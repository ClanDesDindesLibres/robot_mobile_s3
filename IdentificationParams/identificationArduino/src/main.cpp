/* 
 * GRO 302 - Conception d'un robot mobile
 * Code de démarrage
 * Auteurs: Jean-Samuel Lauzon     
 * date: 1 mai 2019
*/

/*------------------------------ Librairies ---------------------------------*/
#include <LibS3GRO.h>
#include <ArduinoJson.h>
#include <libExample.h> // Vos propres librairies




/*------------------------------ Constantes ---------------------------------*/

#define BAUD            115200      // Frequence de transmission serielle
#define UPDATE_PERIODE  100         // Periode (ms) d'envoie d'etat general

#define MAGPIN          32          // Port numerique pour electroaimant
#define POTPIN          A5          // Port analogique pour le potentiometre
#define LIMITSWITCH     42          // Port numérique pour la limite switch

#define PASPARTOUR      64          // Nombre de pas par tour du moteur
#define RAPPORTVITESSE  50          // Rapport de vitesse du moteur

// enum pour les étapes du séquencement
enum Etat { restart, approchePrise, approcheDepot, prise, oscillation, stabilisation, depot, passageObstacle};
/*---------------------------- variables globales ---------------------------*/

ArduinoX AX_;                       // objet arduinoX
MegaServo servo_;                   // objet servomoteur
VexQuadEncoder vexEncoder_;         // objet encodeur vex
IMU9DOF imu_;                       // objet imu
PID pid_;                           // objet PID
PID AGpid_;                         // objet PId angle
MotorControl motor_; 
LS7366Counter encoder_;
 
double t1;
double d1; 
double cur_pos;
double commande = 0;

volatile bool shouldSend_ = false;  // drapeau prêt à envoyer un message
volatile bool shouldRead_ = false;  // drapeau prêt à lire un message
volatile bool shouldPulse_ = false; // drapeau pour effectuer un pulse
volatile bool isInPulse_ = false;   // drapeau pour effectuer un pulse

SoftTimer timerSendMsg_;            // chronometre d'envoie de messages
SoftTimer timerPulse_;              // chronometre pour la duree d'un pulse

uint16_t pulseTime_ = 0;            // temps dun pulse en ms
float pulsePWM_ = 0;                // Amplitude de la tension au moteur [-1,1]


float Axyz[3];                      // tableau pour accelerometre
float Gxyz[3];                      // tableau pour giroscope
float Mxyz[3];                      // tableau pour magnetometre

/*------------------------- Prototypes de fonctions -------------------------*/

void timerCallback();
void startPulse();
void endPulse();
void sendMsg(); 
void readMsg();
void serialEvent();
int GetAngle();
bool oscille();
void GestionEtat(Etat state);

// Fonctions pour le PID
double PIDmeasurement();
void PIDcommand(double cmd);
void PIDgoalReached();
double AGPIDmeasurement();
void AGPIDcommand(double cmd);
void AGPIDgoalReached();

/*---------------------------- fonctions "Main" -----------------------------*/

void setup() {
  Serial.begin(BAUD);               // initialisation de la communication serielle
  AX_.init();                       // initialisation de la carte ArduinoX 
  imu_.init();                      // initialisation de la centrale inertielle
  vexEncoder_.init(2,3);            // initialisation de l'encodeur VEX
  t1 =0;
  d1= AX_.readEncoder(0);
  // attache de l'interruption pour encodeur vex
  //attachInterrupt(vexEncoder_.getPinInt(), []{vexEncoder_.isr();}, FALLING);
  
  motor_.init(5,30);  //initialisation des pins moteur

  //Initialisation des pins
  pinMode(MAGPIN,OUTPUT);      //Initialisation de la pin de l'électroaimant en sortie
  pinMode(LIMITSWITCH,INPUT);  //Initialisation de la pin de la limite switch en entrée


  // Chronometre envoie message
  timerSendMsg_.setDelay(UPDATE_PERIODE);
  timerSendMsg_.setCallback(timerCallback);
  timerSendMsg_.enable();

  // Chronometre duration pulse
  timerPulse_.setCallback(endPulse);
  
  // Initialisation du PID
  pid_.setGains(0.25,0.1 ,0);
  // Attache des fonctions de retour
  pid_.setMeasurementFunc(PIDmeasurement);
  pid_.setCommandFunc(PIDcommand);
  pid_.setAtGoalFunc(PIDgoalReached);
  pid_.setEpsilon(0.001);
  pid_.setPeriod(200);
}

/* Boucle principale (infinie)*/
void loop() {

  if(shouldRead_){
    readMsg();
  }
  if(shouldSend_){
    sendMsg();
  }
  if(shouldPulse_){
    startPulse();
  }

  GestionEtat(restart);     //Gestiondes états pour le séquencement

  // mise a jour des chronometres
  timerSendMsg_.update();
  timerPulse_.update();
  
  // mise à jour du PID
  GetAngle();
  pid_.run();

}


/*---------------------------Definition de fonctions ------------------------*/

void serialEvent(){shouldRead_ = true;}

void timerCallback(){shouldSend_ = true;}

void startPulse(){
  /* Demarrage d'un pulse */
  timerPulse_.setDelay(pulseTime_);
  timerPulse_.enable();
  timerPulse_.setRepetition(1);
  AX_.setMotorPWM(0, pulsePWM_);
  AX_.setMotorPWM(1, pulsePWM_);
  shouldPulse_ = false;
  isInPulse_ = true;
}

void endPulse(){
  /* Rappel du chronometre */
  AX_.setMotorPWM(0,0);
  AX_.setMotorPWM(1,0);
  timerPulse_.disable();
  isInPulse_ = false;
}

void sendMsg(){
  /* Envoit du message Json sur le port seriel */
  StaticJsonDocument<500> doc;
  // Elements du message

  doc["time"] = millis();
  doc["potVex"] = analogRead(POTPIN);
  doc["encVex"] = vexEncoder_.getCount();
  doc["goal"] = pid_.getGoal();
  doc["measurements"] = PIDmeasurement();
  doc["voltage"] = AX_.getVoltage();
  doc["current"] = AX_.getCurrent(); 
  doc["pulsePWM"] = pulsePWM_;
  doc["pulseTime"] = pulseTime_;
  doc["inPulse"] = isInPulse_;
  doc["accelX"] = imu_.getAccelX();
  doc["accelY"] = imu_.getAccelY();
  doc["accelZ"] = imu_.getAccelZ();
  doc["gyroX"] = imu_.getGyroX();
  doc["gyroY"] = imu_.getGyroY();
  doc["gyroZ"] = imu_.getGyroZ();
  doc["isGoal"] = pid_.isAtGoal();
  doc["actualTime"] = pid_.getActualDt();
  doc["cur_vel"] = PIDmeasurement();
  doc["cur_pos"] = cur_pos;
  doc["cmd"] = commande;

  // Serialisation
  serializeJson(doc, Serial);
  // Envoit
  Serial.println();
  shouldSend_ = false;
}

void readMsg(){
  // Lecture du message Json
  StaticJsonDocument<500> doc;
  JsonVariant parse_msg;

  // Lecture sur le port Seriel
  DeserializationError error = deserializeJson(doc, Serial);
  shouldRead_ = false;

  // Si erreur dans le message
  if (error) {
    Serial.print("deserialize() failed: ");
    Serial.println(error.c_str());
    return;
  }
  
  // Analyse des éléments du message message
  parse_msg = doc["pulsePWM"];
  if(!parse_msg.isNull()){
     pulsePWM_ = doc["pulsePWM"].as<float>();
  }

  parse_msg = doc["pulseTime"];
  if(!parse_msg.isNull()){
     pulseTime_ = doc["pulseTime"].as<float>();
  }

  parse_msg = doc["pulse"];
  if(!parse_msg.isNull()){
     shouldPulse_ = doc["pulse"];
  }
  parse_msg = doc["setGoal"];
  if(!parse_msg.isNull()){
    pid_.disable();
    pid_.setGains(doc["setGoal"][0], doc["setGoal"][1], doc["setGoal"][2]);
    pid_.setEpsilon(doc["setGoal"][3]);
    pid_.setGoal(doc["setGoal"][4]);
    pid_.enable();
  }
}


// Fonctions pour le PID
double PIDmeasurement(){
  // double vitesse; 
  //double temps = (millis()-t1)/1000;
  float nbre_encodeur = AX_.readEncoder(0);
  double distance = ((nbre_encodeur)/(2400))*PI*.15;
 
  cur_pos = (((nbre_encodeur))/(2400))*PI*.15;

  return distance;
}
void PIDcommand(double cmd){

  if(pid_.getGoal()>cur_pos){
    AX_.setMotorPWM(0, 0.2);
  }
  else if(pid_.getGoal()<cur_pos){
    AX_.setMotorPWM(0, -0.2);
  }


}
void PIDgoalReached(){
    //Serial.println("GoalReached");
      AX_.setMotorPWM(0, 0);
}

//Fonction pour la gestion d'état
void GestionEtat(Etat state){
  switch (state) {

    case restart: // Retourne au début du rail
      AX_.setMotorPWM(0,-0.2);
      //Faite reculer le robot à une vitesse pas trop vite vers la fin
      // if(AX_.readEncoder(0) <= 500){ //Vérifier la distance pour l'encodeur et pour le ID 
      //   AX_.setMotorPWM(0,-0.2);  //Réduire la vitesse
      // }
      Serial.print("Limit Switch : ");
      Serial.println(digitalRead(LIMITSWITCH));
      if(digitalRead(LIMITSWITCH) == true){ // Reset la valeur de l'encodeur quand rendue au bout du rail
        AX_.resetEncoder(0);
        AX_.setMotorPWM(0,0);
      } 
    break;

    case approchePrise: // Approche au dessus du sapin
      //Valeur de distance pour la prise du sapin
      if(AX_.readEncoder(0) == 500){ // Arrête à la distance du sapin
        AX_.setMotorPWM(0,0);  //Arrête le robot
      }
    break;

    case approcheDepot: // Approche au dessus du bac de dépot
      //Valeur de distance pour la prise du sapin
      if(AX_.readEncoder(0) == 500){ // Arrête à la distance du bac
        AX_.setMotorPWM(0,0);  //Arrête le robot
      }
    break;

    case passageObstacle: // Passe l'obstacle d'un coup

    break;

    case prise:
      digitalWrite(MAGPIN,HIGH);
      delay(500); // vérifier le délai de prise du sapin (si besoin)
    break;

    case depot:
      digitalWrite(MAGPIN,LOW);
      delay(500);//Vérifier le délai de relachement du sapin
    break;

    case oscillation:
    oscille();

    break;

    case stabilisation: // Permet de stabiliser le pendule au dessus de l'objet/bac de dépot

    break;

  }

}
int GetAngle(){
  // capteur
  int angle=0;
  int val = 0;
  val=analogRead(A5);
  angle = (val-560)*180/900;
//Serial.println("Angle : " + String(angle));
  return angle;
}
double AGPIDmeasurement(){return 0.0;}
void AGPIDcommand(double cmd){}
void AGPIDgoalReached(){}
bool oscille(){
  int angle=0;
  angle=GetAngle();
      pid_.setGoal(0.2);
    while(PIDmeasurement()<=0.2)
    {angle=GetAngle();
     if(angle <= -65)
     {return true;}
    }
    pid_.setGoal(-0.2);
    while(PIDmeasurement()>=-0.2)
    {angle=GetAngle();
    if (angle<= -65)
      {return true;}
    }
  return false;}