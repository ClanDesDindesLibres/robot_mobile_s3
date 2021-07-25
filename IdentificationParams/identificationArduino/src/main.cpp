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
#include <math.h>




/*------------------------------ Constantes ---------------------------------*/

#define BAUD            115200      // Frequence de transmission serielle
#define UPDATE_PERIODE  100         // Periode (ms) d'envoie d'etat general

#define MAGPIN          32          // Port numerique pour electroaimant
#define POTPIN          A5          // Port analogique pour le potentiometre
#define LIMITSWITCH     42          // Port numérique pour la limite switch

#define PASPARTOUR      64          // Nombre de pas par tour du moteur
#define RAPPORTVITESSE  50          // Rapport de vitesse du moteur

// enum pour les étapes du séquencement
enum Etat { restart, avancePrise, prise, oscillation, avanceDepot, stabilisation, depot};
int State = 1;
  
/*---------------------------- variables globales ---------------------------*/

ArduinoX AX_;                       // objet arduinoX
MegaServo servo_;                   // objet servomoteur
VexQuadEncoder vexEncoder_;         // objet encodeur vex
IMU9DOF imu_;                       // objet imu
PID pid_pos_;                           // objet PID
PID AGpid_;                         // objet PId angle
MotorControl motor_; 
LS7366Counter encoder_;
 
double t1;
double d1; 
double cur_pos;

//Variables 
double commande = 0;
double angleGlob = 0;
bool comment = true;

//Oscillation du pendule
float T = (2*PI)*sqrt(0.37/9.81);
//float T = 1;
float w = (2*PI)/T;
float Amp = -0.5;
float v;
bool osci = true;
long double tinit;
long double t;

volatile bool shouldSend_ = false;  // drapeau prêt à envoyer un message
volatile bool shouldRead_ = false;  // drapeau prêt à lire un message
volatile bool shouldPulse_ = false; // drapeau pour effectuer un pulse
volatile bool isInPulse_ = false;   // drapeau pour effectuer un pulse

bool test_pid;


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
void SetUp_PID(PID, double);
void serialEvent();
int GetAngle();
bool oscille();
void GestionEtat();

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
  
  //initialisation des pins moteur
  motor_.init(5,30);

  //Initialisation de l'état initiale
  State = restart;

  //Pendule paramètres d'oscillation
  Serial.print("T = ");Serial.println(T);
  Serial.print("w = ");Serial.println(w); 

  //Initialisation des pins
  pinMode(MAGPIN,OUTPUT);      //Initialisation de la pin de l'électroaimant en sortie
  pinMode(LIMITSWITCH,INPUT);  //Initialisation de la pin de la limite switch en entrée


  // Chronometre envoie message
  timerSendMsg_.setDelay(UPDATE_PERIODE);
  timerSendMsg_.setCallback(timerCallback);
  //timerSendMsg_.enable();

  // Chronometre duration pulse
  timerPulse_.setCallback(endPulse);
  
  //
  test_pid = false;

  // Initialisation du PID
  pid_pos_.setGains(5, 0.4 ,0.0004);
  // Attache des fonctions de retour
  pid_pos_.setMeasurementFunc(PIDmeasurement);
  pid_pos_.setCommandFunc(PIDcommand);
  pid_pos_.setAtGoalFunc(PIDgoalReached);
  pid_pos_.setEpsilon(0.05);
  pid_pos_.setPeriod(200);

  //pid_pos_.enable();
}

/* Boucle principale (infinie)*/
void loop() {

  if(shouldRead_){
    //readMsg();
  }
  if(shouldSend_){
    sendMsg();
  }
  if(shouldPulse_){
    startPulse();
  }
  // mise a jour des chronometres
  timerSendMsg_.update();
  timerPulse_.update();
  
  // mise à jour du PI
  pid_pos_.run();

  //Fonction pour la gestion d'état
  GestionEtat();
  //Serial.println(State);
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
  doc["goal"] = pid_pos_.getGoal();
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
  doc["isGoal"] = pid_pos_.isAtGoal();
  doc["actualTime"] = pid_pos_.getActualDt();
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
    pid_pos_.disable();
    pid_pos_.setGains(doc["setGoal"][0], doc["setGoal"][1], doc["setGoal"][2]);
    pid_pos_.setEpsilon(doc["setGoal"][3]);
    pid_pos_.setGoal(doc["setGoal"][4]);
    pid_pos_.enable();
    
  }
}

  // void SetUp_PID(PID pid, double goal){

  //   pid.disable();
  //   pid.setGains(0.4, 0.04, 0.0004);
  //   pid.setEpsilon(0.05);
  //   pid.setGoal(goal);
  //   pid.enable();
  // Serial.println("SETUP");
  // }

// Fonctions pour le PID
double PIDmeasurement(){
  // double vitesse; 
  // double temps = (millis()-t1)/1000;
  float nbre_encodeur = AX_.readEncoder(0);
  double distance = ((nbre_encodeur)/(2400))*PI*.15; // metres
  cur_pos = distance;
  Serial.print("Position : "); Serial.println(cur_pos);
  return distance;

}
void PIDcommand(double cmd){
  commande = cmd;
  float vitesse = commande/((PI*1.5));
  Serial.print("Commande : "); Serial.println(commande);
  Serial.print("Destination : "); Serial.println(pid_pos_.getGoal());
  //Évite de prendre en compte le PID lors du passage de l'obstacle
  if(State == avanceDepot){}
  else{AX_.setMotorPWM(0, vitesse);}
  //AX_.setMotorPWM(0, vitesse);
}

void PIDgoalReached(){
  Serial.println("Goal Reached");
  //Désactive le moteur pour immobiliser le robot
  AX_.setMotorPWM(0, 0);
}





//Fonction pour la gestion d'état
void GestionEtat(){
  switch (State) {

    case restart: // Retourne au début du rail
      if (comment){Serial.println("RESTART"); comment = false;}
      //Diminution de vitesse vers le début du rail
      if(AX_.readEncoder(0) <= 500){ //Vérifier la distance pour l'encodeur et pour le ID 
        AX_.setMotorPWM(0,-0.15);
      }
      //Fait reculer le robot
      else{AX_.setMotorPWM(0,-0.6);}

      //Serial.print("Limit Switch : "); Serial.println(digitalRead(LIMITSWITCH));
      if(digitalRead(LIMITSWITCH) == true){
        //Arrête le moteur
        AX_.setMotorPWM(0,0);
        //Reset l'encodeur
        AX_.resetEncoder(0);
        //Changement d'état
        State++;
        comment = true;
      } 
    break;

    // Approche au dessus du sapin
    case avancePrise:
      if (comment){
        Serial.println("AVANCER PRISE");
        //Active le PID
        pid_pos_.enable(); 
        comment = false;
      }
      //Valeur de distance pour la prise du sapin
      pid_pos_.setGoal(0.4);
      //Serial.println("PID ACTIF");
      if (pid_pos_.isAtGoal()){State++;comment = true;}
    break;

    //Prise du sapin
    case prise:
      if (comment){Serial.println("PRISE"); comment = false;}
      //Active le magnet
      digitalWrite(MAGPIN,LOW);//Mettre à High, low pour test sans sapin
      delay(3000); // vérifier le délai de prise du sapin (si besoin)
      State++;
      comment = true;
    break;

    //Phase d'oscillation du pendule
    case oscillation:
      //Débute le timer de l'oscillation
      if (osci) {tinit = millis(); osci = false;}
      t = (millis() - tinit)/1000;//Change la valeur en secondes

      if (comment){Serial.println("OSCILLATION"); comment = false;}
      //Calcul de la vitesse du moteur
      v = Amp*sin((w*t));
      AX_.setMotorPWM(0,v);
      //Serial.print("Vitesse: "); Serial.println(v);
      if (v < 0 && GetAngle() <= -50){
        while(GetAngle() >= 20);//Attend que le pendule reparte vers l'avant pour partir
        AX_.setMotorPWM(0,0.55);//Fait partir le robot vers le dépot// problème si pas l'à
        osci = true;
        comment = true;
        State++;
      }

    break;

    // Passe l'obstacle d'un coup
    case avanceDepot:
      if (comment){
        Serial.println("AVANCE DEPOT"); 
        comment = false;
        //Active le PID
        pid_pos_.enable();
      }
      //Valeur de distance pour la prise du sapin
      pid_pos_.setGoal(1); // Dépasse la valeur trop vite donc continue à avancer
      //On doit trouver une façon de le faire arrêter même si il dépasse la distance et l'erreur
      //Serial.println("PID ACTIF");
      if (pid_pos_.isAtGoal()){State++;comment = true;}
    break;

    //Permet de stabiliser le pendule // à faire
    case stabilisation:
      if (comment){Serial.println("STABILISATION"); comment = false;}
      //v = 0.2*sin(GetAngle());
      //AX_.setMotorPWM(0,v);
      //comment = true;
      //State++;
    break;

    case depot:
      if (comment){Serial.println("DEPOT"); comment = false;}
      digitalWrite(MAGPIN,LOW);
      delay(500);//Vérifier le délai de relachement du sapin
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
/*
double AGPIDmeasurement(){
  double angle = GetAngle();
  angleGlob = angle;
  return angle;
}
void AGPIDcommand(double cmd){
  if(AGpid_.getGoal()>angleGlob){
    AX_.setMotorPWM(0, -0.2);
  }
  else if(AGpid_.getGoal()<angleGlob){
    AX_.setMotorPWM(0, 0.2);
  }
}
void AGPIDgoalReached(){
  AX_.setMotorPWM(0, 0);
}
bool oscille(){
  int angle=0;
  angle=GetAngle();
      pid_pos_.setGoal(0.5);
    while(PIDmeasurement()<=0.5)
    {angle=GetAngle();
     if(angle <= -65)
     {return true;}
    }
      pid_pos_.setGoal(0.1);
    while(PIDmeasurement()>=0.1)
    {angle=GetAngle();
    if (angle<= -65)
      {return true;}
    }
  return false;}
  */
  