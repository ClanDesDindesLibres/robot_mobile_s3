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
enum Etat { restart, avance, recule, prise, oscillation, stabilisation, depot, passageObstacle};
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
double commande = 0;
double angleGlob = 0;

volatile bool shouldSend_ = false;  // drapeau prêt à envoyer un message
volatile bool shouldRead_ = false;  // drapeau prêt à lire un message
volatile bool shouldPulse_ = false; // drapeau pour effectuer un pulse
volatile bool isInPulse_ = false;   // drapeau pour effectuer un pulse

bool test_pid;
bool GOAL = false;

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

      while(digitalRead(LIMITSWITCH) == false)
      {AX_.setMotorPWM(0, -0.15);}

      if(digitalRead(LIMITSWITCH) == true){ // Reset la valeur de l'encodeur quand rendue au bout du rail
        AX_.resetEncoder(0);
          // Serial.print("Limit Switch : ");
          // Serial.println(digitalRead(LIMITSWITCH));
        AX_.setMotorPWM(0,0);}

      //PRISE SAPIN
      //digitalWrite(MAGPIN,HIGH);
      delay(500); // vérifier le délai de prise du sapin (si besoin)


  //Initialisation des pins
  pinMode(MAGPIN,OUTPUT);      //Initialisation de la pin de l'électroaimant en sortie
  pinMode(LIMITSWITCH,INPUT);  //Initialisation de la pin de la limite switch en entrée


  // Chronometre envoie message
  timerSendMsg_.setDelay(UPDATE_PERIODE);
  timerSendMsg_.setCallback(timerCallback);
  timerSendMsg_.enable();

  // Chronometre duration pulse
  timerPulse_.setCallback(endPulse);
  
  //
  test_pid = false;

  // Initialisation du PID
  pid_pos_.setGains(7, 0.4 ,0.0004);
  // Attache des fonctions de retour
  pid_pos_.setMeasurementFunc(PIDmeasurement);
  pid_pos_.setCommandFunc(PIDcommand);
  pid_pos_.setAtGoalFunc(PIDgoalReached);
  pid_pos_.setEpsilon(0.05);
  pid_pos_.setPeriod(200);

  pid_pos_.enable();
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

  //GestionEtat(restart);     //Gestiondes états pour le séquencement

  // mise a jour des chronometres
  timerSendMsg_.update();
  timerPulse_.update();
  
  // mise à jour du PID
<<<<<<< HEAD
  pid_pos_.run();

//Fonction pour la gestion d'état
  switch (State) 
  {
    case restart: // Retourne au début du rail
      // AX_.setMotorPWM(0, -0.1);

      // if(digitalRead(LIMITSWITCH) == true){ // Reset la valeur de l'encodeur quand rendue au bout du rail
      //   AX_.resetEncoder(0);
      //     // Serial.print("Limit Switch : ");
      //     // Serial.println(digitalRead(LIMITSWITCH));
      //   AX_.setMotorPWM(0,0);
      //   State++;
      // } 

    break;
 

    case avance: // Approche au dessus du sapin
      //Valeur de distance pour la prise du sapin
      //Serial.print("approcheprise");
      pid_pos_.setGoal(0.5);
      
       if(GOAL)
       {
         Serial.println("changement etat 1 ");
         State++;
       }
      GOAL = false;
    break;

    case recule: // Approche au dessus du sapin
      //Valeur de distance pour la prise du sapin
      //Serial.print("approcheprise");
      pid_pos_.setGoal(0.2);
      Serial.println("GETGOAL STATS: ");
      Serial.print(pid_pos_.getGoal());
      
      if(GOAL)
      {
         Serial.println("changement etat 2 ");
        State--;
      }
      GOAL = false;
    break;
       

    // case approcheDepot: // Approche au dessus du bac de dépot
    //   //Valeur de distance pour la prise du sapin
    //   while(true){
    //     Serial.println("WAZAAAAA");
    //   }
    //   if(AX_.readEncoder(0) == 500){ // Arrête à la distance du bac
    //     AX_.setMotorPWM(0,0);  //Arrête le robot
    //   }
    // break;
/*
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
*/
  }
=======
  GetAngle();
  pid_.run();
  AGpid_.run();
>>>>>>> 5812513c1cd704c9ef0e63513a1e89a026a2dedf

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

 // doc["time"] = millis();
  // doc["potVex"] = analogRead(POTPIN);
  // doc["encVex"] = vexEncoder_.getCount();
  doc["goal"] = pid_pos_.getGoal();
  doc["measurements"] = PIDmeasurement();
  // doc["voltage"] = AX_.getVoltage();
  // doc["current"] = AX_.getCurrent(); 
  // doc["pulsePWM"] = pulsePWM_;
  // doc["pulseTime"] = pulseTime_;
  // doc["inPulse"] = isInPulse_;
  // doc["accelX"] = imu_.getAccelX();
  // doc["accelY"] = imu_.getAccelY();
  // doc["accelZ"] = imu_.getAccelZ();
  // doc["gyroX"] = imu_.getGyroX();
  // doc["gyroY"] = imu_.getGyroY();
  // doc["gyroZ"] = imu_.getGyroZ();
  // doc["isGoal"] = pid_pos_.isAtGoal();
  //doc["actualTime"] = pid_pos_.getActualDt();
  //doc["cur_vel"] = PIDmeasurement();
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
   Serial.println("Distance: ");
   Serial.println(distance);
  cur_pos = (((nbre_encodeur))/(2400))*PI*.15;
  return distance;

}
void PIDcommand(double cmd){
  GOAL = false;
  commande = cmd;
  AX_.setMotorPWM(0, cmd/6);
// //OSCILLATION
//   if(pid_pos_.getGoal()>cur_pos){
//     AX_.setMotorPWM(0, 0.2);
//   }
//   else if(pid_pos_.getGoal()<cur_pos){
//     AX_.setMotorPWM(0, -0.2);
//    }
}


void PIDgoalReached(){
    //Serial.println("GoalReached");
    AX_.setMotorPWM(0, 0);
    GOAL = true;
}





/*
//Fonction pour la gestion d'état
void GestionEtat(Etat state){
  switch (state) {

    case restart: // Retourne au début du rail
      AX_.setMotorPWM(0,-0.2);
      //Faite reculer le robot à une vitesse pas trop vite vers la fin
      // if(AX_.readEncoder(0) <= 500){ //Vérifier la distance pour l'encodeur et pour le ID 
      //   AX_.setMotorPWM(0,-0.2);  //Réduire la vitesse
      // }
      // Serial.print("Limit Switch : ");
      // Serial.println(digitalRead(LIMITSWITCH));
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
<<<<<<< HEAD
double AGPIDmeasurement(){return 0.0;}
void AGPIDcommand(double cmd){}
void AGPIDgoalReached(){}
/*
bool oscille(){
  int angle=0;
  while(GetAngle()<= -30){
        pid_pos_.setGoal(0.5);
        pid_pos_.setGoal(0.1);
  }
  return false;}


=======
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
>>>>>>> 5812513c1cd704c9ef0e63513a1e89a026a2dedf
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
  