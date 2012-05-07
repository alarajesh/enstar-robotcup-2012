#include <Wire.h>
#include <SoftwareSerial.h>
#define MD25ADDRESS 0x58 // Adresse de la carte moteur //0x58
#define SPEED1              (byte)0x00  // Adresse où est stockée la vitesse du moteur 1
#define SPEED2              0x01        // Adresse où est stockée la vitesse du moteur 2
#define ENCODER1            0x02        // Adresse du registre contenat la valeur de l'encodeur du moteur 1
#define ENCODER2            0x06  // Adresse du registre contenant la valeur de l'encodeur du moteur 2
#define CURRENT1            0xB   // Adresse du registre contenant la valeur de l'intensité du courant du moteur 1
#define CURRENT2            0xC   // Adresse du registre contenant la valeur de l'intensité du courant du moteur 2
#define CMD                 0x10
#define ACCELERATION_RATE   0x0E  // Adresse du registre contrôlant l'accélération du robot
long encoder1;
long encoder2;
byte current1;
byte current2;
 

float distance_current; 
long orientation_current;// Orientation du robot issue de la dernière mesure
long distance_old;  

float Kp = 0.1;//Coefficient de la commande proportionnelle :Kd = 0.3
float Kd = 0.1;// Coefficient de la commande dérivée// KP=0.3
int commande;// Commande calculée per l'asservissement
unsigned long time_begin;// Date du début de la période d'échantillonnage en cours
unsigned long sampling_period = 100; // Periode d'échantillonage en millisecondes
long waitingTime;// Temps d'attente de la fin de la période d'échantillonnage
int  count ;// Pour l'asservissement 


void setup(){
  // Initialisation de la communication série avec l'ordinateur
  Serial.begin(9600);
  //Initialisation de la communication I2C (protocole pour communiquer avec la carte moteur
  Wire.begin();
  //Moteurs à l'arret
  setSpeed1(128);
  setSpeed2(128);
  //Encodeurs mis à zéro
  resetEncoders();
  delay(2000);// Attente de 2 secondes
  setAccelerationRate(1);
  getAccelerationRate();
  encoder1 = getEncoder1();
  encoder2 = getEncoder2();
  Serial.println(encoder1);
  Serial.println(encoder2);
  //Initialisation de la position du robot 
  distance_old = 0;
  distance_current = 0;
  Serial.print("Commande proportionnelle : ");
  Serial.println(Kp);
  Serial.println("Initialisation terminée");
  delay(2000);

  time_begin = millis();
}



void loop(){


  translate(700);
  delay(6000);
  translate(0);
  delay(6000);




}



// Translate le robot 
void translate (long x){// 700 <---> dépacer de 30 cm

  time_begin = millis();

  /* On entre dans la boucle d'asservissement. On ne sort de cette boucle que lorsque l'asservissement est considéré comme terminé.
   C'est-à_dire quand le compteur count à atteint la valeur 5           */
  while(count <= 5){

    /* Si l'erreur est en valeur absolue inférieure à une certaine valeur, on incrémente */
    if ( (getEncoder1() + getEncoder2() - x) <= 10 && (getEncoder1() + getEncoder2() - x >= -10)){
      count++;     
    }

    distance_old = distance_current;// la dernière mesure de l'orientation devient l'ancienne mesure

    /* Si la période d'échantillonnage est trop courte */
    if (waitingTime = ( sampling_period - (millis() - time_begin) ) < 0){
      Serial.println("Erreur : la boucle dure plus longtemps que la période d'échantillonnage : il faut augmenter la période d'échantillonnage-------------------------------------------------");
    }
    delay(waitingTime );// On attend la fin de la période d'échantillonnage

    Serial.print("Temps d'attente  " );
    Serial.println(waitingTime);

    // début de la nouvelle période d'échantillonnage
    time_begin = millis();
    // Appel à la fonction qui va procéder à l'asservissment
    asservissement_distance(x);

  }

  // L'asservissement est terminé, on remet le compteur à zéro
  count = 0;

}


// Fonction appelée par translate : fait l'asservissement en translation
void asservissement_distance(long x){
  Serial.println("move");
  distance_current=(getEncoder1() + getEncoder2());
  Serial.println(distance_current);
  // Calcul de la comande
  commande = Kp*(x - distance_current) + Kd*(distance_old - distance_current);
  Serial.print("Commande : ");
  Serial.println(commande);
  Serial.println(" ");
  // Saturation en vitesse
  if(commande >= 128 ){
    setSpeed1(255);
    setSpeed2(255);
  }
  else if(commande <= -128){
    setSpeed1(0);
    setSpeed2(0);
  }
  else{

    setSpeed1(128 + commande);
    setSpeed2(128 + commande);

  }



}

// Garde fou : arrête les moteurs si le courant dans les moteurs est supérieur à x
void watchCurrents(float x){
  if(getCurrent1() > x || getCurrent2() > x ){
    setSpeed1(128);
    setSpeed2(128);
    Serial.println("intensité trop grande !--------------------------------------------------------------------------------------------");
  }
}

// Commande de vitesse du moteur 1 : x =128 : arrêt ; x = 0: marche arrière max; x =255 : marche avant max
void setSpeed1(int x){
  // Début de la communication avec la carte moteur
  Wire.beginTransmission(MD25ADDRESS);
  // Accès au registre stockant la vitesse du moteur 1
  Wire.write(SPEED1);
  // Ecriture dans le registre de la consigne en vitesse
  Wire.write(x);   
  // Fin de la communication
  Wire.endTransmission();
}

// Commande de vitesse du moteur 2
void setSpeed2(int x){
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(SPEED2);
  Wire.write(x);
  Wire.endTransmission();
}


long getEncoder1(){ // Lecture de l'encodeur du moteur 1
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(ENCODER1);
  Wire.endTransmission();

  Wire.requestFrom(MD25ADDRESS,4);
  while(Wire.available() < 4);// l'encodeur code sur 32 bits. Il faut donc attendre que la carte moteur renvoit 4 bytes
  long encoder1 = Wire.read(); // Reception des 8 bits de poids les plus forts
  encoder1 <<= 8;
  encoder1 += Wire.read();
  encoder1 <<= 8;
  encoder1 += Wire.read();
  encoder1 <<= 8;
  encoder1 += Wire.read();// Reception des 8 bits de poids les plus faibles

  return(encoder1);
}


long getEncoder2(){ // Lecture de l'encodeur du moteur 2
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(ENCODER2);
  Wire.endTransmission();

  Wire.requestFrom(MD25ADDRESS,4);
  while(Wire.available() < 4);
  long encoder2 = Wire.read();
  encoder2 <<= 8;
  encoder2 += Wire.read();
  encoder2 <<= 8;
  encoder2 += Wire.read();
  encoder2 <<= 8;
  encoder2 += Wire.read();

  return(encoder2);




}
void resetEncoders(){// Met à 0 les encodeurs
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(CMD);
  Wire.write(0x20);
  Wire.endTransmission();

}

byte getCurrent1(){ // Lit le courant dans le moteur 1 : la valeur retournée est égale à 10 fois l'intensite : 10 <--> 1A
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(CURRENT1);
  Wire.endTransmission();

  Wire.requestFrom(MD25ADDRESS,1);
  while(Wire.available() < 1);
  byte current1 = Wire.read();

  return(current1);
}

byte getCurrent2(){ // Lit le courant dans le moterur 2 : la valeur retournée est égale à 10 fois l'intensite : 10 <--> 1A
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(CURRENT2);
  Wire.endTransmission();

  Wire.requestFrom(MD25ADDRESS,1);
  while(Wire.available() < 1);
  byte current2 = Wire.read();

  return(current2);

}

void setAccelerationRate(int x){
  if(x >=1 && x <=10){
    Wire.beginTransmission(MD25ADDRESS);
    Wire.write(ACCELERATION_RATE);
    Wire.write(x);
    Wire.endTransmission();
  }
  else{
    Serial.println("Valeur rentrée incorrecte pour le taux d'accélération");
  }
}

void getAccelerationRate(){// Affiche à l'écran le taux d'accélération : 1(minimale) à 10 (maximal)
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(ACCELERATION_RATE);
  Wire.endTransmission();
  Wire.requestFrom(MD25ADDRESS,1); 
  while(Wire.available() < 1); 
  byte acceleration = Wire.read();
  Serial.println(acceleration);
}





