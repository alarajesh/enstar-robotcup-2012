// vim:ft=cpp

/*
 * Ce fichier contient le programme commandant les pinces du robot.
 * L'adresse de la carte moteur est 0x58.
 * Deux fonctions sont à utiliser : OuvrirPince() (ouvre les pinces)
 * et FermerPince() (ferme les pinces).
 * L'amplitude de l'ouverture fermeture se règle dans le code de ces
 * fonctions : changer la valeur seuil de getEncoder1() et getEncoder2().
 * Lorque les encodeurs1 et 2 sont à zéro, cela correspond à la position
 * ouverte des pinces
 */

#include <Wire.h>
#include <SoftwareSerial.h>
#define MD25ADDRESS 0x58        // Adresse de la carte moteur
#define SPEED1      (byte)0x00  // Adresse où est stockée la vitesse du moteur 1
#define SPEED2      0x01        // Adresse où est stockée la vitesse du moteur 2
#define ENCODER1    0x02        // Adresse du registre contenat la valeur de l'encodeur du moteur 1
#define ENCODER2    0x06        // Adresse du registre contenant la valeur de l'encodeur du moteur 2
#define CURRENT1    0xB         // Adresse du registre contenant la valeur de l'intensité du courant du moteur 1
#define CURRENT2    0xC   // Adresse du registre contenant la valeur de l'intensité du courant du moteur 2
#define CMD         0x10
#define ACCELERATION_RATE   0x0E  // Adresse du registre contrôlant l'accélération du robot
#define SLAMP_MAX   200

long encoder1;
long encoder2;
byte current1;
byte current2;
long distance_reference = 360;
long orientation_reference = 0;// Consigne en orientation du robot
float distance_current; 
long orientation_current = 0;// Orientation du robot issue de la dernière mesure
long distance_old;  
long orientation_old = 0;// Orientation du robot issue de l'avant dernière mesure
float Kp = 0.2;//Coefficient de la commande proportionnelle :Kd = 0.3
float Kd = 0;// Coefficient de la commande dérivée// KP=0.3
int commande;
unsigned long time_begin;
unsigned long time_end;
unsigned long sampling_period = 100; // Periode d'échantillonage en millisecondes
long waitingTime;

void setup()
{
	// Initialisation de la communication série avec l'ordinateur
	Serial.begin(9600);
	//Initialisation de la communication I2C (protocole pour communiquer avec la carte moteur
	Wire.begin();
	// Moteurs à l'arrêt
	setSpeed1(128);
	setSpeed2(128);
	//Initialisation des encodeurs
	resetEncoders();
	delay(2000);// Attente de 2 secondes
	setAccelerationRate(5);
	getAccelerationRate();
	encoder1 = getEncoder1();
	encoder2 = getEncoder2();
	Serial.print("Encoder 1 = ");
	Serial.println(encoder1);
	Serial.print("Encodeur2 = ");
	Serial.println(encoder2);
	Serial.println();
	//Initialisation de l'orientation du robot 
	distance_old = 0;
	distance_current = 0;
	Serial.print("Commande proportionnelle : ");
	Serial.println(Kp);
	Serial.println("Initialisation terminée");
	delay(2000);
	time_begin = millis();
}


/* Les pinces se ferment puis s'ouvrent périodiquement */
void loop()
{
	FermerPince();
	Serial.println("Les pinces sont en position fermée");
	delay(3000);
	OuvrirPince();
	Serial.println("Les pinces sont en position ouverte");
	delay(3000);
}

// Commande en fermeture de la pince
/* Lors de la fermeture des pinces, le moteur tourne dans le sens horaire. Quand la pince est en position ouverte, l'encodeur est à 0. Donc tant que
   l'encodeur est en dessous d'une certaine valeur (SLAMP_MAX par exemple), on ordonne au moteur de tourner dans le sens antihoraire
   */
void FermerPince()
{

	while(getEncoder1() <= SLAMP_MAX || getEncoder2() <= SLAMP_MAX )
{// Faire tourner le(s) moteur(s) qui n'ont pas encore atteint la valeur seuil

		Serial.print("Fermre encodeur 1 :");
		Serial.println(getEncoder1());
		Serial.print("Ferme encodeur 2 :");
		Serial.println(getEncoder2());

		if(getEncoder1() <= SLAMP_MAX)
{// Tourner moteur1 tant que encodeur 1 est en dessous de la valeur seuil
			setSpeed1(140);
		}
		else if(getEncoder1() > SLAMP_MAX)
{// Arrêter le moteur 1 si la valeur seuil est atteinte
			setSpeed1(128);
		}
		if(getEncoder2() <= SLAMP_MAX)
{// Tourner moteur2 tant que encodeur 2 est en dessous de la valeur seuil
			setSpeed2(140);
		}
		else if(getEncoder1() > SLAMP_MAX)
{// Arrêter le moteur 2 si la valeur seuil est atteinte
			setSpeed2(128);
		}

	}
	setSpeed1(128);
	setSpeed2(128);
}

// Commande en ouverture de la pince 
/*Lors de l'ouverture des pinces, les moteurs tournent dans le sens antihoraire(la valeur des encodeurs diminue donc). On fait tourner
 * les moteurs tant que cette valeur est positive (Rappel : la position zéro correspond à la position ouverte des pinces
 */
void OuvrirPince()
{

	while(getEncoder1() >= 0 || getEncoder2() >=0  )
{// Faire tourner le(s) moteur(s) qui n'ont pas encore atteint la valeur seuil
		Serial.println("Ouverture");
		Serial.print("Valeur encod1 pour ouverture : ");
		Serial.println(getEncoder1());
		Serial.print("Valeur encod2 pour ouverture ");
		Serial.println(getEncoder2());

		if(getEncoder1() >= 0)
{// L'encodeur 1 a une valeur >=0, on fait tourner le moteur1 dans le sens antihoraire
			setSpeed1(116);
		}
		else if(getEncoder1() < 0)
{// La valeur de l'encodeur1 est <0, arrêter moteur1 car la pince est en position ouverte
			setSpeed1(128);
		}
		if(getEncoder2() >= 0)
{// L'encodeur 2 a une valeur >=0, on fait tourner le moteur2 dans le sens antihoraire
			setSpeed2(116);
		}
		else if(getEncoder2() < 0)
{// La valeur de l'encodeur2 est <0, arrêter moteur2 car la pince est en position ouverte
			setSpeed2(128);
		}
	}

	setSpeed1(128);
	setSpeed2(128);
}


// Commande de vitesse du moteur 1 : x =128 : arrêt ; x = 0: marche arrière max; x =255 : marche avant max
void setSpeed1(int x)
{
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
void setSpeed2(int x)
{
	Wire.beginTransmission(MD25ADDRESS);
	Wire.write(SPEED2);
	Wire.write(x);
	Wire.endTransmission();
}


long getEncoder1()
{ // Lecture de l'encodeur du moteur 1
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


long getEncoder2()
{ // Lecture de l'encodeur du moteur 2
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
void resetEncoders()
{// Met à 0 les encodeurs
	Wire.beginTransmission(MD25ADDRESS);
	Wire.write(CMD);
	Wire.write(0x20);
	Wire.endTransmission();

}

byte getCurrent1()
{ // Lit le courant dans le moteur 1 : la valeur retournée est égale à 10 fois l'intensite : 10 <--> 1A
	Wire.beginTransmission(MD25ADDRESS);
	Wire.write(CURRENT1);
	Wire.endTransmission();

	Wire.requestFrom(MD25ADDRESS,1);
	while(Wire.available() < 1);
	byte current1 = Wire.read();

	return(current1);
}

byte getCurrent2()
{ // Lit le courant dans le moterur 2 : la valeur retournée est égale à 10 fois l'intensite : 10 <--> 1A
	Wire.beginTransmission(MD25ADDRESS);
	Wire.write(CURRENT2);
	Wire.endTransmission();

	Wire.requestFrom(MD25ADDRESS,1);
	while(Wire.available() < 1);
	byte current2 = Wire.read();

	return(current2);

}

void setAccelerationRate(int x)
{
	if(x >=1 && x <=10)
	{
		Wire.beginTransmission(MD25ADDRESS);
		Wire.write(ACCELERATION_RATE);
		Wire.write(x);
		Wire.endTransmission();
	}
	else
	{
		Serial.println("Valeur rentrée incorrecte pour le taux d'accélération");
	}
}

void getAccelerationRate()
{
	// Affiche à l'écran le taux d'accélération : 1(minimale) à 10 (maximal)
	Wire.beginTransmission(MD25ADDRESS);
	Wire.write(ACCELERATION_RATE);
	Wire.endTransmission();
	Wire.requestFrom(MD25ADDRESS,1);
	while(Wire.available() < 1);
	byte acceleration = Wire.read();
	Serial.println(acceleration);
}

void rotate(int x)
{// Fait tourner le robot : x=458 <--> 90°

	/* Calcul de l'orientation actuelle du robot à partir de la mesure des encodeurs des roue */
	Serial.print("Encodeur 1 : ");
	Serial.println(getEncoder1());

	Serial.print("Encodeur 2 : ");
	Serial.println(getEncoder2());
	orientation_current = getEncoder1() - getEncoder2();
	Serial.print("Orientation : ");
	Serial.println(orientation_current);

	//Calcul de la commande à partir de cette mesure
	commande =  Kp*(x - orientation_current) + Kd*(orientation_old - orientation_current) ; // J'ai changé le signe!!
	Serial.print("Commande : ");
	Serial.println(commande);
	Serial.println(" ");
	if (commande <= 127 && commande >= -127 )
{

		setSpeed1(commande + 128);
		setSpeed2(-commande + 128);
	}
	else if (commande >= 128)
{ // Saturation de la vitesse
		setSpeed1(255);
		setSpeed2(0);
	}
	else if(commande <= -128)
{ // Saturation de la vitesse
		setSpeed1(0);
		setSpeed2(255);
	}



}


void move(long x)
{
	Serial.println("move");
	distance_current=(getEncoder1() + getEncoder2());
	Serial.println(distance_current);
	commande = Kp*(distance_reference - distance_current) + Kd*(distance_old - distance_current);
	Serial.print("Commance : ");
	Serial.println(commande);
	Serial.println(" ");
	//Faire les saturations!!
	setSpeed1(128+ commande);
	setSpeed2(128 + commande);





}


