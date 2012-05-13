// vim:ft=c
/*  Ce programme défini une trajectoire que le robot doit suivre */

#include <Wire.h>
#include <SoftwareSerial.h>

/*******************************************************************
 **** Définition des adresses des registres de la carte moteur  ****
 *******************************************************************/

#define MD25ADDRESS_ROUES 0x5F        // Adresse de la carte moteur pour les roues
#define SPEED1            (byte) 0x00 // Adresse où est stockée la vitesse du moteur 1
#define SPEED2            0x01        // Adresse où est stockée la vitesse du moteur 2
#define ENCODER1          0x02        // Adresse du registre contenat la valeur de l'encodeur du moteur 1
#define ENCODER2          0x06        // Adresse du registre contenant la valeur de l'encodeur du moteur 2
#define CURRENT1          0xB         // Adresse du registre contenant la valeur de l'intensité du courant du moteur 1
#define CURRENT2          0xC         // Adresse du registre contenant la valeur de l'intensité du courant du moteur 2
#define CMD               0x10
#define ACCELERATION_RATE 0x0E        // Adresse du registre contrôlant l'accélération du robot

	/****************************
	 **** Variables globales ****
	 ****************************/

long encoder1; // variable dans laquelle sera stockée la valeur de l'encodeur du moteur 1
long encoder2; // variable dans laquelle sera stockée la valeur de l'encodeur du moteur 2
byte current1; // variable dans laquelle sera stockée la valeur du courant dans le moteur 1
byte current2; // variable dans laquelle sera stockée la valeur du courant dans le moteur 2

long orientation_current; // Orientation du robot issue de la dernière mesure
long orientation_old;     // Orientation du robot issue de l'avant dernière mesure

long distance_current;    // Position actuelle du robot
long distance_old;        // Position du robot durant la période d'échantillonnage précédente

float Kp_rotation = 0.1;  // Coefficient de la commande proportionnelle : 
float Kd_rotation = 0;    // Coefficient de la commande dérivée// 0.1 si acceleration = 0.5
float Kp_translation = 0.1;
float Kd_translation = 0.1;
int commande_rotation;    // commande calculée lors de l'asservissement en rotation
int commande_translation; // commande calculée lors de l'asservissement en translation
unsigned long time_begin; // Date du début de la période d'échantillonnage en cours

unsigned long sampling_period_translation = 100;
unsigned long sampling_period_rotation = 200;
                          // Periode d'échantillonage (pour asservissement en rotation) en milliseconde
                          // XXX : Si elle trop grande ou petite, l'asservissement devient instable
long waitingTime;         // Temps d'attente de la fin de la période d'échantillonnage
int  count ;              // Pour l'asservissement

bool flag = false;        // Flag levé lorsque l'arrêt d'urgence est enclenché
                          // (cf. attachInterrupt dans le setup() )

void setup()
{
	// Initialisation de la communication série avec l'ordinateur
	Serial.begin(9600);

	//Initialisation de la communication I2C (protocole pour communiquer avec la carte moteur)
	Wire.begin();

	// Moteurs à l'arret
	setSpeed1( 128, MD25ADDRESS_ROUES );
	setSpeed2( 128, MD25ADDRESS_ROUES );

	// Interruption pour arrêt d'urgence
	Serial.println( "avant" );
	attachInterrupt( 0, arretUrgence, CHANGE ); // Appel à la fonction arretUrgence
	                                            // quand l'interruption est déclenchée
	Serial.println( "après" );

	//Initialisation des encodeurs
	resetEncoders( MD25ADDRESS_ROUES );
	delay( 2000 ); // Attente de 2 secondes
	setAccelerationRate(1,MD25ADDRESS_ROUES); // Réglage de l'accélération max (1 : mini; max : maxi)
	getAccelerationRate(MD25ADDRESS_ROUES);
	encoder1 = getEncoder1(MD25ADDRESS_ROUES);
	encoder2 = getEncoder2(MD25ADDRESS_ROUES);
	Serial.println(encoder1);
	Serial.println(encoder2);

	//Initialisation de la position du robot
	distance_old = 0;
	distance_current = 0;

	//Initialisation de l'orientation du robot
	orientation_old = 0;
	orientation_current = 0;

	delay(2000);
	count = 0;
	Serial.println("OK");
}

void loop()
{
	translate(1200);
	tourne(458);
	translate(2370);
	tourne(0);
	translate(1300);
	translate(2500);

	delay(100000);
}

// Translate le robot
// 700 <---> dépacer de 30 cm
void translate ( long x )
{
	time_begin = millis();

	/*
	 * On entre dans la boucle d'asservissement.
	 * On ne sort de cette boucle que lorsque l'asservissement
	 * est considéré comme terminé.
	 * C'est-à_dire quand le compteur count à atteint la valeur 5
	 */
	while( count <= 5 )
	{
		/* Si l'erreur est en valeur absolue inférieure
		 * à une certaine valeur, on incrémente */
		if ( ( getEncoder1(MD25ADDRESS_ROUES) + getEncoder2(MD25ADDRESS_ROUES) - x ) <= 10
				&& ( getEncoder1(MD25ADDRESS_ROUES) + getEncoder2(MD25ADDRESS_ROUES) - x >= -10 ) )
		{
			++count;
		}

		// la dernière mesure de l'orientation devient l'ancienne mesure
		distance_old = distance_current;

		/* Si la période d'échantillonnage est trop courte */
		if ( waitingTime = ( sampling_period_translation - (millis() - time_begin) ) < 0 )
		{
			Serial.println( "Erreur : la boucle dure plus longtemps que la période d'échantillonnage : il faut augmenter la période d'échantillonnage-------------------------------------------------" );
		}

		delay( waitingTime ); // On attend la fin de la période d'échantillonnage

		Serial.print( "Temps d'attente  " );
		Serial.println( waitingTime );

		if ( flag == true )
		{
			Serial.println("ARRETTRANSMATIOn---------------------------------------------");
			setSpeed1(128,MD25ADDRESS_ROUES);
			setSpeed2(128,MD25ADDRESS_ROUES);
			while(1); // Le robot reste arrêté
			flag = false;
		}

		// début de la nouvelle période d'échantillonnage
		time_begin = millis();

		// Appel à la fonction qui va procéder à l'asservissment
		asservissement_distance(x);
	}

	// L'asservissement est terminé, on remet le compteur à zéro
	count = 0;
}

// Fonction appelée par translate : fait l'asservissement en translation
void asservissement_distance( long x )
{
	Serial.println( "move" );
	distance_current = ( getEncoder1( MD25ADDRESS_ROUES ) + getEncoder2( MD25ADDRESS_ROUES ) );
	Serial.println( distance_current );

	// Calcul de la comande
	commande_translation =
		  Kp_translation * (x - distance_current)
		+ Kd_translation * (distance_old - distance_current);
	Serial.print( "Commande : " );
	Serial.println( commande_translation );
	Serial.println( " " );

	// Saturation en vitesse
	if ( commande_translation >= 128 )
	{
		setSpeed1( 255,MD25ADDRESS_ROUES );
		setSpeed2( 255,MD25ADDRESS_ROUES );
	}
	else if( commande_translation <= -128 )
	{
		setSpeed1( 0, MD25ADDRESS_ROUES );
		setSpeed2( 0, MD25ADDRESS_ROUES );
	}
	else
	{
		setSpeed1( 128 + commande_translation,MD25ADDRESS_ROUES );
		setSpeed2( 128 + commande_translation,MD25ADDRESS_ROUES );
	}
}

// Consigne en rotation du robot : x=458 <--> tourne de 90°
void tourne( long x )
{
	Serial.print( "Count " );
	Serial.println( count );

	time_begin = millis();

	// Début de la boucle d'asservissement
	// count : compteur qui permet de détecter quand le
	// robot est stabilisé et donc que l'asservissement est termniné
	// Si count < 5, alors on continue l'asservissement,sinon on sort de la boucle while
	while( count <= 5 )
	{
		/*
		 * Si l'erreur en orientation par rapport
		 * à la consigne est inférieure à une certaine
		 * valeur, on incrémente count
		 */
		if ( ( getEncoder1( MD25ADDRESS_ROUES ) - getEncoder2( MD25ADDRESS_ROUES ) - x ) <= 10
				&& ( getEncoder1( MD25ADDRESS_ROUES ) - getEncoder2( MD25ADDRESS_ROUES ) - x ) >= -10 )
		{
			++count;
			Serial.println( "Incrément" );
		}

		Serial.print( "L'Erreur est de :  " );
		Serial.println( getEncoder1( MD25ADDRESS_ROUES ) - getEncoder2( MD25ADDRESS_ROUES ) - x );

		Serial.println( "Allez!" );
		orientation_old = orientation_current; // la dernière mesure de l'orientation
		                                       // devient l'ancienne mesure
		Serial.println( "Allez2!" );
		Serial.println( sampling_period_rotation - ( millis() - time_begin ) );

		// Si la période d'échantillonnage est trop courte...
		if ( waitingTime = ( sampling_period_rotation - (millis() - time_begin) ) < 0 )
		{
			Serial.println( "Erreur : la boucle dure plus longtemps que la période d'échantillonnage : il faut augmenter la période d'échantillonnage-------------------------------------------------" );
		}

		// On attend la fin de la période d'échantillonnage
		delay( waitingTime );
		Serial.print( "Temps d'attente  " );
		Serial.println( waitingTime );

		if( flag == true )
		{
			Serial.println( "ARRETROTATION-----------------------------" );
			setSpeed1( 128, MD25ADDRESS_ROUES );
			setSpeed2( 128, MD25ADDRESS_ROUES );
			while( 1 );
			flag = false;
		}

		// Début de la nouvelle période d'échantillonnage
		time_begin = millis();

		// La fonction "rotate" va calculer la commande à envoyer aux moteurs
		asservissement_rotation(x);
	}

	// L'asservissement est terminé, on remet le compteur à zéro
	count = 0;
}

// Fonction appelée par la fonction "tourne"
// Fait tourner le robot : x=458 <--> 90°
void asservissement_rotation( int x )
{
	/* Calcul de l'orientation actuelle du robot
	 * à partir de la mesure des encodeurs des roues */
	Serial.print( "Encodeur 1 : " );
	Serial.println( getEncoder1( MD25ADDRESS_ROUES ) );

	Serial.print( "Encodeur 2 : " );
	Serial.println( getEncoder2( MD25ADDRESS_ROUES ) );

	// L'orientation est :orientation_current
	orientation_current = getEncoder1( MD25ADDRESS_ROUES ) - getEncoder2( MD25ADDRESS_ROUES );
	Serial.print( "Orientation : " );
	Serial.println( orientation_current );

	//Calcul de la commande à partir de cette mesure
	commande_rotation =
		  Kp_rotation * ( x - orientation_current )
		+ Kd_rotation * ( orientation_old - orientation_current ); // J'ai changé le signe!!
	Serial.print( "Commande : " );
	Serial.println( commande_rotation );
	Serial.println( " " );

	if ( commande_rotation <= 127 && commande_rotation >= -127 )
	{
		setSpeed1(  commande_rotation + 128, MD25ADDRESS_ROUES );
		setSpeed2( -commande_rotation + 128, MD25ADDRESS_ROUES );
	}
	else if ( commande_rotation >= 128 )
	{
		// Saturation de la vitesse
		setSpeed1( 255, MD25ADDRESS_ROUES );
		setSpeed2( 0, MD25ADDRESS_ROUES );
	}
	else if ( commande_rotation <= -128 )
	{
		// Saturation de la vitesse
		setSpeed1( 0, MD25ADDRESS_ROUES );
		setSpeed2( 255, MD25ADDRESS_ROUES );
	}
}

// Garde fou : arrête les moteurs si le courant dans les moteurs est supérieur à x
void watchCurrents( float x, int address )
{
	if( getCurrent1(address) > x
			|| getCurrent2(address) > x )
	{
		setSpeed1( 128, address );
		setSpeed2( 128, address );
		Serial.println( "intensité trop grande !--------------------------------------------------------------------------------------------");
	}
}

