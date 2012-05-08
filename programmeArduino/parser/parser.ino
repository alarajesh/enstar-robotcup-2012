// vim:ft=c

#include <Wire.h>
#include <SoftwareSerial.h>

#define DEBUG
#define SHOW_MESSAGE_RECEIVED

////////////
// MACROS //
////////////

/*{*/

/********************
 * Parsing constant *
 ********************/

#define MSG_SIZE  8              // Input message size
#define BEGIN_MSG (byte) '\001'  // Input message begin
#define END_MSG   (byte) '\004'  // Input message end
#define CMD_TOKEN '$'            // Output message begin on statement transmission
#define ERR_TOKEN '#'            // Output message begin on error transmission
#define MSG_TOKEN '%'            // Output message begin on comment transmission
#define DIODE_ID  13             // Control diode

/*****************************************
 * Register addresses of the motor card  *
 *****************************************/

#define MD25ADDRESS_ROUES 0x5F        // Adresse de la carte moteur pour les roues
#define SPEED1            (byte) 0x00 // Adresse où est stockée la vitesse du moteur 1
#define SPEED2            0x01        // Adresse où est stockée la vitesse du moteur 2
#define ENCODER1          0x02        // Adresse du registre contenant la
                                      //   valeur de l'encodeur du moteur 1
#define ENCODER2          0x06        // Adresse du registre contenant la
                                      //   valeur de l'encodeur du moteur 2
#define CURRENT1          0xB         // Adresse du registre contenant la
                                      //   valeur de l'intensité du courant du moteur 1
#define CURRENT2          0xC         // Adresse du registre contenant la
                                      //   valeur de l'intensité du courant du moteur 2
#define CMD               0x10
#define ACCELERATION_RATE 0x0E        // Adresse du registre contrôlant l'accélération du robot

/**************
 * Attributes *
 **************/

#define ACCELERATION 0x01
#define CURRENT      0x03

/******************
 * Emergency Stop *
 ******************/

#define EMERGENCY_PIN  0  // this coorespond to the PIN 2 on the arduino

/*********
 * Debug *
 *********/

#ifdef DEBUG
	#define DEBUG_MSG( msg ) \
		sendMessage( msg );
#else
	#define DEBUG_MSG( msg )
#endif

/*}*/

/////////////////
// GLOBAL VARS //
/////////////////

/*{*/

/***********
 * Message *
 ***********/

boolean msgComplete = false; // whether the message is complete
boolean onError     = false; // error while msg reception
byte    funcId      = 0;     // 0 : Clamp and 1 : Movement
byte    funcType    = 0;     // if funcId == 0 then { 0 : close and 1 : open }
                             //   if funcId == 1 then { 0 : translate and 1 : rotate }
byte    msgValue[4];         // computer message divided in 4 bytes
int*    msgInt =             // int transmitted in the computer message
		(int*) msgValue;
int     inputSize   = 0;     // size of partial msg received

/*************
 *
 */

long encoder1; // variable dans laquelle sera stockée
               //   la valeur de l'encodeur du moteur 1
long encoder2; // variable dans laquelle sera stockée
               //   la valeur de l'encodeur du moteur 2
byte current1; // variable dans laquelle sera stockée
               //   la valeur du courant dans le moteur 1
byte current2; // variable dans laquelle sera stockée
               //   la valeur du courant dans le moteur 2

long orientation_current; // Orientation du robot issue de la dernière mesure
long orientation_old;     // Orientation du robot issue de l'avant dernière mesure

long distance_current;    // Position actuelle du robot
long distance_old;        // Position du robot durant la période d'échantillonnage précédente

float Kp_rotation = 0.1;  // Coefficient de la commande proportionnelle
float Kd_rotation = 0;    // Coefficient de la commande dérivée// 0.1 si acceleration = 0.5
float Kp_translation = 0.1;
float Kd_translation = 0.1;
int commande_rotation;    // commande calculée lors de l'asservissement en rotation
int commande_translation; // commande calculée lors de l'asservissement en translation
unsigned long time_begin; // Date du début de la période d'échantillonnage en cours

unsigned long sampling_period_translation = 100;
unsigned long sampling_period_rotation = 200;
                          // Periode d'échantillonage (pour asservissement en rotation) en milliseconde
                          //   XXX : Si elle trop grande ou petite, l'asservissement devient instable
long waitingTime;         // Temps d'attente de la fin de la période d'échantillonnage
int  count ;              // Pour l'asservissement

bool flag = false;        // Flag levé lorsque l'arrêt d'urgence est enclenché
                          //   (cf. attachInterrupt dans le setup() )

/*}*/

////////////////////////////////
// TRANSMISSION WITH COMPUTER //
////////////////////////////////

/*{*/

void sendError( String error )
{
	Serial.print( ERR_TOKEN );
	Serial.print( error );
	Serial.print( '\n' );
}

void sendMessage( String message )
{
	Serial.print( MSG_TOKEN );
	Serial.print( message );
	Serial.print( '\n' );
}

void sendCommand( int deviceId, int deviceAttr, int value )
{
	Serial.print( CMD_TOKEN );
	Serial.print( deviceId );
	Serial.print( deviceAttr );
	Serial.print( value );
	Serial.print( '\n' );
}

/*}*/

////////////////////
// MAIN FUNCTIONS //
////////////////////

/*{*/

void setup()
{
	// initialize serial
	Serial.begin(9600);

	// initialize I2C
	DEBUG_MSG("Initialize I2C");
	Wire.begin();

	// initialize the digital pin as an output.
	// Pin 13 has an LED connected on most Arduino boards
	DEBUG_MSG("Initialize Diode");
	pinMode(13, OUTPUT);

	// Stop Motors
	DEBUG_MSG("Stop Motors");
	setSpeed1( 128, MD25ADDRESS_ROUES );
	setSpeed2( 128, MD25ADDRESS_ROUES );

	// Set the emergency stop
	DEBUG_MSG("Set emergency stop");
	attachInterrupt( EMERGENCY_PIN, emergencyStop, CHANGE );

	// Initialize encoders
	DEBUG_MSG("Initialize Encoders");
	resetEncoders( MD25ADDRESS_ROUES );
	delay( 2000 ); // Wait 2 seconds
	setAccelerationRate( 1, MD25ADDRESS_ROUES ); // Set maximal acceleration (1 : mini; max : maxi)
	getAccelerationRate( MD25ADDRESS_ROUES );
	encoder1 = getEncoder1( MD25ADDRESS_ROUES );
	encoder2 = getEncoder2( MD25ADDRESS_ROUES );

	// Initialize robot position
	DEBUG_MSG("Initialize Robot Position");
	distance_old = 0;
	distance_current = 0;

	// Initialize robot orientation
	DEBUG_MSG("Initialize Robot Orientation");
	orientation_old = 0;
	orientation_current = 0;

	delay(2000);
	count = 0;
	
	DEBUG_MSG("Setup complete");
}

void loop()
{
	#ifdef SHOW_MESSAGE_RECEIVED /*{*/
		// print the message when a newline arrives
		if (msgComplete)
		{
			Serial.print( MSG_TOKEN );
			Serial.print( "command received : " );
			Serial.print( funcId );
			Serial.print( funcType );
			Serial.print( *msgInt );
			Serial.print( '\n' );
		}
	#endif // SHOW_MESSAGE_RECEIVED /*}*/
	
	if (msgComplete)
	{
		switch ( funcId )
		{
			// Clamp
			case 0 :
				switch ( funcType )
				{
					case 0 :
						break;
					case 1 :
						break;
					default:
						sendError( "unknown funType" );
						break;
				}
				break;

			// Movement
			case 1:
				switch ( funcType )
				{
					// Translate
					case 0 :
						break;
					// Rotate
					case 1 :
						rotate( *msgInt );
						break;
					default:
						sendError( "unknown funType" );
						break;
				}
				break;

			default:
				sendError( "unknown funId" );
				break;
		}

		// clear the message
		msgComplete = false;
		inputSize   = 0;
	}

	rotate(439);
	delay(100000);
}

/*!
 * \brief This function get the computer message and parse it.
 */
void serialEvent()
{
	while ( Serial.available() && ( inputSize < MSG_SIZE ) && !msgComplete )
	{
		// get the new byte:
		byte inByte = (byte) Serial.read();

		switch ( inputSize )
		{
			case 0 :
				if ( inByte != BEGIN_MSG )
				{
					sendError( "bad begin message" );
					onError = true;
				}
				else
				{
					onError = false;
				}

				digitalWrite( DIODE_ID, HIGH ); // set the LED on
				break;

			case 1 :
				funcId = inByte;
				break;

			case 2 :
				funcType = inByte;
				break;

			case 3 :
				msgValue[0] = inByte;
				break;

			case 4 :
				msgValue[1] = inByte;
				break;

			case 5 :
				msgValue[2] = inByte;
				break;

			case 6 :
				msgValue[3] = inByte;
				break;

			case 7 :
				if ( inByte != END_MSG )
				{
					sendError( "bad end message" );
					onError = true;
				}
				digitalWrite( DIODE_ID, LOW ); // set the LED off
				break;

			default :
				sendError( "oh my god !" );
				onError = true;
				break;
		}

		if ( !onError )
		{
			++inputSize;
		}
		else
		{
			inputSize = 0;
		}

		// when the message has the good size, set a flag
		// so the main loop can do something about it
		if ( inputSize == MSG_SIZE )
		{
			msgComplete = true;
		}
	}
}

/*}*/

/////////////
// SETTERS //
/////////////

/*{*/

// Consigne en rotation du robot : x=458 <--> tourne de 90°
void rotate( int x )
{
	DEBUG_MSG( String( "Count " ) + String( count ) );

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
			DEBUG_MSG( "Increase Count" );
			++count;
		}

		DEBUG_MSG( String( "Gap : " ) +
				String( getEncoder1( MD25ADDRESS_ROUES )
					- getEncoder2( MD25ADDRESS_ROUES )
					- x  )
				);

		orientation_old = orientation_current; // la dernière mesure de l'orientation
		                                       // devient l'ancienne mesure
		
		DEBUG_MSG( String( "Waiting Time : " ) +
				String( sampling_period_rotation
					- ( millis() - time_begin ) )
				);

		// Si la période d'échantillonnage est trop courte...
		if ( waitingTime = ( sampling_period_rotation - ( millis() - time_begin ) ) < 0 )
		{
			sendError( "loop period greater than sampling period" );
		}

		// On attend la fin de la période d'échantillonnage
		delay( waitingTime );
		DEBUG_MSG( String( "Temps d'attente : " ) + String( waitingTime ) );

		if( flag == true )
		{
			DEBUG_MSG( "Emergency Stop : Stop rotation" );
			setSpeed1( 128, MD25ADDRESS_ROUES );
			setSpeed2( 128, MD25ADDRESS_ROUES );
			while( 1 );
			flag = false;
		}

		// Début de la nouvelle période d'échantillonnage
		time_begin = millis();

		// La fonction "rotate" va calculer la commande à envoyer aux moteurs
		setRotation( x );
	}

	// L'asservissement est terminé, on remet le compteur à zéro
	count = 0;
}

void setRotation( int x )
{
	// L'orientation est orientation_current
	orientation_current =
		  getEncoder1( MD25ADDRESS_ROUES )
		- getEncoder2( MD25ADDRESS_ROUES );

	// Calcul de la commande à partir de cette mesure
	commande_rotation =
		  Kp_rotation * ( x - orientation_current )
		+ Kd_rotation * ( orientation_old - orientation_current );

	String res = "commande rotation : ";
	String rot = String( commande_rotation );
	sendMessage( (res + rot) );

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

void setAccelerationRate(int x, int address)
{
	if( x >=1 && x <=10 )
	{
		Wire.beginTransmission( address );
		Wire.write( ACCELERATION_RATE );
		Wire.write( x );
		Wire.endTransmission();
	}
	else
	{
		DEBUG_MSG("Bad value : acceleration rate");
	}
}

void setSpeed1( int x, int address )
{
	// Begin communication with motor card
	Wire.beginTransmission( address );

	// Get Motor1 speed register access
	Wire.write( SPEED1 );

	// Set Speed in register
	Wire.write( x );

	// End communication
	Wire.endTransmission();
}

void setSpeed2( int x, int address )
{
	// Begin communication with motor card
	Wire.beginTransmission( address );

	// Get Motor1 speed register access
	Wire.write( SPEED2 );

	// Set Speed in register
	Wire.write( x );

	// End communication
	Wire.endTransmission();
}

void resetEncoders( int address )
{
	// Set to 0 encoders
	Wire.beginTransmission( address );
	Wire.write( CMD );
	Wire.write( 0x20 );
	Wire.endTransmission();
}

/*}*/

/////////////
// GETTERS //
/////////////

/*{*/

void getAccelerationRate( int address )
{
	// Affiche à l'écran le taux d'accélération : 1(minimale) à 10 (maximal)
	Wire.beginTransmission( address );
	Wire.write( ACCELERATION_RATE );
	Wire.endTransmission();
	Wire.requestFrom( address, 1 );

	while( Wire.available() < 1 );

	byte acceleration = Wire.read();
	sendCommand( address, ACCELERATION, acceleration );
}

int getEncoder1( int address )
{
	// Motor 1 encoder reading
	Wire.beginTransmission( address );
	Wire.write( ENCODER1 );
	Wire.endTransmission();

	Wire.requestFrom( address, 4 );

	while( Wire.available() < 4 );
	// l'encodeur code sur 32 bits.
	// Il faut donc attendre que la carte moteur renvoit 4 bytes

	byte encoder1[4];
	int* value = (int*) encoder1;
	encoder1[0] = Wire.read();
	encoder1[1] = Wire.read();
	encoder1[2] = Wire.read();
	encoder1[3] = Wire.read();

	sendCommand( address, ENCODER1, *value );

	return *value;
}

int getEncoder2( int address )
{
	// Motor 2 encoder reading
	Wire.beginTransmission( address );
	Wire.write( ENCODER2 );
	Wire.endTransmission();

	Wire.requestFrom( address, 4 );
	while( Wire.available() < 4 );

	byte encoder2[4];
	int* value = (int*) encoder2;
	encoder2[0] = Wire.read();
	encoder2[1] = Wire.read();
	encoder2[2] = Wire.read();
	encoder2[3] = Wire.read();

	return *value;
}

byte getCurrent1( int address )
{
	// Read the current in the Motor 1 : 10 <--> 1A
	Wire.beginTransmission( address );
	Wire.write( CURRENT1 );
	Wire.endTransmission();

	Wire.requestFrom( address, 1 );
	while ( Wire.available() < 1 );

	byte current1 = Wire.read();

	return current1;
}

byte getCurrent2( int address )
{
	// Read the current in the Motor 2 : 10 <--> 1A
	Wire.beginTransmission( address );
	Wire.write( CURRENT2 );
	Wire.endTransmission();

	Wire.requestFrom( address, 1 );
	while ( Wire.available() < 1 );

	byte current2 = Wire.read();

	return current2;
}

/*}*/

//////////////
// SECURITY //
//////////////

/*{*/

void emergencyStop()
{
	flag = true;
}

void watchCurrents( float x, int address )
{
	if( getCurrent1(address) > x
			|| getCurrent2(address) > x )
	{
		setSpeed1( 128, address );
		setSpeed2( 128, address );
		sendError( "Too much intensity" );
	}
}

/*}*/

