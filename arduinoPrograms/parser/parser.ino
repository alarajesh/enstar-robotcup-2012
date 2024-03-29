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

#define MSG_SIZE    8              // Input message size
#define BEGIN_MSG   (byte) '\001'  // Input message begin
#define END_MSG     (byte) '\004'  // Input message end
#define CMD_TOKEN   '$'            // Output message begin on statement transmission
#define ERR_TOKEN   '#'            // Output message begin on error transmission
#define MSG_TOKEN   '%'            // Output message begin on comment transmission
#define DIODE_ID    13             // Control diode
#define INFO_PERIOD 500            // Period for information sending (millisecond)
#define TRANSLATION_TKN 1         // The communication code for the translation end
#define ROTATION_TKN 2             // ""                             rotation ""
#define SONAR_TKN 0                // com code for sonar
#define START_TKN 3

/*****************************************
 * Register addresses of the motor card  *
 *****************************************/

#define MD25_WHEEL_ADDRESS 0x5F        // Adresse de la carte moteur pour les roues
#define MD25ADDRESS_PINCE 0x58        // Adresse de la carte moteur pour les pinces
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
#define SLAMP_MAX         150         // Valeur max de l'encodeur pour la fermeture des pinces
#define SLAMP_TIME_MAX    4000        // Temps maximal d'attente pour l'ouverture et la fermeture des pinces

/**************
 * Attributes *
 **************/

#define ACCELERATION 0x01
#define CURRENT      0x03

/******************
 * Emergency Stop *
 ******************/

#define EMERGENCY_PIN  22  // this correspond to the PIN 22 on the arduino

/*********
 * Debug *
 *********/

#ifdef DEBUG
#define DEBUG_MSG( msg )			\
  sendMessage( msg );
#else
#define DEBUG_MSG( msg )
#endif

/*}*/

/**********
 * SONAR  *
 **********/

#define SONAR_PIN 35      // this correspond to the pin the sonar is plugged in

// The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
// Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
#define LOW_PULSE 2
#define HIGH_PULSE 5

// the timeout defines how much time we wait before giving up on
// the sonar pulse response cm*2*29 n= duration, this one correspond to 30 cm
#define SONAR_TIMEOUT 3480  


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
byte current1; // variable dans laquelle sera stockée
               //   la valeur du courant dans le moteur 1
byte current2; // variable dans laquelle sera stockée
               //   la valeur du courant dans le moteur 2

long orientation_current; // Orientation du robot issue de la dernière mesure
long orientation_old;     // Orientation du robot issue de l'avant dernière mesure

int mv_tr = 0; // where to go (translation)
int mv_rt = 0; // where to go (rotation)

int distance_current;    // Position actuelle du robot
int distance_old;        // Position du robot durant la période d'échantillonnage précédente

float Kp_rotation = 0.1;  // Coefficient de la commande proportionnelle
float Kd_rotation = 0;    // Coefficient de la commande dérivée// 0.1 si acceleration = 0.5
float Kp_translation = 0.1;
float Kd_translation = 0.15;
int commande_rotation;    // commande calculée lors de l'asservissement en rotation
int commande_translation; // commande calculée lors de l'asservissement en translation

unsigned long sampling_period_translation = 150;
unsigned long sampling_period_rotation = 150;
// Periode d'échantillonage (pour asservissement en rotation) en milliseconde
//   XXX : Si elle trop grande ou petite, l'asservissement devient instable

bool flag = false;        // Flag levé lorsque l'arrêt d'urgence est enclenché
                          //   (cf. attachInterrupt dans le setup() )

bool isClamped = false; //Coefficient de fermeture des pinces (0 si ouvert, 1 si fermé)

long send_info_timer;
long send_info_abs_timer;
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

void sendCommand( int code, int value )
{
  Serial.print( CMD_TOKEN );
  Serial.print( code );
  Serial.print( String(value) );
  Serial.print( '\n' );
}

/*}*/

////////////////////////////////
//   STARTING AND STOPPING    //
////////////////////////////////

#define START_PIN 18         // the pin where the start button is fixed

////////////////////
// MAIN FUNCTIONS ///
///////////////////

/*{*/

void setup()
{
  // initalize the starting interruption
  pinMode(START_PIN,INPUT);

  // initialize serial
  Serial.begin( 9600 );
  
  // initialize I2C
  DEBUG_MSG( "Initialize I2C" );
  Wire.begin();
  
  // initialize the digital pin as an output.
  // Pin 13 has an LED connected on most Arduino boards
  DEBUG_MSG( "Initialize Diode" );
  pinMode( DIODE_ID, OUTPUT );
  
  // Stop Motors
  DEBUG_MSG( "Stop Motors" );
  setSpeed1( 128, MD25_WHEEL_ADDRESS );
  setSpeed2( 128, MD25_WHEEL_ADDRESS );
  
  // Set the emergency stop
  DEBUG_MSG( "Set emergency stop" );
  pinMode( EMERGENCY_PIN, INPUT );
  //attachInterrupt( EMERGENCY_PIN, emergencyStop, CHANGE );
  
  // Initialize encoders
  DEBUG_MSG( "Initialize Encoders" );
  DEBUG_MSG( "    Clamp..." );
  resetEncoders( MD25ADDRESS_PINCE );
  DEBUG_MSG( String( "      1 : " ) + String( getEncoder1( MD25ADDRESS_PINCE ) ) );
  DEBUG_MSG( String( "      2 : " ) + String( getEncoder2( MD25ADDRESS_PINCE ) ) );
  DEBUG_MSG( "    Wheels..." );
  resetEncoders( MD25_WHEEL_ADDRESS );
  resetEncoders( MD25_WHEEL_ADDRESS );
  DEBUG_MSG( String( "      1 : " ) + String( getEncoder1( MD25_WHEEL_ADDRESS ) ) );
  DEBUG_MSG( String( "      2 : " ) + String( getEncoder2( MD25_WHEEL_ADDRESS ) ) );
  
  delay( 2000 ); // Wait 2 seconds
  
  DEBUG_MSG( "    Set Acceleration..." );
  setAccelerationRate( 1, MD25_WHEEL_ADDRESS ); // Set maximal acceleration (1 : mini; max : maxi)
  DEBUG_MSG( String( "      Wheels :" ) + String( getAccelerationRate( MD25_WHEEL_ADDRESS ) ) );
  DEBUG_MSG( String( "      Clamp  :" ) + String( getAccelerationRate( MD25ADDRESS_PINCE ) ) );
  
  // Initialize robot position
  DEBUG_MSG("Initialize Robot Position");
  distance_old = 0;
  distance_current = 0;
  
  // Initialize robot orientation
  DEBUG_MSG("Initialize Robot Orientation");
  orientation_old = 0;
  orientation_current = 0;
  
  // Initialize info timer
  DEBUG_MSG("Initialize Info Timer");
  send_info_timer = 0;
  send_info_abs_timer = millis();
 
  DEBUG_MSG("Setup complete");
}

void loop()
{
  // wait for robot to start
  static bool start = false;
  while(digitalRead(START_PIN) == LOW)
    {
      DEBUG_MSG(String("waiting to start"));
      DEBUG_MSG(String("start pin: ")+digitalRead(START_PIN));
      delay(100);
    }
  if(!start)
    {
      start = true;
      sendCommand(START_TKN, 0);
    }
  
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
	      // Close
	    case 0 :
	      if ( !isClamped )
		{
		  DEBUG_MSG( "closing Clamp" );
		  closeClamp();
		  DEBUG_MSG( "clamp closed" );
		  isClamped = true;
		  break;
		}
	      else
		{
		  sendError( "Clamping Error : already closed" );
		  break;
		}
	      // Open
	    case 1 :
	      if ( isClamped )
		{
		  DEBUG_MSG( "opening Clamp" );
		  openClamp() ;
		  DEBUG_MSG( "clamp open" );
		  isClamped = false;
		  break ;
		}
	      else
		{
		  sendError( "Clamping Error : already open");
		  break;
		}
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
	      DEBUG_MSG( "Begin translation" );
	      translate( *msgInt );
	      DEBUG_MSG( "Translation done" );
	      sendCommand( TRANSLATION_TKN, 0 );
	      break;
	      // Rotate
	    case 1 :
	      DEBUG_MSG( "Begin rotation" );
	      rotate( *msgInt );
	      DEBUG_MSG( "Rotation done" );
	      sendCommand( ROTATION_TKN, 0 );
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
  
  // send info to PC
  send_info_timer += millis() - send_info_abs_timer;
  send_info_abs_timer = millis();
  if ( send_info_timer > INFO_PERIOD )
    {
      send_info_timer -= INFO_PERIOD;
      sendCommand( SONAR_TKN, (int)getSonarRange() );
    }
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

void openClamp()
{
  unsigned long time_in = millis();
  int enco1;
  int enco2;

  do
    {
      while ( digitalRead( EMERGENCY_PIN ) == 1 )
	{
	  DEBUG_MSG( "Emergency Stop : Stop Clamp opening" );
	  setSpeed1( 128, MD25_WHEEL_ADDRESS );
	  setSpeed2( 128, MD25_WHEEL_ADDRESS );
	}

      //  Faire tourner le(s) moteur(s) qui n'ont pas encore atteint la valeur seuil
      enco1 = getEncoder1( MD25ADDRESS_PINCE );
      DEBUG_MSG( String( "Clamp : encoder1.value -> " ) + String( enco1 ) );
      if ( enco1 >= 0 )
	{
	  // L'encodeur 1 a une valeur >=0, on fait tourner le moteur1 dans le sens antihoraire
	  setSpeed1(116, MD25ADDRESS_PINCE);
	  DEBUG_MSG( "Clamp : motor1.speed -> 116" );
	}
      else
	{
	  // La valeur de l'encodeur1 est <0, arrêter moteur1 car la pince est en position ouverte
	  setSpeed1(128,  MD25ADDRESS_PINCE );
	  DEBUG_MSG( "Clamp : motor1.speed -> 128" );
	}

      enco2 = getEncoder2( MD25ADDRESS_PINCE );
      DEBUG_MSG( String( "Clamp : encoder2.value -> " ) + String( enco2 ) );
      if ( enco2 >= 0 )
	{
	  // L'encodeur 2 a une valeur >=0, on fait tourner le moteur2 dans le sens antihoraire
	  setSpeed2(116,  MD25ADDRESS_PINCE );
	  DEBUG_MSG( "Clamp : motor2.speed -> 116" );
	}
      else
	{
	  // La valeur de l'encodeur2 est <0, arrêter moteur2 car la pince est en position ouverte
	  setSpeed2(128,  MD25ADDRESS_PINCE );
	  DEBUG_MSG( "Clamp : motor2.speed -> 128" );
	}

      DEBUG_MSG( String( "Time_in : " ) + String( millis() - time_in ) );
    }
  while( ( enco1 >= 0 || enco2 >=0 )
	 && ( millis() - time_in < SLAMP_TIME_MAX ) );

  setSpeed1(128,  MD25ADDRESS_PINCE );
  setSpeed2(128,  MD25ADDRESS_PINCE );
}

void closeClamp()
{
  unsigned long time_in = millis();
  int enco1;
  int enco2;

  do
    {
      while ( digitalRead( EMERGENCY_PIN ) == 1 )
	{
	  DEBUG_MSG( "Emergency Stop : Stop Clamp closing" );
	  setSpeed1( 128, MD25_WHEEL_ADDRESS );
	  setSpeed2( 128, MD25_WHEEL_ADDRESS );
	}

      // Faire tourner le(s) moteur(s) qui n'ont pas encore atteint la valeur seuil
      enco1 = getEncoder1( MD25ADDRESS_PINCE );
      DEBUG_MSG( String( "Clamp : encoder1.value -> " ) + String( enco1 ) );
      if( enco1 <= SLAMP_MAX )
	{
	  // Tourner moteur1 tant que encodeur 1 est en dessous de la valeur seuil
	  setSpeed1( 140, MD25ADDRESS_PINCE );
	  DEBUG_MSG( "Clamp : motor1.speed -> 140" );
	}
      else
	{
	  // Arrêter le moteur 1 si la valeur seuil est atteinte
	  DEBUG_MSG( "Clamp : motor1.speed -> 128" );
	  setSpeed1(128,  MD25ADDRESS_PINCE );
	}

      enco2 = getEncoder2( MD25ADDRESS_PINCE );
      DEBUG_MSG( String( "Clamp : encoder2.value -> " ) + String( enco2 ) );
      if( enco2 <= SLAMP_MAX )
	{
	  // Tourner moteur2 tant que encodeur 2 est en dessous de la valeur seuil
	  setSpeed2(140,  MD25ADDRESS_PINCE );
	  DEBUG_MSG( "Clamp : motor2.speed -> 140" );
	}
      else
	{
	  // Arrêter le moteur 2 si la valeur seuil est atteinte
	  setSpeed2(128,  MD25ADDRESS_PINCE );
	  DEBUG_MSG( "Clamp : motor2.speed -> 128" );
	}

      DEBUG_MSG( String( "Time_in : " ) + String( millis() - time_in ) );
    }
  while ( ( enco1 <= SLAMP_MAX || enco2 <= SLAMP_MAX )
	  && ( millis() - time_in < SLAMP_TIME_MAX ) );

  setSpeed1(128, MD25ADDRESS_PINCE );
  setSpeed2(128,  MD25ADDRESS_PINCE );
}

// Translate le robot
// 700 <---> dépacer de 30 cm
void translate ( int x )
{
  unsigned long time_begin_loop = millis();
  int count = 0;

  mv_tr += x;

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
      if ( ( getEncoder1(MD25_WHEEL_ADDRESS) + getEncoder2(MD25_WHEEL_ADDRESS) - mv_tr ) <= 10
	   && ( getEncoder1(MD25_WHEEL_ADDRESS) + getEncoder2(MD25_WHEEL_ADDRESS) - mv_tr >= -10 ) )
	{
	  ++count;
	}

      // la dernière mesure de l'orientation devient l'ancienne mesure
      distance_old = distance_current;

      long waitingTime =
	(long) sampling_period_rotation
	- ( (long) millis() - (long) time_begin_loop );

      DEBUG_MSG( String( "Waiting Time : " ) +
		 String( waitingTime ) );

      /* Si la période d'échantillonnage est trop courte */
      if ( waitingTime < 0 )
	{
	  sendError( "loop too long" );
	}

      // On attend la fin de la période d'échantillonnage
      delay( waitingTime );

      while ( digitalRead( EMERGENCY_PIN ) == 1 )
	{
	  DEBUG_MSG( "Emergency Stop : Stop translation" );
	  setSpeed1( 128, MD25_WHEEL_ADDRESS );
	  setSpeed2( 128, MD25_WHEEL_ADDRESS );
	}

      // début de la nouvelle période d'échantillonnage
      time_begin_loop = millis();

      // Appel à la fonction qui va procéder à l'asservissment
      setTranslation( mv_tr );
    }
}

// Fonction appelée par translate : fait l'asservissement en translation
void setTranslation( int x )
{
  distance_current = ( getEncoder1( MD25_WHEEL_ADDRESS ) + getEncoder2( MD25_WHEEL_ADDRESS ) );
  DEBUG_MSG( String( "Current distance : " ) + String( distance_current ) );

  // Calcul de la comande
  commande_translation =
    Kp_translation * ( x - distance_current )
    + Kd_translation * ( distance_old - distance_current );
  DEBUG_MSG( String( "Translation commande : " ) + String( commande_translation ) );

  // Saturation en vitesse
  if ( commande_translation >= 128 )
    {
      setSpeed1( 255,MD25_WHEEL_ADDRESS );
      setSpeed2( 255,MD25_WHEEL_ADDRESS );
    }
  else if( commande_translation <= -128 )
    {
      setSpeed1( 0, MD25_WHEEL_ADDRESS );
      setSpeed2( 0, MD25_WHEEL_ADDRESS );
    }
  else
    {
      setSpeed1( 128 + commande_translation,MD25_WHEEL_ADDRESS );
      setSpeed2( 128 + commande_translation,MD25_WHEEL_ADDRESS );
    }
}

// Consigne en rotation du robot : x=458 <--> tourne de 90°
void rotate( int x )
{
  long int time_begin_loop = millis();
  int count = 0;

  mv_rt += x;

  DEBUG_MSG( String( "Count " ) + String( count ) );

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
      int dEnc = getEncoder1( MD25_WHEEL_ADDRESS ) - getEncoder2( MD25_WHEEL_ADDRESS );

      if ( ( dEnc - mv_rt ) <= 10 && ( dEnc - mv_rt ) >= -10 )
	{
	  DEBUG_MSG( "Increase Count" );
	  ++count;
	}

      DEBUG_MSG( String( "Gap : " ) + String( dEnc - mv_rt ) );

      long waitingTime =
	(long) sampling_period_rotation
	- ( (long) millis() - (long) time_begin_loop );

      DEBUG_MSG( String( "Waiting time : " ) + String( waitingTime ) );

      // Si la période d'échantillonnage est trop courte...
      if ( waitingTime  < 0 )
	{
	  sendError( "loop period greater than sampling period" );
	}

      // On attend la fin de la période d'échantillonnage
      delay( waitingTime );

      while( digitalRead( EMERGENCY_PIN ) == 1 )
	{
	  DEBUG_MSG( "Emergency Stop : Stop rotation" );
	  setSpeed1( 128, MD25_WHEEL_ADDRESS );
	  setSpeed2( 128, MD25_WHEEL_ADDRESS );
	}

      // Début de la nouvelle période d'échantillonnage
      time_begin_loop = millis();

      // La fonction "rotate" va calculer la commande à envoyer aux moteurs
      setRotation( mv_rt );
    }
}

void setRotation( int x )
{
  orientation_old = orientation_current; // la dernière mesure de l'orientation
  // devient l'ancienne mesure

  // L'orientation est orientation_current
  orientation_current =
    getEncoder1( MD25_WHEEL_ADDRESS )
    - getEncoder2( MD25_WHEEL_ADDRESS );

  DEBUG_MSG( String( "Current orientation : " ) + String( orientation_current ) );
  DEBUG_MSG( String( "Old     orientation : " ) + String( orientation_old ) );

  // Calcul de la commande à partir de cette mesure
  commande_rotation =
    Kp_rotation * ( x - orientation_current )
    + Kd_rotation * ( orientation_old - orientation_current );

  DEBUG_MSG( String( "commande rotation : ") + String( commande_rotation ) );

  if ( commande_rotation < 128 && commande_rotation > -128 )
    {
      setSpeed1(  commande_rotation + 128, MD25_WHEEL_ADDRESS );
      setSpeed2( -commande_rotation + 128, MD25_WHEEL_ADDRESS );
    }
  else if ( commande_rotation >= 128 )
    {
      // Saturation de la vitesse
      setSpeed1( 255, MD25_WHEEL_ADDRESS );
      setSpeed2( 0, MD25_WHEEL_ADDRESS );
    }
  else if ( commande_rotation <= -128 )
    {
      // Saturation de la vitesse
      setSpeed1( 0, MD25_WHEEL_ADDRESS );
      setSpeed2( 255, MD25_WHEEL_ADDRESS );
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

byte getAccelerationRate( int address )
{
  // Affiche à l'écran le taux d'accélération : 1(minimale) à 10 (maximal)
  Wire.beginTransmission( address );
  Wire.write( ACCELERATION_RATE );
  Wire.endTransmission();
  Wire.requestFrom( address, 1 );

  while( Wire.available() < 1 );

  byte acceleration = Wire.read();
  //sendCommand( address, ACCELERATION, acceleration );
  return acceleration;
}

long getEncoder1( int address )
{
  // Motor 1 encoder reading
  Wire.beginTransmission( address );
  Wire.write( ENCODER1 );
  Wire.endTransmission();

  Wire.requestFrom( address, 4 );

  // l'encodeur code sur 32 bits.
  // Il faut donc attendre que la carte moteur renvoit 4 bytes
  while( Wire.available() < 4 )
    {
      //DEBUG_MSG( String( " available : " ) + String( Wire.available() ) );
    }

  byte encoder1[4];
  int* value = (int*) encoder1;
  encoder1[3] = Wire.read();
  encoder1[2] = Wire.read();
  encoder1[1] = Wire.read();
  encoder1[0] = Wire.read();

  return (long) *value;
}

long getEncoder2( int address )
{
  // Motor 2 encoder reading
  Wire.beginTransmission( address );
  Wire.write( ENCODER2 );
  Wire.endTransmission();

  Wire.requestFrom( address, 4 );
  while( Wire.available() < 4 );

  byte encoder2[4];
  int* value = (int*) encoder2;
  encoder2[3] = Wire.read();
  encoder2[2] = Wire.read();
  encoder2[1] = Wire.read();
  encoder2[0] = Wire.read();

  //sendCommand( address, ENCODER1, *value );

  return (long) *value;
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
  sendMessage( String( digitalRead( 2 ) ) );
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

////////////
// SONAR  //
///////////

// trigger the sonar, wait for the echo and the return the range in cm
long getSonarRange()
{
  // establish variables for duration of the ping, 
  // and the distance result in inches and centimeters:
  unsigned long duration, cm;

  pinMode(SONAR_PIN, OUTPUT);
  digitalWrite(SONAR_PIN, LOW);
  delayMicroseconds(LOW_PULSE);
  digitalWrite(SONAR_PIN, HIGH);
  delayMicroseconds(HIGH_PULSE);
  digitalWrite(SONAR_PIN, LOW);
  
  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(SONAR_PIN, INPUT);

  // FIXME: handle bade cases when the pulseIn get
  // another signal than our echo
  // ie if the duration is to short, also
  // need to determine the timeout in accord with the maximum
  // range we want to read, and then in case of timeout return a certain
  // value, finally we have to make sure we waited enough before emmitting 
  // again in order to avoir bad echo coming in
  duration = pulseIn(SONAR_PIN, HIGH, SONAR_TIMEOUT);

  if(duration == 0)
    duration = SONAR_TIMEOUT;

  // convert the time into a distance

  // 29 microsecondes par centimetre.
  // on divise par deux pour prendre en compte l'aller-retour
  // WARNING: calibration needed
  cm = duration / 29 / 2;
 
  // FIXME: the delay waited here should take into account
  // the delay waited before.
  // the sonar datasheet claimed that a sonar can make measures
  // at best every 50ms
  // take into account the loop time length
  //delay(50);

  return cm;
}
