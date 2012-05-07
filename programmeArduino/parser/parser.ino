// vim:ft=c

#define MSG_SIZE  8
#define DIODE_ID  13
#define BEGIN_MSG (byte) '\001'
#define END_MSG   (byte) '\004'
#define OK        '$'
#define NOT_OK    '#'

boolean msgComplete = false; // whether the message is complete
boolean onError     = false; // caca
byte    cardId      = 0;     //
byte    deviceId    = 0;     //
byte    msgValue[4];         //
int*    msgInt =
		(int*) msgValue;
int     inputSize   = 0;     // size of partial msg received

void sendError( String error )
{
	Serial.print( NOT_OK );
	Serial.print( error );
	Serial.print( '\n' );
}

void setup()
{
	// initialize serial:
	Serial.begin(9600);

	// initialize the digital pin as an output.
	// Pin 13 has an LED connected on most Arduino boards:
	pinMode(13, OUTPUT);
}

void loop()
{
	// print the message when a newline arrives:
	if (msgComplete)
	{
		Serial.print( OK );
		Serial.print( cardId );
		Serial.print( deviceId );
		Serial.print( *msgInt );
		Serial.print('\n');
		// clear the message:
		msgComplete = false;
		inputSize   = 0;
	}
}

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

				digitalWrite( DIODE_ID, HIGH ); // set the LED on
				break;

			case 1 :
				cardId = inByte;
				break;

			case 2 :
				deviceId = inByte;
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

		// if the incoming character is a newline, set a flag
		// so the main loop can do something about it:
		if ( inputSize == MSG_SIZE )
		{
			msgComplete = true;
		}
	}
}

