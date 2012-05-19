#include "Urduino.hh"

#include <cmath>

Urduino::Urduino(const std::string &n) :
  urbi::UObject(n),
  Arduino()
{
  UBindFunction( Urduino, init );
}

int Urduino::init(std::string serialport, int baudrate)
{
  int res = Arduino::init(serialport, baudrate);

  // Binding
  UBindFunction( Urduino, openClamp );
  UBindFunction( Urduino, closeClamp );
  UBindFunction( Urduino, rotate );
  UBindFunction( Urduino, translate );
  UBindFunction( Urduino, getSonar );
  UBindFunction( Urduino, setPidRotation );
  UBindFunction( Urduino, setPidTranslation );

  UBindVar( Urduino, started );
  started = false;

  UBindVar( Urduino, translationEnded );
  UBindVar( Urduino, rotationEnded );
  translationEnded = false;
  rotationEnded = false;

  UBindEvent( Urduino, endTranslation );
  UBindEvent( Urduino, endRotation );

  serialPort = fdopen( Arduino::mFd, "w+" );

  listeningFunctor = Functor< Urduino >(
					this,
					&Urduino::listenPort,
					NULL);

  listeningFunctor.startThreaded( &listeningThread, NULL );
  pthread_detach( listeningThread );

  sonar = 0;

  return res;
}

void* Urduino::listenPort( void* )
{
  std::string current_msg;
  fd_set read_set;
  char buffer[1];

  while ( true )
    {
      //usleep(20);
      //std::cout << "begin loop" << std::endl;
      FD_SET( Arduino::mFd, &read_set );
      select( Arduino::mFd + 1, &read_set, NULL, NULL, NULL );

      if ( FD_ISSET( Arduino::mFd, &read_set ) )
	{
	  // read the next char
	  int n = read( Arduino::mFd, buffer, 1);

	  if ( n <= 0 ) continue;

	  if ( buffer[0] != '\n' )
	    {
	      current_msg.append( 1, buffer[0] );
	    }
	  else
	    {
	      if ( current_msg.size() < 1 )
		{
		  current_msg.clear();
		  continue;
		}

	      // data
	      //std::cout << "message received : " << current_msg << std::endl;
	      switch ( current_msg.at(0) )
		{
		case '$':
		  if ( current_msg.size() < 2 )
		    {
		      std::cout << "bad size captor" << std::endl;
		      current_msg.clear();
		      continue;
		    }

		  switch (current_msg.at(1))
		    {
		    case SONAR_TKN:
		      if ( current_msg.size() < 3 )
			{
			  std::cout << "bad size sonar" << std::endl;
			  current_msg.clear();
			  continue;
			}
		      pthread_mutex_lock( &sonar_lock );
		      sonar = atoi( current_msg.c_str() + 2 );
		      //std::cout << "sonar rec : "
		      //	<< sonar << std::endl;
		      pthread_mutex_unlock( &sonar_lock );
		      break;
		    case TRANSLATION_TKN:
		      endTranslation.emit();
		      /*
		      pthread_mutex_lock( &state_lock );
		      translationEnded = true;
		      pthread_mutex_unlock( &state_lock );
		      */
		      break;
		    case ROTATION_TKN:
		      endRotation.emit();
		      /*
		      pthread_mutex_lock( &state_lock );
		      rotationEnded = true;
		      pthread_mutex_unlock( &state_lock );
		      */
		      break;
		    case START_TKN:
		      started = true;
		      break;
		    default:
		      std::cerr << "caca" << std::endl;
		      break;
		    }
		  break;
		case '%':
		  std::cout << current_msg << std::endl;
		  break;
		case '#':
		  std::cerr << current_msg << std::endl;
		  break;
		default:
		  std::cerr << "super caca" << std::endl;
		}

	      current_msg.clear();
	    }
	}
      //fflush(stdout);
    }
}

void Urduino::writeCommand( int type, int id, int value )
{
  uint8_t* tab = (uint8_t*) &value;

  writeByte( BEGIN_TRANSMISSION );
  writeByte( (uint8_t) type );
  writeByte( (uint8_t) id );
  writeByte( tab[0] );
  writeByte( tab[1] );
  writeByte( tab[2] );
  writeByte( tab[3] );
  writeByte( END_TRANSMISSION );
  fflush( serialPort );
}

void Urduino::rotate( ufloat rad )
{
  int value = (int) ( (rad * 458 * 2) / M_PI );

  writeCommand( 1, 1, value );

  usleep(100);
  /*
  pthread_mutex_lock( &state_lock );
  rotationEnded = false;
  pthread_mutex_unlock( &state_lock );
  */
}

void Urduino::translate( ufloat meters )
{
  int value = (int) ( (meters * 700) / 0.3 );

  writeCommand( 1, 0, value );
  
  usleep(100);
  /*
  pthread_mutex_lock( &state_lock );
  translationEnded = false;
  pthread_mutex_unlock( &state_lock );
  */
}

void Urduino::openClamp()
{
  writeCommand( 0, 1, 0 );
}

void Urduino::closeClamp()
{
  writeCommand( 0, 0, 0 );
}

ufloat Urduino::getSonar()
{
  pthread_mutex_lock( &sonar_lock );
  ufloat res = sonar;
  pthread_mutex_unlock( &sonar_lock );

  return res;
}

void Urduino::setPidTranslation(int p, int i, int d)
{
  writeCommand(4, 0, p);// send p value
  writeCommand(4, 1, d);// send p value
  
  usleep(100);
  // FIXME: i not used
  // FIXME: send float, for now the value is *100
}

void Urduino::setPidRotation(int p, int i, int d)
{
  writeCommand(4, 2, p);// send p value
  writeCommand(4, 3, d);// send p value
  
  usleep(100);
  // FIXME: i not used
}

UStart(Urduino);
