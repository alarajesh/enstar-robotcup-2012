#ifndef URDUINO_HH
# define URDUINO_HH

#include <urbi/uobject.hh>
#include <pthread.h>
#include "arduino.hh"
#include "Functor.hh"
#include "Cemaphore.hh"

#define SONAR_TKN       '0'
#define TRANSLATION_TKN '1'
#define ROTATION_TKN    '2'
#define PID_TKN         '4'
#define START_TKN       '3'

#define BEGIN_TRANSMISSION (uint8_t) '\001'
#define END_TRANSMISSION   (uint8_t) '\004'

typedef struct encoder_data
{
  int  enco1;
  int  enco2;
  long millisec;
} encoder_data;

class Urduino: public urbi::UObject, public Arduino
{
public:
  Urduino( const std::string &n );

  int init( std::string serialport, int baudrate );

  void rotate( ufloat rad );

  void translate( ufloat meters );

  void openClamp();

  void closeClamp();

  ufloat getSonar();
  
  void setPidRotation(int p, int i, int d);
  
  void setPidTranslation(int p, int i, int d);
  
private:
  /*!
   * brief Loop which listen the serial port and interpret the command received.
   */
  void* listenPort( void* unused );

  /*!
   * \brief Send command to the Arduino
   * \param type  Clamp : 0 and Movement : 1
   * \param id    type : 0 -> { close : 0 and open : 1 }
   *              and type : 1 -> { translate : 0 and rotate : 1 }
   * \param value if type : 1 then angle or distance
   */
  void writeCommand( int type, int id, int value );

  // sonar
  ufloat sonar;
  pthread_mutex_t sonar_lock;
  pthread_mutex_t state_lock;

  urbi::UVar started;
  urbi::UVar translationEnded;
  urbi::UVar rotationEnded;

  // Event movement
  urbi::UEvent endTranslation;
  urbi::UEvent endRotation;

  // arduino listening stuff
  Functor< Urduino > listeningFunctor;
  pthread_t listeningThread;

  FILE* serialPort;
};

#endif
