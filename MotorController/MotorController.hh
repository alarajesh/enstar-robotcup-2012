#ifndef MOTORCONTROLLER_HH
#define MOTORCONTROLLER_HH

#include <urbi/uobject.hh>

class MotorController : public urbi::UObject
{
public:
	/*!
	 * \brief C++ Constructor
	 */
	MotorController(const std::string& name);

	/*!
	 * \brief Destructor
	 */
	~MotorController();

	/*!
	 * \brief Urbi Constructor
	 */
	void init();

	/*!
	 * \brief Open Serial Port
	 */
	bool openPort();

	bool testWrite();

	bool testRead();

private:
	urbi::UVar inputMotor1Speed; ///< Motor 1 Speed
	urbi::UVar inputMotor2Speed; ///< Motor 2 Speed
	urbi::UVar outputMotor1Current; ///< Motor 1 Current
	urbi::UVar outputMotor2Current; ///< Motor 2 Current
	urbi::UVar outputMotor1Encoder; ///< increment when rotor angle increase from 1 degree
	urbi::UVar outputMotor2Encoder; ///< increment when rotor angle increase from 1 degree
	int fd_serial_port; ///< file descriptor of the serial port
	struct termios serial_port_options; ///< serial port options
};

#endif // MOTORCONTROLLER_HH
