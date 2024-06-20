#include "vnIMU.h"
// Eigen::VectorXd imuData = Eigen::VectorXd::Zero(9);
// void asciiOrBinaryAsyncMessageReceived(void *userData, Packet& p, size_t index);
vnIMU::~vnIMU(){
	vs.unregisterAsyncPacketReceivedHandler();
	vs.disconnect();
}
bool vnIMU::initIMU(){
	uint32_t oldBaud = 0;
	try{
		// Now let's create a VnSensor object and use it to connect to our sensor.
		vs.connect(SensorPort, SensorBaudrate);
		//Change the baudrate
		oldBaud = vs.baudrate();
		// try{
		vs.changeBaudRate(SensorBaudrate2);
		// vs.disconnect();
		// vs.connect(SensorPort, SensorBaudrate2);
	}catch(...){
		try{
			vs.disconnect();
			vs.connect(SensorPort, SensorBaudrate2);
		}catch(...){
			return false;
		}
	}
	try{
		uint32_t newBaud = vs.baudrate();
		cout << "Old Baud Rate: " << oldBaud << " Hz" << endl;
		cout << "New Async Rate: " << newBaud << " Hz" << endl;
		//Close the AsyncDataOutput, set the frequency to 0
		uint32_t oldHz = vs.readAsyncDataOutputFrequency();
		vs.writeAsyncDataOutputFrequency(0);
		uint32_t newHz = vs.readAsyncDataOutputFrequency();
		cout << "Old Async Frequency: " << oldHz << " Hz" << endl;
		cout << "New Async Frequency: " << newHz << " Hz" << endl;
		// We change the heading mode used by the sensor.
		VpeBasicControlRegister vpeReg = vs.readVpeBasicControl();
		cout << "Old Heading Mode: " << vpeReg.headingMode << endl;
		vpeReg.headingMode = HEADINGMODE_RELATIVE;
		vs.writeVpeBasicControl(vpeReg, true);
		vpeReg = vs.readVpeBasicControl();
		cout << "New Heading Mode: " << vpeReg.headingMode << endl;
	}catch(...)
	{
		return false;
	}
     // First we create a structure for setting the configuration information
	// for the binary output register to send yaw, pitch, roll data out at
	// 800 Hz.
	BinaryOutputRegister bor(
		ASYNCMODE_PORT1,
		1,
		COMMONGROUP_YAWPITCHROLL | COMMONGROUP_ANGULARRATE | COMMONGROUP_ACCEL,	// Note use of binary OR to configure flags.
		TIMEGROUP_NONE,
		IMUGROUP_NONE,
    GPSGROUP_NONE,
		ATTITUDEGROUP_NONE,
		INSGROUP_NONE,
    GPSGROUP_NONE);
	std::cout<<"write binary output"<<std::endl;
    vs.writeBinaryOutput1(bor);
	std::cout<<"register"<<std::endl;
    vs.registerAsyncPacketReceivedHandler(NULL, vnIMU::asciiOrBinaryAsyncMessageReceived);

    return true;
}
bool vnIMU::closeIMU(){
    vs.unregisterAsyncPacketReceivedHandler();
	vs.disconnect();

    return true;
}