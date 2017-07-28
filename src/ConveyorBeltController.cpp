#include "ConveyorBeltController.h"


ConveyorBeltController::ConveyorBeltController(ros::NodeHandle &n, float frequency):
	_n(n),
  _loopRate(frequency)
{

	ROS_INFO_STREAM("The conveyor belt controller node is created at: " << _n.getNamespace() << " with freq: " << frequency << "Hz.");
}


bool ConveyorBeltController::init() 
{

  // Pulibsher definitions (mode, measured speed of the conveyor, acceleration, decceleration)
  _pubConveyorBeltMode = _n.advertise<std_msgs::Int32>("conveyor_belt/mode", 1);
  _pubConveyorBeltSpeed = _n.advertise<std_msgs::Int32>("conveyor_belt/speed", 1);
  _pubConveyorBeltAcceleration = _n.advertise<std_msgs::Int32>("conveyor_belt/acc", 1);
  _pubConveyorBeltDecceleration = _n.advertise<std_msgs::Int32>("conveyor_belt/dec", 1);

  // Subscriber definition (desired mode)
  _subConveyorBeltMode = _n.subscribe("/conveyor_belt/desired_mode", 1, &ConveyorBeltController::updateConveyorBeltMode,this,ros::TransportHints().reliable().tcpNoDelay());

  // Dynamic reconfigure configuration
	_dynRecCallback = boost::bind(&ConveyorBeltController::dynamicReconfigureCallback,this, _1, _2);
	_dynRecServer.setCallback(_dynRecCallback);
	_dynRecServer.getConfigDefault(_config);

  // Get default conveyor belt configuration from dynamic reconfigure 
	_mode = _config.mode;
	_desiredSpeed = _config.speed;
	_acceleration = _config.acc;
	_decceleration = _config.dec;

  // Open serial communication
  try
  {
    _serial.setPort("/dev/ttyS0");
    _serial.setBaudrate(9600);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
    _serial.setTimeout(timeout);
    _serial.setBytesize(serial::eightbits);
    _serial.setParity(serial::parity_even);
    _serial.setStopbits(serial::stopbits_one);
    _serial.open();
  }
  catch (serial::IOException& e)
  {
    ROS_ERROR_STREAM("Unable to open port ");
    return false;
  }

  if(_serial.isOpen())
  {
    ROS_INFO_STREAM("Serial Port initialized");
  }
  else
  {
    return false;
  }

	if (_n.ok())
	{ 
  	// Wait for callback to be called
		ros::spinOnce();
		ROS_INFO("The ros node is ready.");
		return true;
	}
	else 
	{
		ROS_ERROR("The ros node has a problem.");
		return false;
	}
}


void ConveyorBeltController::run() 
{

  // Send initial desired conveyor belt configuration 
  sendCommand();

  while (_n.ok()) 
  {

    // Read and process input message from conveyor
    if(_serial.available())
    {
      std::string message;
      message = _serial.read(_serial.available());
      processInputSerialMessage(message);
    }

    // Publish data to topics
    publishData();
    
    ros::spinOnce();
    _loopRate.sleep();

  }

  // Make sure conveyor belt is stopped at then end if killing the node
  stopConveyorBelt();

  // Close serial communication
  _serial.close();
}


void ConveyorBeltController::setDesiredConfig(int mode, int speed, int acceleration, int decceleration) 
{
	_mode = mode;
	_desiredSpeed = speed;
	_acceleration = acceleration;
	_decceleration = decceleration;

	_config.mode = _mode;
	_config.speed = _desiredSpeed;
	_config.acc = _acceleration;
	_config.dec = _decceleration;

	_dynRecServer.updateConfig(_config);
}


void ConveyorBeltController::publishData()
{

  _pubConveyorBeltMode.publish(_modeMessage);  
  _pubConveyorBeltSpeed.publish(_speedMessage);
  _pubConveyorBeltAcceleration.publish(_accelerationMessage);  
  _pubConveyorBeltDecceleration.publish(_deccelerationMessage);  

}


void ConveyorBeltController::stopConveyorBelt()
{

  _mode = 0;    
  _config.mode = _mode;
  _modeMessage.data = _mode;
  _dynRecServer.updateConfig(_config);

  sendCommand();
}


void ConveyorBeltController::processInputSerialMessage(std::string input)
{
  std::string s1,s2;

  // Extract conveyor belt status (0 = OK / 1 = ERROR)
  s1 = input.substr(0,1);
  s2 = input.substr(1);

  int status = atoi(s1.c_str());

  // If status OK update the measured speed otherwise set it to 0 (?)
  if(!status)
  {
    _measuredSpeed = atoi(s2.c_str());  
  }
  else
  {
    _measuredSpeed = 0;
    ROS_INFO("An error occured, set measured speed to 0");
  }

  _speedMessage.data = _measuredSpeed;
}


void ConveyorBeltController::sendCommand()
{
  // Build output message to the conveyor
  buildOutputSerialMessage();
  std::cerr << "Message sent to conveyor belt: " << _outputSerialMessage << std::endl;

  // Write message to serial
  _serial.write(_outputSerialMessage);
}


void ConveyorBeltController::buildOutputSerialMessage()
{

  // The output message of the conveyor should be: 
  // m|ssss|aaa|ddd <=> mode|speed|acceleration|decceleration

  _outputSerialMessage = zeroPaddedStringConversion(_mode,1)
              + zeroPaddedStringConversion(_desiredSpeed,4)
              + zeroPaddedStringConversion(_acceleration,3)
              + zeroPaddedStringConversion(_decceleration,3);
}


std::string ConveyorBeltController::zeroPaddedStringConversion(int value, int desiredSize)
{

    std::stringstream ss;
    
    // The integer value is converted to string with the help of stringstream
    ss << value; 
    std::string result;
    ss >> result;
    
    // Append zero chars to match the desired string size
    int initialSize = result.length();
    for (int k = 0; k < desiredSize-initialSize; k++)
    {
        result = "0" + result;
    }

    return result;
}


void ConveyorBeltController::updateConveyorBeltMode(const std_msgs::Int32::ConstPtr& msg) 
{
  _mode = msg->data;
  _config.mode = _mode;
  _modeMessage.data = _mode;
  _dynRecServer.updateConfig(_config);
  sendCommand();
}


void ConveyorBeltController::dynamicReconfigureCallback(conveyor_belt_ros::conveyorBelt_paramsConfig &config, uint32_t level) 
{

  ROS_INFO("Reconfigure request. Updatig the parameters ...");
  
  _mode = config.mode;
  _desiredSpeed = config.speed;
  _acceleration = config.acc;
  _decceleration = config.dec;

  _modeMessage.data = _mode;
  _accelerationMessage.data = _acceleration;
  _deccelerationMessage.data = _decceleration;

	sendCommand();

}