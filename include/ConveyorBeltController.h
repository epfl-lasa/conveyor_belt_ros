#ifndef __CONVOYER_BELT_CONTROLLER_H__
#define __CONVOYER_BELT_CONTROLLER_H__

#include "ros/ros.h"
#include <serial/serial.h>
#include "std_msgs/Int32.h"
// #include <mutex>

#include <dynamic_reconfigure/server.h>
#include <conveyor_belt_ros/conveyorBelt_paramsConfig.h>


class ConveyorBeltController 
{

	private:

		// ROS variables
		ros::NodeHandle _n;
		ros::Rate _loopRate;

		// Subscribers and publishers declaration
		ros::Publisher _pubConveyorBeltMode;              // Publish conveyor belt mode
		ros::Publisher _pubConveyorBeltSpeed;             // Publish conveyor belt speed 
		ros::Publisher _pubConveyorBeltAcceleration;      // Publish conveyor belt acceleration
		ros::Publisher _pubConveyorBeltDecceleration;     // Publish conveyor belt decceleration
		ros::Subscriber _subConveyorBeltMode;             // Subscribe to conveyor belt desired mpde

		// Publisher messages declaration
		std_msgs::Int32 _modeMessage;
		std_msgs::Int32 _speedMessage;
		std_msgs::Int32 _accelerationMessage;
		std_msgs::Int32 _deccelerationMessage;

		// Conveyor belt variables
		int _mode;							// Conveyor belt mode [-] (0: stop, 1: start right, 2: start left) 
		int _desiredSpeed;					// Desired conveyor belt speed [mm/s] [40,1500]
		int _measuredSpeed;             	// Measured conveyor belt speed [mm/s] [40,1500]
		int _acceleration;              	// Desired conveyor belt acceleration time [0.1s] to go from 0 t0 1500 mm/s [10,600]
		int _decceleration;             	// Desired conveyor belt decceleration time [0.1s] to go from 1500 t0 0 mm/s [10,600]

		// Serial interface
		serial::Serial _serial;				// Serial interface object
		std::string _outputSerialMessage;   // Output message to send via serial interface

		// std::mutex _mutex;

		// Dynamic reconfigure (server+callback+config)
		dynamic_reconfigure::Server<conveyor_belt_ros::conveyorBelt_paramsConfig> _dynRecServer;
		dynamic_reconfigure::Server<conveyor_belt_ros::conveyorBelt_paramsConfig>::CallbackType _dynRecCallback;
		conveyor_belt_ros::conveyorBelt_paramsConfig _config;


	public:

		// Class constructor
		ConveyorBeltController(ros::NodeHandle &n, float frequency);

		// Initialize node
		bool init();

		// Run node main loop
		void run();

		// Set desired conveyor belt configuration
		void setDesiredConfig(int mode, int speed, int acceleration, int decceleration);


	private:

		// Publish data to topics
		void publishData();
		
		// Stop conveyor belt
		void stopConveyorBelt();

		// Process input message from the covneyor belt
		void processInputSerialMessage(std::string input);

		// Send command to conveyor belt
		void sendCommand();

		// Build output message to be sent to the conveyor belt
		void buildOutputSerialMessage();

		// Build zero padded string from input integer and desired string size
		std::string zeroPaddedStringConversion(int value, int desiredSize);

		// Update conveyor belt mode externally from other programs
		void updateConveyorBeltMode(const std_msgs::Int32::ConstPtr& msg); 

		// Dynamic reconfigure callback
		void dynamicReconfigureCallback(conveyor_belt_ros::conveyorBelt_paramsConfig &config, uint32_t level); 
};


#endif
