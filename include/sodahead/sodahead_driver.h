#ifndef SODAHEAD_NODE_H
#define SODAHEAD_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sodahead_ctl/CalibrateServo.h>
#include <std_srvs/Empty.h>
#include <XmlRpcValue.h>
#include <string>
#include <vector>
#include <queue>
#include "sodahead.h"

namespace sodahead
{

struct Joint
{
	struct JointProperties
	{
		int channel;
		int pulse_offset;
		double min_angle;
		double max_angle;
		double offset_angle; // this angle is considered to be 1500 uS
		double default_angle; // angle that the joint is initialized to (defaults to the offset_angle)
		bool initialize; // Indicates whether to initialize the servo to the default angle on startup.
		bool invert;
	};

	std::string name;
	JointProperties properties;
};

namespace ControllerTypes
{
	enum ControllerType
	{
		JointController,
		DiffDriveController
	};
}
typedef ControllerTypes::ControllerType ControllerType;

class SodaheadDriver;

struct Controller
{
	std::string name;
	ControllerType type;
	std::vector<Joint*> joints; // Pointer to the joints in this controller
	bool publish_joint_states;
	double publish_rate;

	private:
		double expected_publish_time;
		ros::Time last_publish_time;
		friend class SodaheadDriver;
};

class SodaheadDriver
{
	public:
		SodaheadDriver( ros::NodeHandle &nh, boost::asio::io_service *ioService );
		~SodaheadDriver( );
		bool init( );
		bool spin( );
		bool start( );
		void stop( );
		void update( );

	private:
		void publishJointStates( );
		bool readCalibrationData(int ch, servo_calibration_data_t &calib);
		void servoStateCallback(int ch, const servo_state_t &state);
		bool calibrateCallback(sodahead_ctl::CalibrateServo::Request &cmd, sodahead_ctl::CalibrateServo::Response &r);
		void calibrationCompletedCallback(int ch, const servo_calibration_data_t &calib);
		void jointCallback( const ros::MessageEvent<trajectory_msgs::JointTrajectory const>& event );
		void execute_command( std::string );

		ros::NodeHandle nh;
		std::vector<ros::Subscriber> joint_subs;
		std::map<std::string, ros::Publisher> joint_state_pubs_map;
		ros::ServiceServer calibration_service;

		Joint *channels[32];

		std::string port;
		std::string calib_dir;
		int baud;
		bool publish_joint_states;
		double range_scale;
		double scale;
		std::vector<Controller*> controllers;
		std::map<std::string, Controller*> controllers_map;
		std::map<std::string, Joint*> joints_map;

		Sodahead sodahead_dev;

		ros::Time current_time;
		ros::Time last_time;

		/*!
		 * \brief Class that gives access to an XmlRpcValue's ValueStruct or ValueArray.
		 */
		class XmlRpcValueAccess : private XmlRpc::XmlRpcValue
		{
			public:
				XmlRpcValueAccess( XmlRpc::XmlRpcValue xml_rpc_value ) :
					XmlRpc::XmlRpcValue( xml_rpc_value ) { }

				XmlRpc::XmlRpcValue::ValueStruct getValueStruct( )
				{
					assertStruct( );
					return *_value.asStruct;
				}

				XmlRpc::XmlRpcValue::ValueArray getValueArray( )
				{
					assertArray( size( ) );
					return *_value.asArray;
				}
		};
};

};

#endif // SSC32_SSC32_NODE_H
