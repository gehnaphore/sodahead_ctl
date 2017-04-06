
#include <sodahead/sodahead_driver.h>
#include <ros/package.h>
#include "yaml-cpp/yaml.h"
#include "math.h"
#include <algorithm>
#include <fstream>

namespace sodahead
{

SodaheadDriver::SodaheadDriver( ros::NodeHandle &nh, boost::asio::io_service *ioService ) :
	nh( nh ),
	sodahead_dev(ioService)
{
	for( int i = 0; i < 32; i++ )
		channels[i] = NULL;

	ros::NodeHandle priv_nh( "~" );

	priv_nh.param<std::string>( "port", port, "/dev/ttyFOO" );
	priv_nh.param<int>( "baud", baud, 115200 );
	priv_nh.param<std::string>( "calib_dir", calib_dir, ros::package::getPath("sodahead_ctl")+"/calib" );
	priv_nh.param<bool>( "publish_joint_states", publish_joint_states, true );

	range_scale = 1.0;
	scale = range_scale * 2000.0 / M_PI;

	// Parse joints ros param
	XmlRpc::XmlRpcValue joints_list;
	if( priv_nh.getParam( "joints", joints_list ) )
	{
		ROS_ASSERT( joints_list.getType( ) == XmlRpc::XmlRpcValue::TypeStruct );

		XmlRpcValueAccess joints_struct_access( joints_list );
		XmlRpc::XmlRpcValue::ValueStruct joints_struct = joints_struct_access.getValueStruct( );

		XmlRpc::XmlRpcValue::ValueStruct::iterator joints_it;

		for( joints_it = joints_struct.begin( ); joints_it != joints_struct.end( ); joints_it++ )
		{
			Joint *joint = new Joint;
			joint->name = static_cast<std::string>( joints_it->first );

			std::string joint_graph_name = "joints/" + joint->name + "/";

			priv_nh.param<int>( joint_graph_name + "channel", joint->properties.channel, 0 );

			// Channel must be between 0 and 31, inclusive
			ROS_ASSERT( joint->properties.channel >= 0 );
			ROS_ASSERT( joint->properties.channel <= 31 );

			priv_nh.param<double>( joint_graph_name + "max_angle", joint->properties.max_angle, M_PI_2 );
			priv_nh.param<double>( joint_graph_name + "min_angle", joint->properties.min_angle, -M_PI_2 );
			priv_nh.param<double>( joint_graph_name + "offset_angle", joint->properties.offset_angle, 0 );
			priv_nh.param<double>( joint_graph_name + "default_angle", joint->properties.default_angle, joint->properties.offset_angle );
			priv_nh.param<bool>( joint_graph_name + "initialize", joint->properties.initialize, true );
			priv_nh.param<bool>( joint_graph_name + "invert", joint->properties.invert, false );
			priv_nh.param<int>( joint_graph_name + "pulse_offset", joint->properties.pulse_offset, 0 );

			// Make sure no two joints have the same channel
			ROS_ASSERT( channels[joint->properties.channel] == NULL );

			// Make sure no two joints have the same name
			ROS_ASSERT( joints_map.find( joint->name ) == joints_map.end( ) );

			channels[joint->properties.channel] = joint;
			joints_map[joint->name] = joint;
		}
	}
	else
	{
		ROS_FATAL( "No joints were given" );
		ROS_BREAK( );
	}

	// Parse controllers ros param
	XmlRpc::XmlRpcValue controllers_list;
	if( priv_nh.getParam( "controllers", controllers_list ) )
	{
		ROS_ASSERT( controllers_list.getType( ) == XmlRpc::XmlRpcValue::TypeStruct );

		// Get the controllers ValueStruct
		XmlRpcValueAccess controllers_struct_access( controllers_list );
		XmlRpc::XmlRpcValue::ValueStruct controllers_struct = controllers_struct_access.getValueStruct( );

		XmlRpc::XmlRpcValue::ValueStruct::iterator controllers_it;

		// For each controller, parse its type and the joints associated with the controller
		for( controllers_it = controllers_struct.begin( ); controllers_it != controllers_struct.end( ); controllers_it++ )
		{
			Controller *controller = new Controller;
			controller->name = static_cast<std::string>( controllers_it->first );

			std::string controller_graph_name = "controllers/" + controller->name + "/";

			std::string controller_type;
			priv_nh.param<std::string>( controller_graph_name + "type", controller_type, "joint_controller" );

			// Validate the controller type
			if( controller_type == "joint_controller" )
				controller->type = ControllerTypes::JointController;
			else if( controller_type == "diff_drive_controller" )
				controller->type = ControllerTypes::DiffDriveController;
			else
			{
				ROS_FATAL( "Unknown controller type [%s] for controller [%s]",
					controller_type.c_str( ), controller->name.c_str( ) );
				delete controller;
				ROS_BREAK( );
			}

			priv_nh.param<bool>( controller_graph_name + "publish_joint_states", controller->publish_joint_states, true );

			// Get publish rate
			priv_nh.param<double>( controller_graph_name + "publish_rate", controller->publish_rate, 10.0 );
			if( controller->publish_rate <= 0.0 )
			{
				controller->expected_publish_time = 0.0;
				controller->publish_joint_states = false;
			}
			else
				controller->expected_publish_time = ( 1.0 / controller->publish_rate );

			// Make sure the controller has joints
			if( priv_nh.getParam( controller_graph_name + "joints", joints_list ) )
			{
				ROS_ASSERT( joints_list.getType( ) == XmlRpc::XmlRpcValue::TypeArray );

				// Get joints array
				XmlRpcValueAccess joints_array_access( joints_list );
				XmlRpc::XmlRpcValue::ValueArray joints_array = joints_array_access.getValueArray( );

				// Parse the joint names and verify the joint exists
				for( unsigned int i = 0; i < joints_array.size( ); i++ )
				{
					std::string joint_name = static_cast<std::string>( joints_array[i] );

					if( joints_map.find( joint_name ) != joints_map.end( ) )
					{
						controller->joints.push_back( joints_map[joint_name] );
					}
					else // joint doesn't exist
					{
						ROS_FATAL( "Joint [%s] for controller [%s] does not exist",
							joint_name.c_str( ), controller->name.c_str( ) );
						delete controller;
						ROS_BREAK( );
					}
				}
			}
			else // No joints were provided
			{
				ROS_FATAL( "Controller [%s] has no joints listed.", controller->name.c_str( ) );
				delete controller;
				ROS_BREAK( );
			}

			controllers.push_back( controller );
			controllers_map[controller->name] = controller;
		}
	}
	else
	{
		/*!
		 * \todo Instead of throwing an error if no controllers are give,
		 *       maybe put all joints into a 'global' controller that can
		 *       be controlled over the topic cmd_joint_traj.
		 */
		ROS_FATAL( "No controllers were given" );
		ROS_BREAK( );
	}

	calibration_service = nh.advertiseService( "calibrate", &SodaheadDriver::calibrateCallback, this );

}

SodaheadDriver::~SodaheadDriver( )
{
	stop( );

	for( int i = 0; i < 32; i++ )
		if( channels[i] )
			delete channels[i];

	while( !controllers.empty( ) )
	{
		Controller *controller = controllers.back( );
		controllers.pop_back( );
		delete controller;
	}
}

bool SodaheadDriver::readCalibrationData(int ch, servo_calibration_data_t &calib) {
	std::stringstream ss;
	ss << calib_dir << "/servo" << ch << ".yaml";
	std::cout << "Looking for servo " << ch << " calibration data at " << ss.str() << std::endl;
	try {
		YAML::Node node = YAML::LoadFile(ss.str());

		if (node.Type() != YAML::NodeType::Map) return false;

		calib.minPW = node["minPulseWidth"].as<int>();
		calib.maxPW = node["maxPulseWidth"].as<int>();
		calib.minRadians = node["minRadians"].as<double>();
		calib.maxRadians = node["maxRadians"].as<double>();

		const YAML::Node &fwd = node["forwardPoints"];
		calib.forwardPoints.clear();
		for (std::size_t i=0;i<fwd.size();i++) {
			calibration_point_t p;
			YAML::Node pt = fwd[i];
			p.pulseWidth = pt[0].as<int>();
			p.positionMeasurement = pt[1].as<int>();
			calib.forwardPoints.push_back(p);
		}

		const YAML::Node &rvs = node["reversePoints"];
		calib.reversePoints.clear();
		for (std::size_t i=0;i<rvs.size();i++) {
			calibration_point_t p;
			YAML::Node pt = rvs[i];
			p.pulseWidth = pt[0].as<int>();
			p.positionMeasurement = pt[1].as<int>();
			calib.reversePoints.push_back(p);
		}

		return true;
	} catch (...) {
		std::cout << "No calibration data for servo" << ch << std::endl;
		return false;
	}
}

bool SodaheadDriver::init( )
{
	bool success = true;

	// Initialize each controller
	for( unsigned int i = 0; i < controllers.size( ); i++ )
	{
		ROS_DEBUG( "Initializing controller %s", controllers[i]->name.c_str( ) );

		// Only initialize the controller if it's a joint controller
		if( controllers[i]->type == ControllerTypes::JointController )
		{
			for( unsigned int j = 0; j < controllers[i]->joints.size( ); j++ )
			{
				Joint *joint = controllers[i]->joints[j];
				int ch = joint->properties.channel;
				servo_calibration_data_t calib;
				if (readCalibrationData(j,calib)) {
					sodahead_dev.setCalibrationData(ch,calib);
				}

				//sodahead_dev.setPulseOffset(joint->properties.channel, joint->properties.pulse_offset);
				if( joint->properties.initialize ) {
					if (sodahead_dev.isCalibrated(ch)) {
						ROS_INFO( "Initializing channel %d to radians == 0", ch);
						sodahead_dev.moveServoRadians(ch,0);
					} else {
						int pw = ( unsigned int )( scale * ( joint->properties.default_angle - joint->properties.offset_angle ) + 1500 + 0.5 );
						ROS_INFO( "Initializing channel %d to pulse width %d", ch, pw );
						if( pw < 500 ) pw = 500;
						else if( pw > 2500 ) pw = 2500;
						sodahead_dev.moveServo(ch,pw);
					}
				}
			}
		}
	}

	return success;
}

bool SodaheadDriver::spin( )
{
	ros::Rate loop_rate(10);
	bool result = true;

	if( start( ) && init( ) )
	{
		ROS_INFO( "Spinning..." );

		while( ros::ok( ) )
		{
			update( );
			ros::spinOnce( );
			loop_rate.sleep();
			ros::spinOnce( );
		}
	}
	else
	{
		ROS_ERROR( "Failed to start" );
		result = false;
	}

	stop( );

	return result;
}

bool SodaheadDriver::start( )
{
	ROS_INFO( "Starting SodaheadDriver" );

	// Start device
	if( !sodahead_dev.openPort( port.c_str( ), baud ) )
		return false;

	// Subscribe and advertise for every controller
	for( unsigned int i = 0; i < controllers.size( ); i++ )
	{
		joint_state_pubs_map[controllers[i]->name] = nh.advertise<sensor_msgs::JointState>( /*controllers[i]->name +*/ "/joint_states", 1 );
		joint_subs.push_back( nh.subscribe( controllers[i]->name + "/command", 1, &SodaheadDriver::jointCallback, this ) );
	}

	sodahead_dev.setServoStateCallback(std::bind(&SodaheadDriver::servoStateCallback,this,
		std::placeholders::_1, std::placeholders::_2));
	sodahead_dev.setServoCalibrationCallback(std::bind(&SodaheadDriver::calibrationCompletedCallback,this,
		std::placeholders::_1, std::placeholders::_2));

	return true;
}

void SodaheadDriver::stop( )
{
	ROS_INFO( "Stopping SodaheadDriver" );

	nh.shutdown( );

	joint_state_pubs_map.clear( );

	joint_subs.clear( );

	sodahead_dev.closePort( );
}

void SodaheadDriver::update( )
{
	current_time = ros::Time::now( );
/*
	if( publish_joint_states )
	{
		publishJointStates( );
	}

	for( std::map<std::string, std::queue<Command> >::iterator it = command_queues.begin( ); it != command_queues.end( ); it++ )
		execute_command( it->first );
*/
	last_time = current_time;
}

void SodaheadDriver::publishJointStates( )
{
	for( unsigned int i = 0; i < controllers.size( ); i++ )
	{
		if( controllers[i]->publish_joint_states &&
			fabs( ( controllers[i]->last_publish_time - current_time ).toSec( ) ) >= controllers[i]->expected_publish_time)
		{
			sensor_msgs::JointState joints;
			joints.header.stamp = current_time;

			for( unsigned int j = 0; j < controllers[i]->joints.size( ); j++ )
			{
				joints.name.push_back( controllers[i]->joints[j]->name );

				servo_state_t state = sodahead_dev.queryServo( controllers[i]->joints[j]->properties.channel );
				int pw = state.pulseWidth;

				if( controllers[i]->joints[j]->properties.invert )
					pw = 3000 - pw;

				double angle = ( ( double ) pw - 1500.0 ) / scale + controllers[i]->joints[j]->properties.offset_angle;

				joints.position.push_back( angle );
			}

			joint_state_pubs_map[controllers[i]->name].publish( joints );

			controllers[i]->last_publish_time = current_time;
		}
	}
}
/*
struct calibration_point_t {
	int16_t pulseWidth;
	int32_t positionMeasurement;
};

struct servo_calibration_data_t {
	int minPW, maxPW;
	double minRadians, maxRadians;
	std::vector<calibration_point_t> forwardPoints;
	std::vector<calibration_point_t> reversePoints;
};
*/
void SodaheadDriver::calibrationCompletedCallback(int ch, const servo_calibration_data_t &calib)
{
	YAML::Emitter out;
	out << YAML::BeginMap;

	out << YAML::Key << "minPulseWidth";
	out << YAML::Value << calib.minPW;

	out << YAML::Key << "maxPulseWidth";
	out << YAML::Value << calib.maxPW;

	out << YAML::Key << "minRadians";
	out << YAML::Value << calib.minRadians;

	out << YAML::Key << "maxRadians";
	out << YAML::Value << calib.maxRadians;

	out << YAML::Key << "forwardPoints";
	out << YAML::Value << YAML::BeginSeq;
	for (int i=0;i<calib.forwardPoints.size();i++) {
		const calibration_point_t &p = calib.forwardPoints[i];
		out << YAML::Flow << YAML::BeginSeq << p.pulseWidth << p.positionMeasurement << YAML::EndSeq;
	}
	out << YAML::EndSeq;

	out << YAML::Key << "reversePoints";
	out << YAML::Value << YAML::BeginSeq;
	for (int i=0;i<calib.reversePoints.size();i++) {
		const calibration_point_t &p = calib.reversePoints[i];
		out << YAML::Flow << YAML::BeginSeq << p.pulseWidth << p.positionMeasurement << YAML::EndSeq;
	}
	out << YAML::EndSeq;
	
	out << YAML::EndMap;

	std::stringstream ss;
	ss << calib_dir << "/servo" << ch << ".yaml";
	std::ofstream f(ss.str());
	f << out.c_str();
    f.close();
}

void SodaheadDriver::servoStateCallback(int ch, const servo_state_t &state)
{
	ros::Time now = ros::Time::now( );

	sensor_msgs::JointState joints;
	joints.header.stamp = now;

	joints.name.push_back( controllers[0]->joints[ch]->name );
/*
	int pw = state.pulseWidth;

	if( controllers[0]->joints[ch]->properties.invert )
		pw = 3000 - pw;

	double angle = ( ( double ) pw - 1500.0 ) / scale + controllers[0]->joints[ch]->properties.offset_angle;
*/	
	double angle = state.positionInRadians + controllers[0]->joints[ch]->properties.offset_angle;
//	printf("%s:%lf\n",controllers[0]->joints[ch]->name.c_str(),angle);

	joints.position.push_back( angle );
	joint_state_pubs_map[controllers[0]->name].publish( joints );
	controllers[0]->last_publish_time = now;
}

bool SodaheadDriver::calibrateCallback(sodahead_ctl::CalibrateServo::Request &cmd, sodahead_ctl::CalibrateServo::Response &r)
{
	sodahead_dev.calibrateServo(cmd.channel, cmd.minPulseWidth, cmd.minRadians, cmd.maxPulseWidth, cmd.maxRadians );
	r.errorCode = 0;
	return true;
}

void SodaheadDriver::jointCallback( const ros::MessageEvent<trajectory_msgs::JointTrajectory const>& event )
{
	ros::M_string connection_header = event.getConnectionHeader( );
	const trajectory_msgs::JointTrajectoryConstPtr &msg = event.getMessage( );

	std::string topic = connection_header["topic"];

	if( topic.empty( ) )
	{
		ROS_ERROR( "The connection header topic is empty" );
		return;
	}

	// Remove beginning '/'
	if( topic[0] == '/')
		topic.erase( 0, 1 );

	// Extract the controller name from the topic
	std::string::iterator it = find( topic.begin( ), topic.end( ), '/' );
	if( it != topic.end( ) )
		topic.erase( it, topic.end( ) );

	// Validate the controller name
	if( controllers_map.find( topic ) == controllers_map.end() )
	{
		ROS_ERROR( "[%s] is not a valid controller name.", topic.c_str( ) );
		return;
	}

	int num_joints = controllers_map[topic]->joints.size( );

	ros::Duration prev_time_from_start = ros::Duration( 0 );

	for( unsigned int i = 0; i < msg->points.size(); i++ )
	{
		bool invalid = false;

		for( unsigned int j = 0; j < msg->joint_names.size( ) && !invalid; j++ )
		{
			if( joints_map.find( msg->joint_names[j] ) != joints_map.end( ) )
			{
				Joint *joint = joints_map[msg->joint_names[j]];

				int ch = joint->properties.channel;
				if( msg->points[i].velocities.size( ) > j /*&& msg->points[i].velocities[j] > 0 */) {
					// If velocity is present, use that
					double radiansPerSec = msg->points[i].velocities[j];
					double angle = sodahead_dev.queryServo(ch).positionInRadians;
					double secondsToTake = 1.0;
					if ((angle + radiansPerSec) > joint->properties.max_angle) { 
						double untilMax = joint->properties.max_angle - angle;
						secondsToTake = secondsToTake * untilMax/radiansPerSec;
						radiansPerSec = untilMax;
					} else if ((angle + radiansPerSec) < joint->properties.min_angle) { 
						double untilMin = joint->properties.min_angle - angle;
						secondsToTake = secondsToTake * untilMin/radiansPerSec;
						radiansPerSec = untilMin;
					}
					sodahead_dev.moveServoRadians(ch, angle + radiansPerSec, 1000*secondsToTake);
				} else {
					double angle = msg->points[i].positions[j];
					if( angle >= joint->properties.min_angle && angle <= joint->properties.max_angle ) {
						sodahead_dev.moveServoRadians(ch, angle - joint->properties.offset_angle);
					} else {
						invalid = true;
						ROS_ERROR( "The given position [%f] for joint [%s] is invalid", angle, joint->name.c_str( ) );
					}
				}
			}
			else
			{
				invalid = true;
				ROS_ERROR( "Joint [%s] does not exist", msg->joint_names[i].c_str( ) );
			}
		}
	}
}

}
