
#include <ros/ros.h>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/utility/in_place_factory.hpp>
#include "sodahead/sodahead_driver.h"

int main( int argc, char** argv )
{
	ros::init( argc, argv, "sodahead_node" );
	ros::NodeHandle nh;

	boost::asio::io_service io_service;
	boost::optional<boost::asio::io_service::work> work(io_service);
	boost::thread io_thread(boost::bind(&boost::asio::io_service::run, &io_service));
	sodahead::SodaheadDriver sodahead( nh, &io_service );

	sodahead.spin( );

	return 0;
}
