#ifndef SODAHEAD_H
#define SODAHEAD_H

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <math.h>
#include <string>
#include <sstream>
#include <queue>
#include <functional>
#include <boost/asio.hpp>
#include <cstdio>

namespace sodahead
{

#define MAX_SERVOS 16

struct servo_state_t {
	int pulseWidth;
	int position;
	double positionInRadians;
	int velocity;
	int lastPotDir;
	bool beingMoved:1;
	bool inMotion:1;
	bool inCalibration:1;
};

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

struct servo_t {
	int pulseWidthOffset;
	int maxPulseWidth, minPulseWidth;
	servo_calibration_data_t calib;
	servo_state_t state;
};

class Sodahead
{
	public:
		const static unsigned int MAX_PULSE_WIDTH =	2500;
		const static unsigned int CENTER_PULSE_WIDTH =	1500;
		const static unsigned int MIN_PULSE_WIDTH =	500;

		Sodahead( boost::asio::io_service *ioService );
		~Sodahead( );

		bool openPort( const char *port, int baud );
		bool isConnected( );
		bool isCalibrated( int ch );
		void closePort( );

		void setPulseOffset( int ch, int value );
		void setCalibrationData( int ch, const servo_calibration_data_t &data );

		void moveServo( int ch, int pulseWidth, int time = -1 );
		void moveServoRadians( int ch, double rad, int time = -1 );
		servo_state_t queryServo( int ch );
		void calibrateServo(  int ch, int minUs, double minRad, int maxUs, double maxRad );

		void setServoStateCallback(std::function< void(int, const servo_state_t &) > cb);
		void setServoCalibrationCallback(std::function< void(int, const servo_calibration_data_t &) > cb);

	private:

		enum find_result_t {
			NO_CALIBRATION_DATA = 0,
			BETWEEN_PT0_PT1 = 1,
			BEFORE_PT0 = 2,
			AFTER_PT1 = 3
		};

		void updateServoState(int id, servo_state_t &state);
		void finishCalibration(int id);
		bool sendMessage( std::string msg );
		void parseData(const std::string &msg);
		void doRead();
		void doWrite();

		find_result_t findCalibrationNeighborsForPW(int id, int pw, int lastDir, calibration_point_t &pt0, calibration_point_t &pt1);
		find_result_t findCalibrationNeighborsForPot(int id, int pot, int lastDir, calibration_point_t &pt0, calibration_point_t &pt1);
		int32_t estimatePotForPW(int id, int pw, int lastDir);
		double estimateRadiansForPot(int id, int pot, int lastDir);

		servo_t m_servos[MAX_SERVOS];
		boost::asio::serial_port m_port;
		boost::asio::io_service *m_ioService;
		std::deque<std::string> m_outMsgs;
		std::function< void(int, const servo_state_t &) > m_servoStateCB;
		std::function< void(int, const servo_calibration_data_t &) > m_servoCalibrationCB;
		#define INBUF_SIZE 1024
		char m_inBuf[INBUF_SIZE+1];
		int m_inBufLen;
};

} //namespace

#endif
