
#include <sodahead/sodahead.h>

#ifndef DEBUG
#define DEBUG 0
#endif

namespace sodahead
{

//Constructor
Sodahead::Sodahead( boost::asio::io_service *ioService ) : m_port(*ioService), m_ioService(ioService)
{
	m_inBufLen = 0;
	m_inBuf[0] = 0;
	memset(m_servos,sizeof(m_servos),0);
}

//Destructor
Sodahead::~Sodahead( )
{
	closePort( );
}

bool Sodahead::openPort( const char *port, int baud_rate )
{
	std::cout << "openPort " << port << " @ " << baud_rate << std::endl;
	m_port.open(port);
	m_port.set_option(boost::asio::serial_port_base::baud_rate      (baud_rate                                        ));
	m_port.set_option(boost::asio::serial_port_base::character_size (8                                                ));
	m_port.set_option(boost::asio::serial_port_base::flow_control   (boost::asio::serial_port_base::flow_control::none));
	m_port.set_option(boost::asio::serial_port_base::parity         (boost::asio::serial_port_base::parity::none      ));
	m_port.set_option(boost::asio::serial_port_base::stop_bits      (boost::asio::serial_port_base::stop_bits::one    ));

	doRead();

	return true;
}

void Sodahead::setServoStateCallback(std::function< void(int, const servo_state_t &) > cb)
{
	m_servoStateCB = cb;
}

void Sodahead::setServoCalibrationCallback(std::function< void(int, const servo_calibration_data_t &) > cb) {
	m_servoCalibrationCB = cb;
}

bool Sodahead::isConnected( )
{
	return m_port.is_open();
}

bool Sodahead::isCalibrated( int ch )
{
	return m_servos[ch].calib.forwardPoints.size() > 0;
}

void Sodahead::closePort( )
{
	if (isConnected()) {
		m_port.close();
	}
}

bool Sodahead::sendMessage( std::string msg )
{
//	std::cout << "sendMessage '" << msg << "'" << std::endl;
	
	m_ioService->post(
		[this, msg]()
		{
			bool write_in_progress = !m_outMsgs.empty();
			m_outMsgs.push_back(msg);
			if (!write_in_progress) doWrite();
		});
		
}

void Sodahead::parseData(const std::string &msg) {
	if (msg[0] == 'S') {
		int id;
		char beingMoved,inMotion,inCalibration;
		servo_state_t state;std:
//		std::cout << "parsing: '" << msg << "'" << std::endl;
		int r = sscanf(msg.c_str(),"S%d:%c%c%c,%d,%d,%d,%d",
			&id,&beingMoved,&inMotion,&inCalibration,
			&state.pulseWidth,&state.position,&state.velocity,
			&state.lastPotDir);
		
		if ((r == 8) && (id >= 0) && (id <= MAX_SERVOS)) {
			state.beingMoved = beingMoved == '1';
			state.inMotion = inMotion == '1';
			state.inCalibration = inCalibration == '1';
			updateServoState(id, state);
		}
	} else if (msg[0] == 'C') {
		int id,dir;
		calibration_point_t pt;
		printf("%s\n",msg.c_str());
		sscanf(msg.c_str(),"C%d:%hd,%d,%d",
			&id,&pt.pulseWidth,&pt.positionMeasurement,&dir);
		if (dir == 1) {
			printf("Received forward calibration point %lu\n",m_servos[id].calib.forwardPoints.size());
			m_servos[id].calib.forwardPoints.push_back(pt);
		} else if (dir == -1) {
			printf("Received reverse calibration point %lu\n",m_servos[id].calib.reversePoints.size());
			m_servos[id].calib.reversePoints.push_back(pt);
		}
	} else {
		std::cout << "Bad data: '" << msg << "'" << std::endl;
	}
}

void Sodahead::doRead()
{
//	std::cout << "doRead" << std::endl;
	m_port.async_read_some(
		boost::asio::buffer(m_inBuf+m_inBufLen, INBUF_SIZE - m_inBufLen),
        [this](const boost::system::error_code &ec, std::size_t len)
        {
          if (!ec) {
//			  std::cout << "read complete " << len << std::endl;
			  if (len > 0) {
				  m_inBufLen += len; 
				  m_inBuf[m_inBufLen] = 0;
//				  std::cout << "buffer: -->" << std::endl << m_inBuf << std::endl << "<--" << std::endl;
				  char *p = strchr(m_inBuf, '\n');
				  while (p) {
					  *p = 0;
					  std::string msg(m_inBuf);
//					  std::cout << "newline found at " << (p-m_inBuf) << ", '" << msg << "'" << std::endl;
					  memmove(m_inBuf, p+1, m_inBufLen - (p-m_inBuf));
					  m_inBufLen -= (p-m_inBuf)+1;
					  m_inBuf[m_inBufLen] = 0;
//					  std::cout << "buffer now: -->" << std::endl << m_inBuf << std::endl << "<--" << std::endl;
					  parseData(msg);
					  p = strchr(m_inBuf, '\n');
				  }

				  if (m_inBufLen == INBUF_SIZE) {
					  // BURN IT ALL DOWN
					  m_inBufLen = 0;
				  }
			  }
			  doRead();
          } else {
		    std::cout << "read error " << ec << std::endl;
            m_port.close();
          }
        });	
}

void Sodahead::doWrite()
{
//	std::cout << "doWrite (queue depth: " << m_outMsgs.size() << ")" << std::endl;
	boost::asio::async_write(m_port,
        boost::asio::buffer(m_outMsgs.front().data(),
          m_outMsgs.front().length()),
        [this](boost::system::error_code ec, std::size_t /*length*/)
        {
//		  std::cout << "write complete" << std::endl;
          if (!ec)
          {
            m_outMsgs.pop_front();
            if (!m_outMsgs.empty())
            {
              doWrite();
            }
          }
          else
          {
            closePort();
          }
        });
}

void Sodahead::finishCalibration(int id) {
	printf("Processing calibration data...\n");
	servo_t &s = m_servos[id];
	int currentDir = 0;

	sort(s.calib.forwardPoints.begin(), s.calib.forwardPoints.end(), 
		[](const calibration_point_t & a, const calibration_point_t & b) -> bool
	{ 
		return (a.pulseWidth < b.pulseWidth);
	});	

	sort(s.calib.reversePoints.begin(), s.calib.reversePoints.end(), 
		[](const calibration_point_t & a, const calibration_point_t & b) -> bool
	{ 
		return (a.pulseWidth < b.pulseWidth);
	});	

	for (int i=0; i<s.calib.forwardPoints.size();i++) {
		calibration_point_t &pt = s.calib.forwardPoints[i];
		printf("C%d:%d,%d,%d\n",id,pt.pulseWidth,pt.positionMeasurement,1);
	}

	for (int i=0; i<s.calib.reversePoints.size();i++) {
		calibration_point_t &pt = s.calib.reversePoints[i];
		printf("C%d:%d,%d,%d\n",id,pt.pulseWidth,pt.positionMeasurement,-1);
	}

	if (m_servoCalibrationCB) m_servoCalibrationCB(id,s.calib);
}

Sodahead::find_result_t Sodahead::findCalibrationNeighborsForPW(int id, int pw, int lastDir, calibration_point_t &pt0, calibration_point_t &pt1) {
	servo_t &s = m_servos[id];
	std::vector<calibration_point_t> &pts = (lastDir == 1) ? s.calib.forwardPoints : s.calib.reversePoints;

	if (!pts.size()) return NO_CALIBRATION_DATA;

	if (pw < pts[0].pulseWidth) {
		pt0 = pts[0];
		pt1 = pts[1];
		return BEFORE_PT0;
	}

	if (pw < pts[pts.size()-1].pulseWidth) {
		pt0 = pts[pts.size()-2];
		pt1 = pts[pts.size()-1];
		return AFTER_PT1;
	}

	for (int i=1;i<pts.size();i++) {
		calibration_point_t &p0 = pts[i-1];
		calibration_point_t &p1 = pts[i];
		if ((p0.pulseWidth <= pw) && (p1.pulseWidth >= pw)) {
			pt0 = p0;
			pt1 = p1;
			return BETWEEN_PT0_PT1;
		}
	}

	return NO_CALIBRATION_DATA;	
}

Sodahead::find_result_t Sodahead::findCalibrationNeighborsForPot(int id, int pot, int lastDir, calibration_point_t &pt0, calibration_point_t &pt1) {
	servo_t &s = m_servos[id];
	std::vector<calibration_point_t> &pts = (lastDir == 1) ? s.calib.forwardPoints : s.calib.reversePoints;

	if (!pts.size()) return NO_CALIBRATION_DATA;

	if (pot > pts[0].positionMeasurement) {
		pt0 = pts[0];
		pt1 = pts[1];
		return BEFORE_PT0;
	}

	if (pot < pts[pts.size()-1].positionMeasurement) {
		pt0 = pts[pts.size()-2];
		pt1 = pts[pts.size()-1];
		return AFTER_PT1;
	}

	for (int i=1;i<pts.size();i++) {
		calibration_point_t &p0 = pts[i-1];
		calibration_point_t &p1 = pts[i];
		if ((p0.positionMeasurement >= pot) && (p1.positionMeasurement <= pot)) {
			pt0 = p0;
			pt1 = p1;
			return BETWEEN_PT0_PT1;
		}
	}

	return NO_CALIBRATION_DATA;	
}

int32_t Sodahead::estimatePotForPW(int id, int pw, int lastDir) {
	calibration_point_t pt0,pt1;
	int result = findCalibrationNeighborsForPW(id,pw,lastDir,pt0,pt1);
	switch (result) {
		case BEFORE_PT0:
		case AFTER_PT1:
		case BETWEEN_PT0_PT1: {
			int deltaPW = (int32_t)pt1.pulseWidth - pt0.pulseWidth;
			int deltaPotv = (int32_t)pt1.positionMeasurement - pt0.positionMeasurement;
			if (deltaPW == 0) return (pt0.positionMeasurement + pt1.positionMeasurement)/2;
			int potv = pt0.positionMeasurement + (pw - pt0.pulseWidth) * deltaPotv / deltaPW;
			return potv;
		}
	}

	return -1;
}

double Sodahead::estimateRadiansForPot(int id, int pot, int lastDir) {
	calibration_point_t pt0,pt1;
	int result = findCalibrationNeighborsForPot(id,pot,lastDir,pt0,pt1);
	switch (result) {
		case BEFORE_PT0:
		case AFTER_PT1:
		case BETWEEN_PT0_PT1: {
			servo_t &s = m_servos[id];
			int deltaPW = (int32_t)pt1.pulseWidth - pt0.pulseWidth;
			int deltaPotv = (int32_t)pt1.positionMeasurement - pt0.positionMeasurement;
			double radPerPW = (s.calib.maxRadians - s.calib.minRadians) / (s.calib.maxPW - s.calib.minPW);
			double r0 = s.calib.minRadians + (pt0.pulseWidth - s.calib.minPW) * radPerPW;
			double r1 = s.calib.minRadians + (pt1.pulseWidth - s.calib.minPW) * radPerPW;
			if (deltaPW == 0) return (r0+r1)/2;
			double r = r0 + (r1-r0) * (pot - pt0.positionMeasurement) / deltaPotv;
			return r;
		}
	}

	return NAN;
}

void Sodahead::updateServoState(int id, servo_state_t &state)
{
	if (!state.inCalibration && m_servos[id].state.inCalibration) {
		finishCalibration(id);
	}

	servo_t &s = m_servos[id];
	if (!s.calib.forwardPoints.empty()) {
		int32_t potv = estimatePotForPW(id, state.pulseWidth, state.lastPotDir);
		state.positionInRadians = estimateRadiansForPot(id, state.position, state.lastPotDir);
		//std::cout << id << ":" << state.pulseWidth << "," << state.position << " (" << potv << ")," << state.positionInRadians << ", " << state.lastPotDir << std::endl;
	}
	s.state = state;
	if (m_servoStateCB) m_servoStateCB(id,state);
}

void Sodahead::moveServoRadians( int ch, double rad, int time )
{
	servo_t &s = m_servos[ch];
	double radPerPW = (s.calib.maxRadians - s.calib.minRadians) / (s.calib.maxPW - s.calib.minPW);
	int pw = s.calib.minPW + ((rad - s.calib.minRadians) / radPerPW);
	moveServo(ch,pw,time);
}

void Sodahead::moveServo( int ch, int pulseWidth, int time )
{
	char buf[256];
	if (m_servos[ch].calib.minPW) {
		if (pulseWidth < m_servos[ch].calib.minPW) pulseWidth = m_servos[ch].calib.minPW;
		else if (pulseWidth > m_servos[ch].calib.maxPW) pulseWidth = m_servos[ch].calib.maxPW;
	}
	if (time == -1) {
		sprintf(buf,"M%d:%d\n",ch,pulseWidth);
	} else {
		sprintf(buf,"M%d:%d,%d\n",ch,pulseWidth,time);
	}
	sendMessage(std::string(buf));
}

void Sodahead::calibrateServo( int ch, int minUs, double minRad, int maxUs, double maxRad ) {
	printf("Starting calibration [(%d,%lf),(%d,%lf)]...\n",minUs,minRad,maxUs,maxRad);
	char buf[256];
	m_servos[ch].calib.forwardPoints.clear();
	m_servos[ch].calib.reversePoints.clear();
	m_servos[ch].calib.minPW = minUs; 
	m_servos[ch].calib.maxPW = maxUs; 
	m_servos[ch].calib.minRadians = minRad; 
	m_servos[ch].calib.maxRadians = maxRad; 
	sprintf(buf,"C%d:%d,%d\n",ch,minUs,maxUs);
	sendMessage(std::string(buf));
}

servo_state_t Sodahead::queryServo( int ch )
{
	return m_servos[ch].state;
}

void Sodahead::setPulseOffset( int ch, int value)
{
	m_servos[ch].pulseWidthOffset = value;
}

void Sodahead::setCalibrationData( int ch, const servo_calibration_data_t &data )
{
	m_servos[ch].calib = data;
}

} // namespace
