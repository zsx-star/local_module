#ifndef POS_H
#define POS_H

#include <sstream>
#include <iostream>
#include<iostream>
#include<ros/ros.h>
#include<cmath>
#include<serial/serial.h>
#include<boost/thread.hpp>
#include<boost/bind.hpp>
#include<local_module/Local.h>
#include<nav_msgs/Odometry.h>


#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))

int A0A1,A0A2,A1A2;

PACK(
struct posmodule_t
{
	
	double mask;
	double rang0;
	double rang1;
	double rang2;
	double rang3;
	
	double nRangs;
	double resQ;
	double debug;
//	double aT_A;
	
});

class Posmodule
{
public:
	Posmodule();
	~Posmodule();
	
	bool init();
	void startReading();
	void stopReading();
private:
	bool openSerial(const std::string& port,int baudrate);
	void closeSerial();
	void readSerialThread();
	void parseIncomingData(uint8_t* buffer,size_t len);
	
	void parseMa(const char* buffer);
	void parseMr(const char* buffer);
	void parseMc(const char* buffer);
	
private:
	ros::Publisher m_pub_pos;
	serial::Serial *m_serial_port;
	//! shared pointer to Boost thread for listening for data from novatel
	boost::shared_ptr<boost::thread> m_read_thread_ptr;
	bool m_reading_status;  //!< True if the read thread is running, false otherwise.
	uint8_t * const m_pkg_buffer;
	nav_msgs::Odometry m_pos;
};


#endif
