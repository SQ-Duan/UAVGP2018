#ifndef __MYUTILS_H__
#define __MYUTILS_H__

#include <fstream>
#include <string>
#include <iostream>

#include <unistd.h>
#include <fcntl.h> 
#include <pthread.h>
#include <termios.h>
#include <string.h>

typedef struct
{
  bool isReadSuccess;
  
  int nGrabPoints;
  int aimedDoneNumber;
  int tryTimes;
  double batteryThreshold;
  
  double grab_xP;
  double grab_xI;
  double grab_xD;
  double flight_xP;
  double flight_xI;
  double flight_xD;
  
  double grab_yP;
  double grab_yI;
  double grab_yD;
  double flight_yP;
  double flight_yI;
  double flight_yD;

  double cam_ts;
  double cam_tv;
  int cam_switch;
  double timeThreshold;
  float grabHeight;
} ControlArgs;

class SerialPort
{
public:
	SerialPort(const char* port, const int baud);
	~SerialPort(void);
	bool isOpened(void);
	int write_cmd_no_ack(char cmd);
	bool write_cmd(char cmd);
	char read_ack(void);
  bool wait_ack(char ch, int32_t timeout=60000);
	void stopThread(void);
	void release(void);
	void tciflush(void);
	static void* grabserial_task(void* _this);
private:
	std::string m_port;
	pthread_t m_thread;
	int m_serial_fd;
	pthread_mutex_t m_plock;
	int m_status;	// bit0:串口是否打开，bit1:线程是否开启
	bool m_worksig;	//true时工作，false时线程结束
	char m_rxbuf[10];
	bool m_rxflag;	//true表示收到信号
	char m_txbuf[10];
};

ControlArgs read_args(std::string filename);

#endif
