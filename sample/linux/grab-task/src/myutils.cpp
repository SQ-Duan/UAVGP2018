#include "myutils.h"

using namespace std;

SerialPort::SerialPort(const char* port, const int baud)
{
	m_port=port;
	m_status=0;
	m_worksig=true;
	m_rxflag=false;

	m_serial_fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);

	if (m_serial_fd != -1)
	{
		cout << "Open SerialPort " << port << " successfully." << endl;
		m_status |= 0x01;

		fcntl(m_serial_fd, F_SETFL, FNDELAY);	

		struct termios options;
		tcgetattr(m_serial_fd, &options);
		bzero(&options, sizeof(options));
		speed_t speed;
		switch (baud){
			case 110  :     speed=B110; break;
			case 300  :     speed=B300; break;   
			case 600  :     speed=B600; break;
			case 1200 :     speed=B1200; break;
			case 2400 :     speed=B2400; break;
			case 4800 :     speed=B4800; break;
			case 9600 :     speed=B9600; break;
			case 19200 :    speed=B19200; break;
			case 38400 :    speed=B38400; break;
			case 57600 :    speed=B57600; break;
			case 115200 :   speed=B115200; break;
		}
		
		cfsetispeed(&options, speed);
		cfsetospeed(&options, speed);

		options.c_cflag |= ( CLOCAL | CREAD |  CS8);

		//c_cc数组的VSTART和VSTOP元素被设定成DC1和DC3，代表ASCII标准的XON和XOFF字符，如果在传输这两个字符的时候就传不过去，需要把软件流控制屏蔽，即：
		options.c_iflag &= ~ (IXON | IXOFF | IXANY);
		//有时候，在用write发送数据时没有键入回车，信息就发送不出去，这主要是因为我们在输入输出时是按照规范模式接收到回车或换行才发送，而更多情况下我们是不必键入回车或换行的。此时应转换到行方式输入，不经处理直接发送，设置如下：
		options.c_lflag &= ~ (ICANON | ECHO | ECHOE | ISIG);
		//还存在这样的情况：发送字符0X0d的时候，往往接收端得到的字符是0X0a，原因是因为在串口设置中c_iflag和c_oflag中存在从NL-CR和CR-NL的映射，即串口能把回车和换行当成同一个字符，可以进行如下设置屏蔽之：
		options.c_iflag &= ~ (INLCR | ICRNL | IGNCR);
		options.c_oflag &= ~(ONLCR | OCRNL);
		//options.c_iflag |= ( IGNPAR | IGNBRK );     

		options.c_cc[VTIME]=0;			
		options.c_cc[VMIN]=0;			
		tcsetattr(m_serial_fd, TCSANOW, &options);

		//if(pthread_create(&this->m_thread, NULL, SerialPort::grabserial_task, this) == 0)
		//{
		//	pthread_mutex_init(&this->m_plock, NULL);
		//	m_status |= 0x02;
		//	cout << "SerialPort thread created." << endl;
		//}
		//else
		//{
		//	close(m_serial_fd);
		//	m_status = 0;
		//	cout << "Cannot create SerialPort thread." << endl;
		//}
	}
	else
	{
		cout << "Cannot open SerialPort" << port << endl;
	}
}
// 判断串口是否打开
bool SerialPort::isOpened(void)
{
	//if(m_status==0x03)
	if(m_status==0x01)
		return true;
	else
		return false;
}
// 暂不使用
void* SerialPort::grabserial_task(void* _this)
{
	SerialPort* pgs=(SerialPort*)_this;
  while(pgs->m_worksig)
  {
		pthread_mutex_lock(&pgs->m_plock);
    if(read(pgs->m_serial_fd, pgs->m_rxbuf, 1)==1)
		{
			pgs->m_rxflag=true;
		}
		pthread_mutex_unlock(&pgs->m_plock);
		usleep(50);
  }
}
// 读取一个字节，没有读取到时返回0
char SerialPort::read_ack(void)
{
	//char rx, flag;

	//pthread_mutex_lock(&m_plock);
	//flag = m_rxflag;
	//rx = m_rxbuf[0];
	//m_rxflag=false;
	//pthread_mutex_unlock(&m_plock);
	
	//if(flag)
	//{
	//	cout << "Receive : " << rx << endl;
	//	return rx;
	//}
	//else
	//	return 0;
	if(read(m_serial_fd, m_rxbuf, 1)==1)
	{
		cout << '[' << m_port << "] Receive : " << m_rxbuf[0] << endl;
		return m_rxbuf[0];
	}
	else
		return 0;
}

// 等待信号ch
bool SerialPort::wait_ack(char ch, int32_t timeout)
{
	int32_t elapsed = 0;
	while(1)
	{
		if(read(m_serial_fd, m_rxbuf, 1)==1 && m_rxbuf[0]==ch)
		{
			cout << '[' << m_port << "] Receive : " << m_rxbuf[0] << endl;
			return true;
		}
		usleep(50000);
		elapsed += 50;
		if(elapsed >= timeout)
		{
			cout << '[' << m_port << "] Timeout : " << ch << " not received." << endl;
			return false;
		}
	}
}

// 直接发送，不等待响应
int SerialPort::write_cmd_no_ack(char cmd)
{
	m_txbuf[0]=cmd;
	cout << '[' << m_port << "] Send : " << cmd << endl;
	return write(m_serial_fd, m_txbuf, 1);
}
// 发送信号，如果没有响应，一直发送直至Timeout
bool SerialPort::write_cmd(char cmd)
{
	int i=0;
	m_txbuf[0]=cmd;
	tciflush();
	for(;i<10;++i)
	{
		if(write(m_serial_fd, m_txbuf, 1)==1)
		{
			usleep(100000);
			if(read(m_serial_fd, m_rxbuf, 1)==1 && m_rxbuf[0]=='a')
				break;
		}
	}
	if(i<10)
	{
		cout << '[' << m_port << "] i=" << i << "; Send : " << cmd << endl;
		return true;
	}
	else
	{
		cout <<'[' << m_port << "] Error; Send : " << cmd << endl;
		return false;
	}
}

void SerialPort::release(void)
{
	//if(m_status == 0x03)
	if(m_status == 0x01)
	{
		//m_worksig = false;
		//pthread_join(m_thread, NULL);
		//pthread_mutex_destroy(&m_plock);

		close(m_serial_fd);
		m_status = 0;
	}
}

SerialPort::~SerialPort(void)
{
	release();
}

// 清除输入缓冲
void SerialPort::tciflush(void)
{
	tcflush(m_serial_fd,TCIFLUSH);
}

ControlArgs read_args(std::string filename)
{
  ControlArgs _controlargs;
  
  std::string linebuf;
  unsigned int flag=0;
  
  ifstream fin(filename.c_str());
  if(!fin.is_open())
  {
    std::cout<<"[Error]: Can not open ControlArgs file."<<endl;
    _controlargs.isReadSuccess=false;
    return _controlargs;
  }
  
  while(getline(fin,linebuf))
  {
    if(linebuf.empty())
      continue;
    if(sscanf(linebuf.c_str(), "nGrabPoints:%d", &_controlargs.nGrabPoints))
    {
      flag |= 0x0001;
    }
    else if(sscanf(linebuf.c_str(), "grab_xP:%lf", &_controlargs.grab_xP))
    {
      flag |= 0x0002;
    }
    else if(sscanf(linebuf.c_str(), "grab_xI:%lf", &_controlargs.grab_xI))
    {
      flag |= 0x0004;
    }
    else if(sscanf(linebuf.c_str(), "grab_xD:%lf", &_controlargs.grab_xD))
    {
      flag |= 0x0008;
    }
    else if(sscanf(linebuf.c_str(), "flight_xP:%lf", &_controlargs.flight_xP))
    {
      flag |= 0x0010;
    }
    else if(sscanf(linebuf.c_str(), "flight_xI:%lf", &_controlargs.flight_xI))
    {
      flag |= 0x0020;
    }
    else if(sscanf(linebuf.c_str(), "flight_xD:%lf", &_controlargs.flight_xD))
    {
      flag |= 0x0040;
    }
    else if(sscanf(linebuf.c_str(), "grab_yP:%lf", &_controlargs.grab_yP))
    {
      flag |= 0x0080;
    }
    else if(sscanf(linebuf.c_str(), "grab_yI:%lf", &_controlargs.grab_yI))
    {
      flag |= 0x0100;
    }
    else if(sscanf(linebuf.c_str(), "grab_yD:%lf", &_controlargs.grab_yD))
    {
      flag |= 0x0200;
    }
    else if(sscanf(linebuf.c_str(), "flight_yP:%lf", &_controlargs.flight_yP))
    {
      flag |= 0x0400;
    }
    else if(sscanf(linebuf.c_str(), "flight_yI:%lf", &_controlargs.flight_yI))
    {
      flag |= 0x0800;
    }
    else if(sscanf(linebuf.c_str(), "flight_yD:%lf", &_controlargs.flight_yD))
    {
      flag |= 0x1000;
    }
    else if(sscanf(linebuf.c_str(), "tryTimes:%d", &_controlargs.tryTimes))
    {
      flag |= 0x2000;
    }
    else if(sscanf(linebuf.c_str(), "aimedDoneNumber:%d", &_controlargs.aimedDoneNumber))
    {
      flag |= 0x4000;
    }
    else if(sscanf(linebuf.c_str(), "cam_ts:%lf", &_controlargs.cam_ts))
    {
      flag |= 0x8000;
    }
    else if(sscanf(linebuf.c_str(), "cam_tv:%lf", &_controlargs.cam_tv))
    {
      flag |= 0x10000;
    }
    else if(sscanf(linebuf.c_str(), "batteryThreshold:%lf", &_controlargs.batteryThreshold))
    {
      flag |= 0x20000;
    }
    else if(sscanf(linebuf.c_str(), "cam_switch:%d", &_controlargs.cam_switch))
    {
      flag |= 0x40000;
    }
    else if(sscanf(linebuf.c_str(), "timeThreshold:%lf", &_controlargs.timeThreshold))
    {
      flag |= 0x80000;
    }
    else if(sscanf(linebuf.c_str(), "grabHeight:%f", &_controlargs.grabHeight))
    {
      flag |= 0x100000;
    }
  }
  
  if(flag != 0x1fffff)
  {
    std::cout<<"[Error]: Too few args in ControlArgs file."<<endl;
    _controlargs.isReadSuccess=false;
    return _controlargs;
  }
  
  _controlargs.isReadSuccess=true;
  return _controlargs; 
}


