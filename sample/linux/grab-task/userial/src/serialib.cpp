#include "serialib.h"



int serial_open(serial **s, const char* port, const unsigned int baud)
{
	*s = (serial *) malloc(sizeof(serial));

	(*s)->port = (char *) malloc((strlen(port) + 1) * sizeof(char));
	strcpy((*s)->port, port);

	
	struct termios options;
	
	(*s)->fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
	
	if ((*s)->fd == -1)
		return -1;
	
	fcntl((*s)->fd, F_SETFL, FNDELAY);	
	
	tcgetattr((*s)->fd, &options);
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
	tcsetattr((*s)->fd, TCSANOW, &options);
	return 0;
}

//success is 0,fail is -1
int serial_read_char(serial *s, char *p)
{
	return (read(s->fd, p, 1) == 1) ? 0 : -1; 
}

int serial_read(serial *s, char *buf, char eol, unsigned int len)
{
	int i = 0;
	while (i < len-1) {
		int j = serial_read_char(s, &buf[i++]);
		
		if (j != 0){
			buf[i-1] = '\0';
			return i-1;
			// return -1;
		}

		if (buf[i-1] == eol){
			buf[i] = '\0';
			return i;
			// return 1;
		}
	}

	buf[i] = '\0';
	return i;
	// return 1;
}

int serial_write_char(serial *s, char p)
{
	return (write(s->fd, &p, 1) == 1) ? 0: -1;
}

int serial_writesb(serial *s, char* str,uint16_t len){

	if (write(s->fd, str, len) != len)
		return -1;
	return 0;

}


int serial_write(serial *s, char* str)
{
	int len = strlen(str);
	if (write(s->fd, str, len) != len)
		return -1;
	return 0;
}



