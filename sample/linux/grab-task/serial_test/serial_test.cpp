#include <myutils.h>

int main(int argc,char** argv)
{
	SerialPort xbee("/dev/ttyUSB0",115200);
	SerialPort grab("/dev/ttyTHS0", 9600);
	if(!xbee.isOpened() || !grab.isOpened())
		return -1;
	char ch1, ch2;
	while(1)
	{
		ch1=xbee.read_ack();
		ch2=grab.read_ack();
		if(ch1 != 0)
		{
			if(ch1 == 'a')
				break;
			else
			{
				grab.write_cmd_no_ack(ch1);
			}
		}
		if(ch2 != 0)
		{
				xbee.write_cmd_no_ack(ch2);
		}
		usleep(500);
	}
}


