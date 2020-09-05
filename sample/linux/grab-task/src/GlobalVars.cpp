#include "myutils.h"
#include "GlobalVars.h"

const ControlArgs CONTROLARGS=read_args("args.txt");
SerialPort grabserial("/dev/ttyTHS0", 9600);
