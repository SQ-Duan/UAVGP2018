#include <stdint.h>

typedef  float float32_t;
typedef  double float64_t;


// 发送数据队列格式
typedef struct  __attribute__((packed)) 
{
	uint8_t index;
	uint8_t enable;  		// 是否处于激活状态  0：失效状态  1：使能状态
	uint16_t n_1ms;		    // 该队列 运行时间，定义运行时间变量，最小分辨率为1ms
	uint16_t period_ms;		// 该队列 连续发送周期，指所要求的定时发送周期
	uint16_t regaddr;		// 寄存器变量起始地址
	uint16_t objAddr;		// 寄存器变量起始地址  
	uint8_t  length;			// 数据长度
	uint8_t  header;          //(方)添加，用于判断将要打包发送的数据属于哪个数据文件中

} sendbufQ;