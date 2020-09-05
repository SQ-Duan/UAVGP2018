#include "protocol.h"
#include "gcs_thread.h"


#include <pthread.h>
#include <stdio.h>

#include <string.h>
#include <unistd.h>
#include <stdint.h>

#include <signal.h>
#include <sys/time.h>


void init_time(void) ;
void init_sigaction(void);
int init_serial(void);

uint32_t number = 0;
//pthread_mutex_t mut;

//uint32_t limit = 0;

int rec_len = 0;

serial *s;

uint8_t buffer[128];

void timeout_info(int signo) {

    int kc =0;
    
    rec_len = serial_read(s, (char*)::buffer, '\n', 50);

    if(rec_len>1)
    {
        printf("rec len = %d\n",rec_len);
        for(kc = 0;kc<rec_len;kc++)
        {
           printf("%x    ",::buffer[kc]);
        }

         printf("\n");

        received_task((char*)::buffer,rec_len);
    }
        
    CommProtocol_task();
    //printf("time is %10.3f \n",(limit++)*0.1);
}

/* init sigaction */
void init_sigaction(void) {
	struct sigaction act;

	act.sa_handler = timeout_info;
	act.sa_flags = 0;
	sigemptyset(&act.sa_mask);
	sigaction(SIGPROF, &act, NULL);
}

/* init */
void init_time(void) {
	struct itimerval val;

	val.it_value.tv_sec = 0;
	val.it_value.tv_usec = 50000;
	val.it_interval = val.it_value;
	setitimer(ITIMER_PROF, &val, NULL);
}

int init_serial(void){

    if (serial_open(&s, "/dev/ttyUSB0", 115200) == 0){
		printf("Port opened.\n");

	} else {
		printf("Problem with port opening\n");
		return -1;
	}
	printf("%s -> %d\n", s->port, s->fd);
    return 0;
}

int gcs_interface_init(void){
    
    if(init_serial()==-1)
        return -1;
    init_sigaction();
	init_time();

    CommProtocol_init();

    return 0;
}


serial * get_local_port(void){
    return s;
}