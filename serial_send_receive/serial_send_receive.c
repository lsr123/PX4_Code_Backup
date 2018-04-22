/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

/*#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>

__EXPORT int serial_send_receive_main(int argc, char *argv[]);

int serial_send_receive_main(int argc, char *argv[])
{


	PX4_INFO("Hello Sky!");

	

	PX4_INFO("exiting");

	return 0;
}*/


/* limit the update rate to 5 Hz */
	//orb_set_interval(sensor_sub_fd, 200);
//orb_copy  orb_publish  orb_subscribe
//PX4_INFO
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>
#include <drivers/drv_hrt.h>
#include <fcntl.h>

#include "rw_uart_sonar_topic.h"

#include <px4_config.h>
#include <px4_tasks.h>


#include <float.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <drivers/drv_hrt.h>
#include <limits.h>
#include <math.h>
#include <uORB/uORB.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <poll.h>



#include <time.h>
#include <sys/ioctl.h>

#include <drivers/drv_hrt.h>
#include <arch/board/board.h>

#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/wind_estimate.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_global_position.h>
#include <systemlib/param/param.h>

#include <systemlib/mavlink_log.h>
#include <lib/ecl/geo/geo.h>
#include <dataman/dataman.h>
#include <mathlib/mathlib.h>

#include <string.h>

/* 定义主题 */
//ORB_DEFINE(rw_uart_sonar, struct rw_uart_sonar_data_s);

static bool thread_should_exit = false;
static bool thread_running = false;
static int daemon_task;


//__EXPORT int rw_uart_main(int argc, char *argv[]);
__EXPORT int serial_send_receive_main(int argc, char *argv[]);
int rw_uart_thread_main(int argc, char *argv[]);

static int uart_init(const char * uart_name);
static int set_uart_baudrate(const int fd, unsigned int baud);
static void usage(const char *reason);



int set_uart_baudrate(const int fd, unsigned int baud)
{
    int speed;

    switch (baud) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        case 921600: speed = B921600; break;
        default:
           	warnx("ERR: baudrate: %d\n", baud);
            return -EINVAL;
    }

    struct termios uart_config;

    int termios_state;

    /* fill the struct for the new configuration */
    tcgetattr(fd, &uart_config);
    /* clear ONLCR flag (which appends a CR for every LF) */
    uart_config.c_oflag &= ~ONLCR;
    /* no parity, one stop bit */
    uart_config.c_cflag &= ~(CSTOPB | PARENB);
    /* set baud rate */
    if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
        warnx("ERR: %d (cfsetispeed)\n", termios_state);
        return false;
    }

    if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
        warnx("ERR: %d (cfsetospeed)\n", termios_state);
        return false;
    }

    if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
        warnx("ERR: %d (tcsetattr)\n", termios_state);
        return false;
    }

    return true;
}


int uart_init(const char * uart_name)
{
    int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);

    if (serial_fd < 0) {
        err(1, "failed to open port: %s", uart_name);
        return false;
    }
    return serial_fd;
}

static void usage(const char *reason)
{
    if (reason) {
        fprintf(stderr, "%s\n", reason);
    }

    fprintf(stderr, "usage: position_estimator_inav {start|stop|status} [param]\n\n");
    exit(1);
}

//int rw_uart_main(int argc, char *argv[])
int serial_send_receive_main(int argc, char *argv[])
{
    if (argc < 2) {
        usage("[YCM]missing command");
    }

    if (!strcmp(argv[1], "start")) {
        if (thread_running) {
            warnx("[YCM]already running\n");
            exit(0);
        }

        thread_should_exit = false;
        daemon_task = px4_task_spawn_cmd("rw_uart",
                         SCHED_DEFAULT,
                         SCHED_PRIORITY_MAX - 5,
                         2000,
                         rw_uart_thread_main,
                         (argv) ? (char * const *)&argv[2] : (char * const *)NULL);
        exit(0);
    }

    if (!strcmp(argv[1], "stop")) {
        thread_should_exit = true;
        exit(0);
    }

    if (!strcmp(argv[1], "status")) {
        if (thread_running) {
            warnx("[YCM]running");

        } else {
            warnx("[YCM]stopped");
        }

        exit(0);
    }

    usage("unrecognized command");
    exit(1);
}

int rw_uart_thread_main(int argc, char *argv[])
{

    if (argc < 2) {
        errx(1, "[YCM]need a serial port name as argument");
        usage("eg:");
    }

    const char *uart_name = argv[1];    //rw_uart start /dev/ttyS6  启动命令

    warnx("[YCM]opening port %s", uart_name);
    
    //char buffer[5] = "";
    /*
     * TELEM1 : /dev/ttyS1
     * TELEM2 : /dev/ttyS2
     * GPS    : /dev/ttyS3
     * NSH    : /dev/ttyS5
     * SERIAL4: /dev/ttyS6
     * N/A    : /dev/ttyS4
     * IO DEBUG (RX only):/dev/ttyS0
     */

    char data = '0';
    int16_t read_data[3];
    //int16_t read_head = 0;
    memset(read_data, 0, sizeof(read_data));

    int uart_read = uart_init(uart_name);
    if(false == uart_read)return -1;
    if(false == set_uart_baudrate(uart_read,9600)){
        printf("[YCM]set_uart_baudrate is failed\n");
        return -1;
    }
    printf("[YCM]uart init is successful\n");

    thread_running = true;

    /*初始化数据结构体 */
    struct rw_uart_sonar_data_s sonardata;
    memset(&sonardata, 0, sizeof(sonardata));
    /* 公告主题 */
    //orb_advert_t rw_uart_sonar_pub = orb_advertise(ORB_ID(rw_uart_sonar), &sonardata);

    int count = 0;
    int16_t output_data[3];   // int默认是32位 4个字节
    output_data[0] = 32765;
    output_data[1] = -3;
    output_data[2] = -32768;

    //const char *write_data ="ABCDEFGHIJKLMN";
    //char *write_data_pointer = &write_data;
    /*
    Matlab和代码中发的大小端是不一样的。
    代码发，simulink接，直接按int16发送接收。
    simulink发，代码收。发送的是低位前高位后，代码接收后做相应的转换。
    */
    uint8_t read_temp;
    uint8_t buffer[8];
    //int8_t Lbuff = 0;
    int i = 0;
    int16_t a = 0;
    int16_t b = 0;
    int16_t c = 0;
    while(!thread_should_exit){
        //read(uart_read, &read_head,2);    //read the first int16  two bits
        //if(read_head == 32767)
        //{
            //read(uart_read, read_data, sizeof(read_data));
            //read(uart_read, read_data, 1);
        

        count ++;
        //printf("%d\n",count);

            read(uart_read, &read_temp, 1);    //   
            //printf("%d\n", read_temp);
            if(read_temp == 255)
            {
                //printf("ssssss\n");

                for(i = 0;i<7; i++)
                {
                    read(uart_read, &read_temp, 1);
                    buffer[i] = read_temp;
                    read_temp = 0;
                    //printf("%x\n", buffer[i]);
                }
                /*Lbuff = buffer[1] & 0x00ff;
                printf("%d\n", Lbuff);*/
                //printf("sdfasdfasdf\n");
                a = (buffer[2]<<8)|buffer[1];
                b = (buffer[4]<<8)|buffer[3];
                c = (buffer[6]<<8)|buffer[5];
                printf("%d %d %d\n", a, b, c);

               /* read(uart_read, &read_temp,1);
                buffer[0] = read_temp;
                printf("%d\n", buffer[0]);

                read(uart_read, &read_temp,1);
                buffer[1] = read_temp;
                printf("%d\n", buffer[1]);

                read(uart_read, &read_temp,1);
                buffer[2] = read_temp;
                printf("%d\n", buffer[2]);*/

            }
        //}
            /*if(read_temp == 255)
            {
                printf("s");
                for(int i = 0;i < 5; i++)
                {
                    read(uart_read, &read_temp, 1);
                    buffer[i] = read_temp;
                    read_temp = 0;
                }
                printf("%d\n", buffer[0]);
                printf("%d\n", buffer[1]);
                printf("%d\n", buffer[2]);
                printf("%d\n", buffer[3]);
                printf("%d\n", buffer[4]);
            }*/
           

        //memcpy(output_data, read_data, sizeof(output_data));   //read_data to output_data
    	write(uart_read, output_data, sizeof(output_data));  //write a string through serial nuttx ttyS6, baudrate 115200 
        //output_data[1] ++;
        //printf("%d %d %d\n", (int)output_data[0], (int)output_data[1], (int)output_data[2]);


        //printf("%d\n", read_temp);        
        //usleep(10000);  //1000000us = 1s          // receive data rare is 100Hz
        
        if(data == 'R'){
            /*for(int i = 0;i <4;++i){
                read(uart_read,&data,1);
                buffer[i] = data;
                data = '0';
                
            }
            strncpy(sonardata.datastr,buffer,4);
            sonardata.data = atoi(sonardata.datastr);*/
           
            //printf("%d\n", count);
            
            count++;
            //printf("[YCM]sonar.data=%d\n",sonardata.data);
            //orb_publish(ORB_ID(rw_uart_sonar), rw_uart_sonar_pub, &sonardata);
        }

        //PX4_INFO("cycling!\n");
        //sleep(1000);
    }

    warnx("[YCM]exiting");
    thread_running = false;
    close(uart_read);

    fflush(stdout);
    return 0;
}