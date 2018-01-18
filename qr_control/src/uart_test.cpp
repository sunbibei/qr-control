#include <stdio.h>
#include <string.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h> /* File control definitions */
#include <errno.h>
#include <termios.h> /* POSIX terminal control definitions */
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <sstream>
#include <stdlib.h>     /*标准函数库定义*/
#include <sys/types.h>  
#include <sys/stat.h>  
#include <pthread.h>
#include "qr_control/JY901.h"

#define FALSE  -1
#define TRUE   0

int fd; /* File descriptor for the port */
int READ_BUFFER_SIZE =2000;

int speed_arr[] = { B38400, B19200, B9600, B4800, B2400, B1200, B300, B38400, B19200, B9600, B4800, B2400, B1200, B300};
int name_arr[] = {38400,  19200,  9600,  4800,  2400,  1200,  300, 38400, 19200,  9600, 4800, 2400, 1200,  300};

void set_speed(int fd, int speed)
{
  int   i; 
  int   status; 
  struct termios   Opt;
  tcgetattr(fd, &Opt); 
  for(i= 0;  i < sizeof(speed_arr) / sizeof(int); i++)
  { 
    if(speed == name_arr[i]) 
    {     
      tcflush(fd, TCIOFLUSH);     
      cfsetispeed(&Opt, speed_arr[i]);  
      cfsetospeed(&Opt, speed_arr[i]);   
      status = tcsetattr(fd, TCSANOW, &Opt);  

      if(status != 0) 
      {        
        perror("tcsetattr fd1");  
        return;     
      }    
      tcflush(fd,TCIOFLUSH);   
    }  
  }
}

int open_port(void)
{
  fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd == -1)
  {	
    perror("open_port: Unable to open /dev/ttyS0 -");
  }
  else
  {
    fcntl(fd, F_SETFL, 0);
    return (fd);
  }
}

int read_data(const int fd, char *read_buffer)
{
  return read(fd, read_buffer, READ_BUFFER_SIZE);
}

int set_Parity(int fd,int databits,int stopbits,int parity)
{ 
  struct termios options; 
  if(tcgetattr(fd,&options) != 0) 
  { 
    perror("SetupSerial 1");     
    return(FALSE);  
  }
  options.c_cflag &= ~CSIZE; 
  switch(databits) /*设置数据位数*/
  {   
   case 7:  
   options.c_cflag |= CS7; 
   break;
   case 8:     
   options.c_cflag |= CS8;
   break;   
   default:    
   fprintf(stderr,"Unsupported data sizen"); 
   return (FALSE);  
 }
 switch(parity) 
 {   
   case 'n':
   case 'N':    
  options.c_cflag &= ~PARENB;   /* Clear parity enable */
  options.c_iflag &= ~INPCK;     /* Enable parity checking */ 
   break;  
   case 'o':   
   case 'O':     
  options.c_cflag |= (PARODD | PARENB); /* 设置为奇效验*/  
  options.c_iflag |= INPCK;             /* Disnable parity checking */ 
   break;  
   case 'e':  
   case 'E':   
  options.c_cflag |= PARENB;     /* Enable parity */    
  options.c_cflag &= ~PARODD;   /* 转换为偶效验*/     
  options.c_iflag |= INPCK;       /* Disnable parity checking */
   break;
   case 'S': 
 case 's':  /*as no parity*/   
   options.c_cflag &= ~PARENB;
   options.c_cflag &= ~CSTOPB;break;  
   default:   
   fprintf(stderr,"Unsupported parityn");    
   return (FALSE);  
 }  
  /* 设置停止位*/  
 switch(stopbits)
 {   
   case 1:    
   options.c_cflag &= ~CSTOPB;  
   break;  
   case 2:    
   options.c_cflag |= CSTOPB;  
   break;
   default:    
   fprintf(stderr,"Unsupported stop bitsn");  
   return (FALSE); 
 } 
  /* Set input parity option */ 
 if(parity != 'n')   
 {
   options.c_iflag |= INPCK; 
 }

 tcflush(fd,TCIFLUSH);
 options.c_cc[VTIME] = 13; /* 设置超时15 seconds*/   
 options.c_cc[VMIN] = 0; /* Update the options and do it NOW */

 if(tcsetattr(fd,TCSANOW,&options) != 0)   
 { 
   perror("SetupSerial 3");   
   return (FALSE);  
 } 
 return (TRUE);  
}

bool g_run = true;
void *thread(void *ptr)
{
  while(getchar())
  {
    g_run = false;
    break;
  }
  return 0;
}



int main(int argc, char **argv)
{
  ros::init(argc,argv,"uart_test");
  ros::NodeHandle n;

  std_msgs::Float64MultiArray msg;
   // ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("/gesture/IMU", 1);
  boost::scoped_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>> joint_state_publisher_;

  joint_state_publisher_.reset( new realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>(n, "/gesture/IMU", 1000));
   // ros::Rate loop_rate(100);

  char chrBuffer[READ_BUFFER_SIZE];
  unsigned short usLength=0,usCnt=0;
  unsigned long ulBaund=115200;

  fd = open_port();
  set_speed(fd,ulBaund);

  if(set_Parity(fd,8,1,'N') == FALSE)  
  {
    printf("Set Parity Errorn");
    exit (0);
  }

  pthread_t id;
  int ret = pthread_create(&id, NULL, thread, NULL);
  if(ret) 
  {
    std::cout << "Create pthread error!" << std::endl;
    return 1;
  }

  while(ros::ok())
  {
    usLength = read_data(fd, chrBuffer);
    // std::cout<<"usLength:"<<usLength<<std::endl;

    if(usLength>0)
    {
      JY901.CopeSerialData(chrBuffer,usLength);
    }

    if(usCnt++>=0)
    {
      usCnt=0;
      // printf("Time:20%d-%d-%d %d:%d:%.3f\r\n",(short)JY901.stcTime.ucYear,(short)JY901.stcTime.ucMonth,
      //     (short)JY901.stcTime.ucDay,(short)JY901.stcTime.ucHour,(short)JY901.stcTime.ucMinute,(float)JY901.stcTime.ucSecond+(float)JY901.stcTime.usMiliSecond/1000);

      // printf("Acc:%.3f %.3f %.3f\r\n",(float)JY901.stcAcc.a[0]/32768*16,(float)JY901.stcAcc.a[1]/32768*16,(float)JY901.stcAcc.a[2]/32768*16);

      // printf("Gyro:%.3f %.3f %.3f\r\n",(float)JY901.stcGyro.w[0]/32768*2000,(float)JY901.stcGyro.w[1]/32768*2000,(float)JY901.stcGyro.w[2]/32768*2000);

      // printf("Angle:%.3f %.3f %.3f\r\n",(float)JY901.stcAngle.Angle[0]/32768*180,(float)JY901.stcAngle.Angle[1]/32768*180,(float)JY901.stcAngle.Angle[2]/32768*180);

      // printf("Mag:%d %d %d\r\n",JY901.stcMag.h[0],JY901.stcMag.h[1],JY901.stcMag.h[2]);

      // printf("Pressure:%lx Height%.2f\r\n",JY901.stcPress.lPressure,(float)JY901.stcPress.lAltitude/100);

      // printf("DStatus:%d %d %d %d\r\n",JY901.stcDStatus.sDStatus[0],JY901.stcDStatus.sDStatus[1],JY901.stcDStatus.sDStatus[2],JY901.stcDStatus.sDStatus[3]);

      // printf("Longitude:%ldDeg%.5fm Lattitude:%ldDeg%.5fm\r\n",JY901.stcLonLat.lLon/10000000,(double)(JY901.stcLonLat.lLon % 10000000)/1e5,JY901.stcLonLat.lLat/10000000,(double)(JY901.stcLonLat.lLat % 10000000)/1e5);

      // printf("GPSHeight:%.1fm GPSYaw:%.1fDeg GPSV:%.3fkm/h\r\n\r\n",(float)JY901.stcGPSV.sGPSHeight/10,(float)JY901.stcGPSV.sGPSYaw/10,(float)JY901.stcGPSV.lGPSVelocity/1000);
    } 



    if(joint_state_publisher_ && joint_state_publisher_->trylock())
    {
      joint_state_publisher_->msg_.data.clear();
      for(int i=0; i<3; i++)
      {
       joint_state_publisher_->msg_.data.push_back((float)JY901.stcAngle.Angle[i]/32768*180);
      }
      joint_state_publisher_->unlockAndPublish();
    }
  }

close(fd);
}