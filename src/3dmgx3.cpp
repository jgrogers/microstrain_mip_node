/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2008-20010  Willow Garage
 *                      
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 * Modified by jgrogers@gmail.com to work with the 3dmgx3 IMU
 *
 */
#include <ros/ros.h>
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <netinet/in.h>
#include <stdlib.h>

#include <sys/time.h>

//#include <ros/console.h>

#include <microstrain_mip_node/3dmgx3.h>

#include "poll.h"

#define BUFFSIZE 	4096

//! Macro for throwing an exception with a message
#define IMU_EXCEPT(except, msg, ...)					\
  {									\
    char buf[1000];							\
    snprintf(buf, 1000, msg" (in microstrain_3dmgx3_imu::IMU:%s)", ##__VA_ARGS__, __FUNCTION__); \
    throw except(buf);							\
  }

// Some systems (e.g., OS X) require explicit externing of static class
// members.
extern const double microstrain_3dmgx3_imu::IMU::G;
extern const double microstrain_3dmgx3_imu::IMU::KF_K_1;
extern const double microstrain_3dmgx3_imu::IMU::KF_K_2;

//! Code to swap bytes since IMU is big endian
static inline unsigned short bswap_16(unsigned short x) {
  return (x>>8) | (x<<8);
}

//! Code to swap bytes since IMU is big endian
static inline unsigned int bswap_32(unsigned int x) {
  return (bswap_16(x&0xffff)<<16) | (bswap_16(x>>16));
}


//! Code to extract a floating point number from the IMU
static float extract_float(uint8_t* addr) {

  float tmp;

  *((unsigned char*)(&tmp) + 3) = *(addr);
  *((unsigned char*)(&tmp) + 2) = *(addr+1);
  *((unsigned char*)(&tmp) + 1) = *(addr+2);
  *((unsigned char*)(&tmp)) = *(addr+3);

  return tmp;
}


void compute_checksum(char* mip_packet, size_t len, 
		      char& checksum_byte1, char& checksum_byte2) {
  checksum_byte1 = 0;
  checksum_byte2 = 0;
  for(unsigned int i=0; i<len; i++)
    {
      checksum_byte1 += mip_packet[i];
      checksum_byte2 += checksum_byte1;
    }

  //  checksum = ((u16) checksum_byte1 << 8) + (u16) checksum_byte2;
}

void append_checksum(char* mip_packet, size_t len) {
  //Assuming that 2 extra bytes are allocated...
  compute_checksum(mip_packet, len, mip_packet[len], mip_packet[len+1]);
}

//! Helper function to get system time in nanoseconds.
static unsigned long long time_helper()
{
#if POSIX_TIMERS > 0
  struct timespec curtime;
  clock_gettime(CLOCK_REALTIME, &curtime);
  return (unsigned long long)(curtime.tv_sec) * 1000000000 + (unsigned long long)(curtime.tv_nsec);  
#else
  struct timeval timeofday;
  gettimeofday(&timeofday,NULL);
  return (unsigned long long)(timeofday.tv_sec) * 1000000000 + (unsigned long long)(timeofday.tv_usec) * 1000;  
#endif
}


////////////////////////////////////////////////////////////////////////////////
// Constructor
microstrain_3dmgx3_imu::IMU::IMU() : fd(-1), continuous(false)
{ }


////////////////////////////////////////////////////////////////////////////////
// Destructor
microstrain_3dmgx3_imu::IMU::~IMU()
{
  closePort();
}


////////////////////////////////////////////////////////////////////////////////
// Open the IMU port
void
microstrain_3dmgx3_imu::IMU::openPort(const char *port_name)
{
  closePort(); // In case it was previously open, try to close it first.

  // Open the port
  fd = open(port_name, O_RDWR |O_NOCTTY);
  if (fd < 0)
    {
      const char *extra_msg = "";
      switch (errno)
	{
	case EACCES:
	  extra_msg = "You probably don't have premission to open the port for reading and writing.";
	  break;
	case ENOENT:
	  extra_msg = "The requested port does not exist. Is the IMU connected? Was the port name misspelled?";
	  break;
	}

      IMU_EXCEPT(microstrain_3dmgx3_imu::Exception, "Unable to open serial port [%s]. %s. %s", port_name, strerror(errno), extra_msg);
    }

  // Lock the port
  struct flock fl;
  fl.l_type   = F_WRLCK;
  fl.l_whence = SEEK_SET;
  fl.l_start = 0;
  fl.l_len   = 0;
  fl.l_pid   = getpid();

  if (fcntl(fd, F_SETLK, &fl) != 0)
    IMU_EXCEPT(microstrain_3dmgx3_imu::Exception, "Device %s is already locked. Try 'lsof | grep %s' to find other processes that currently have the port open.", port_name, port_name);

  // Change port settings
  struct termios term;
  if (tcgetattr(fd, &term) < 0)
    IMU_EXCEPT(microstrain_3dmgx3_imu::Exception, "Unable to get serial port attributes. The port you specified (%s) may not be a serial port.", port_name);

  cfmakeraw( &term );
  cfsetispeed(&term, B115200);
  cfsetospeed(&term, B115200);

  if (tcsetattr(fd, TCSAFLUSH, &term) < 0 )
    IMU_EXCEPT(microstrain_3dmgx3_imu::Exception, "Unable to set serial port attributes. The port you specified (%s) may not be a serial port.", port_name); /// @todo tcsetattr returns true if at least one attribute was set. Hence, we might not have set everything on success.
  // Stop continuous mode
  stopContinuous();

  // Make sure queues are empty before we begin
  if (tcflush(fd, TCIOFLUSH) != 0)
    IMU_EXCEPT(microstrain_3dmgx3_imu::Exception, "Tcflush failed. Please report this error if you see it.");
}


////////////////////////////////////////////////////////////////////////////////
// Close the IMU port
void
microstrain_3dmgx3_imu::IMU::closePort()
{
  if (fd != -1)
    {
      if (continuous)
	{
	  try {
	    //ROS_DEBUG("stopping continuous");
	    stopContinuous();

	  } catch (microstrain_3dmgx3_imu::Exception &e) {
	    // Exceptions here are fine since we are closing anyways
	  }
	}

      if (close(fd) != 0)
	IMU_EXCEPT(microstrain_3dmgx3_imu::Exception, "Unable to close serial port; [%s]", strerror(errno));
      fd = -1;
    }
}



////////////////////////////////////////////////////////////////////////////////
// Initialize time information
void
microstrain_3dmgx3_imu::IMU::initTime()
{
  wraps = 0;

  //Sets internal tick counter to 0
  send_command("75650C0808360101000000002E58");
  start_time = time_helper();

  offset_ticks = 0;
  last_ticks = 0;

  // reset kalman filter state
  offset = 0;
  d_offset = 0;
  sum_meas = 0;
  counter = 0;

  // fixed offset
  fixed_offset = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Initialize IMU gyros
void
microstrain_3dmgx3_imu::IMU::initGyros(double* bias_x, double* bias_y, double* bias_z)
{
  wraps = 0;
  send_command("75650C04043927105EE0");
  std::vector<MIP_Data_Response> mip_responses;
  readPort(mip_responses);
  if (mip_responses.size() != 2 ||
      !checkAckResponse(mip_responses[0], 0x39) ||
      mip_responses[1].command_desc != 0x9b) {
    ROS_ERROR("Unable to initialize gyro bias");
    exit(0);
  }
  
  if (bias_x)
    *bias_x = mip_responses[1].x;
  
  if (bias_y)
    *bias_y = mip_responses[1].y;

  if (bias_z)
    *bias_z = mip_responses[1].z;
}


////////////////////////////////////////////////////////////////////////////////
// Put the IMU into continuous mode
bool
microstrain_3dmgx3_imu::IMU::setContinuous()
{

  //Select scaled acceleration, angular rate, and timestamp AHRS msgs
  //send_command("75650C0D0D08010304000A05000A0E000A4195");
  char cmd_string[22] = {0x75, 0x65, 0x0C, 0x10, 0x10,
		      0x08, 0x01, 0x04, 0x04, 0x00,
		      0x0A, 0x05, 0x00, 0x0A, 0x0E,
		      0x00, 0x0A, 0x09, 0x00, 0x0A,
		      0xBE, 0xEF}; //Beef gets replaced by true checksum
  append_checksum(cmd_string, sizeof(cmd_string)-2);
  write(fd, cmd_string, sizeof(cmd_string));
  std::vector<MIP_Data_Response> mip_responses;
  readPort(mip_responses);
  if (mip_responses.size() != 1 ||
      !checkAckResponse(mip_responses[0], 0x08)) {
    ROS_ERROR("Failed to select AHRS message format");
    exit(0);
  }
  //Select GPS component messages
  send_command("75650C0A0A090102040004060004188E");
  readPort(mip_responses);
  if (mip_responses.size()!= 1 ||
      !checkAckResponse(mip_responses[0], 0x09)) {
    ROS_ERROR("Failed to select GPS data format");
    exit(0);
  }
  //Enable streaming
  send_command("75650C0A0511010101051101020123CA");
  readPort(mip_responses);
  if (mip_responses.size()!= 2 ||
      !checkAckResponse(mip_responses[0], 0x011),
      !checkAckResponse(mip_responses[1], 0x011)) {
    ROS_ERROR("Failed to enable streaming");
    exit(0);
  }
  continuous = true;
  return true;
}
////////////////////////////////////////////////////////////////////////////////
// Take the IMU out of continuous mode
void
microstrain_3dmgx3_imu::IMU::stopContinuous()
{

  //Disable streaming
  send_command("75650C0A0511010100051101020021C3");
  std::vector<MIP_Data_Response> mip_responses;
  readPort(mip_responses);
  if (mip_responses.size() != 2 || 
      !checkAckResponse(mip_responses[0], 0x11) ||
      !checkAckResponse(mip_responses[1], 0x11)) {
    ROS_ERROR("Failed to disable streaming");
    exit(0);
  }
  continuous = false;
}



////////////////////////////////////////////////////////////////////////////////
// Receive ACCEL_ANGRATE_MAG message
void
microstrain_3dmgx3_imu::IMU::receiveAccelAngrate(uint64_t *time, double accel[3], double angrate[3])
{

  double imu_time;
  std::vector<MIP_Data_Response> mip_responses;
  readPort(mip_responses);
  for (unsigned int i = 0;i<mip_responses.size();i++) {
    if (mip_responses[i].desc_set == 0x80) {
      //AHRS message
      if (mip_responses[i].command_desc == 0x04) {
	//accel
	accel[0] = mip_responses[i].x;
	accel[1] = mip_responses[i].y;
	accel[2] = mip_responses[i].z;
      }
      else if (mip_responses[i].command_desc == 0x05) {
	angrate[0] = mip_responses[i].x;
	angrate[1] = mip_responses[i].y;
	angrate[2] = mip_responses[i].z;
      }
      else if(mip_responses[i].command_desc == 0x0e) {
	//Time
	imu_time = mip_responses[i].ftime;
	*time = filterTime(imu_time, time_helper());
	
      }
    }
    else if (mip_responses[i].desc_set == 0x81) {
      //GPS data, ignore for now
    }
  }
}
void
microstrain_3dmgx3_imu::IMU::receiveAccelAngrateOrientation(uint64_t *time, double accel[3], double angrate[3], double orientation[9])
{
  double imu_time;
  std::vector<MIP_Data_Response> mip_responses;
  readPort(mip_responses);
  for (unsigned int i = 0;i<mip_responses.size();i++) {
    if (mip_responses[i].desc_set == 0x80) {
      //AHRS message
      if (mip_responses[i].command_desc == 0x04) {
	//accel
	accel[0] = mip_responses[i].x;
	accel[1] = mip_responses[i].y;
	accel[2] = mip_responses[i].z;
      }
      else if (mip_responses[i].command_desc == 0x05) {
	angrate[0] = mip_responses[i].x;
	angrate[1] = mip_responses[i].y;
	angrate[2] = mip_responses[i].z;
      }
      else if(mip_responses[i].command_desc == 0x0e) {
	//Time
	imu_time = mip_responses[i].ftime;
	*time = filterTime(imu_time, time_helper());
	
	
      }
      else if (mip_responses[i].command_desc == 0x09) {
	for (unsigned int j = 0;j<9;j++) {
	  orientation[j] = mip_responses[i].orientation[j];
	}
      }
    }
    else if (mip_responses[i].desc_set == 0x81) {
      //GPS data, ignore for now
    }
  }
}


////////////////////////////////////////////////////////////////////////////////
// Extract time and process rollover
uint64_t
microstrain_3dmgx3_imu::IMU::extractTime(unsigned int ticks)
{
  if (ticks < last_ticks) {
    wraps += 1;
  }

  last_ticks = ticks;

  uint64_t all_ticks = ((uint64_t)wraps << 32) - offset_ticks + ticks;

  return  start_time + ((double)all_ticks / (double)62500);

}



bool microstrain_3dmgx3_imu::IMU::
checkAckResponse(const MIP_Data_Response& resp, unsigned char cmd) {
  ROS_INFO("Resp error code %x, cmd echo %x, expecting %x",
	   resp.error_code, resp.command_echo, cmd);
  if (resp.error_code == 0x00 &&
      resp.command_echo == cmd) return true;
  else return false;
  
}
void microstrain_3dmgx3_imu::IMU::
checkDataResponse(const MIP_Data_Response& resp) {
  if (resp.desc_set == 0x80) {
    if (resp.command_desc == 0x04) 
      ROS_INFO("Got scaled acceleration (g) (%f, %f, %f)", 
	       resp.x, resp.y, resp.z);
    if (resp.command_desc == 0x05)
      ROS_INFO("Got angular rate (rad/sec) (%f, %f, %f)", 
	       resp.x, resp.y, resp.z);
    if (resp.command_desc == 0x0e) 
      ROS_INFO("Got time value %u ftime %f",
	       resp.time, resp.ftime);
  }
  else if (resp.desc_set == 0x81) {
    if (resp.command_desc == 0x04) 
      ROS_INFO("Got ECEF position (%f %f %f) acc %f, vf: %u",
	       resp.x, resp.y, resp.z, resp.position_acc, resp.flags);
    if (resp.command_desc == 0x06) 
      ROS_INFO("Got ECEF velocity (%f %f %f) acc %f, vf: %u",
	       resp.x, resp.y, resp.z, resp.position_acc, resp.flags);
  }
	   
}
////////////////////////////////////////////////////////////////////////////////
// Send a packet and wait for a reply from the IMU.
// Returns the number of bytes read.
bool microstrain_3dmgx3_imu::IMU::
send_command(const char command_string[]) {

 int number_hex_chars;
 int string_length;
 unsigned char* hexs;
 int i;
 
 //deterine length of command string
 string_length = strlen(command_string);

 //each byte is represented by 2 ascii chars
 //so the length of the string should always be even
 if(string_length%2!=0){
  ROS_INFO("Invalid Command\n");
  return false;
 }

 //each hex char is only 1 byte from the two entered
 number_hex_chars=string_length/2;

 //allocate memory for the byte-level command string
 hexs=(unsigned char*)malloc(number_hex_chars*sizeof(unsigned char));

 //process each 2-char set from an ascii representation of the hexadecimal
 //command byte to the associated byte
 for(i=0;i<number_hex_chars;i++){
   if(sscanf(&command_string[i*2],"%2x",&hexs[i])<1){
     ROS_INFO("Invalid Command\n");
     return false;
   }
 }
 //write command 
 write(fd, hexs, number_hex_chars); 

 //deallocate the memory for the hex bytes
 free(hexs); 

 return true;
}

enum MIP_DATA_PARSE_T { FieldLength, CmdDesc, FieldData};

//attempts to read one packet from the open port
int microstrain_3dmgx3_imu::IMU::
readPort(std::vector<MIP_Data_Response>& resp){
  resp.clear();
 int size;
 //struct termios initial_settings, new_settings;
 unsigned char response[BUFFSIZE] = {0};

 //get current terminal settings 
 // tcgetattr(0,&initial_settings);

 // new_settings = initial_settings;
 // new_settings.c_lflag &= ~ICANON;
 // new_settings.c_cc[VMIN] = 0;
 // new_settings.c_cc[VTIME] = 0;

 // tcsetattr(0, TCSANOW, &new_settings);

 //determine if there are bytes to read
 size = 1;
 while (response[0] != 0x75 && size) {
   size = read(fd, &response[0], 1);//Read sync1
 } 
 //Synced
 size = read(fd, &response[1], 3);; //Read the rest of the header
 if (response[0] != 0x75 || 
     response[1] != 0x65) {
   ROS_ERROR("Error in sync");
   return 0;
 }
 unsigned char desc_set = response[2];
 unsigned char payload_length = response[3];
 size = read(fd, &response[4], payload_length+2);
 if (size != payload_length+2) {
   ROS_ERROR("Could not read enough");
   return 0;
 }
 //Parse data fields
 MIP_DATA_PARSE_T data_parse_t = FieldLength;
 unsigned char field_length;
 unsigned char cmd_desc;
 for(unsigned int byte_ind = 0; byte_ind < payload_length;) {
   switch(data_parse_t) {
   case FieldLength:
     field_length = response[4+byte_ind++];
     data_parse_t = CmdDesc;
     break;
   case CmdDesc:
     cmd_desc = response[4+byte_ind++];
     data_parse_t = FieldData;
     break;
   case FieldData:
     MIP_Data_Response rsp_one;
     rsp_one.command_desc = cmd_desc;
     rsp_one.desc_set = desc_set;
     if (cmd_desc == 0xf1){ //Ack type
       rsp_one.command_echo = response[4+byte_ind];
       rsp_one.error_code = response[4+byte_ind+1];
     }
     else if (desc_set == 0x80) {
       if (cmd_desc >= 0x01 && cmd_desc <= 0x08) {
	 float x,y,z;
	 unsigned char* xp = (unsigned char*) &x;
	 unsigned char* yp = (unsigned char*) &y;
	 unsigned char* zp = (unsigned char*) &z;
	 for (int i = 0;i<4;i++) {
	   xp[3-i] = response[4+byte_ind+i];
	   yp[3-i] = response[4+byte_ind+i+4];
	   zp[3-i] = response[4+byte_ind+i+8];
	 }
	 rsp_one.x = x;
	 rsp_one.y = y;
	 rsp_one.z = z;
       }
       else if (cmd_desc == 0x0e) {
	 unsigned int time;
	 unsigned char* tp = (unsigned char*) &time;
	 for (int i = 0;i<4;i++) {
	   tp[3-i] = response[4+byte_ind+i];
	 }
	 rsp_one.time = time;
	 rsp_one.ftime = (double)time / (double)(62500);
       }else if (cmd_desc == 0x09) {
	 //Orientation
	 for (unsigned int j = 0;j<9;j++) {
	   unsigned char* rp = (unsigned char*) &rsp_one.orientation[j];
	   for (unsigned int k = 0;k<4;k++) {
	     rp[3-k] = response[4+byte_ind+k+j*4];
	   }
	 }
       }
     }
     else if (desc_set == 0x81) {
       if (cmd_desc == 0x04) {
	 double x,y,z;
	 float acc; 
	 unsigned short valid;
	 unsigned char* xp = (unsigned char*) &x;
	 unsigned char* yp = (unsigned char*) &y;
	 unsigned char* zp = (unsigned char*) &z;
	 unsigned char* accp = (unsigned char*) &acc;
	 unsigned char* vp = (unsigned char*) &valid;
	 for (int i = 0;i<8;i++) {
	   xp[7-i] = response[4+byte_ind+i];
	   yp[7-i] = response[4+byte_ind+i+8];
	   zp[7-i] = response[4+byte_ind+i+16];
	 }
	 for (int i = 0;i<4;i++) {
	   accp[3-i] = response[4+byte_ind+i+24];
	 }
	 for (int i = 0;i<2;i++) {
	   vp[1-i] = response[4+byte_ind+i+28];
	 }
	 rsp_one.x = x;
	 rsp_one.y = y;
	 rsp_one.z = z;
	 rsp_one.position_acc = acc;
	 rsp_one.flags = valid;
       }
       if (cmd_desc == 0x06) {
	 //ECEF velocity
	 float x,y,z;
	 float acc; 
	 unsigned short valid;
	 unsigned char* xp = (unsigned char*) &x;
	 unsigned char* yp = (unsigned char*) &y;
	 unsigned char* zp = (unsigned char*) &z;
	 unsigned char* accp = (unsigned char*) &acc;
	 unsigned char* vp = (unsigned char*) &valid;
	 for (int i = 0;i<4;i++) {
	   xp[3-i] = response[4+byte_ind+i];
	   yp[3-i] = response[4+byte_ind+i+4];
	   zp[3-i] = response[4+byte_ind+i+8];
	 }
	 for (int i = 0;i<4;i++) {
	   accp[3-i] = response[4+byte_ind+i+12];
	 }
	 for (int i = 0;i<2;i++) {
	   vp[1-i] = response[4+byte_ind+i+16];
	 }
	 rsp_one.x = x;
	 rsp_one.y = y;
	 rsp_one.z = z;
	 rsp_one.position_acc = acc;
	 rsp_one.flags = valid;
       }
     }
     else if (desc_set == 0x0c) {
       if (cmd_desc == 0x9b) {
	 //Bias record, 3 floats
	 float x,y,z;
	 unsigned char* xp = (unsigned char*) &x;
	 unsigned char* yp = (unsigned char*) &y;
	 unsigned char* zp = (unsigned char*) &z;
	 for (int i = 0;i<4;i++) {
	   xp[3-i] = response[4+byte_ind+i];
	   yp[3-i] = response[4+byte_ind+i+4];
	   zp[3-i] = response[4+byte_ind+i+8];
	 }
	 rsp_one.x = x;
	 rsp_one.y = y;
	 rsp_one.z = z;
       }
     }
     else {
       ROS_ERROR("Unhandled message");
       exit(0);
     }
     resp.push_back(rsp_one);
     byte_ind += field_length - 2;
     data_parse_t = FieldLength;
     break;
   }
 }
 
 // tcsetattr(0, TCSANOW, &initial_settings);

 return 1;

}
////////////////////////////////////////////////////////////////////////////////
// Kalman filter for time estimation
double microstrain_3dmgx3_imu::IMU::filterTime(double imu_time, double sys_time)
{
  // first calculate the sum of KF_NUM_SUM measurements
  if (counter < KF_NUM_SUM){
    counter ++;
    sum_meas += imu_time - sys_time;
  }
  // update kalman filter with fixed innovation
  else{
    // system update
    offset += d_offset;

    // measurement update
    double meas_diff = (sum_meas/KF_NUM_SUM) - offset;
    offset   += KF_K_1 * meas_diff;
    d_offset += KF_K_2 * meas_diff;

    // reset counter and average
    counter = 0; sum_meas = 0;
  }
  return imu_time - offset + fixed_offset;
}


