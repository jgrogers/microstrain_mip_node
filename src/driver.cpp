
#include <termios.h> // terminal io (serial port) interface
#include <fcntl.h>  // File control definitions
#include <errno.h>  // Error number definitions
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <stdlib.h>
#include <unistd.h>


#include <ros/ros.h>


#define BUFFSIZE 	4096

typedef int ComPortHandle;
typedef unsigned char Byte;

// Utility functions for working with a com port in Linux

// Purge
// Clears the com port's read and write buffers

bool Purge(ComPortHandle comPortHandle){

  if (tcflush(comPortHandle,TCIOFLUSH)==-1){

  ROS_INFO("flush failed\n");
  return false;

 }

 return true;

}

// OpenComPort
// Opens a com port with the correct settings for communicating with a 
// MicroStrain 3DM-GX3-25 sensor

ComPortHandle OpenComPort(std::string& comPortPath){

  ComPortHandle comPort = open(comPortPath.c_str(), O_RDWR | O_NOCTTY);
  
  if (comPort== -1){ //Opening of port failed
    
    ROS_INFO("Unable to open com Port %s\n Errno = %i\n", comPortPath.c_str(), errno);
    return -1;
    
  }
 
 //Get the current options for the port...
 struct termios options;
 tcgetattr(comPort, &options);

 //set the baud rate to 115200
 int baudRate = B115200;
 cfsetospeed(&options, baudRate);
 cfsetispeed(&options, baudRate);

 //set the number of data bits.
 options.c_cflag &= ~CSIZE; // Mask the character size bits
 options.c_cflag |= CS8;

 //set the number of stop bits to 1
 options.c_cflag &= ~CSTOPB;

 //Set parity to None
 options.c_cflag &=~PARENB;

 //set for non-canonical (raw processing, no echo, etc.)
 options.c_iflag = IGNPAR; // ignore parity check close_port(int
 options.c_oflag = 0; // raw output
 options.c_lflag = 0; // raw input

 //Time-Outs -- won't work with NDELAY option in the call to open
 options.c_cc[VMIN] = 0;  // block reading until RX x characers. If x = 0, 
               // it is non-blocking.
 options.c_cc[VTIME] = 1;  // Inter-Character Timer -- i.e. timeout= x*.1 s

 //Set local mode and enable the receiver
 options.c_cflag |= (CLOCAL | CREAD);

 //Purge serial port buffers
 Purge(comPort);

 //Set the new options for the port...
 int status=tcsetattr(comPort, TCSANOW, &options);

 if (status != 0){ //For error message

  ROS_INFO("Configuring comport failed\n");
  return status;

 }

 //Purge serial port buffers
 Purge(comPort);

 return comPort;

}

// CloseComPort
// Closes a port that was previously opened with OpenComPort
void CloseComPort(ComPortHandle comPort){

 close(comPort);

}

// readComPort
// read the specivied number of bytes from the com port
int readComPort(ComPortHandle comPort, Byte* bytes, int bytesToRead){

 int bytesRead = read(comPort, bytes, bytesToRead);
 return bytesRead;  

}

// writeComPort
// send bytes to the com port
int writeComPort(ComPortHandle comPort, unsigned char* bytesToWrite, int size){

 return write(comPort, bytesToWrite, size);

}

// Simple Linux Console interface function
struct MIP_Data_Response {
  unsigned char desc_set;
  unsigned char command_desc;
  unsigned char command_echo;
  unsigned char error_code;
  double x,  y, z;
  float position_acc;
  unsigned short flags;
  unsigned int time;
  double ftime;
};

bool checkAckResponse(const MIP_Data_Response& resp, unsigned char cmd) {
  ROS_INFO("Resp error code %x, cmd echo %x, expecting %x",
	 resp.error_code, resp.command_echo, cmd);
  if (resp.error_code == 0x00 &&
      resp.command_echo == cmd) return true;
  else return false;
  
}
void checkDataResponse(const MIP_Data_Response& resp) {
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
enum MIP_DATA_PARSE_T { FieldLength, CmdDesc, FieldData};
//attempts to read one packet from the open port
int readPort(ComPortHandle comPort, std::vector<MIP_Data_Response>& resp){
  resp.clear();
 int size;
 struct termios initial_settings, new_settings;
 Byte response[BUFFSIZE] = {0};

 //get current terminal settings 
 tcgetattr(0,&initial_settings);

 new_settings = initial_settings;
 new_settings.c_lflag &= ~ICANON;
 new_settings.c_cc[VMIN] = 0;
 new_settings.c_cc[VTIME] = 0;

 tcsetattr(0, TCSANOW, &new_settings);

 //determine if there are bytes to read
 size = 1;
 while (response[0] != 0x75 && size) {
   size = readComPort(comPort, &response[0], 1);//Read sync1
 } 
 //Synced
 size = readComPort(comPort, &response[1], 3);; //Read the rest of the header
 if (response[0] != 0x75 || 
     response[1] != 0x65) {
   ROS_ERROR("Error in sync");
   return 0;
 }
 unsigned char desc_set = response[2];
 unsigned char payload_length = response[3];
 size = readComPort(comPort, &response[4], payload_length+2);
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
 
 tcsetattr(0, TCSANOW, &initial_settings);

 return 1;

}

//scandev
//finds attached microstrain devices and prompts user for choice then returns 
//selected portname
std::string scandev(){
 
 FILE *instream;
 char devnames[255][255];//allows for up to 256 devices with path links up to 
             //255 characters long each
 int devct=0; //counter for number of devices
 int i=0;
 int j=0;
 int userchoice=0;
 char *device;

 //search /dev/serial for microstrain devices
 const char *command = "find /dev/serial -print | grep -i microstrain";
 
 ROS_INFO("Searching for devices...\n");

 instream=popen(command, "r");//execute piped command in read mode

 if(!instream){//SOMETHING WRONG WITH THE SYSTEM COMMAND PIPE...EXITING
  ROS_INFO("ERROR BROKEN PIPELINE %s\n", command);
  return std::string("");
 }

 //load char array of device addresses
 for(i=0;i<255&&(fgets(devnames[i],sizeof(devnames[i]), instream));i++){
  ++devct;
 }

 for(i=0;i<devct;i++){
  for(j=0;j<sizeof(devnames[i]);j++){
   if(devnames[i][j]=='\n'){
    devnames[i][j]='\0';//replaces newline inserted by pipe reader with 
              //char array terminator character 
    break;//breaks loop after replacement
   }
  }
  ROS_INFO("Device Found:\n%d: %s\n",i,devnames[i]);
 }

 //CHOOSE DEVICE TO CONNECT TO AND CONNECT TO IT (IF THERE ARE ANY CONNECTED)

 if(devct>0){
  ROS_INFO("Number of devices = %d\n", devct);
  if(devct>1){
   ROS_INFO(
      "Please choose the number of the device to connect to (0 to %i):\n",
                                   devct-1);
    //check that there's input and in the correct range
    while(scanf("%i",&userchoice)==0||userchoice<0||userchoice>devct-1){
     ROS_INFO("Invalid choice...Please choose again between 0 and %d:\n", 
                                   devct-1);

     getchar();//clear carriage return from keyboard buffer after invalid 
          //choice
    }
  }
  device=devnames[userchoice];
  return std::string(device);

 }
 else{
  ROS_INFO("No MicroStrain devices found.\n");
  return std::string(device);
 }

}

bool send_command(const char command_string[], ComPortHandle comPort){

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
 writeComPort(comPort, hexs, number_hex_chars); 

 //deallocate the memory for the hex bytes
 free(hexs); 

 return true;
}

// main
int main(int argc, char* argv[]){
  ros::init(argc, argv, "microstrain_mip_node");
  ros::NodeHandle nh;
  ComPortHandle comPort;
  int ret = 0;
  std::string port;
  nh.param("port", port, std::string("/dev/ttyACM0"));
  ROS_INFO("Attempting to open port...%s\n",port.c_str());
  comPort = OpenComPort(port);
  if (comPort == -1) {
    ROS_ERROR("Cannot attach to IMU at port %s, trying automatic initialization",
	      port.c_str());
  }
  else{
    std::string dev=scandev();  
    if(strcmp(dev.c_str(),"")!=0){
      
      printf("Attempting to open port...%s\n",dev.c_str());
      comPort = OpenComPort(dev);
    }
  }
 if(comPort > 0){ 
   
   ROS_INFO("Connected to IMU.");

   std::vector<MIP_Data_Response> mip_responses;

   
   //Disable streaming
   ret = send_command("75650C0A0511010100051101020021C3", comPort);
   ret = readPort(comPort, mip_responses);
   if (mip_responses.size() != 2 || 
       !checkAckResponse(mip_responses[0], 0x11) ||
       !checkAckResponse(mip_responses[1], 0x11)) {
     ROS_ERROR("Failed to disable streaming");
     exit(0);
   }
   //Select scaled acceleration, angular rate, and timestamp AHRS msgs
   ret = send_command("75650C0D0D08010304000A05000A0E000A4195", comPort);
   ret = readPort(comPort, mip_responses);
   if (mip_responses.size() != 1 ||
       !checkAckResponse(mip_responses[0], 0x08)) {
     ROS_ERROR("Failed to select AHRS message format");
     exit(0);
   }
   //Select GPS component messages
   ret = send_command("75650C0A0A090102040004060004188E", comPort);
   ret = readPort(comPort, mip_responses);
   if (mip_responses.size()!= 1 ||
       !checkAckResponse(mip_responses[0], 0x09)) {
     ROS_ERROR("Failed to select GPS data format");
     exit(0);
   }
   //Enable streaming
   ret = send_command("75650C0A0511010101051101020123CA", comPort);
   ret = readPort(comPort, mip_responses);
   if (mip_responses.size()!= 2 ||
       !checkAckResponse(mip_responses[0], 0x011),
       !checkAckResponse(mip_responses[1], 0x011)) {
     ROS_ERROR("Failed to enable streaming");
     exit(0);
   }

   while (ros::ok() ){
     /*     ret = send_command("75650C0404010000EFDA", comPort);
	    ret = readPort(comPort, mip_responses);
	    if (mip_responses.size() != 1 ||
	    !checkAckResponse(mip_responses[0], 0x01)) {
	    ROS_ERROR("Failed to ack polling command properly");
	    exit(0);
	    }*/
     ret = readPort(comPort, mip_responses);
     for (size_t i = 0;i<mip_responses.size();i++) {
       checkDataResponse(mip_responses[i]);
     }
     
   }
   ret = send_command("75650C0A0511010100051101020021C3", comPort);
   ret = readPort(comPort, mip_responses);
   if (mip_responses.size() != 2 || 
       !checkAckResponse(mip_responses[0], 0x11) ||
       !checkAckResponse(mip_responses[1], 0x11)) {
     ROS_ERROR("Failed to disable streaming");
     exit(0);
   }else {
     ROS_INFO("Streaming mode disabled");
   }
   
   ROS_INFO("EXITING\n"); 

   CloseComPort(comPort);

 }

 return ret;

}
