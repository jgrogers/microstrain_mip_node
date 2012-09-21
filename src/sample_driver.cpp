//Example code for interfacing with the Microstrain Inertial Sensor through its
//serial connection

/* compile using:


 gcc linux_serial_hex_driver.c -lpthread -o BINFILENAME

 Once compiled the desired device can be specified using a command line 
 argument.

EXAMPLES:(FOR USB)

./BINFILENAME /dev/ttyACM0

or: (FOR SERIAL)

./BINFILENAME /dev/ttyS0


 -OR-

 On some linux distributions, (specifically ubuntu/debian) the program will 
 scan /dev/serial/by-id for attached devices by name if connected to the USB 
 port AND no device is specified at the command line. The USB connected 
 3DM-GX3-xx will usually show up in /dev/ttyACM0 to  ttyACM# where represents 
 the device number by the order the devices were attached.

 Copyright 2012 Microstrain.                                                  
                                                                              
 THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING            
 CUSTOMERS WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER          
 FOR THEM TO SAVE TIME. AS A RESULT, MICROSTRAIN SHALL NOT BE HELD LIABLE     
 FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY        
 CLAIMS ARISING FROM THE CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY      
 CUSTOMERS OF THE CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH      
 THEIR PRODUCTS.                                                              

*/

#include <termios.h> // terminal io (serial port) interface
#include <fcntl.h>  // File control definitions
#include <errno.h>  // Error number definitions
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <semaphore.h>
#include <stdlib.h>
#include <unistd.h>

#define TRUE 		1
#define FALSE 		0
#define EXIT		-1
#define BUFFSIZE 	4096

typedef int ComPortHandle;
typedef unsigned char Byte;
int send_command(char command_string[], ComPortHandle comPort);

//function to remove whitespace from commands
void remove_space(char *string_with_spaces){
 
 int i=0;
 int parse_index = 0;
 char *no_space_string;

 if(string_with_spaces==0)
  return;

 no_space_string=string_with_spaces;

 for(i;i<strlen(string_with_spaces);i++){
  if(!(string_with_spaces[i]==' '||string_with_spaces[i]=='\t'))
   *no_space_string++ = string_with_spaces[i];  
 }
 
 *no_space_string = '\0';

 no_space_string = 0;

 return; 
}

// Utility functions for working with a com port in Linux

// Purge
// Clears the com port's read and write buffers

int Purge(ComPortHandle comPortHandle){

 if (tcflush(comPortHandle,TCIOFLUSH)==-1){

  printf("flush failed\n");
  return FALSE;

 }

 return TRUE;

}

// OpenComPort
// Opens a com port with the correct settings for communicating with a 
// MicroStrain 3DM-GX3-25 sensor

ComPortHandle OpenComPort(const char* comPortPath){

 ComPortHandle comPort = open(comPortPath, O_RDWR | O_NOCTTY);

 if (comPort== -1){ //Opening of port failed

  printf("Unable to open com Port %s\n Errno = %i\n", comPortPath, errno);
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

  printf("Configuring comport failed\n");
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

//attemots to read BUFFSIZE bytes at a time from the open port
int readPort(ComPortHandle comPort){

 int i=0;
 int size;
 struct termios initial_settings, new_settings;
 char command_string[255];
 char c;
 int ret;
 Byte response[BUFFSIZE] = {0};

 //get current terminal settings 
 tcgetattr(0,&initial_settings);

 new_settings = initial_settings;
 new_settings.c_lflag &= ~ICANON;
 new_settings.c_cc[VMIN] = 0;
 new_settings.c_cc[VTIME] = 0;

 tcsetattr(0, TCSANOW, &new_settings);

 //determine if there are bytes to read
 size = readComPort(comPort, &response[0], BUFFSIZE);

 if(size<0)
  return FALSE;
 
 //if ther are bytes to read output them and check again
 while(size>0){

  //write out any read bytes
  for(i=0;i<size;i++){

    printf("%.2x ",response[i]|0x00);

  }
  c = getchar();

  if(c != EOF)
  {
    tcsetattr(0, TCSANOW, &initial_settings);
    ungetc(c,stdin);
    gets(command_string);
    remove_space(command_string);
    ret=send_command(command_string,comPort);       
    if(ret==EXIT)
     return EXIT;
    tcsetattr(0, TCSANOW, &new_settings);
  }

  size = readComPort(comPort, &response[0], BUFFSIZE);

 }

 tcsetattr(0, TCSANOW, &initial_settings);

 return TRUE;

}

//scandev
//finds attached microstrain devices and prompts user for choice then returns 
//selected portname
char* scandev(){
 
 FILE *instream;
 char devnames[255][255];//allows for up to 256 devices with path links up to 
             //255 characters long each
 int devct=0; //counter for number of devices
 int i=0;
 int j=0;
 int userchoice=0;
 char *device;

 //search /dev/serial for microstrain devices
 char *command = "find /dev/serial -print | grep -i microstrain";
 
 printf("Searching for devices...\n");

 instream=popen(command, "r");//execute piped command in read mode

 if(!instream){//SOMETHING WRONG WITH THE SYSTEM COMMAND PIPE...EXITING
  printf("ERROR BROKEN PIPELINE %s\n", command);
  return device;
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
  printf("Device Found:\n%d: %s\n",i,devnames[i]);
 }

 //CHOOSE DEVICE TO CONNECT TO AND CONNECT TO IT (IF THERE ARE ANY CONNECTED)

 if(devct>0){
  printf("Number of devices = %d\n", devct);
  if(devct>1){
   printf(
      "Please choose the number of the device to connect to (0 to %i):\n",
                                   devct-1);
    //check that there's input and in the correct range
    while(scanf("%i",&userchoice)==0||userchoice<0||userchoice>devct-1){
     printf("Invalid choice...Please choose again between 0 and %d:\n", 
                                   devct-1);

     getchar();//clear carriage return from keyboard buffer after invalid 
          //choice
    }
  }
  device=devnames[userchoice];
  return device;

 }
 else{
  printf("No MicroStrain devices found.\n");
  return device; 
 }

}

int send_command(char command_string[], ComPortHandle comPort){

 int number_hex_chars;
 int string_length;
 unsigned char* hexs;
 int i;
 
 if(strcmp(command_string,"exit")==0)
  return EXIT;
 
 //deterine length of command string
 string_length = strlen(command_string);

 //each byte is represented by 2 ascii chars
 //so the length of the string should always be even
 if(string_length%2!=0){
  printf("Invalid Command\n");
  return FALSE;
 }

 //each hex char is only 1 byte from the two entered
 number_hex_chars=string_length/2;

 //allocate memory for the byte-level command string
 hexs=(unsigned char*)malloc(number_hex_chars*sizeof(unsigned char));

 //process each 2-char set from an ascii representation of the hexadecimal
 //command byte to the associated byte
 for(i=0;i<number_hex_chars;i++){
  if(sscanf(&command_string[i*2],"%2x",&hexs[i])<1){
   printf("Invalid Command\n");
   return FALSE;
  }
 }
 //write command 
 writeComPort(comPort, hexs, number_hex_chars); 

 //deallocate the memory for the hex bytes
 free(hexs); 

 return TRUE;
}

// main
int main(int argc, char* argv[]){

 ComPortHandle comPort;
 int go = TRUE; 
 char *dev;
 char a;
 char command_string[255];
 int ret;

 a='\0';

 dev=&a;

 if(argc<2){//No port specified at commandline so search for attached devices

  dev=scandev();  
  if(strcmp(dev,"")!=0){
 
   printf("Attempting to open port...%s\n",dev);
   comPort = OpenComPort(dev);

  }
  else{

   printf("Failed to find attached device.\n");
   return FALSE;

  }

 }
 else{//Open port specified at commandline

  printf("Attempting to open port...%s\n",argv[1]);
  comPort = OpenComPort(argv[1]);

 }

 printf("comPort=%i\n",comPort);

 if(comPort > 0){ 

  printf("Connected. \n\nEnter Command as specified in 3DM-GX3 MIP DCP");
  printf(" followed by an <ENTER> Then utility will attempt to read");
  printf("reply on port.\n The resulting read will read all characters ");
  printf(" available currently then return for user command.\n");
  
  while(1){//continue until user chooses to exit

   printf("\nEnter Command (\"exit\" to exit)> "); 
   //get command from user
   gets(command_string);    
   remove_space(command_string);
   ret=send_command(command_string,comPort);
   //exit if user specified this
   if(ret==EXIT)
    break;
   //read port if user didn't ask for exit
   ret=readPort(comPort);
   //exit if user specified this
   if(ret==EXIT)
    break;

  }

  printf("EXITING\n"); 
  CloseComPort(comPort);

 }

 return 0;

}
