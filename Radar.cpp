#include "Radar.h"
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include<iostream>


#define DATA_PORT "/dev/ttyACM1"
#define BAUD_RATE 921600

using namespace std;

void Radar::test(){
	int fd;/*File Descriptor*/

	printf("\n +----------------------------------+");
	printf("\n |        Serial Port Read          |");
	printf("\n +----------------------------------+");

	/*------------------------------- Opening the Serial Port -------------------------------*/

	/* Change /dev/ttyUSB0 to the one corresponding to your system */

	fd = open(DATA_PORT, O_RDWR | O_NOCTTY); /* ttyUSB0 is the FT232 based USB2SERIAL Converter   */
	/* O_RDWR   - Read/Write access to serial port       */
	/* O_NOCTTY - No terminal will control the process   */
	/* Open in blocking mode,read will wait              */

	if (fd == -1) /* Error Checking */
		printf("\n  Error! in Opening ttyUSB0  ");
	else
		printf("\n Opened Successfully ");

	/*---------- Setting the Attributes of the serial port using termios structure --------- */

	struct termios SerialPortSettings; /* Create the structure                          */

	tcgetattr(fd, &SerialPortSettings); /* Get the current attributes of the Serial port */

	/* Setting the Baud rate */
	cfsetispeed(&SerialPortSettings, B921600); /* Set Read  Speed as 921600                     */
	cfsetospeed(&SerialPortSettings, B921600); /* Set Write Speed as 9600                       */

	/* 8N1 Mode */
	SerialPortSettings.c_cflag &= ~PARENB; /* Disables the Parity Enable bit(PARENB),So No Parity   */
	SerialPortSettings.c_cflag &= ~CSTOPB; /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
	SerialPortSettings.c_cflag &= ~CSIZE; /* Clears the mask for setting the data size             */
	SerialPortSettings.c_cflag |= CS8; /* Set the data bits = 8                                 */

	SerialPortSettings.c_cflag &= ~CRTSCTS; /* No Hardware flow Control                         */
	SerialPortSettings.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */

	SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY); /* Disable XON/XOFF flow control both i/p and o/p */
	SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG); /* Non Cannonical mode                            */

	SerialPortSettings.c_oflag &= ~OPOST;/*No Output Processing*/

	/* Setting Time outs */
	SerialPortSettings.c_cc[VMIN] = 10; /* Read at least 10 characters */
	SerialPortSettings.c_cc[VTIME] = 0; /* Wait indefinetly   */

	if ((tcsetattr(fd, TCSANOW, &SerialPortSettings)) != 0){ /* Set the attributes to the termios structure*/
		printf("\n  ERROR ! in Setting attributes");
	}
	else{
		printf("\n  BaudRate = 921600 \n  StopBits = 1 \n  Parity   = none");
	}
	/*------------------------------- Read data from serial port -----------------------------*/

	tcflush(fd, TCIFLUSH); /* Discards old data in the rx buffer            */

	//Reading Header-------------------------------------------------------------
	char read_buffer[36]; /* Buffer to store the data received              */
	int bytes_read = 0; /* Number of bytes read by the read() system call */

	bytes_read = read(fd, &read_buffer, sizeof(read_buffer)); /* Read the data                   */

	printf("\n\nHeader: Total bytes: %d", bytes_read); /* Print the number of bytes read */
	printf("\n\n");

	//reading magicWord
	cout << "Magic Word: ";
	for (int i = 0; i < 8; i++) /*printing only the received characters*/
		printf("%d ", read_buffer[i]);
	cout << endl;

	unsigned int version = 0;
	for(int i = 11; i >= 8 ;i--){
		version <<= 8;
		version = version + (int)read_buffer[i];
		cout << (int)read_buffer[i] << " ";
	}
	cout << "Version: " << version << endl;

	unsigned int packetLength =0;
	for(int i = 15; i >= 12 ;i--){
		packetLength <<= 8;
		packetLength = packetLength + (int)read_buffer[i];
	}
	cout << "PacketLength: " << packetLength << endl;

	unsigned int platform = 0;
	for(int i = 19; i >= 16 ;i--){
		platform <<= 8;
		platform = platform + (int)read_buffer[i];
	}
	cout << "Platform: " << platform << endl;

	unsigned int frameNumber = 0;
	for(int i = 23; i >= 20 ;i--){
		frameNumber <<= 8;
		frameNumber = frameNumber + (int)read_buffer[i];
	}
	cout << "Frame Number: " << frameNumber << endl;

	unsigned int time = 0;
	for(int i = 27; i >= 24 ;i--){
		time <<= 8;
		time = time + (int)read_buffer[i];
	}
	cout << "Time: " << time << endl;


	unsigned int detectedObjects = 0;
	for (int i = 31; i >= 28; i--) {
		detectedObjects <<= 8;
		detectedObjects = detectedObjects + (int) read_buffer[i];
		//cout << (int) read_buffer[i] << " ";
	}
	cout << "Number of Detected Objects: " << detectedObjects << endl;

	unsigned int dataStructNum = 0;
	for (int i = 35; i >= 32; i--) {
		dataStructNum <<= 8;
		dataStructNum = dataStructNum + (int) read_buffer[i];
		//cout << (int) read_buffer[i] << " ";
	}
	cout << "Number of Data Structures: " << dataStructNum << endl;


	printf("\n +----------------------------------+\n\n\n");

	//Reading Header------------------------------------------------

	close(fd); /* Close the serial port */

}
