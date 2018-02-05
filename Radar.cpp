#include "Radar.h"
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include<iostream>
#include<cmath>


#include <sys/stat.h>
#include <sys/types.h>
#include <string>
#include <string.h>
#include <iomanip> // setprecision
#include <sstream> // stringstream

#define DATA_PORT "/dev/ttyACM1"
#define BAUD_RATE 921600


using namespace std;

void Radar::startRadar(){
	fd = open(DATA_PORT, O_RDWR | O_NOCTTY);
	if (fd == -1) /* Error Checking */
		printf("\n  Error! in Opening ttyUSB0  ");
	else
		printf("\n Opened Successfully ");
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

	if ((tcsetattr(fd, TCSANOW, &SerialPortSettings)) != 0) { /* Set the attributes to the termios structure*/
		printf("\n  ERROR ! in Setting attributes");
	} else {
		printf("\n  BaudRate = 921600 \n  StopBits = 1 \n  Parity   = none");
	}
#if VISUAL
	//mkfifo(myfifo, 0666);
	//fdf = open(myfifo, O_WRONLY);
#endif
}

void Radar::closeRadar(){
	close(fd); /* Close the serial port */
#if VISUAL
/*	close(fdf);
	unlink(myfifo);*/
#endif
}

bool Radar::checkMagicWord(int readWord, int position){
	bool pass = (readWord == magicWordInts[position]);
	return pass;
}

//returns 1 if the data was read correctly, 0 otherwise
int Radar::readInfo(){
	tcflush(fd, TCIFLUSH); /* Discards old data in the rx buffer            */

	//Reading Header-------------------------------------------------------------
	char read_buffer[1024]; /* Buffer to store the data received              */
	int bytes_read = 0; /* Number of bytes read by the read() system call */

	bytes_read = read(fd, &read_buffer, sizeof(read_buffer));
	if(bytes_read < 128){ //minimum number of the packet
		return 0;
	}
	/*printf("\n\n +----------------------------------+");
	printf("\n\nTotal bytes: %d", bytes_read);  Print the number of bytes read
	printf("\n\n");*/

	//reading magicWord
	//cout << "Magic Word: ";
	bool pass = true;
	for (int i = 0; i < 8; i++){ /*printing only the received characters*/
		//printf("%d ", read_buffer[i]);
		pass = pass && checkMagicWord(read_buffer[i],i);
	}
	//cout << endl;
	if(!pass){
		return 0;
	}


	unsigned int packetLength = 0;
	for (int i = 15; i >= 12; i--) {
		packetLength <<= 8;
		packetLength = packetLength + (int) read_buffer[i];
	}
	//cout << "PacketLength: " << packetLength << endl;

	unsigned int detectedObjects = 0;
	for (int i = 31; i >= 28; i--) {
		detectedObjects <<= 8;
		detectedObjects = detectedObjects + (int) read_buffer[i];
		//cout << (int) read_buffer[i] << " ";
	}
	//cout << "Number of Detected Objects: " << detectedObjects << endl;

	//printf("\n +----------------------------------+ \n\n");
	//Reading  Detected Objects
	int sizeDetectedObjects = 12 + detectedObjects * 12;
	int byteToReadB = 36;

	unsigned int structureTag = 0;
	for (int i = byteToReadB + 3; i >= byteToReadB; i--) {
		structureTag <<= 8;
		structureTag = structureTag + (int) read_buffer[i];
		//cout << (int) read_buffer[i] << " ";
	}
	//cout << "Structure Tag: " << structureTag << endl;
	byteToReadB = byteToReadB + 4;
	unsigned int lengthStruct = 0;
	for (int i = byteToReadB + 3; i >= byteToReadB; i--) {
		lengthStruct <<= 8;
		lengthStruct = lengthStruct + (int) read_buffer[i];
		//cout << (int) read_buffer[i] << " ";
	}
	//cout << "Length Structure: " << lengthStruct << endl;
	byteToReadB = byteToReadB + 4;

	unsigned int Descriptor1 = 0;
	for (int i = byteToReadB + 1; i >= byteToReadB; i--) {
		Descriptor1 <<= 8;
		Descriptor1 = Descriptor1 + (int) read_buffer[i];
		//cout << (int) read_buffer[i] << " ";
	}
	byteToReadB = byteToReadB + 2;
	//cout << "Descriptor Num of detected Objects: " << Descriptor1 << endl;
	unsigned int Descriptor2 = 0;
	for (int i = byteToReadB + 1; i >= byteToReadB; i--) {
		Descriptor2 <<= 8;
		Descriptor2 = Descriptor2 + (int) read_buffer[i];
		//cout << "Byte " << (unsigned int) read_buffer[i] << endl;
		//cout << (int) read_buffer[i] << " ";
	}
	//Descriptor2 = (unsigned int)read_bufferDO[11];
	//cout << "Descriptor 2:  " << Descriptor2 << endl;
	//cout << "Descriptor XYZ Q format: " << pow(2, Descriptor2) << endl;
	float xyzQ = (float) pow((double) 2, (double) (Descriptor2));
	byteToReadB = byteToReadB + 2;
	//cout << "Starting location Read byte: " << byteToReadB << endl;
	//cout << "\nObject Information: \n";
	//This area depends on the number of Objects detected
	//Reading detected objects Range, doppler, peak, x,y,z each 2 bytes

	//data to be send ---------------
#if VISUAL
	mkfifo(myfifo, 0666);
	fdf = open(myfifo, O_WRONLY);

	stringstream stream;
	stream << detectedObjects;
#endif
	cout << "Number Num: " << detectedObjects << " ";
	for (int obj = 0; obj < detectedObjects; obj++) {
		cout << "Byte: " << byteToReadB << " ";
		//cout << "\nObject Number " << obj << " :\n";
		unsigned int range = 0;
		unsigned int doppler = 0;
		unsigned int peak = 0;
		int x = 0;
		int y = 0;
		int z = 0;

		//reading range
		for (int i = byteToReadB + 1; i >= byteToReadB; i--) {
			range <<= 8;
			range = range + (unsigned int) read_buffer[i];
			//cout << (int) read_buffer[i] << " ";
		}
		//cout << "Range: " << range << endl;

		for (int i = byteToReadB + 3; i >= byteToReadB + 2; i--) {
			doppler <<= 8;
			doppler = doppler + (int) read_buffer[i];
			//cout << (int) read_buffer[i] << " ";
		}
		//cout << "Doopler: " << doppler << endl;

		for (int i = byteToReadB + 5; i >= byteToReadB + 4; i--) {
			peak <<= 8;
			peak = peak + (int) read_buffer[i];
			//cout << (int) read_buffer[i] << " ";
		}
		//cout << "Peak: " << peak << endl;

		//Points
		for (int i = byteToReadB + 7; i >= byteToReadB + 6; i--) {
			x <<= 8;
			x = x + (int) read_buffer[i];
			//cout << (int) read_buffer[i] << " ";
		}
		for (int i = byteToReadB + 9; i >= byteToReadB + 8; i--) {
			y <<= 8;
			y = y + (int) read_buffer[i];
			//cout << (int) read_buffer[i] << " ";
		}
		for (int i = byteToReadB + 11; i >= byteToReadB + 10; i--) {
			z <<= 8;
			z = z + (int) read_buffer[i];
			//cout << (int) read_buffer[i] << " ";
		}
		if(x>32767){
			x = x - 65536;
		}
		if (y > 32767) {
			y = y - 65536;
		}

		float xCoo = ((float) x) / xyzQ;
		float yCoo = ((float) y) / xyzQ;
		cout << "Coordinates: " << xCoo << " " << yCoo << " "
				<< (float) (z / Descriptor2) << endl;
		byteToReadB = byteToReadB + 12;
#if VISUAL
		stream << "\n";
		stream << fixed << setprecision(4) << xCoo;
		stream << " ";
		stream << fixed << setprecision(4) << yCoo;
#endif
	}
#if VISUAL
	string SendInfo = stream.str();
	write(fdf, SendInfo.c_str(), SendInfo.length());
	close(fdf);
	unlink(myfifo);
#endif
	cout << "done Passing info \n";
	return 1;
}
