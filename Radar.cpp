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
#include<math.h>

#define DATA_PORT "/dev/ttyACM1"
#define BAUD_RATE 921600


using namespace std;

void Radar::startRadar(){
	fd = open(DATA_PORT,  O_RDWR | O_NOCTTY | O_SYNC);
	if (fd == -1) /* Error Checking */
		printf("\n  Error! in Opening USB  ");
	else
		printf("\n Opened Successfully ");
	tcgetattr(fd, &SerialPortSettings); /* Get the current attributes of the Serial port */

	/* Setting the Baud rate */
	cfsetispeed(&SerialPortSettings, B921600); /* Set Read  Speed as 921600                     */
	cfsetospeed(&SerialPortSettings, B921600); /* Set Write Speed as 9600                       */

	//8N1 Mode
	SerialPortSettings.c_cflag &= ~PARENB;  //Disables the Parity Enable bit(PARENB),So No Parity
	SerialPortSettings.c_cflag &= ~CSTOPB;  //CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit
	SerialPortSettings.c_cflag &= ~CSIZE;  //Clears the mask for setting the data size
	SerialPortSettings.c_cflag |= CS8;  //Set the data bits = 8

	SerialPortSettings.c_cflag &= ~CRTSCTS;//  No Hardware flow Control
	SerialPortSettings.c_cflag |= CREAD | CLOCAL;//  Enable receiver,Ignore Modem Control lines

	SerialPortSettings.c_iflag &= (IXON | IXOFF | IXANY);//  Disable XON/XOFF flow control both i/p and o/p
	SerialPortSettings.c_iflag &= (ICANON | ECHO | ECHOE | ISIG);//  Non Cannonical mode

	SerialPortSettings.c_oflag &= ~OPOST;//No Output Processing

	 //Setting Time outs
	SerialPortSettings.c_cc[VMIN] = 10;  //Read at least 10 characters
	SerialPortSettings.c_cc[VTIME] = 0; // Wait indefinetly
	SerialPortSettings.c_oflag &= ~OPOST;//No Output Processing

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

void concatenateInfo(char& readInfo, int bytes_read){

}

//returns 1 if the data was read correctly, 0 otherwise
int Radar::readInfo(){
	tcflush(fd, TCIFLUSH); /* Discards old data in the rx buffer            */

	//Reading Header-------------------------------------------------------------
	char read_buffer[1824]; /* Buffer to store the data received              */
	memset(read_buffer,0,1824);
	size_t bytes_read= 0, bytes_expected = 1824;

	do {
	  ssize_t result = read(fd, read_buffer + bytes_read, bytes_expected - bytes_read);
	  if (0 >= result)
	  {
	    if (0 > result)
	    {
	      perror("read()");
	    }

	    break;
	  }
	  bool pass = true;
	  for(int i = 0; i <= 7;i++){
		 // cout << (int)read_buffer[i] << " ";
		  pass = pass && checkMagicWord(read_buffer[i],i);
	  }
	  //cout << endl;
	  if(pass){
		  bytes_read+= result; //should not increase until magic word
	  }
	} while (bytes_read < bytes_expected);

	//int bytes_read = 0; /* Number of bytes read by the read() system call */
	//bytes_read = read(fd, &read_buffer, sizeof(read_buffer));
/*	printf("\n\n +----------------------------------+");
	printf("\n\nTotal bytes: %d", bytes_read);  //Print the number of bytes read
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
	//cout << "PacketLength: " << packetLength <<endl;

	unsigned int detectedObjects = 0;
	for (int i = 31; i >= 28; i--) {
		detectedObjects <<= 8;
		detectedObjects = detectedObjects + (int) read_buffer[i];
		//cout << (int) read_buffer[i] << " ";
	}
	//cout << "Number of Detected Objects: " << detectedObjects << endl;



	//printf("\n +----------------------------------+ \n\n");
	//Reading  Detected Objects ----------------------------------------------------------
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
	float xyzQ = (float) pow((float) 2, (float) (Descriptor2));
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
	dataPointNum = detectedObjects;
	distanceToSB.clear();
	xCoordinates.clear();
	cout << "Number Num: " << detectedObjects << " \n";
	for (int obj = 0; obj < detectedObjects; obj++) {
		//cout << "Byte: " << byteToReadB << "\n ";
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
		/*for (int i = byteToReadB + 7; i >= byteToReadB + 6; i--) {
			x <<= 8;
			x = x + (int) read_buffer[i];
			//cout << (int) read_buffer[i] << " ";
		}
		for (int i = byteToReadB + 9; i >= byteToReadB + 8; i--) {
			y <<= 8;
			y = y + (int) read_buffer[i];
			//cout << (int) read_buffer[i] << " ";
		}*/
		for (int i = byteToReadB + 11; i >= byteToReadB + 10; i--) {
			z <<= 8;
			z = z + (int) read_buffer[i];
			//cout << (int) read_buffer[i] << " ";
		}

		//float xCoo = ((float) x) / xyzQ;
		//float yCoo = ((float) y) / xyzQ;

		union {
			char chars[2];
			int16_t in;
		} u1,u2;

		u1.chars[0] = read_buffer[byteToReadB + 6];
		u1.chars[1] = read_buffer[byteToReadB + 7];
		float xCoo = float(u1.in) / xyzQ;

		u2.chars[0] = read_buffer[byteToReadB + 8];
		u2.chars[1] = read_buffer[byteToReadB + 9];
		float yCoo = float(u2.in) / xyzQ;

		//Polar Coordinates
		//if(yCoo >= 0){
			double mag = sqrt(xCoo*xCoo + yCoo*yCoo);
			double ang = atan2(yCoo,xCoo);
			if(xCoo < 0){
				ang = M_PI - ang;
			}
			distanceToSB.push_back(getDistancePointToStereo(mag,ang));
			xCoordinates.push_back(mag*cos(ang));
			cout << "Point: " << xCoo << " " << yCoo <<" \tPolar Coo: "<< mag << ' ' << (ang*180/M_PI) << " \tDistance to SB: " << getDistancePointToStereo(mag,ang)<<endl;
		//}
		/*else{
			union {
			  char chars[2];
			  float f;
			} u;
			u.chars[0] = read_buffer[byteToReadB+9];
			u.chars[1] = read_buffer[byteToReadB+8];
			y = u.f/xyzQ;
			uint16_t y = float((read_buffer[byteToReadB+8] << 8) + read_buffer[byteToReadB+9]);
			cout << "Y: " << y/xyzQ << " " <<(float)y/xyzQ << " " << u.f/xyzQ << "\t";
			for(int i = byteToReadB; i <= byteToReadB + 11; i++){

				cout << (uint16_t) read_buffer[i] << " ";
			}
			cout << " wrong \n";
		}*/
		//cout << "Coordinates: " << xCoo << " " << yCoo << " " << (float) (z / Descriptor2) << " End byte "<< (byteToReadB + 12) <<endl;
		byteToReadB = byteToReadB + 12;
#if VISUAL
		stream << "\n";
		stream << fixed << setprecision(7) << xCoo;
		stream << " ";
		stream << fixed << setprecision(7) << yCoo;
#endif
	}
#if VISUAL
	string SendInfo = stream.str();
	write(fdf, SendInfo.c_str(), SendInfo.length());
	close(fdf);
	unlink(myfifo);
#endif
	//cout << "done Passing info \n";
	return 1;
}

double Radar::getDistancePointToStereo(double mag, double ang){
	//cout << mag << " " << ang << " ";
	double z = Rts + mag*sin(ang);
	return z;
}

void Radar::setData(double distanceToSBNum){
	Rts = distanceToSBNum;
}

int Radar::getDataPointNum(){
	return dataPointNum;
}
