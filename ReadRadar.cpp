#include "ReadRadar.h"
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

#define DATA_PORT "/dev/ttyACM1"
#define BAUD_RATE 921600

using namespace std;

void ReadRadar::Connect(){

	fd1 = open(DATA_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
	if(fd1 == -1){
		printf("Error Connecting \n");
	}
	else{
		fcntl(fd1,F_SETFL,0);
		printf("Connected Successfully \n");
	}
	memset (&tty, 0, sizeof tty);

	tty_old = tty;

	cfsetospeed (&tty, (speed_t)B921600);
	cfsetispeed (&tty, (speed_t)B921600);

	/* Setting other Port Stuff */
	tty.c_cflag     &=  ~PARENB;            // Make 8n1
	tty.c_cflag     &=  ~CSTOPB;
	tty.c_cflag     &=  ~CSIZE;
	tty.c_cflag     |=  CS8;

	tty.c_cflag     &=  ~CRTSCTS;           // no flow control
	tty.c_cc[VMIN]   =  1;                  // read doesn't block
	tty.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
	tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

	/* Make raw */
	cfmakeraw(&tty);

	/* Flush Port, then applies attributes */
	tcflush( fd1, TCIFLUSH );
	/*if ( tcsetattr ( fd1, TCSANOW, &tty ) != 0) {
	   cout << "Error " << errno << " from tcsetattr\n";
	}*/
}
/*void ReadRadar::Read(){
	int n = 0,
	    spot = 0;
	char buf = '\0';

	 Whole response
	char response[1024];
	memset(response, '\0', sizeof response);

	do {
	    n = read( USB, &buf, 1 );
	    sprintf( &response[spot], "%c", buf );
	    spot += n;
	} while( buf != '\r' && n > 0);

	if (n < 0) {
	    std::cout << "Error reading: " << strerror(errno) << std::endl;
	}
	else if (n == 0) {
	    std::cout << "Read nothing!" << std::endl;
	}
	else {
	    std::cout << "Response: " << response << std::endl;
	}
}*/
