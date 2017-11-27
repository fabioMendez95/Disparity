#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

class ReadRadar{
public:
	//void TestConnection();
	void Connect();
	void Read();
private:
	int fd1,fd2;
	char *buff,*buffer,*bufptr;
	struct termios tty;
	struct termios tty_old;
};
