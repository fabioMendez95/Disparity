#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

#include <sys/stat.h>
#include <sys/types.h>
#include <string>
#include <string.h>
#include <iomanip> // setprecision
#include <sstream> // stringstream


#define VISUAL true

class Radar{
public:
	int readInfo();
	void startRadar();
	void closeRadar();
	void sendSingleToPy();
private:
	//Variable initialization
	int fd;
	struct termios SerialPortSettings;// Create the structure
#if VISUAL
	int fdf;
	char* myfifo = "/tmp/FIFO";
#endif
	int magicWordInts[8] = {2,1,4,3,6,5,8,7};

	//Funcrtions
	bool checkMagicWord(int readWord, int position);

};
