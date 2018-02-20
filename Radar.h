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
#include <vector>


#define VISUAL true

using namespace std;

class Radar{
public:
	int readInfo();
	void startRadar();
	void closeRadar();
	void sendSingleToPy();

	int getDataPointNum();
	vector<double> distanceToSB;

	void setData(int distanceToSB);
private:
	int Rts = 0;
	int dataPointNum = 0;

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

	double getDistancePointToStereo(double mag, double ang);

};
