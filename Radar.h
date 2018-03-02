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

#include "opencv2/core/core.hpp"
#include "map"

#define VISUAL true

using namespace std;
using namespace cv;

#pragma once
struct FusionInfo{
	double distanceToSB;
	double displacement;
};

struct PointD{
	double x,y;

	bool operator==(const PointD &o) const{
		return x == o.x && y == o.y;
	}
	bool operator<(const PointD o) const{
		return x < o.x || (x == o.x && y < o.y);
	}
};



class Radar{
public:
	int readInfo();
	void startRadar();
	void closeRadar();
	void sendSingleToPy();

	int getDataPointNum();
	vector<double> distanceToSB;
	vector<double> xCoordinates;
	vector<FusionInfo> fusion;

	map<PointD ,int> getLabelMap();

	void saveImage();
	void setData(double distanceToSBNum);
private:
	bool saveData = false;
	double dbscanEPS = 0.2;
	int minClusterSize = 10;

	double Rts = 0;
	int dataPointNum = 0;

	void concatenateInfo(char& readInfo, int bytes_read);

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


	map<PointD, int> labels;
	int DBSCAN(vector<PointD> DB, double eps, int minPts, map<PointD, int> &labels);
	void expandCluster(PointD P, vector<PointD> sphere_points, int C, double eps, int minPts, map<PointD, int> &labels, vector<PointD> DB);
	vector<PointD> RangeQuery(vector<PointD> DB, PointD Q,double eps);
	double EuclidianDistance(PointD a, PointD b);
	void showClusterResults(map<PointD,int> labels);
	void expandVector(vector<PointD> &toExpand, vector<PointD> adder);

	//get centroids of DBSCAN
	vector<PointD> getCentroidsOfClusters(map<PointD,int> labels, int numberOfClusters);

};
