#include<vector>
#include<stdio.h>
#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/calib3d/calib3d.hpp"

using namespace std;
using namespace cv;

struct ObjectDR{
	int width;
	int length;
	Point centre;
};

class Fusion{
public:
	void radarPointsInImage(Mat disparitySrc, vector<double> radarToSB);
	void setValuesFusion(int minX, int minY, Size s);
	Mat displayOnImage(Mat& Image);

	vector<ObjectDR> getObjects();

private:
	double errorValue = 0.2;

	int xDown;
	int xUp;
	int yDown;
	int yUp;

	int xRep;
	int yRep;

	int radarLine;
	Mat getFilterImage(Mat disparitySrc);
	int widthOfObject(Mat disparity, int x);
	ObjectDR getObjectDimensions(Mat disparity, int width, int x);

	int pixelError = 3;

	vector<ObjectDR> objects;

	Size size;
};
