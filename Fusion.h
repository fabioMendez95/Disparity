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
	double distance;
};

class Fusion{
public:
	void radarPointsInImage(Mat disparitySrc, vector<double> radarToSB);
	void setValuesFusion(int minX, int minY, Size s);
	Mat displayOnImage(Mat& Image);

	void pointMatchOnImage(Mat disparitySrc, vector<double> radarToSB, vector<double> displacements);
	vector<ObjectDR> getObjects();

	void dontShow();

private:
	double errorValue = 0.1;

	bool showFusionProcess = true;

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

	double getXcooMeters(double Dos, double displacement);

	int pixelError = 3;

	vector<ObjectDR> objects;

	Size size;
};
