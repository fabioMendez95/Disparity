#include<vector>
#include<stdio.h>
#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "Radar.h"

using namespace std;
using namespace cv;

struct ObjectDR{
	int width;
	int length;
	Point centre;
	double distance;
	int pointsInside;
};

struct DivisionImage{
	int startX;
	int endX;
	int startY;
	int endY;
};

class Fusion{
public:
	void radarPointsInImage(Mat disparitySrc, vector<double> radarToSB);
	void setValuesFusion(int minX, int minY, Size s);
	Mat displayOnImage(Mat& Image);

	void pointMatchOnImage(Mat disparitySrc, vector<FusionInfo> fus);
	vector<ObjectDR> getObjects();

	void dontShow();

private:
	double errorValue = 0.5;

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

	double getXcooMeters(double Dos, double displacement);

	int pixelError = 2;

	vector<ObjectDR> objects;
	vector<DivisionImage> getDisivionOnImage(Mat disparity);
	void getObjectLength(Mat disparity, DivisionImage& segment);
	void drawSegments(Mat disparity, vector<DivisionImage> segments);

	Size size;
};
