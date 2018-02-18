#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <iostream>

using namespace cv;
using namespace std;

struct StereoPair{
	Mat left;
	Mat right;
};

class Camera{
public:
	void getCamera();
	void initCamera(int device);
	Mat getRight();
	Mat getLeft();
	void extractImage();

	//Mari es preciosa
	void createCameraSample(int num);

	void close();

private:
	VideoCapture stream;
	int imageWidth;
	int imageHeight;

	//Left Camera parameters
	Mat CameraMatrixLeft;
	Mat DistortionCoeffLeft;
	//float camDatLeft[9] = {6.7354280072906420e+02, 0, 6.3950000000000000e+0, 0,
	//	    6.7354280072906420e+02, 3.5950000000000000e+02, 0, 0, 1};
	float camDatLeft[9] = {1389.16, 0, 1155.59, 0,
			1389.16, 661.689, 0, 0, 1};

	/*float camCoeffLeft[5] = {-2.0435760493430152e-01, 7.1501399601846899e-02, 0, 0,
		    -1.8224940787071004e-02};*/
	float camCoeffLeft[4] = {-0.175745, 0.0278024,0,0};

	Mat viewL, rviewL, map1L, map2L;

	//Right Camera Parameters
	Mat CameraMatrixRight;
	Mat DistortionCoeffRight;
/*	float camDatRight[9] = { 6.8740482095599828e+02, 0, 6.3950000000000000e+02, 0,
		    6.8740482095599828e+02, 3.5950000000000000e+02, 0, 0, 1.};*/
	float camDatRight[9] = { 1395.04, 0, 1160.77, 0,
			1395.04, 707.253, 0, 0, 1.};
	/*float camCoeffRight[5] = { -1.6863923311532500e-0, 9.9014670322185843e-03, 0, 0,
		    9.5270171834983628e-03};*/
	float camCoeffRight[4] = {-0.175673, 0.0276058, 0, 0};
	Mat viewR, rviewR, map1R, map2R;

	String getX(int i);

	Mat left;
	Mat rigth;

	StereoPair getImages();
};
