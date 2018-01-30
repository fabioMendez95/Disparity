#include <stdio.h>
#include <iostream>
#include <string.h>
#include <limits>

#include "opencv2/core/core.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"

#include<sys/resource.h>
#include <time.h>
#include<sys/time.h>

using namespace std;
using namespace cv;

class Comparison{
public:
	void CompareDisparities();
private:
	void CostComputation (Mat left, Mat right, int* cost, int maxDisparity, int width, int length, int blockX, int blockY);
	void AggregateCostCom(int* cost, int* L, int width, int length,int maxDisparity, int directionx, int directiony);
	int minBetweenNumbersInt(int a, int b, int c, int d);
	Mat DisparitySelectionP(int* L2, int* L4, int* L5, int* L7,int* L1, int* L3, int* L6, int* L8, int maxDisparity, int width, int length);

	Mat SimpleDisparityCalculations(Mat left, Mat right, int blockX, int blockY, int width, int length, int maxDisparity);
	void DivideImagesCam(Mat completeImage, Mat* left, Mat* right);


	void CensusTransformation (Mat image, int widthW, int lengthW, unsigned int* censusArray);
	void CostComputationCensus (unsigned int* censusL, unsigned int* censusR, int* cost, int maxDisparity, int width, int length);
	int HammingDistanceNumbers (unsigned int a, unsigned int b);



};
