#include <stdio.h>
#include <iostream>
#include <string.h>
#include <limits>
//#include "ReadRadar.h"

#include "opencv2/core/core.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <sl/Camera.hpp>

#include<sys/resource.h>
#include <time.h>
#include<sys/time.h>

#include "Comparison.h"
#include "Kernels.cuh"

using namespace std;
using namespace cv;

#define threadx 16
#define thready 16

#define USECAMARA false
#define Camara 1
#define WIDTHIMAGE 1280
#define LENGTHIMAGE 720

__host__ void SMG();
__host__ void AggregateCostCom(int* cost, int* L, int width, int length,int maxDisparity, int directionx, int directiony);
__host__ int minBetweenNumbersInt(int a, int b, int c, int d);
__host__ Mat DisparitySelectionP(int* L2, int* L4, int* L5, int* L7,int* L1, int* L3, int* L6, int* L8, int maxDisparity, int width, int length);
__host__ void DivideImagesCam(Mat completeImage, Mat* left, Mat* right);
__host__ void CostComputationCensus (unsigned int* censusL, unsigned int* censusR, int* cost, int maxDisparity, int width, int length);
__host__ int HammingDistanceNumbers (unsigned int a, unsigned int b);


//sl::Camera zed;

int main (int argc, char** argv){
	//Initialising Radar
	//ReadRadar radar;
	//radar.Connect();
	//radar.Read();
	//Initialising Radar


	//Comparison Calculations
	//Comparison comp;
	//comp.CompareDisparities();
	//------------------------

	SMG();
	return 0;
}

__host__ void SMG(){
	struct timeval timstr;
	Mat left, right;
	Mat completeImage;
	//Texture Creation
	Mat leftD,rightD;
	uchar* imageLeftA,*imageRightA,*leftC,*rightC;

	//CensusDeclarations
	unsigned int* censusLa;
	unsigned int* censusRa;
	int* costK;

	//Maximum box value is depending on bytes used
	int BoxCostX = 9;
	int BoxCostY = 7;
	int maxDisparity = 100;


#if USECAMARA
	VideoCapture stream(Camara);
	for (int i =0; i<100;i++){
		stream.read(completeImage);
	}

	DivideImagesCam(completeImage,&right,&left);
#else
	//Reading Images
	left = imread("Images/KITTY/left/0000000000.png", CV_LOAD_IMAGE_COLOR);
	right= imread("Images/KITTY/right/0000000000.png", CV_LOAD_IMAGE_COLOR);

#endif

	//Converting Images
	Mat leftBlack;
	cvtColor( left, leftBlack, CV_BGR2GRAY );
	Mat rightBlack;
	cvtColor( right, rightBlack, CV_BGR2GRAY );

	//Passing images to kernels----------------------------
	imageLeftA = (uchar*)malloc((sizeof(uchar))*(left.cols)*(left.rows));
	imageRightA= (uchar*)malloc((sizeof(uchar))*(right.cols)*(right.rows));
	imageLeftA = leftBlack.data;
	imageRightA= rightBlack.data;
	cudaMalloc(&leftC,(sizeof(uchar))*(left.cols)*(left.rows));
	cudaMalloc(&rightC,(sizeof(uchar))*(left.cols)*(left.rows));
	cudaMemcpy(leftC,imageLeftA,(sizeof(uchar))*(left.cols)*(left.rows),cudaMemcpyHostToDevice);
	cudaMemcpy(rightC,imageRightA,(sizeof(uchar))*(left.cols)*(left.rows),cudaMemcpyHostToDevice);
	//Done passing images to kernels------------------------
	//Done Converting Images

	int decreseX = BoxCostX/2 + BoxCostX/2;
	int decreseY = BoxCostY/2 + BoxCostY/2;

	int dimX = ((leftBlack.cols-decreseX) / threadx);
	int dimY = ((leftBlack.rows-decreseY) / thready);
	dim3 dimGrid(dimX,dimY);
	dim3 dimBlock(threadx,thready);

	//Timing
	gettimeofday(&timstr, NULL);
	double begin = timstr.tv_sec + (timstr.tv_usec / 1000000.0);

	//Census Transform
	unsigned int* CLK =(unsigned int*)malloc((sizeof(unsigned int))*(left.cols-decreseX)*(left.rows-decreseY));
	unsigned int* CRK =(unsigned int*)malloc((sizeof(unsigned int))*(right.cols-decreseX)*(right.rows-decreseY));
	//CensusTransformation(leftBlack,BoxCostX,BoxCostY,censusLa);
	//CensusTransformation(rightBlack,BoxCostX,BoxCostY,censusRa);

	cudaMalloc(&censusLa,(sizeof(unsigned int))*(left.cols-decreseX)*(left.rows-decreseY));
	cudaMalloc(&censusRa,(sizeof(unsigned int))*(right.cols-decreseX)*(right.rows-decreseY));
	cudaMalloc(&costK,(sizeof(int))*(left.cols-decreseX)*(left.rows-decreseY)*(maxDisparity+1));

	KernelDisparityCalculations<<<dimGrid,dimBlock>>>(BoxCostX,BoxCostY,censusLa,censusRa,left.cols-decreseX,left.rows-decreseY,leftC,rightC,costK);
	cudaDeviceSynchronize();


	//cudaMemcpy(CLK,censusLa,(sizeof(unsigned int))*(left.cols-decreseX)*(left.rows-decreseY),cudaMemcpyDeviceToHost);
	//cudaMemcpy(CRK,censusRa,(sizeof(unsigned int))*(left.cols-decreseX)*(left.rows-decreseY),cudaMemcpyDeviceToHost);

	//cout << "census ok" << endl;

	//Cost Computation
	int* cost = (int*)malloc((sizeof(int))*(left.cols-decreseX)*(left.rows-decreseY)*(maxDisparity+1));
	cudaMemcpy(cost,costK,(sizeof(int))*(left.cols-decreseX)*(left.rows-decreseY)*(maxDisparity+1),cudaMemcpyDeviceToHost);
	//CostComputation(leftBlack,rightBlack,cost,maxDisparity,left.cols,left.rows,BoxCostX,BoxCostY);
	//CostComputationCensus(CLK,CRK,cost,maxDisparity,left.cols-decreseX,left.rows-decreseY);
	//cout << "cost ok" << endl;
	cout << "Done CUDA " << endl;

	//Aggregate Cost
	int* L1 = (int*)malloc((sizeof(int))*(left.cols-decreseX)*(left.rows-decreseY)*(maxDisparity+1));
	AggregateCostCom(cost,L1,left.cols-decreseX,left.rows-decreseY,maxDisparity,-1,-1);
	cout << "Done" << endl;

	int* L2 = (int*)malloc((sizeof(int))*(left.cols-decreseX)*(left.rows-decreseY)*(maxDisparity+1));
	AggregateCostCom(cost,L2,left.cols-decreseX,left.rows-decreseY,maxDisparity,0,-1);
	cout << "Done" << endl;

	int* L3 = (int*)malloc((sizeof(int))*(left.cols-decreseX)*(left.rows-decreseY)*(maxDisparity+1));
	AggregateCostCom(cost,L3,left.cols-decreseX,left.rows-decreseY,maxDisparity,1,-1);
	cout << "Done" << endl;

	int* L4 = (int*)malloc((sizeof(int))*(left.cols-decreseX)*(left.rows-decreseY)*(maxDisparity+1));
	AggregateCostCom(cost,L4,left.cols-decreseX,left.rows-decreseY,maxDisparity,-1,0);
	cout << "Done" << endl;

	int* L5 = (int*)malloc((sizeof(int))*(left.cols-decreseX)*(left.rows-decreseY)*(maxDisparity+1));
	AggregateCostCom(cost,L5,left.cols-decreseX,left.rows-decreseY,maxDisparity,1,0);
	cout << "Done" << endl;

	int* L6 = (int*)malloc((sizeof(int))*(left.cols-decreseX)*(left.rows-decreseY)*(maxDisparity+1));
	AggregateCostCom(cost,L6,left.cols-decreseX,left.rows-decreseY,maxDisparity,-1,1);
	cout << "Done" << endl;

	int* L7 = (int*)malloc((sizeof(int))*(left.cols-decreseX)*(left.rows-decreseY)*(maxDisparity+1));
	AggregateCostCom(cost,L7,left.cols-decreseX,left.rows-decreseY,maxDisparity,0,1);
	cout << "Done" << endl;

	int* L8 = (int*)malloc((sizeof(int))*(left.cols-decreseX)*(left.rows-decreseY)*(maxDisparity+1));
	AggregateCostCom(cost,L8,left.cols-decreseX,left.rows-decreseY,maxDisparity,1,1);
	cout << "Done" << endl;




	//Disparity Selection
	Mat disparity = DisparitySelectionP(L1,L2,L3,L4,L5,L6,L7,L8,maxDisparity,left.cols-decreseX,left.rows-decreseY);
	gettimeofday(&timstr, NULL);
	double end = timstr.tv_sec + (timstr.tv_usec / 1000000.0);
	printf("Elapsed time Local Matching:\t\t\t%.6lf (s)\n", end - begin);


	namedWindow("SMG");
	imshow("SMG",disparity);
	imwrite("disparity.png",disparity);
	waitKey(0);


	//free Memory
	cudaFree(leftC);
	cudaFree(rightC);
	free(censusLa);
	free(censusRa);
	free(CLK);
	free(CRK);
	free(costK);
	free(cost);
	free(imageLeftA);
	free(imageRightA);
	free(cost);
	free(L1);
	free(L2);
	free(L3);
	free(L4);
	free(L5);
	free(L6);
	free(L7);
	free(L8);
}
//Disparity Selection Process
__host__ Mat DisparitySelectionP(int* L2, int* L4, int* L5, int* L7,int* L1, int* L3, int* L6, int* L8, int maxDisparity, int width, int length){
	Mat disparity(length,width,CV_8U);

	for(int y=0; y<length; y++){
		for(int x=0; x<width; x++){
			int costA = 99999;
			int disPix = 0;
			for (int d = 0; d<maxDisparity; d++){
				int sumAgg = L1[width*(y+d*length)+x] + L2[width*(y+d*(length))+x]+
						L3[width*(y+d*length)+x] + L4[width*(y+d*(length))+x] + L5[width*(y+d*(length))+x]+
						L6[width*(y+d*length)+x] + L7[width*(y+d*(length))+x] + L8[width*(y+d*length)+x];
				if (sumAgg < costA){
					costA = sumAgg;
					disPix = d;
				}
 			}
			//cout << "disparity pixel " <<x << " " << y << " is " << disPix << " with value "<< costA<< endl;
			disparity.at<uchar>(y,x) = disPix;
		}
	}
	return disparity;
}


//Gets the cost Computation Across all Paths
__host__ void AggregateCostCom(int* cost, int* L, int width, int length, int maxDisparity, int directionx, int directiony){
	//penalties
	int p1 = 5;
	int p2 = 100;

	int startX,startY, increaseX, increaseY;

	if(directionx <= 0){
		startX = 0;
		increaseX = 1;
	}
	else{
		startX = width-1;
		increaseX = -1;
	}
	if(directiony <= 0){
		startY = 0;
		increaseY = 1;
	}
	else{
		startY = length-1;
		increaseY = -1;
	}

	int* minimuns = (int*)malloc((sizeof(int))*(width)*(length));
	int x;
	int y = startY;

	for(int yC=0; yC<length; yC++){
		int influenceY = y + directiony;
		x = startX;
		for(int xC=0; xC<width; xC++){
			int influenceX = x + directionx;

			int minimunValue = 999999;

			if(influenceX > width || influenceX < 0 || influenceY > length || influenceY < 0){
				for (int d = 0; d<maxDisparity; d++){
					int costPixel = cost[width*(y+d*(length))+x];
					L[width*(y+d*(length))+x] = costPixel;
					if(costPixel < minimunValue){
						minimunValue = costPixel;
						//cout << "minimum ->  " << costPixel;
					}
				}
			}
			else{
				/*cout << x << " " << y <<endl;
				waitKey(0);*/
				for (int d = 0;  d<maxDisparity; d++){
					int costPixel = cost[width*(y+d*(length))+x];
					int currentD = L[width*(influenceY+d*(length))+influenceX];
					int previousD= (d-1>=0)? L[width*(influenceY+(d-1)*(length))+influenceX]: 8888;
					int nextD    = (d+1<maxDisparity)? L[width*(influenceY+(d+1)*(length))+influenceX] : 8888;

					//int minValue  = minArray(L,influenceX,influenceY,maxDisparity);
					int minValue =minimuns[influenceY*width+influenceX];
					int valueToAssign =  costPixel + minBetweenNumbersInt(currentD,nextD+p1,previousD+p1,minValue+p2) - minValue;

					L[width*(y+d*(length))+x] = valueToAssign;
					if(valueToAssign < minimunValue){
						minimunValue = valueToAssign;
					}

					/*if(valueToAssign <= 0){
						cout << x<<" "<<y<<"\t influence "<< influenceX << " " << influenceY << endl;
						cout << "Pixels: "<<x<<"-"<<y<<"\t"<<"Formula: " <<costPixel << "\t + min["<< currentD << "\t"<< previousD << "\t"
							<<nextD<<"\t"<<minValue<< "]" <<"\t Result Value: " <<valueToAssign<< "\t"<< d <<endl;
						waitKey(0);
					}*/
				}
				minimuns[y*width+x] = minimunValue;
			}
			x = x + increaseX;
		}
		y = y + increaseY;
	}


	cout << "done Aggregate Cost direction "<<directionx << " " << directiony << endl;
	free(minimuns);
}


__host__ int minBetweenNumbersInt(int a, int b, int c, int d){
	int min = a;

	if (min > b){
		min = b;
	}
	if (min > c){
		min = c;
	}
	if (min > d){
		min = d;
	}
	return min;
}


//Testing Census---------------
__host__ void CostComputationCensus (unsigned int* censusL, unsigned int* censusR, int* cost, int maxDisparity, int width, int length){
	for(int y=0;y<length;y++){
		for(int xl=0;xl<width;xl++){
			int start = xl-(maxDisparity);

			unsigned int valueLeft = censusL[y*width+xl];
			//cout << "Value Left " << valueLeft <<endl;
			for(int xr = start; xr<=xl; xr++){
				int valueToAssigned;
				int dis = xl-xr;
				//cout << dis<<" "<<xr << endl;
				if(xr>=0){
					unsigned int valueRight = censusR[y*width+xr];
					//cout << "Value Right " << valueRight << endl;
					valueToAssigned = HammingDistanceNumbers(valueLeft,valueRight);
				}
				else{
					valueToAssigned = 99999;
				}
				cost[width*(y+dis*(length))+xl] =valueToAssigned;

			}
		}
	}
}

__host__ int HammingDistanceNumbers (unsigned int a, unsigned int b){
	unsigned int val = a ^ b;
	int dist = 0;
	while(val != 0){
		val = val & (val-1);
		dist++;
	}
	return dist;
}


__host__ void DivideImagesCam(Mat completeImage, Mat* left, Mat* right){
	*left = completeImage(Rect(0,0,WIDTHIMAGE,LENGTHIMAGE));
	*right = completeImage(Rect(1280,0,WIDTHIMAGE,LENGTHIMAGE));
}

