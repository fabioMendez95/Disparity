#include <stdio.h>
#include <iostream>
#include <string.h>
#include <limits>

#include "opencv2/core/core.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/contrib/contrib.hpp>
#include"string"

#include<sys/resource.h>
#include<time.h>
#include<sys/time.h>


#include "Comparison.h"
#include "Kernels.cuh"
#include "ZedCamera.h"
#include "Radar.h"

using namespace std;
using namespace cv;

#define threadx 16
#define thready 16

#define USECAMARA false
#define Camara 1
#define WIDTHIMAGE 1280
#define LENGTHIMAGE 720

__host__ void SGM();
__host__ Mat DisparitySelectionP(int* L1, int* L2, int* L3, int* L4,/*int* L5, int* L6, int* L7, int* L8,*/ int width, int length);
__host__ void DivideImagesCam(Mat completeImage, Mat* left, Mat* right);
__host__ Mat DisparitySelectionOneArray(int*L, int width, int length);

__host__ String numberOfZeros(int number);
__host__ String getImageLocation(int frame, String side);

__host__ pointCoo getPoint(int ID, int pathX, int pathY, int width, int length);
__host__ void getKernelInitialInformation(int pathNumber, startInfo* pixelDi, int width, int length);

//sl::Camera zed;

int main (int argc, char** argv){
	//Comparison Calculations
	//Comparison comp;
	//comp.CompareDisparities();
	//------------------------

	//Radar info
/*	Radar radar;
	int samples = 100;
	radar.startRadar();
	while(samples > 0){
		bool correctlyRead = radar.readInfo();
		while(correctlyRead == 0){
			bool correctlyRead = radar.readInfo();
		}
		waitKey(50);
		samples = samples - 1;
	}
	radar.closeRadar();*/


	SGM();
	return 0;
}

__host__ void SGM(){
	int frameNumber = 30;
	int frame = 0;

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
	ZedCamera zed;
#if USECAMARA
	zed.initCamera();
	zed.grabImage();
	left = zed.getLeftImage();
	right = zed.getRightImage();
	/*char key = ' ';
	while(key != 'q'){
		zed.grabImage();
		left = zed.getLeftImage();
		right = zed.getRightImage();
		namedWindow("left");
		namedWindow("right");
		imshow("left", left);
		imshow("right", right);
		key = waitKey(10);
	}

	namedWindow("left");
	namedWindow("right");
	imshow("left", left);
	imshow("right", right);*/

#else
	//Reading Images
	left = imread("Images/KITTY/left/0000000000.png", CV_LOAD_IMAGE_COLOR);
	right= imread("Images/KITTY/right/0000000000.png", CV_LOAD_IMAGE_COLOR);
#endif

	//Initialisation Parameters---------------
	//First Kernel Params
	imageLeftA = (uchar*)malloc((sizeof(uchar))*(left.cols)*(left.rows));
	imageRightA= (uchar*)malloc((sizeof(uchar))*(right.cols)*(right.rows));

	int decreseX = BoxCostX/2 + BoxCostX/2;
	int decreseY = BoxCostY/2 + BoxCostY/2;

	int dimX = ((left.cols-decreseX) / threadx);
	int dimY = ((left.rows-decreseY) / thready);
	cout <<"Dimensions Grid: " << dimX << " " << dimY << endl;
	cout << "Dimensions Block: " << threadx << " " <<thready << endl;

	dim3 dimGrid(dimX,dimY);
	dim3 dimBlock(threadx,thready);
	//First Kernel, Census and cost Computation


	//Second Kernel Params
	int widthR = left.cols-decreseX;
	int lengthR = left.rows-decreseY;
	cout << "Disparity Size : " << widthR << " " << lengthR << endl;

	int threadNum = 258; // pathNumber is divisible by this, 437 blocks
	int pathNumber =/*(widthR+lengthR-1)*4 + */widthR *2 + lengthR*2;
	dim3 dimGrid2(pathNumber/threadNum);
	dim3 dimBlock2(threadNum);
	//Assigning Paths

	//Setting Up initial Info, this is done just once in the algorithm.


	int* L1S = (int*) malloc((sizeof(int)) * (left.cols - decreseX) * (left.rows - decreseY)* (maxDisparity + 1));
	//Done Initialisation Parameters----------

	//--------------------------------------------------------------------------------------------------
	//----------------------------Main Loop-------------------------------------------------------------
	//--------------------------------------------------------------------------------------------------
	while (frame < frameNumber || USECAMARA) {
		int* L1;
		cout <<"CUDA malloc " <<cudaMalloc(&L1,(sizeof(int)*(widthR)*(lengthR)*(maxDisparity+1)))<<endl;
		cout << "CUDA malloc 1: "<<cudaMalloc(&costK,(sizeof(int))*(left.cols-decreseX)*(left.rows-decreseY)*(maxDisparity+1)) << endl;
		cudaMalloc(&censusLa,(sizeof(unsigned int))*(left.cols-decreseX)*(left.rows-decreseY));
		cudaMalloc(&censusRa,(sizeof(unsigned int))*(right.cols-decreseX)*(right.rows-decreseY));


		startInfo* initialInfo, *initialInfoToKernel;
		initialInfo = (startInfo*) malloc((sizeof(startInfo)) * pathNumber);
		getKernelInitialInformation(pathNumber, initialInfo, widthR, lengthR);
		cudaMalloc(&initialInfoToKernel, (sizeof(startInfo)) * pathNumber);
		cudaMemcpy(initialInfoToKernel, initialInfo, (sizeof(startInfo)) * pathNumber, cudaMemcpyHostToDevice);

#if USECAMARA
		zed.grabImage();
		left = zed.getLeftImage();
		right = zed.getRightImage();
#else
		left = imread(getImageLocation(frame,"left"), CV_LOAD_IMAGE_COLOR);
		right= imread(getImageLocation(frame,"right"), CV_LOAD_IMAGE_COLOR);
#endif
		//Timing
		gettimeofday(&timstr, NULL);
		double begin = timstr.tv_sec + (timstr.tv_usec / 1000000.0);
		if (waitKey(1) >= 1) {
			cout << "Stoped at frame " << frame << endl;
			break;
		}

		namedWindow("left");
		namedWindow("right");
		imshow("left", left);
		imshow("right", right);
		cout << " here frame" << frame << endl;

		//Converting Images
#if USECAMARA
		Mat leftBlack;
		left.copyTo(leftBlack);
		Mat rightBlack;
		right.copyTo(rightBlack);
		namedWindow("leftB");
		namedWindow("rightB");
		//This line is wrong, not showing the correct image
		imshow("leftB", left);
		imshow("rightB", right);

		imageLeftA = left.data;
		imageRightA = right.data;
#else
		Mat leftBlack;
		cvtColor(left, leftBlack, CV_BGR2GRAY);
		Mat rightBlack;
		cvtColor(right, rightBlack, CV_BGR2GRAY);
		imageLeftA = leftBlack.data;
		imageRightA = rightBlack.data;
#endif
		cudaMalloc(&leftC, (sizeof(uchar)) * (left.cols) * (left.rows));
		cudaMalloc(&rightC, (sizeof(uchar)) * (left.cols) * (left.rows));
		cudaMemcpy(leftC, imageLeftA, (sizeof(uchar)) * (left.cols) * (left.rows),cudaMemcpyHostToDevice);
		cudaMemcpy(rightC, imageRightA,(sizeof(uchar)) * (left.cols) * (left.rows),cudaMemcpyHostToDevice);
		//Done COnverting Images

		gettimeofday(&timstr, NULL);
		double begin1 = timstr.tv_sec + (timstr.tv_usec / 1000000.0);
		//CensusAndCostKernel<<<dimGrid,dimBlock>>>(BoxCostX,BoxCostY,widthR,lengthR,leftC,rightC,costK); //Not running
		KernelDisparityCalculations<<<dimGrid,dimBlock>>>(BoxCostX,BoxCostY,censusLa,censusRa,left.cols-decreseX,left.rows-decreseY,leftC,rightC,costK);
		cout << "Synchronise status kernel 1: " << cudaDeviceSynchronize()<< "\n";
		//------------Done-First-Kernel-----------------------------
		gettimeofday(&timstr, NULL);
		double end1 = timstr.tv_sec + (timstr.tv_usec / 1000000.0);
		printf("Elapsed time Census and Cost:\t\t\t%.6lf (s)\n", end1 - begin1);

		gettimeofday(&timstr, NULL);
		double begin2 = timstr.tv_sec + (timstr.tv_usec / 1000000.0);
		//Second Kernel, Semi global matching and disparity Selection

		KernelSemiGlobal<<<dimGrid2,dimBlock2>>>(costK,widthR,lengthR,initialInfoToKernel,L1);
		cout << "Synchronise status kernel 2: " << cudaDeviceSynchronize() << "\n";
		//------------Done-Second-Kernel-----------------------------

		gettimeofday(&timstr, NULL);
		double end2 = timstr.tv_sec + (timstr.tv_usec / 1000000.0);
		printf("Elapsed time Aggregate Cost:\t\t\t%.6lf (s)\n", end2 - begin2);

		cout << "Done CUDA " << endl;

		cudaMemcpy(L1S, L1,(sizeof(int)) * (left.cols - decreseX) * (left.rows - decreseY)* (maxDisparity + 1), cudaMemcpyDeviceToHost);

		cout << "Path Number " << pathNumber << endl;

		//Disparity Selection--------------------------------------------------------------------------------------------
		Mat disparity = DisparitySelectionOneArray(L1S, widthR, lengthR);

		//---------------------------------------------------------------------------------------------------------------

		gettimeofday(&timstr, NULL);
		double end = timstr.tv_sec + (timstr.tv_usec / 1000000.0);
		printf("Elapsed time Disparity:\t\t\t%.6lf (s)\n", end - begin);
		namedWindow("SMG");
		Mat display;
		double minVal, maxVal;
		minMaxIdx(disparity, &minVal, &maxVal);
		disparity.convertTo(display, CV_8UC3, 255 / (maxVal - minVal), -minVal);
		applyColorMap(display,display,COLORMAP_JET);
		imshow("SMG", display);
		//imwrite("disparit.png",disparity);
		frame ++;

		/*cudaFree(leftC);
		cudaFree(rightC);
		cudaFree(censusLa);
		cudaFree(censusRa);
		cudaFree(costK);
		cudaFree(L1);
		cudaFree(initialInfoToKernel);*/
		free(initialInfo);
		cudaDeviceReset();
	}
	//--------------------------------------------------------------------------------------------------
	//----------------------------Main Loop-------------------------------------------------------------
	//--------------------------------------------------------------------------------------------------

	//free Memory
	free(L1S);
	//free(imageLeftA);
	//free(imageRightA);


#if USECAMARA
	zed.closeCamera();
#endif
}


__host__ pointCoo getPoint(int ID, int pathX, int pathY, int width, int length){
	pointCoo point;
	int startX;
	int startY;
	//Initial Point, this is the corner of the diagonals
	if(ID == 0){
		startX = 0;
		startY = 0;
		if(pathX == -1){
			startX = width - 1;
		}
		if(pathY == -1){
			startY = length - 1;
		}
	}
	//As diagonals go through the width and length, this needs to be divided into two segments
	//Segment 1 across the x axis of the image, fix y coordinate of the image
	else if (ID < width){
		startX = ID;
		startY = 0;
		if(pathX == -1){
			startX = startX - 1;
		}
		if(pathY == -1){
			startY = length-1;
		}
	}
	//Segment 2 across the y axis of the image, fix x coordinate
	else if (ID < width + length - 1){
		int newID = ID - width;
		startX = 0;
		startY = newID;
		if (pathX == -1){
			startX = width -1;
		}
		if (pathY == -1){
			startY = startY -1;
		}
	}

	point.x = startX;
	point.y = startY;
	return point;
}

__host__ void getKernelInitialInformation(int pathNumber, startInfo* pixelDi, int width, int length) {
	for (int blockID = 0; blockID < pathNumber; blockID++) {
		int ID = blockID;
		int LA = blockID; // Location in Array

		if (blockID < width) {
			pixelDi[LA].startX = ID;
			pixelDi[LA].startY = 0;
			pixelDi[LA].directionX = 0;
			pixelDi[LA].directionY = 1;
		} else if (blockID < 2 * width) {
			ID = blockID - width;
			pixelDi[LA].startX = ID;
			pixelDi[LA].startY = length - 1;
			pixelDi[LA].directionX = 0;
			pixelDi[LA].directionY = -1;
		} else if (blockID < 2 * width + length) {
			ID = blockID - 2 * width;
			pixelDi[LA].startX = 0;
			pixelDi[LA].startY = ID;
			pixelDi[LA].directionX = 1;
			pixelDi[LA].directionY = 0;
		} else if (blockID < 2 * width + 2 * length) {
			ID = blockID - 2 * width - length;
			pixelDi[LA].startX = width - 1;
			pixelDi[LA].startY = ID;
			pixelDi[LA].directionX = -1;
			pixelDi[LA].directionY = 0;
		} else if (blockID < 2 * width + 2 * length + (width + length - 1)) {
			cout << "Should not happen \n";
			ID = blockID - 2 * width - 2 * length;
			pixelDi[LA].directionX = 1;
			pixelDi[LA].directionY = 1;
			pointCoo point = getPoint(ID, 1, 1, width, length);
			pixelDi[LA].startX = point.x;
			pixelDi[LA].startY = point.y;
		} else if (blockID < 2 * width + 2 * length + 2 * (width + length - 1)) {
			ID = blockID - 2 * width - 2 * length - (width + length - 1);
			pixelDi[LA].directionX = -1;
			pixelDi[LA].directionY = -1;
			pointCoo point = getPoint(ID, -1, -1, width, length);
			pixelDi[LA].startX = point.x;
			pixelDi[LA].startY = point.y;
		} else if (blockID < 2 * width + 2 * length + 3 * (width + length - 1)) {
			ID = blockID - 2 * width - 2 * length - 2 * (width + length - 1);
			pixelDi[LA].directionX = 1;
			pixelDi[LA].directionY = -1;
			pointCoo point = getPoint(ID, 1, -1, width, length);
			pixelDi[LA].startX = point.x;
			pixelDi[LA].startY = point.y;
		} else if (blockID < 2 * width + 2 * length + 4 * (width + length - 1)) {
			ID = blockID - 2 * width - 2 * length - 3 * (width + length - 1);
			pixelDi[LA].directionX = -1;
			pixelDi[LA].directionY = 1;
			pointCoo point = getPoint(ID, -1, 1, width, length);
			pixelDi[LA].startX = point.x;
			pixelDi[LA].startY = point.y;
		} else {
			printf("ifs are wrong \n");
		}
	}
}


//Disparity Selection Process
__host__ Mat DisparitySelectionP(int* L1, int* L2, int* L3, int* L4/*,int* L5, int* L6, int* L7, int* L8*/, int width, int length){
	Mat disparity(length,width,CV_8U);

	for(int y=0; y<length; y++){
		for(int x=0; x<width; x++){
			int costA = 99999;
			int disPix = 0;
			for (int d = 0; d<maxDisparity; d++){
				int sumAgg = L1[width*(y+d*length)+x] + L2[width*(y+d*(length))+x]+
						L3[width*(y+d*length)+x] + L4[width*(y+d*(length))+x] /*+ L5[width*(y+d*(length))+x]+
						L6[width*(y+d*length)+x] + L7[width*(y+d*(length))+x] + L8[width*(y+d*length)+x]*/;
				if(x == 100 && y == 369 && d == 99){
					cout << x << " " << y<< " : "<<L1[width*(y+d*length)+x] << endl;
					cout << width << " " << length << endl;
				}
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

__host__ Mat DisparitySelectionOneArray(int*L, int width, int length){
	Mat disparity(length,width,CV_8U);
	for(int y=0;y<length;y++){
		for(int x=0;x<width;x++){
			int costA = 99999;
			int disPix = 0;
			for (int d=0;d<maxDisparity;d++){
				int value = L[width*(y+d*length)+x];
				if(value < costA){
					costA = value;
					disPix = d;
				}
			}
			disparity.at<uchar>(y,x) = disPix;
		}
	}
	return disparity;
}

__host__ void DivideImagesCam(Mat completeImage, Mat* left, Mat* right){
	*left = completeImage(Rect(0,0,WIDTHIMAGE,LENGTHIMAGE));
	*right = completeImage(Rect(1280,0,WIDTHIMAGE,LENGTHIMAGE));
}


//side = left, right
__host__ String getImageLocation(int frame, String side){
	//"Images/KITTY/left/0000000000.png"
	String imageLocation = "Images/KITTY/"+side+"/"+numberOfZeros(frame);
	ostringstream convert;
	convert << frame;
	imageLocation = imageLocation + convert.str() +  ".png";
	return imageLocation;
}

__host__ String numberOfZeros(int number){
	if(number < 10){
		return "000000000";
	}
	else if (number < 100){
		return "00000000";
	}
	else if (number < 1000){
		return "0000000";
	}
	else{
		return "000000";
	}
}


