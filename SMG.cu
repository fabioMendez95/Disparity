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

#include<mutex>
#include<thread>

#include "Comparison.h"
#include "Kernels.cuh"
//#include "ZedCamera.h"
#include "Radar.h"
#include "Camera.h"

using namespace std;
using namespace cv;

#define threadx 16
#define thready 16

#define USERADAR true

#define SAVEIMAGE false
#define Profile true

#define DISPLAY true

#define USECAMARA true
#define Paths8 true
#define Camara 1
#define WIDTHIMAGE 1280
#define LENGTHIMAGE 720

__host__ void SGM();
__host__ Mat DisparityCreation(int* imageFromKernel, int width, int len);

__host__ String numberOfZeros(int number);
__host__ String getImageLocation(int frame, String side);

__host__ pointCoo getPoint(int ID, int pathX, int pathY, int width, int length);
__host__ void getKernelInitialInformation(int pathNumber, startInfo* pixelDi, int width, int length);
__host__ void radarThread();

bool readRadar = true;
mutex radarMtx;

int main (int argc, char** argv){
	//Comparison Calculations
	//Comparison comp;
	//comp.CompareDisparities();
	//------------------------

	SGM();
	return 0;
}

__host__ void radarThread(){
	/*bool correctlyRead = radar.readInfo();
	while (correctlyRead == 0) {
		correctlyRead = radar.readInfo();
	}*/
	Radar radar;
	radar.startRadar();
	bool copyReadRadar = true;
	while(copyReadRadar){
		radar.readInfo();
		radarMtx.lock();
		copyReadRadar = readRadar;
		radarMtx.unlock();
	}
	radar.closeRadar();
}

__host__ void SGM(){
	int frameNumber = 30;
	int frame = 0;
	Size size(720,400);
	cout << "Starting process \n";

	//Radar Info
#if USERADAR
	thread thrRadar(radarThread);
#endif
	struct timeval timstr;
	struct timeval timstrTotal;
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

	cout <<"Initialising camera\n";

#if USECAMARA
/*	ZedCamera zed;
	zed.initCamera();
	zed.grabImage();
	left = zed.getLeftImage();
	right = zed.getRightImage();*/
	Camera cam;
	cam.initCamera(1);
	cam.extractImage();
	right = cam.getRight();
	left = cam.getLeft();

#else
	//Reading Images
	left = imread("Images/KITTY/left/0000000000.png", CV_LOAD_IMAGE_COLOR);
	right= imread("Images/KITTY/right/0000000000.png", CV_LOAD_IMAGE_COLOR);
#endif

	cout << "Camera ready \n";
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

#if USECAMARA && Paths8
	int threadNum = 382;
#elif USECAMARA
	int threadNum = 255;
#else
	int threadNum = 458; // pathNumber is divisible by this, 437 blocks
#endif

#if Paths8
	int pathNumber =(widthR+lengthR-1)*4 + widthR *2 + lengthR*2;
#else
	int pathNumber = widthR *2 + lengthR*2;
#endif
	dim3 dimGrid2(pathNumber/threadNum);
	dim3 dimBlock2(threadNum);
	//Assigning Paths

	//Setting Up initial Info, this is done just once in the algorithm.


	int* L1S = (int*) malloc((sizeof(int)) * (left.cols - decreseX) * (left.rows - decreseY)* (maxDisparity));
	//Done Initialisation Parameters----------


	startInfo* initialInfo, *initialInfoToKernel;
	initialInfo = (startInfo*) malloc((sizeof(startInfo)) * pathNumber);
	getKernelInitialInformation(pathNumber, initialInfo, widthR, lengthR);
	cudaMalloc(&initialInfoToKernel, (sizeof(startInfo)) * pathNumber);
	cudaMemcpy(initialInfoToKernel, initialInfo, (sizeof(startInfo)) * pathNumber, cudaMemcpyHostToDevice);
	cudaMalloc(&leftC, (sizeof(uchar)) * (left.cols) * (left.rows));
	cudaMalloc(&rightC, (sizeof(uchar)) * (left.cols) * (left.rows));
	cudaMalloc(&censusLa,(sizeof(unsigned int))*(left.cols-decreseX)*(left.rows-decreseY));
	cudaMalloc(&censusRa,(sizeof(unsigned int))*(right.cols-decreseX)*(right.rows-decreseY));
	cout << "CUDA malloc 1: "<<cudaMalloc(&costK,(sizeof(int))*(left.cols-decreseX)*(left.rows-decreseY)*(maxDisparity)) << endl;

	int* disKernel;
	int* disFromKernel = (int*) malloc((sizeof(int)) * (widthR) * (lengthR));
	int errorCUDAMALLOCdis = cudaMalloc(&disKernel,(sizeof(int)*(widthR)*(lengthR)));
	cout << "Malloc dis: " << errorCUDAMALLOCdis << endl;
	//Timing Total
	gettimeofday(&timstrTotal, NULL);
	double beginTotal = timstrTotal.tv_sec + (timstrTotal.tv_usec / 1000000.0);


	int* L1;
	int errorCUDAMALLOC1 = cudaMalloc(&L1,(sizeof(int)*(widthR)*(lengthR)*(maxDisparity)));
#if Profile
	cout <<"CUDA malloc " << errorCUDAMALLOC1 <<endl;
#endif

	cout << "\n\nStarting main loop \n";
	//--------------------------------------------------------------------------------------------------
	//----------------------------Main Loop-------------------------------------------------------------
	//--------------------------------------------------------------------------------------------------
	while (frame < frameNumber || USECAMARA) {
		//Stop Condition
		if (waitKey(1) >= 1) {
			cout << "Stoped at frame " << frame << endl;
			break;
		}
		//Read Image---------------------------------------
#if USECAMARA
/*		zed.grabImage();
		left = zed.getLeftImage();
		right = zed.getRightImage();*/
		cam.extractImage();
		right = cam.getRight();
		left = cam.getLeft();
#else
		left = imread(getImageLocation(frame,"left"), CV_LOAD_IMAGE_COLOR);
		right= imread(getImageLocation(frame,"right"), CV_LOAD_IMAGE_COLOR);
#endif
		//Done Read Image----------------------------------

#if Profile
		//Timing
		gettimeofday(&timstr, NULL);
		double begin = timstr.tv_sec + (timstr.tv_usec / 1000000.0);
#endif
		//Converting Images--------------------------------
#if USECAMARA
/*		imag launching thread then read from iteLeftA = left.data;
		imageRightA = right.data;*/
		Mat leftBlack;
		cvtColor(left, leftBlack, CV_BGR2GRAY);
		Mat rightBlack;
		cvtColor(right, rightBlack, CV_BGR2GRAY);
		imageLeftA = leftBlack.data;
		imageRightA = rightBlack.data;
#if DISPLAY
		namedWindow("left");
		imshow("left",left);
#endif
#else
		Mat leftBlack;
		cvtColor(left, leftBlack, CV_BGR2GRAY);
		Mat rightBlack;
		cvtColor(right, rightBlack, CV_BGR2GRAY);
		imageLeftA = leftBlack.data;
		imageRightA = rightBlack.data;
#endif
		int errorMemCpy1 = cudaMemcpy(leftC, imageLeftA, (sizeof(uchar)) * (left.cols) * (left.rows),cudaMemcpyHostToDevice);
		int errorMemCpy2 = cudaMemcpy(rightC, imageRightA,(sizeof(uchar)) * (left.cols) * (left.rows),cudaMemcpyHostToDevice);
#if Profile
		cout<< "Cuda copy1: " << errorMemCpy1 << endl;
		cout<< "Cuda copy2: " << errorMemCpy2 << endl;
		//Done converting images---------------------------
#endif
		//------------First kernel cost Computation----------------------
		KernelDisparityCalculations<<<dimGrid,dimBlock>>>(BoxCostX,BoxCostY,censusLa,censusRa,widthR,lengthR,leftC,rightC,costK,L1,disKernel); //Old copy L1
		//------------Done-First-Kernel-----------------------------

		Mat disparity = DisparityCreation(disFromKernel,widthR,lengthR);
#if DISPLAY
		//Display Logic----------------------------------
		namedWindow("SMG");
		Mat display;
		disparity.convertTo(display, CV_8UC3, 255 / (maxDisparity), 0);
		applyColorMap(display,display,COLORMAP_HOT);
		resize(display,display,size);
		imshow("SMG", display);
		//Done Display Logic-----------------------------
#endif
		//----> Radar point fetch should be done here <---- TODO
#if USERADAR
		/*bool correctlyRead = radar.readInfo();
		while (correctlyRead == 0) {
			correctlyRead = radar.readInfo();
		}*/
#endif
		//-----------------------------------------------------



		//Second Kernel, Semi global matching and disparity Selection---------
		KernelSemiGlobal<<<dimGrid2,dimBlock2>>>(costK,widthR,lengthR,initialInfoToKernel,L1); //new Copy L1

		int syncStatus = cudaDeviceSynchronize();
#if Profile
		if(syncStatus > 0){
			cout << "Error Kernel synchronise \t\tError type: " << syncStatus << "\n";
			cout << cudaGetErrorString(cudaGetLastError()) << endl;
		}
#endif
		//------------Done-Second-Kernel-----------------------------
		//----> Junction of information should be done here <---- TODO


		//int errorMemCpy3 = cudaMemcpy(L1S, L1,(sizeof(int)) * (left.cols - decreseX) * (left.rows - decreseY)* (maxDisparity + 1), cudaMemcpyDeviceToHost);
		int errorMemCpy3 = cudaMemcpy(disFromKernel, disKernel,(sizeof(int)) * (widthR) * (lengthR), cudaMemcpyDeviceToHost);
#if Profile
		cout<< "Cuda copy3: " << errorMemCpy3 << endl;
		cout << "Path Number " << pathNumber << endl;
#endif
		//Disparity Selection--------------------------------------------------------------------------------------------
		//Mat disparity = DisparitySelectionOneArray(L1S, widthR, lengthR);
		//Mat disparity(lengthR,widthR,CV_8U,disFromKernel);

		//---------------------------------------------------------------------------------------------------------------
#if Profile
		gettimeofday(&timstr, NULL);
		double end = timstr.tv_sec + (timstr.tv_usec / 1000000.0);
		printf("Elapsed time Disparity:\t\t\t%.6lf (s)\n", end - begin);
#endif

#if SAVEIMAGE
		ostringstream imageSaveLocation;
		imageSaveLocation << "Results/"<<frame << ".png";
		imwrite(imageSaveLocation.str(),left);
#endif

		frame ++;
	}
	//--------------------------------------------------------------------------------------------------
	//----------------------------Main Loop-------------------------------------------------------------
	//--------------------------------------------------------------------------------------------------

	gettimeofday(&timstrTotal, NULL);
	double endTotal = timstrTotal.tv_sec + (timstrTotal.tv_usec / 1000000.0);
	printf("\n\nElapsed time:\t\t\t%.6lf (s)\n", endTotal - beginTotal);
	printf("Frames Analysed:\t\t%d frames\n",frame);
	printf("Frames per second:\t\t%f \n",frame/(endTotal-beginTotal));



#if USERADAR
	radarMtx.lock();
	readRadar = false;
	radarMtx.unlock();
	thrRadar.join();
#endif

#if USECAMARA
	//zed.closeCamera();
	cam.close();
#endif

	free(disFromKernel);
	free(initialInfo);
	free(L1S);
	cudaDeviceReset();

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
		}

		else if (blockID < 2 * width + 2 * length + (width + length - 1)) {
			//cout << "Should not happen \n";
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

__host__ Mat DisparityCreation(int* imageFromKernel, int width, int len){
	Mat disparity(len,width,CV_8U);
	for(int y=0; y<len; y++){
		for(int x=0; x<width; x++){
			disparity.at<uchar>(y,x) = imageFromKernel[y*width+x];
		}
	}
	return disparity;
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


