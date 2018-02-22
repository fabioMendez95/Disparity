#include<vector>
#include<stdio.h>
#include "opencv2/core/core.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/contrib/contrib.hpp>
#include"Fusion.h"

using namespace std;
using namespace cv;

void Fusion::radarPointsInImage(Mat disparitySrc, vector<double> radarToSB){
	Mat disparity = getFilterImage(disparitySrc);

	Mat display;
	double minVal, maxVal;
	minMaxIdx(disparitySrc, &minVal, &maxVal);
	disparity.convertTo(display, CV_8UC3, 255 / (maxVal), minVal);

	radarLine = disparity.rows / 2;
	vector<double> local = radarToSB;
	objects.clear();

	for(int x = 0; x < disparity.cols; x++){
		double Z = 3.36 / (double)((int)disparity.at<uchar>(radarLine,x)+1);
		int num = 0;
		//cout <<"\nZ: " << Z <<" Size: " << local.size()  <<endl;
		for(double distance : local){
			//cout << distance << " ";
			if(Z > distance - errorValue && Z < distance+errorValue){
				//cout << "Match " << x << ": "<<((int)disparity.at<uchar>(radarLine,x)) << endl;
				int width = widthOfObject(disparity,x);
				ObjectDR obj = getObjectDimensions(disparity,width,x);
				local.erase(local.begin()+num);
				objects.push_back(obj);

				circle(display,Point(x,radarLine),5,Scalar(255,0,0),-1,3);
				line(display,Point(x,radarLine),Point(x+width,radarLine),Scalar(0,0,0),2);
				line(display,Point(x+width/2,radarLine-obj.length/2),Point(x+width/2,radarLine+obj.length/2),Scalar(0,0,0),2);
				num ++;
				x = x + width;
				break;
			}
		}
	}
	applyColorMap(display, display, COLORMAP_HOT);
	resize(display, display, size);
	namedWindow("Radar and Disparity");
	imshow("Radar and Disparity",display);
}

Mat Fusion::displayOnImage(Mat& image){
	Mat result;
	image.copyTo(result);
	for(ObjectDR object : objects){
		int newXC = object.centre.x*xRep;
		int newYC = object.centre.y*yRep;
		if(object.width > 2){
			Point pt1(newXC-object.width*xRep/2,newYC-object.length*yRep/2);
			Point pt2(newXC+object.width*xRep/2,newYC+object.length*yRep/2);
			rectangle(result,pt1,pt2,Scalar(0,255,0),2);
		}
		//circle(result,Point(newX,radarLine*yRep),5,Scalar(255,0,0),-1,3);
	}
	return result;
}

vector<ObjectDR> Fusion::getObjects(){
	return objects;
}

ObjectDR Fusion::getObjectDimensions(Mat disparity, int width, int x){
	ObjectDR result; //width, length, Point centre
	int midObject = x + width/2;
	int disparityValue = disparity.at<uchar>(radarLine,midObject); //maybe use x

	bool checkUp = true;
	bool checkDown = true;
	result.centre.x = midObject;
	result.width = width;

	int ycD = radarLine+1;
	int ycU = radarLine-1;
	int length = 0;
	int centreY = radarLine;
	while(ycD < disparity.rows && ycU > 0){
		int disparityValueUp = disparity.at<uchar>(ycU,midObject);
		int disparityValueDown = disparity.at<uchar>(ycD,midObject);
		if ((disparityValue < disparityValueUp + pixelError && disparityValue > disparityValueUp - pixelError) && checkUp) {
			length ++;
			centreY --;
		}
		else{
			checkUp = false;
		}
		if ((disparityValue < disparityValueDown + pixelError && disparityValue > disparityValueDown - pixelError) && checkDown) {
			width = width + 1;
			length ++;
			centreY ++;
		}
		else{
			checkDown = false;
		}
		if(!checkDown && !checkUp){
			break;
		}
		ycD ++;
		ycU --;
	}
	result.length = length;
	result.centre.y = centreY;

	return result;
}

int Fusion::widthOfObject(Mat disparity, int x){
	int disparityValue = disparity.at<uchar>(radarLine,x);
	int width = 0;
	for(int cx = x+1; cx<xDown; cx ++){
		int compare = (int)disparity.at<uchar>(radarLine,cx);
		cout << x << ": "<<disparityValue <<" " << compare << endl;
		if((disparityValue < compare + pixelError && disparityValue > compare - pixelError)){
			width = width+1;
		}
		else{
			break;
		}
	}
	return width;
}

Mat Fusion::getFilterImage(Mat disparitySrc){
	Mat disparity2;
	bilateralFilter(disparitySrc,disparity2,9,20,20);
	return disparity2;
}

void Fusion::setValuesFusion(int minX, int minY, Size s){
	xUp = s.width;
	xDown = minX;
	yUp = s.height;
	yDown = minY;

	yRep = s.height/minY;
	xRep = s.width/minX;

	size.height = s.height;
	size.width = s.width;
}
