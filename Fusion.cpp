#include<vector>
#include<stdio.h>
#include "opencv2/core/core.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/contrib/contrib.hpp>
#include"Fusion.h"
#include<math.h>

using namespace std;
using namespace cv;

void Fusion::pointMatchOnImage(Mat disparitySrc, vector<double> radarToSB, vector<double> displacements){
	Mat disparity = getFilterImage(disparitySrc);
	Mat display;

	//vector<DivisionImage> divisions = getDisivionOnImage(disparity);
	objects.clear();

	if (showFusionProcess) {
		double minVal, maxVal;
		minMaxIdx(disparitySrc, &minVal, &maxVal);
		disparity.convertTo(display, CV_8UC3, 255 / (maxVal), minVal);
	}
	radarLine = disparity.rows / 2;
	vector<double> local = radarToSB;
	vector<double> displace = displacements;
	objects.clear();
	int num = 0;
	cout << "Sanity check: " << (radarToSB.size() == displacements.size()) <<endl << radarToSB.size() << " " << displacements.size() << endl;
	for(double distance : local){
		double xCoordinate = (getXcooMeters(distance,displace.at(num))/(0.004)/xRep);
		int x = (int)floor(xCoordinate/0.001); //X coordante on image

	//	cout <<" distance: " << distance << " at: " << x << " (" << xCoordinate << ") " << "displacement "<< displace.at(num)<< " xMeters " << getXcooMeters(distance,displace.at(num))
	//		 << endl;

		//cout << displace.at(num) + 0.06 << " "<<distance <<endl;
		if(x<disparity.cols && x > 0){
			double Z = 336
					/ ((double) ((int) disparity.at<uchar>(radarLine, x))
							* 0.004 * xRep);
			Z = Z / 1000;

			if (distance < 4/*fabs(Z-distance) <= 0.5*/) {
				//create Objects
				cout << "Match " << distance << " on image: " << Z << " " << x << endl;
				int width = widthOfObject(disparity, x);
				ObjectDR obj;
				obj.centre = Point(x, radarLine);
				obj.distance = distance;
				obj.length = 50;
				obj.width = 50;
				objects.push_back(obj);
			}

			num ++;

			if (showFusionProcess) {
				circle(display, Point(x, radarLine), 5, Scalar(255, 0, 0), -1, 1);
			}
		}
	}

	if(showFusionProcess){
		applyColorMap(display, display, COLORMAP_HOT);
		resize(display, display, size);
		namedWindow("Radar and Disparity");
		imshow("Radar and Disparity", display);
	}

}

double Fusion::getXcooMeters(double Dos, double displacement){//Dos in meters
	double Dcr = 0.06 + displacement;

	double fov = 85.316268452;
	double fovRad = fov*M_PI/180;
	double importantAngle = (((180-fov)/2)*M_PI/180);

	double beta;
	double alpha;
	if(Dcr >= 0){
		alpha = atan(Dos/Dcr);
		beta  = (fovRad+importantAngle) - alpha;
	}
	else{
		Dcr = fabs(Dcr);
		alpha = atan(Dos/Dcr);
		beta  = (alpha-importantAngle);
	}

	double d     = 0.0028 / sin(alpha);
	double x     = d * sin(beta)/sin(importantAngle);

	return x;
}

vector<DivisionImage> Fusion::getDisivionOnImage(Mat disparity){
	vector<DivisionImage> divisions;
	for(int x=0;x<disparity.cols;x++){
		DivisionImage segment;
		segment.startX = x;
		int widthSegment = 0;
		int disparityValue =  disparity.at<uchar>(radarLine,x);
		int extraCount = 0;
		for(int x2=x+1;x2<disparity.cols;x2++){
			int disparyValueComp = disparity.at<uchar>(radarLine,x2);
			if(abs(disparityValue - disparyValueComp) <= pixelError){
				widthSegment = x2 - x;
				segment.endX = x2;
				extraCount = 0;
			}
			else if(extraCount == 1){
				break;
			}
			else{
				extraCount ++;
			}
		}
		if (widthSegment > 5){
		getObjectLength(disparity,segment);
		x = x + widthSegment;

		cout << segment.startX << " " << segment.startY << " " <<segment.endX << " " <<segment.endY << endl;
		divisions.push_back(segment);
		}
	}

	drawSegments(disparity,divisions);

	return divisions;
}

void Fusion::drawSegments(Mat disparity, vector<DivisionImage> segments){
	for(DivisionImage segment : segments) {
		Point pt1(segment.startX,segment.startY);
		Point pt2(segment.endX,segment.endY);
		rectangle(disparity,pt1,pt2,Scalar(0,255,0),2);
	}
}

void Fusion::getObjectLength(Mat disparity, DivisionImage& segment){
	int midObject = segment.startX + int((segment.endX - segment.startX) / 2);
	//cout << midObject << " this " << (segment.endX - segment.startX) << " " << segment.startX << " "<< segment.endX << endl;
	int disparityValue = disparity.at<uchar>(radarLine, midObject);
	bool checkUp = true;
	bool checkDown = true;
	int ycD = radarLine + 1;
	int ycU = radarLine - 1;

	while (ycD < disparity.rows || ycU > 0) {
		int disparityValueUp = disparity.at<uchar>(ycU, midObject);
		int disparityValueDown = disparity.at<uchar>(ycD, midObject);
		if (abs(disparityValue-disparityValueUp) <= pixelError
				&& checkUp && ycU > 0) {
			ycU--;
		} else {
			checkUp = false;
		}
		if (abs(disparityValue - disparityValueDown) <= pixelError
				&& checkDown && ycD < disparity.rows) {
			ycD++;
		} else {
			checkDown = false;
		}
		if (!checkDown && !checkUp) {
			break;
		}
	}
	segment.startY = ycU;
	segment.endY = ycD;
}

Mat Fusion::displayOnImage(Mat& image){
	Mat result;
	image.copyTo(result);
	for(ObjectDR object : objects){
		int newXC = object.centre.x*xRep;
		int newYC = object.centre.y*yRep;
		if(object.width > 5 && object.length > 5){
			Point pt1(newXC-object.width*xRep/2,newYC-object.length*yRep/2);
			Point pt2(newXC+object.width*xRep/2,newYC+object.length*yRep/2);
			rectangle(result,pt1,pt2,Scalar(0,255,0),2);
			putText(result,to_string(object.distance),Point(newXC,newYC),FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);
		}
		//circle(result,Point(newX,radarLine*yRep),5,Scalar(255,0,0),-1,3);
	}
	return result;
}

vector<ObjectDR> Fusion::getObjects(){
	return objects;
}

int Fusion::widthOfObject(Mat disparity, int x){
	int disparityValue = disparity.at<uchar>(radarLine,x);
	int width = 0;
	for(int cx = x+1; cx<xDown; cx ++){
		int compare = (int)disparity.at<uchar>(radarLine,cx);
		//cout << x << ": "<<disparityValue <<" " << compare << endl;
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
	Mat kernel;
	int kernelSize = 20;
	kernel = Mat::ones(Size(kernelSize,kernelSize),CV_32F)/(float)(kernelSize*kernelSize);
	filter2D(disparitySrc,disparity2,-1,kernel,Point(-1,-1),0,BORDER_DEFAULT);
	//bilateralFilter(disparitySrc,disparity2,5,10,5);
	//disparitySrc.copyTo(disparity2);
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

void Fusion::dontShow(){
	showFusionProcess = false;
}
