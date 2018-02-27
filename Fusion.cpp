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
	cout << "Sanity check: " << (radarToSB.size() == displacements.size()) <<endl;
	for(double distance : local){
		double xCoordinate = (getXcooMeters(distance,displace.at(num))/(0.004)/xRep);
		double xFull = getXcooMeters(distance,displace.at(num))/0.000004;
		int x = (int)round(xCoordinate/0.001);
	//	cout <<" distance: " << distance << " at: " << x << " (" << xCoordinate << ") " << "displacement "<< displace.at(num)<< " xMeters " << getXcooMeters(distance,displace.at(num))
	//			<<" on complete Image " << xFull << endl;
		if(x<disparity.cols && x > 0){
		double Z = 336 / ((double)((int)disparity.at<uchar>(radarLine,x))*0.004*xRep);
		Z = Z/1000;
		num ++;

		if(fabs(distance - Z) <= 0.1){
			cout << "Match " << distance << " " <<Z <<endl;
			ObjectDR obj;
			obj.centre = Point(x,radarLine);
			obj.distance = distance;
			obj.length=20;
			obj.width = 20;
			objects.push_back(obj);
		}

		if (showFusionProcess) {
			circle(display, Point(x, radarLine), 5, Scalar(255, 0, 0), -1, 1);
			/*line(display, Point(x, radarLine), Point(x + width, radarLine),
					Scalar(0, 0, 0), 2);
			line(display, Point(x + width / 2, radarLine - obj.length / 2),
					Point(x + width / 2, radarLine + obj.length / 2),
					Scalar(0, 0, 0), 2);*/
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

	/*double beta  = atan2(Dos,Dcr);
	double alpha = M_PI - (beta +  0.6108652382); // 30 in rad
	double df    = 0.0028 / sin(beta);

	return (sin(alpha) * df / sin(0.6108652382));
	*/
	return x;
}

void Fusion::radarPointsInImage(Mat disparitySrc, vector<double> radarToSB){
	Mat disparity = getFilterImage(disparitySrc);
	Mat display;

	if (showFusionProcess) {
		double minVal, maxVal;
		minMaxIdx(disparitySrc, &minVal, &maxVal);
		disparity.convertTo(display, CV_8UC3, 255 / (maxVal), minVal);
	}
	radarLine = disparity.rows / 2;
	vector<double> local = radarToSB;
	objects.clear();

	for(int x = 0; x < disparity.cols; x++){
		double Z = 336 / ((double)((int)disparity.at<uchar>(radarLine,x))*0.004*xRep);
		Z = Z/1000;
		//cout << "Distance " << Z << endl;
		int num = 0;

		int toDelete = 0;
		int XtoUse = 0;
		double distanceObj = 0;
		double mimDifference = 10;
		bool match = false;

		cout <<"\nZ: " << Z <<" Size: " << local.size()  <<endl;
		if(!local.empty()){
		for(double distance : local){
			cout << distance << " ";
			double difference = fabs(Z - distance);
			if (difference < mimDifference && difference <= errorValue) {
				cout << "Match: " << distance << endl;
				mimDifference = difference;
				toDelete = num;
				XtoUse = x;
				distanceObj = distance;
				match = true;
			}
			num ++;
		}

		if(match){
			int width = widthOfObject(disparity, XtoUse);
			ObjectDR obj = getObjectDimensions(disparity, width, XtoUse);
			local.erase(local.begin() + toDelete);
			obj.distance = distanceObj;
			objects.push_back(obj);
			if (showFusionProcess) {
				circle(display, Point(x, radarLine), 5, Scalar(255, 0, 0), -1,
						3);
				line(display, Point(x, radarLine), Point(x + width, radarLine),
						Scalar(0, 0, 0), 2);
				line(display, Point(x + width / 2, radarLine - obj.length / 2),
						Point(x + width / 2, radarLine + obj.length / 2),
						Scalar(0, 0, 0), 2);
			}
			x = x + width;
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
	bilateralFilter(disparitySrc,disparity2,9,10,10);
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
