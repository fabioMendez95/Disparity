#include"Camera.h"

using namespace std;
using namespace cv;

void Camera::getCamera(){
	stream.open(1);
	if(!stream.isOpened()){
		cout  << "Cant open camera" << endl;
	}
	while(true){
		Mat image;
		stream.read(image);
		namedWindow("Image");
		imshow("Image", image);
		if(waitKey(30) >= 0){
			break;
		}
	}
	cvDestroyWindow("Image");
}

void Camera::initCamera(int device){
	stream.set(CV_CAP_PROP_OPENNI_MAX_BUFFER_SIZE,0);
	stream.open(device);
	if(!stream.isOpened()){
		cout << "Error: Cant find camera on device: " << device << endl;
	}
	cout << "Enter to continue \n";
	namedWindow("Image: ESC to continue");
	Mat image;
	while(true){
		//stream.read(image);
		stream >> image;
		imshow("Image: ESC to continue", image);
		if(waitKey(30)>=0){
			break;
		}
	}

	CameraMatrixLeft = Mat(3,3,CV_32F,camDatLeft);
	DistortionCoeffLeft = Mat(1,4,CV_32F,camCoeffLeft);
	CameraMatrixRight= Mat(3,3,CV_32F,camDatRight);
	DistortionCoeffRight = Mat(1,4,CV_32F,camCoeffRight);

	imageHeight = image.rows;
	imageWidth = image.cols/2;

	initUndistortRectifyMap(CameraMatrixLeft, DistortionCoeffLeft, Mat(),
	            getOptimalNewCameraMatrix(CameraMatrixLeft, DistortionCoeffLeft, Size(imageWidth,imageHeight), 0, Size(imageWidth,imageHeight), 0),
	            Size(imageWidth,imageHeight), CV_16SC2, map1L, map2L);

	initUndistortRectifyMap(CameraMatrixRight, DistortionCoeffRight, Mat(),
		            getOptimalNewCameraMatrix(CameraMatrixRight, DistortionCoeffRight, Size(imageWidth,imageHeight), 0, Size(imageWidth,imageHeight), 0),
		            Size(imageWidth,imageHeight), CV_16SC2, map1R, map2R);

	cvDestroyWindow("Image: ESC to continue");
}

StereoPair Camera::getImages(){
	StereoPair images;
	Mat completeImage;
	stream.read(completeImage);

    images.left = completeImage(Rect(0,0,imageWidth,imageHeight));
	images.right = completeImage(Rect(imageWidth,0,imageWidth,imageHeight));

	return images;
}

void Camera::close(){
	stream.release();
}

void Camera::extractImage(){
	StereoPair lr = getImages();
	left = lr.left;
	rigth = lr.right;
}

Mat Camera::getLeft(){
	remap(left,rviewL,map1L,map2L,INTER_LINEAR);
	resize(rviewL,rviewL,Size(160,120));//320,240  160,120 1280,720
	return rviewL;
}
Mat Camera::getRight(){
	remap(rigth,rviewR,map1R,map2R,INTER_LINEAR);
	resize(rviewR,rviewR,Size(160,120));//320,240
	return rviewR;
}


void Camera::createCameraSample(int num){
	initCamera(1);
	namedWindow("image");
	namedWindow("right");
	int i = 1;
	while(i<=num){
		ostringstream leftSaveLocation;
		ostringstream rightSaveLocation;
		extractImage();
		Mat left = getLeft();
		Mat right = getRight();

		if (waitKey(30) >= 0) {
			leftSaveLocation << "Calibration/Left/" << getX(i)<<i<< ".jpg";
			rightSaveLocation << "Calibration/Right/" << getX(i)<<i<< ".jpg";
			imwrite(leftSaveLocation.str(), left);
			imwrite(rightSaveLocation.str(), right);
			i++;
		}
		imshow("image", left);
		imshow("right", right);
	}
	cvDestroyAllWindows();
}

String Camera::getX(int i){
	if(i<10){
		return "xx";
	}
	else if(i<100){
		return "x";
	}
	else{
		return "";
	}
}

