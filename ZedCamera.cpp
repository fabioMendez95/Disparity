#include "ZedCamera.h"
using namespace cv;
using namespace std;

void ZedCamera::initCamera(){
	initParameters.camera_resolution = sl::RESOLUTION_VGA;
	initParameters.camera_fps = 10;
	initParameters.camera_buffer_count_linux = 0;
	//initParameters.depth_mode = sl::DEPTH_MODE_PERFORMANCE;
	//initParameters.coordinate_units = sl::UNIT_METER;
	//runtime_parameters.sensing_mode = sl::SENSING_MODE_STANDARD;

	zed.open(initParameters);
	imageSize = zed.getResolution();
	//This might be the problem
	new_width = imageSize.width/2;
	new_height = imageSize.height/2;
}

void ZedCamera::grabImage(){
	sl::Mat	image_zedLeft(new_width,new_height,sl::MAT_TYPE_8U_C4);
	sl::Mat image_zedRight(new_width,new_height,sl::MAT_TYPE_8U_C4);

	int errorGrab = zed.grab(runtime_parameters);
	//cout << "ZED grab: "<<errorGrab << endl;
	if(errorGrab > 0){
		cout << "Error Camera \n";
		sleep(5);
	}
	zed.retrieveImage(image_zedLeft, sl::VIEW_LEFT_GRAY, sl::MEM_CPU, new_width, new_height);
	zed.retrieveImage(image_zedRight, sl::VIEW_RIGHT_GRAY, sl::MEM_CPU, new_width, new_height);
	left = slMat2cvMat(image_zedLeft);
	right= slMat2cvMat(image_zedRight);
}

void ZedCamera::closeCamera(){
	zed.close();
}

Mat ZedCamera::getLeftImage(){
	return left;
}

Mat ZedCamera::getRightImage(){
	return right;
}

Mat ZedCamera::slMat2cvMat(sl::Mat& input){
    int cv_type = -1;
    switch (input.getDataType()) {
        case sl::MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
        case sl::MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
        case sl::MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
        case sl::MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
        case sl::MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
        case sl::MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
        case sl::MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
        case sl::MAT_TYPE_8U_C4: cv_type = CV_8UC4;break;
        default: break;
    }

    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM_CPU));
}

string ZedCamera::type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

