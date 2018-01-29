#include<sl/Camera.hpp>
#include "opencv2/core/core.hpp"

using namespace std;
using namespace cv;

class ZedCamera{
public:
	Mat getLeftImage();
	Mat getRightImage();
	void initCamera();
	void grabImage();
	void closeCamera();
	string type2str(int type);


private:
	int new_width;
	int new_height;

	sl::Camera zed;
	sl::InitParameters initParameters;
	sl::RuntimeParameters runtime_parameters;
	sl::Resolution imageSize;

	Mat left;
	Mat right;

	Mat slMat2cvMat(sl::Mat& input);
};
