#include "Comparison.h"

using namespace std;
using namespace cv;

void Comparison::CompareDisparities(){
	Mat left, right;
	Mat completeImage;
	struct timeval timstr;


	//Maximum box value is depending on bytes used
	int BoxCostX = 9;
	int BoxCostY = 7;
	int maxDisparity = 128;

	//Reading Images
	left = imread("Images/KITTY/left/0000000000.png", CV_LOAD_IMAGE_COLOR);
	right= imread("Images/KITTY/right/0000000000.png", CV_LOAD_IMAGE_COLOR);

	Mat leftBlack;
	cvtColor( left, leftBlack, CV_BGR2GRAY );
	Mat rightBlack;
	cvtColor( right, rightBlack, CV_BGR2GRAY );
	//Image Display
	namedWindow("left");
	namedWindow("right");
	imshow("left",leftBlack);
	imshow("right",rightBlack);
	waitKey(0);


	int decreseX = BoxCostX/2 + BoxCostX/2;
	int decreseY = BoxCostY/2 + BoxCostY/2;


	//Simple Disparity calculations and display----------------------------------
	gettimeofday(&timstr, NULL);
	double begin = timstr.tv_sec + (timstr.tv_usec / 1000000.0);

	Mat disparity2 = SimpleDisparityCalculations(leftBlack,rightBlack,BoxCostX,BoxCostY,left.cols,left.rows,maxDisparity);

	gettimeofday(&timstr, NULL);
	double end = timstr.tv_sec + (timstr.tv_usec / 1000000.0);
	double timeD = double(end - begin)/CLOCKS_PER_SEC;

	namedWindow("disparitySimple");
	imshow("disparitySimple",disparity2);
	printf("Elapsed time Local Matching:\t\t\t%.6lf (s)\n", end - begin);
	waitKey(0);
	//---------------------------------------------------------------------------


	//Census Transform
	gettimeofday(&timstr, NULL);
	begin = timstr.tv_sec + (timstr.tv_usec / 1000000.0);

	unsigned int* censusLa = (unsigned int*)malloc((sizeof(unsigned int))*(left.cols-decreseX)*(left.rows-decreseY));
	unsigned int* censusRa = (unsigned int*)malloc((sizeof(unsigned int))*(right.cols-decreseX)*(right.rows-decreseY));
	CensusTransformation(leftBlack,BoxCostX,BoxCostY,censusLa);
	CensusTransformation(rightBlack,BoxCostX,BoxCostY,censusRa);
	cout << "census ok" << endl;

	//Cost Computation

	int* cost = (int*)malloc((sizeof(int))*(left.cols-decreseX)*(left.rows-decreseY)*(maxDisparity+1));
	//CostComputation(leftBlack,rightBlack,cost,maxDisparity,left.cols,left.rows,BoxCostX,BoxCostY);
	CostComputationCensus(censusLa,censusRa,cost,maxDisparity,left.cols-decreseX,left.rows-decreseY);
	cout << "cost ok" << endl;


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
	end = timstr.tv_sec + (timstr.tv_usec / 1000000.0);
	timeD = double(end - begin)/CLOCKS_PER_SEC;
	printf("Elapsed time SMG:\t\t\t%.6lf (s)\n", end - begin);

	namedWindow("SMG");
	imshow("SMG",disparity);
	imwrite("disparitybox2x2.png",disparity);
	waitKey(0);

	//free Memory
	free(censusLa);
	free(censusRa);
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
Mat Comparison::SimpleDisparityCalculations(Mat left, Mat right, int blockX, int blockY, int width, int length, int maxDisparity){
	Mat disparity(length,width,CV_8U);

	for(int y=blockY/2; y<length-blockY/2; y++){
		for(int xl=blockX/2; xl<width-blockX/2; xl++){

			int startSearch = xl - maxDisparity;
			if(startSearch<0){
				startSearch = 0;
			}
			int minDisparity = maxDisparity;
			int minValueDisparity = 999999;

			for(int xr = startSearch; xr<=xl; xr++){
				//scan Box
				int SAD = 0;
				for (int oy = -blockY/2; oy <= blockY/2; oy++ ){
					for(int ox =-blockX/2; ox<=blockX/2;ox++){
						int valueLeft = left.at<uchar>(y+oy,xl+ox);//left[(y+oy)*width+xl+ox];
						int valueRight= right.at<uchar>(y+oy,xr+ox);//right[(y+oy)*width+xr+ox];
						SAD = SAD + abs(valueLeft - valueRight);
					}
				}
				if (SAD < minValueDisparity){
					minValueDisparity = SAD;
					minDisparity      = xl-xr;
				}
			}
			disparity.at<uchar>(y,xl) = (uchar) minDisparity;

		}
	}

	return disparity;
}

//Disparity Selection Process
Mat Comparison::DisparitySelectionP(int* L2, int* L4, int* L5, int* L7,int* L1, int* L3, int* L6, int* L8, int maxDisparity, int width, int length){
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
void Comparison::AggregateCostCom(int* cost, int* L, int width, int length, int maxDisparity, int directionx, int directiony){
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


int Comparison::minBetweenNumbersInt(int a, int b, int c, int d){
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


//Calculates cost in a 3D matrix width x height x disparity
void Comparison::CostComputation (Mat left, Mat right, int* cost, int maxDisparity, int width, int length, int blockX, int blockY){
	int widthCo = width - blockX/2 - blockX/2;
	int lengthCo = length - blockY/2 - blockY/2;
	for(int y=blockY/2; y<length-blockY/2; y++){
		for(int xl=blockX/2; xl<width-blockX/2; xl++){

			int startSearch = xl - maxDisparity;
			/*if(startSearch<0){
				startSearch = 0;
			}*/

			for(int xr = startSearch; xr<=xl; xr++){
				//scan Box
				int SAD = 0;
				int dis;
				if (xr>=0){
					dis = xl - xr;
					for (int oy = -blockY / 2; oy <= blockY / 2; oy++) {
						for (int ox = -blockX / 2; ox <= blockX / 2; ox++) {
							int valueLeft = left.at<uchar>(y + oy, xl + ox);//left[(y+oy)*width+xl+ox];
							int valueRight = right.at<uchar>(y + oy, xr + ox);//right[(y+oy)*width+xr+ox];
							SAD = SAD + abs(valueLeft - valueRight);
						}
					}
				}
				else{
					SAD = 88888;
					dis = xl-xr;
				}
				int yc = y - blockY/2;
				int xc = xl - blockX/2;
/*				if (xc == 100 && yc == 100 && dis == 100) {
					cout << " Real " << SAD << endl;
				}*/
				cost[widthCo*(yc+dis*(lengthCo))+xc] =SAD;
			}
		}
	}
	cout << "Done" <<endl;
//	cout<<cost[widthCo*(100+100*lengthCo)+100] << " here \n";
}



//Testing Census---------------
void Comparison::CensusTransformation (Mat image, int widthW, int lengthW, unsigned int* censusArray){
	unsigned int census = 0;
	int shiftCount = 0;

	int width = image.cols - widthW/2 - widthW/2;

	for (int y = lengthW / 2; y < image.rows - lengthW / 2; y++) {
		for (int x = widthW / 2; x < image.cols - widthW / 2; x++) {
			census = 0;
			shiftCount = 0;
			int xA = x - widthW / 2;
			int yA = y - lengthW / 2;

			for (int j = y - lengthW / 2; j <= y + lengthW / 2; j++) {
				for (int i = x - widthW / 2; i <= x + widthW / 2; i++) {
					if ((int) image.at<uchar>(y, x) < (int) image.at<uchar>(j, i) && shiftCount != widthW * lengthW / 2) {
						census <<= 1;
						census = census + 1;
					} else if (shiftCount != widthW * lengthW / 2) {
						census <<= 1;
					}
					shiftCount++;
				}
			}
			censusArray[yA * width + xA] = census;
		}
	}
}


void Comparison::CostComputationCensus (unsigned int* censusL, unsigned int* censusR, int* cost, int maxDisparity, int width, int length){
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
					//cout<<"ignored" <<endl;
					valueToAssigned = 99999;
				}
				//cout << "Value to assign " << valueToAssigned << endl;
				cost[width*(y+dis*(length))+xl] =valueToAssigned;

			}
		}
	}
}

int Comparison::HammingDistanceNumbers (unsigned int a, unsigned int b){
	unsigned int val = a ^ b;
	int dist = 0;
	while(val != 0){
		val = val & (val-1);
		dist++;
	}
	return dist;
}
