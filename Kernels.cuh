//Kernel 1 used for cost Computation
//Kernel 2 used for path aggregation and disparity selection

//Structure declarations
struct startInfo{
	int startX;
	int startY;
	int directionX;
	int directionY;
	//int disparityValue; //Dont need this, delete later just in case
};
struct pointCoo{
	int x,y;
};

#define p1 10//5  //10
#define p2 250//200//1000 //150


#define maxDisparity 250

//Kernel 1 main
__global__ void KernelDisparityCalculations(int boxCostX, int boxCostY, unsigned int* censusLa, unsigned int* censusRa,int widthImage, int lenImage, uchar* leftI, uchar* rightI, int* L, int* image);
//Kernel 2 main
__global__ void KernelSemiGlobal(int* cost, int width, int length, startInfo* workInfo, int* L1);

//Kernel 1 functions
__device__ void CensusTransFormationKernel(int widthW, int lengthW, unsigned int* censusLa, unsigned int* censusRa,int widthImage, uchar* leftI, uchar* rightI);
__device__ void CostComputationKernel(unsigned int* censusL, unsigned int* censusR, int* cost, int width, int length);
__device__ int HammingDistanceKernel(unsigned int a, unsigned int b);
__device__ void CostComputationKernelAndDisparitySelection(unsigned int* censusL, unsigned int* censusR, int* cost, int width, int length, int* L, int* image);

//Kernel 2 functions
__device__ void AggregateCostKernel(int* cost, int width, int lenght, startInfo info, int* L);
__device__ int minNumber(int a, int b, int c, int d);
__device__ void swappArrays(int* x, int* y);


__global__ void KernelDisparityCalculations(int boxCostX, int boxCostY, unsigned int* censusLa, unsigned int* censusRa,int widthImage, int lenImage, uchar* leftI, uchar* rightI, int* cost, int* L, int* image){
	//Hardcode size of image so this can be done
	//__shared__ unsigned int* censusLa[widthImage*lenImage];
	//__shared__ unsigned int* censusRa[widthImage*lenImage];
	CensusTransFormationKernel(boxCostX,boxCostY,censusLa,censusRa,widthImage,leftI,rightI);
	__syncthreads();
	//CostComputationKernel(censusLa,censusRa,cost,widthImage,lenImage);
	CostComputationKernelAndDisparitySelection(censusLa,censusRa,cost,widthImage,lenImage,L,image);
	__syncthreads();

	//censusAndCostKernel(boxCostX, boxCostY, widthImage, lenImage, leftI, rightI, cost, censusRa);
}

__global__ void KernelSemiGlobal(int* cost, int width, int length, startInfo* workInfo, int* L1){
	int ID = blockIdx.x * blockDim.x + threadIdx.x;
	//startInfo infoThread = workInfo[blockIdx.x];
	startInfo infoThread = workInfo[ID];
	AggregateCostKernel(cost,width,length,infoThread,L1);
	__syncthreads();
}


__device__ void AggregateCostKernel(int* cost, int width, int length, startInfo info, int* L){
	//int ID = blockIdx.x * blockDim.x + threadIdx.x;
	//int disparityWork = maxDisparity / blockDim.x;
	int startD = 0;//threadIdx.x * disparityWork;
	int endD   = maxDisparity;//startD + disparityWork;


	/*if(blockIdx.x == 0){
		printf("Distribution %d %d \n", startD, endD);
	}*/

	int previousResults[maxDisparity];
	int previousTemp[maxDisparity];

	int x = info.startX;
	int y = info.startY;
	int pathX = info.directionX;
	int pathY = info.directionY;
	int influenceX = -1;
	int influenceY = -1;

	bool done = false;
	int minToUse = 0;

	while(!done){
		int minimum = 8888;
		if(influenceX < 0 || influenceY < 0 || influenceY >= length ||influenceX >= width ){ //This line is different, less checks

			for(int d=startD; d<endD; d++){
				int costPixel = cost[width*(y+d*(length))+x];
				L[width*(y+d*(length))+x] = L[width*(y+d*(length))+x] + costPixel;
				previousResults[d] = costPixel;
				if(costPixel < minimum){
					minimum = costPixel;
				}
			}
		}
		else{
			for(int d = startD; d<endD; d++){
				int costPixel = cost[width*(y+d*(length))+x];
				int currentD = previousResults[d];
				int previousD= (d-1>=0)? previousResults[d-1]: 8888;
				int nextD    = (d+1<maxDisparity)? previousResults[d+1] : 8888;

				int valueToAssign = costPixel + minNumber(currentD,nextD+p1,previousD+p1,minToUse+p2) - minToUse;
				L[width*(y+d*(length))+x] = L[width*(y+d*(length))+x] +  valueToAssign;
				previousTemp[d] = valueToAssign;

				if(valueToAssign < minimum){
					minimum = valueToAssign;
				}
			}
			swappArrays(previousResults,previousTemp);
		}

		minToUse = minimum;
		influenceX = x;
		influenceY = y;
		x = x + pathX;
		y = y + pathY;
		//previousResults = previousTemp;
		if (x >= width || y >= length || x < 0 || y < 0) {
			done = true;
			break;
		}
	}
}

__device__ void swappArrays(int* x, int* y){
	for(int i=0;i<maxDisparity;i++){
		x[i] = y[i];
	}
}

__device__ int minNumber(int a, int b, int c, int d){
	int min = a;
	if(min>b){
		min = b;
	}
	if(min>c){
		min =c;
	}
	if(min>d){
		min = d;
	}
	return min;
}



//Kernel 1 functions
__device__ void CostComputationKernel(unsigned int* censusL, unsigned int* censusR, int* cost, int width, int length){
	int y = threadIdx.y + (blockIdx.y * blockDim.y);
	int xl = threadIdx.x + (blockIdx.x * blockDim.x);
	int start = xl - maxDisparity;
	unsigned valueLeft = censusL[y*width+xl];

	for(int xr = start; xr<=xl;xr++){
		int valueToAssigned = 99999;
		int dis = xl-xr;
		if(xr>=0){
			unsigned int valueRight = censusR[y*width+xr];
			valueToAssigned = HammingDistanceKernel(valueLeft,valueRight);
		}
		cost[width*(y+dis*(length))+xl] = valueToAssigned;
	}
}

__device__ void CostComputationKernelAndDisparitySelection(unsigned int* censusL, unsigned int* censusR, int* cost, int width, int length, int* L, int* image){
	int y = threadIdx.y + (blockIdx.y * blockDim.y);
	int xl = threadIdx.x + (blockIdx.x * blockDim.x);
	int start = (xl - maxDisparity) + 1;
	unsigned valueLeft = censusL[y*width+xl];

	int blockId = blockIdx.x + blockIdx.y * gridDim.x;
	int threadId = blockId * (blockDim.x * blockDim.y) + (threadIdx.y * blockDim.x) + threadIdx.x;

	int minimum = 99999;
	int disToAssign = 0;
	for(int xr = start; xr<=xl;xr++){
		int valueToAssigned = 99999;
		int dis = xl - xr;
		int value = L[width*(y+dis*(length))+xl];
		L[width*(y+dis*(length))+xl] = 0;

		if (xr >= 0) {
			/*if (threadId == 0) {
				printf("\n disparity: %d \n\n", dis);
			}*/

			unsigned int valueRight = censusR[y * width + xr];
			valueToAssigned = HammingDistanceKernel(valueLeft, valueRight);

			//Disparity Selection
			if (value < minimum) {
				minimum = value;
				disToAssign = dis;
			}
		}
		cost[width*(y+dis*(length))+xl] = valueToAssigned;
	}
	image[width*y+xl] = disToAssign;
}




__device__ void CensusTransFormationKernel(int widthW, int lengthW, unsigned int* censusLa, unsigned int* censusRa,int widthImage, uchar* leftI, uchar* rightI){
	int xA = threadIdx.x + (blockIdx.x * blockDim.x);
	int yA = threadIdx.y + (blockIdx.y * blockDim.y);

	int wI = widthImage + widthW/2 + widthW/2;
	int x = xA + widthW/2;
	int y = yA + lengthW/2;
	unsigned int censusLCal = 0;
	unsigned int censusRCal = 0;
	unsigned int shiftCount=0;

	int valueLeft = (int)leftI[y*wI+x];
	int valueRight= (int)rightI[y*wI+x];

	for (int j = y - lengthW / 2; j <= y + lengthW / 2; j++) {
		for (int i = x - widthW / 2; i <= x + widthW / 2; i++) {
			int valueToCheckLeft = (int)leftI[j*wI+i];
			int valueTocheckRight= (int)rightI[j*wI+i];
			if(shiftCount != widthW*lengthW/2){
				//Left Census
				if(valueLeft < valueToCheckLeft){
					censusLCal <<= 1;
					censusLCal = censusLCal + 1;
				}else{
					censusLCal <<= 1;
				}
				//Right Census
				if(valueRight < valueTocheckRight){
					censusRCal <<=1;
					censusRCal = censusRCal + 1;
				}
				else{
					censusRCal <<=1;
				}
			}
		}
	}
	censusLa[yA*widthImage+xA] = censusLCal;
	censusRa[yA*widthImage+xA] = censusRCal;
}



__device__ int HammingDistanceKernel(unsigned int a, unsigned int b){
	unsigned int val = a ^ b;
	int dist = 0;
	while(val != 0){
		val = val & (val-1);
		dist++;
	}
	return dist;
}
