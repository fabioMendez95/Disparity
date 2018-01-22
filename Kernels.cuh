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

//Kernel 1 main
__global__ void KernelDisparityCalculations(int boxCostX, int boxCostY, unsigned int* censusLa, unsigned int* censusRa,int widthImage, int lenImage, uchar* leftI, uchar* rightI);
//Kernel 2 main
__global__ void KernelSemiGlobal(int* cost, int width, int length);

//Kernel 1 functions
__device__ void CensusTransFormationKernel(int widthW, int lengthW, unsigned int* censusLa, unsigned int* censusRa,int widthImage, uchar* leftI, uchar* rightI);
__device__ void CostComputationKernel(unsigned int* censusL, unsigned int* censusR, int* cost, int maxDisparity, int width, int length);
__device__ int HammingDistanceKernel(unsigned int a, unsigned int b);

//Kernel 2 functions
__device__ void AggregateCostKernel(int* cost, int width, int lenght, int maxDisparity);


__global__ void KernelDisparityCalculations(int boxCostX, int boxCostY, unsigned int* censusLa, unsigned int* censusRa,int widthImage, int lenImage, uchar* leftI, uchar* rightI, int* cost){
	//Hardcode size of image so this can be done
	//__shared__ unsigned int* censusLa[widthImage*lenImage];
	//__shared__ unsigned int* censusRa[widthImage*lenImage];

	CensusTransFormationKernel(boxCostX,boxCostY,censusLa,censusRa,widthImage,leftI,rightI);
	__syncthreads();
	CostComputationKernel(censusLa,censusRa,cost,100,widthImage,lenImage);
	__syncthreads();
}

__global__ void KernelSemiGlobal(int* cost, int width, int length){
	int threadNum = threadIdx.x;
	int blockX = blockIdx.x;
	int blockY = blockIdx.y;

	//printf("%d %d %d \n",threadNum, blockX, blockY);
}

//Kernel 2 Functions
__device__ void AggregateCostKernel(int* cost, int width, int lenght, int maxDisparity){
	//Penalties
	int p1 = 5;
	int p2 = 100;
}


//Kernel 1 functions
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

__device__ void CostComputationKernel(unsigned int* censusL, unsigned int* censusR, int* cost, int maxDisparity, int width, int length){
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

__device__ int HammingDistanceKernel(unsigned int a, unsigned int b){
	unsigned int val = a ^ b;
	int dist = 0;
	while(val != 0){
		val = val & (val-1);
		dist++;
	}
	return dist;
}
