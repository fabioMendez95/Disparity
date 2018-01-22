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

#define p1 10
#define p2 100

//Kernel 1 main
__global__ void KernelDisparityCalculations(int boxCostX, int boxCostY, unsigned int* censusLa, unsigned int* censusRa,int widthImage, int lenImage, uchar* leftI, uchar* rightI);
//Kernel 2 main
__global__ void KernelSemiGlobal(int* cost, int width, int length, startInfo* workInfo, int maxDisparity, int* L1, int* L2, int* L3, int* L4/*, int* L5, int* L6, int* L7, int* L8*/);

//Kernel 1 functions
__device__ void CensusTransFormationKernel(int widthW, int lengthW, unsigned int* censusLa, unsigned int* censusRa,int widthImage, uchar* leftI, uchar* rightI);
__device__ void CostComputationKernel(unsigned int* censusL, unsigned int* censusR, int* cost, int maxDisparity, int width, int length);
__device__ int HammingDistanceKernel(unsigned int a, unsigned int b);

//Kernel 2 functions
__device__ void AggregateCostKernel(int* cost, int width, int lenght, int maxDisparity, startInfo info, int* L);
__device__ int minNumber(int a, int b, int c, int d);

__global__ void KernelDisparityCalculations(int boxCostX, int boxCostY, unsigned int* censusLa, unsigned int* censusRa,int widthImage, int lenImage, uchar* leftI, uchar* rightI, int* cost){
	//Hardcode size of image so this can be done
	//__shared__ unsigned int* censusLa[widthImage*lenImage];
	//__shared__ unsigned int* censusRa[widthImage*lenImage];

	CensusTransFormationKernel(boxCostX,boxCostY,censusLa,censusRa,widthImage,leftI,rightI);
	__syncthreads();
	CostComputationKernel(censusLa,censusRa,cost,100,widthImage,lenImage);
	__syncthreads();
}

__global__ void KernelSemiGlobal(int* cost, int width, int length, startInfo* workInfo, int maxDisparity, int* L1, int* L2, int* L3, int* L4/*, int* L5, int* L6, int* L7, int* L8*/){
	//int threadNum = threadIdx.x;
	//int blockX = blockIdx.x;
	int ID = blockIdx.x * blockDim.x + threadIdx.x;
	//Need work Distribution for non-divisible thread number
	startInfo infoThread = workInfo[ID];
	if(ID < width){
		AggregateCostKernel(cost,width,length,maxDisparity,infoThread,L1);
	}
	else if(ID < 2*width){
		AggregateCostKernel(cost,width,length,maxDisparity,infoThread,L2);
	}
	else if(ID < 2*width + length){
		AggregateCostKernel(cost,width,length,maxDisparity,infoThread,L3);
	}
	else if(ID < 2*width + 2*length){
		AggregateCostKernel(cost,width,length,maxDisparity,infoThread,L4);
	}
	/*else if(ID < 2*width +2*length + (width+length -1)){
		AggregateCostKernel(cost,width,length,maxDisparity,infoThread,L5);
	}
	else if(ID < 2*width +2*length + 2*(width+length -1)){
		AggregateCostKernel(cost,width,length,maxDisparity,infoThread,L6);
	}
	else if(ID < 2*width +2*length + 3*(width+length -1)){
		AggregateCostKernel(cost,width,length,maxDisparity,infoThread,L7);
	}
	else if(ID < 2*width +2*length + 4*(width+length -1)){
		AggregateCostKernel(cost,width,length,maxDisparity,infoThread,L8);
	}*/
	if(ID == 100){
		printf("%d \n",L1[width*(369+99*(length))+100]);
	}
	__syncthreads();
	//printf("%d %d %d \n",threadNum, blockX, blockY);
}

//Kernel 2 Functions
//TODO there is an out of bound in the array accessing, also check initial info
__device__ void AggregateCostKernel(int* cost, int width, int length, int maxDisparity, startInfo info, int* L){
	int ID = blockIdx.x * blockDim.x + threadIdx.x;

	int x = info.startX;
	int y = info.startY;
	int pathX = info.directionX;
	int pathY = info.directionY;
	int influenceX = -1;
	int influenceY = -1;

	bool done = false;
	int minToUse = 0;
	while(!done){
		//Quick Check
		if(x >= width || y >= length || x < 0 || y < 0){
			printf(" Error: %d %d \n",x,y);
		}

		int minimum = 8888;
		if(influenceX < 0 || influenceY < 0){ //This line is different, less checks
			for(int d=0; d<maxDisparity; d++){
				int costPixel = cost[width*(y+d*(length))+x];
				L[width*(y+d*(length))+x] = costPixel; //this line is wrong
				if(costPixel < minimum){
					minimum = costPixel;
				}
			}
		}
		else{
			for(int d = 0; d<maxDisparity; d++){
				int costPixel = cost[width*(y+d*(length))+x];
				int currentD = L[width*(influenceY+d*(length))+influenceX];
				int previousD= (d-1>=0)? L[width*(influenceY+(d-1)*(length))+influenceX]: 8888;
				int nextD    = (d+1<maxDisparity)? L[width*(influenceY+(d+1)*(length))+influenceX] : 8888;

				int valueToAssign = costPixel + minNumber(currentD,nextD+p1,previousD+p1,minToUse+p2) - minToUse;
				L[width*(y+d*(length))+x] = valueToAssign;

				if(valueToAssign < minimum){
					minimum = valueToAssign;
				}
				/*if(ID == 100){
					printf("%d %d : %d d: %d path: %d %d\n",x,y,valueToAssign,d,pathX,pathY);
				}*/
			}
		}

		minToUse = minimum;
		influenceX = x;
		influenceY = y;
		x = x + pathX;
		y = y + pathY;
		if(x >= width || y >= length || x < 0 || y < 0){
			done = true;
		}
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
