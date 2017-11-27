__global__ void KernelDisparityCalculations(int boxCostX, int boxCostY, unsigned int* censusLa, unsigned int* censusRa,int widthImage, int lenImage, uchar* leftI, uchar* rightI);
__device__ void CensusTransFormationKernel(int widthW, int lengthW, unsigned int* censusLa, unsigned int* censusRa,int widthImage, uchar* leftI, uchar* rightI);
__device__ void CostComputationKernel(unsigned int* censusL, unsigned int* censusR, int* cost, int maxDisparity, int width, int length);
__device__ int HammingDistanceKernel(unsigned int a, unsigned int b);


__global__ void KernelDisparityCalculations(int boxCostX, int boxCostY, unsigned int* censusLa, unsigned int* censusRa,int widthImage, int lenImage, uchar* leftI, uchar* rightI, int* cost){
  /*int tidX = threadIdx.x;
	int tidY = threadIdx.y;
	int bidX = blockIdx.x;
	int bidY = blockIdx.y;
	int dGX  = gridDim.x;
	int dGY  = gridDim.y;*/
	CensusTransFormationKernel(boxCostX,boxCostY,censusLa,censusRa,widthImage,leftI,rightI);
	__syncthreads();
	CostComputationKernel(censusLa,censusRa,cost,100,widthImage,lenImage);
	__syncthreads();
	//printf("BlockID: %d %d \t ThreadId: %d %d \t GridDims: %d %d \n",bidX,bidY,tidX,tidY,dGX,dGY);
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
