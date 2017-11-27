texture<float,cudaTextureType2D,cudaReadModeElementType> ImageRightTex;
texture<float,cudaTextureType2D,cudaReadModeElementType> ImageLeftTex;

__global__ void KernelDisparityCalculations(int boxCostX, int boxCostY, unsigned int* censusLa, unsigned int* censusRa);
__device__ void CensusTransFormationKernel(int widthW, int lengthW, unsigned int* censusLa, unsigned int* censusRa);


__global__ void KernelDisparityCalculations(int boxCostX, int boxCostY, unsigned int* censusLa, unsigned int* censusRa){
	int tidX = threadIdx.x;
	int tidY = threadIdx.y;
	int bidX = blockIdx.x;
	int bidY = blockIdx.y;
	int dGX  = gridDim.x;
	int dGY  = gridDim.y;
	CensusTransFormationKernel(boxCostX,boxCostY,censusLa,censusRa);
	//printf("BlockID: %d %d \t ThreadId: %d %d \t GridDims: %d %d \n",bidX,bidY,tidX,tidY,dGX,dGY);
}

__device__ void CensusTransFormationKernel(int widthW, int lengthW, unsigned int* censusLa, unsigned int* censusRa){
	int x = threadIdx.x + (blockIdx.x * blockDim.x);
	int y = threadIdx.y + (blockIdx.y * blockDim.y);

	float value = tex2D(ImageLeftTex,x,y);

	unsigned int census = 0;
	printf("BlockID: %d %d \t ThreadId: %d %d \t pixel: %d %d value: %f\n",blockIdx.x,blockIdx.y,threadIdx.x,threadIdx.y,x,y,value);


}
