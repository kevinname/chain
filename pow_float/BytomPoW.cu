#include "BytomPoW.h"
#include <cuda_runtime.h>
#include "cublas_v2.h"

void print_floatMatrix(const char *s, float *devPtr) {

  float f[256 * 256];
  cublasGetMatrix (256, 256, sizeof(float), devPtr, 256, f, 256);
  printf("%s:\n", s);
  for(int i =0; i<256; i++) {
    if(0 == (i%8)) printf("\n");
    printf("%3.1f\t", f[i]);
  }
  printf("\n");
}

void print_int8HostMatrix(const char *s, int8_t *i8Ptr) {

  printf("%s:\n", s);
  for(int i =0; i<256; i++) {
    if(0 == (i%8)) printf("\n");
    printf("%d\t", i8Ptr[i]);
  }
  printf("\n");
}

void initMatVecGpu(BytomMatListGpu* matListGpu_float, BytomMatList* matList_int8) {

  float *tmp = (float *)malloc(sizeof(float) * 256 * 256);
  for(int i=0; i<matList_int8->matVec.size(); i++) {
    int8_t* hostPtr_i8 = (int8_t*)(matList_int8->at(i).d);
    for(int k = 0; k < 256 * 256; k++)
      tmp[k] = hostPtr_i8[k];

    float* devPtr = (float*)(matListGpu_float->at(i));
    cublasStatus_t stat = cublasSetMatrix (256, 256, sizeof(*devPtr), tmp, 256, devPtr, 256);
    if (stat != CUBLAS_STATUS_SUCCESS) {
      std::cerr<<"Fail to Set CuBlas Matrix."<<std::endl;
      exit(EXIT_FAILURE);
    }
  }
}

__global__ void converInt32ToInt8_gpu(float * in, float* out) {
  int32_t data_i32 = in[blockIdx.x * blockDim.x + threadIdx.x];
  int8_t data_i8 = ((data_i32&0xFF)+ ((data_i32>>8)&0xFF))&0xFF;
  out[blockIdx.x * blockDim.x + threadIdx.x] = data_i8;
}

void core_mineBytom_gpu(
    std::vector<uint8_t> fourSeq[4],
    BytomMatListGpu* matListGpu_float,
    uint32_t data[64],
    cublasHandle_t handle) {

  Mat256x256float *idt=new Mat256x256float;
  Mat256x256float *mat=new Mat256x256float;
  Mat256x256float *tmp=new Mat256x256float;
  Mat256x256float *res=new Mat256x256float[4];
  idt->toIdentityMatrix();

  float* devIdt;
  float* devTmp;
  float* devTmp_r;
  cudaMalloc ((void**)&devIdt, 256*256*sizeof(*devIdt));
  cudaMalloc ((void**)&devTmp, 256*256*sizeof(*devTmp));
  cudaMalloc ((void**)&devTmp_r, 256*256*sizeof(*devTmp_r));
  cublasStatus_t stat = cublasSetMatrix (256, 256, sizeof(float), idt->f, 256, devIdt, 256); //HKKUO: A). Memory Set
  const float alpha = 1;
  const float beta = 0;

  for(int k=0; k<4; k++) {
    for(int j=0; j<2; j++) {
      for(int i=0; i<32; i+=2) {
        if (j==0 && i==0)
          stat = cublasSgemm(handle,
                              CUBLAS_OP_N,
                              CUBLAS_OP_N,
                              256,
                              256,
                              256,
                              &alpha,
                              matListGpu_float->at(fourSeq[k][i]),
                              256,
                              devIdt,
                              256,
                              &beta,
                              devTmp_r,
                              256);  //HKKUO: B). General Matrix Multiplication (GEMM)
        else
          stat = cublasSgemm(handle,
                              CUBLAS_OP_N,
                              CUBLAS_OP_N,
                              256,
                              256,
                              256,
                              &alpha,
                              matListGpu_float->at(fourSeq[k][i]),
                              256,
                              devTmp,
                              256,
                              &beta,
                              devTmp_r,
                              256);  //HKKUO: B). General Matrix Multiplication (GEMM)
        if (stat != CUBLAS_STATUS_SUCCESS) {
          std::cerr<<"Fail to Run CuBlas GemmEx1."<<std::endl;
          exit(EXIT_FAILURE);
        }
        converInt32ToInt8_gpu<<<256, 256>>>(devTmp_r, devTmp);
        stat = cublasSgemm(handle,
                            CUBLAS_OP_N,
                            CUBLAS_OP_N,
                            256,
                            256,
                            256,
                            &alpha,
                            matListGpu_float->at(fourSeq[k][i+1]),
                            256,
                            devTmp,
                            256,
                            &beta,
                            devTmp_r,
                            256);  //HKKUO: B). General Matrix Multiplication (GEMM)
        if (stat != CUBLAS_STATUS_SUCCESS) {
          std::cerr<<"Fail to Run CuBlas GemmEx2."<<std::endl;
          exit(EXIT_FAILURE);
        }
        converInt32ToInt8_gpu<<<256, 256>>>(devTmp_r, devTmp);
      }
    }
    stat = cublasGetMatrix (256, 256, sizeof(*devTmp), devTmp, 256, res[k].f, 256);
  }

  mat->add(res[0], res[1]);  //HKKUO: C). Matrix Addition
  tmp->add(*mat, res[2]);    //HKKUO: C). Matrix Addition
  mat->add(*tmp, res[3]);    //HKKUO: C). Matrix Addition

  Arr256x64i32 arr(*mat);
  arr.reduceFNV();           //HKKUO: D). Reduction
  arr.fillWithD0(data);      //HKKUO: E). Memory Set
  delete mat;
  delete tmp;
  delete[] res;
}
