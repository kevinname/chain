#include "BytomPoW.h"
#include <stdio.h>
#include <iostream>
#include <time.h>

BytomMatList* matList_int8;
BytomMatListGpu* matListGpu_float;

static uint32_t g_seed[32] = {
  // 0x00000000, 0x00000000, 0xd89c1260, 0x00007fa2,
  // 0x0000007c, 0x0000005b, 0xd7bb8870, 0x00007fa2,
  // 0x00851000, 0x00000000, 0xd89ab3e0, 0x00007fa2,
  // 0x31bce000, 0x34313536, 0xd89c1260, 0x00007fa2,
  // 0x282522cf, 0x00007fff, 0x027de068, 0x00000000,
  // 0x00000001, 0x00000000, 0x28252548, 0x00007fff,
  // 0x28252538, 0x00007fff, 0x00000001, 0x00000000,
  // 0x026fcc68, 0x00000000, 0xd7bffc99, 0x00007fa2,
  0x00000000, 0x00000000, 0x00000000, 0x00000000,
  0x00000000, 0x00000000, 0x00000000, 0x00000000,
  0x9e629197, 0x0cb44dd9, 0x4008c79b, 0xcaf9d86f,
  0x18b4b49b, 0xa5b2a047, 0x81db7199, 0xed3b9e4e,
  0x3ffcf92d, 0x9c820def, 0x681c81ab, 0x1dffa44c,
  0x3166539a, 0xddb445c7, 0x731921af, 0x69bce8c7,
  0xeb61fef1, 0xa4791943, 0x4cfbddf1, 0xf4d96d49,
  0x2d646946, 0x2d636105, 0xc7c9c1fc, 0xd022da2c,
};

static uint8_t g_msg[5][32] = {
  {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  }, {
    0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
    0xf0, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x1f, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    0x1f, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
  }, {
    0x0f, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0xf9, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
    0x10, 0xf1, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    0x18, 0x19, 0xfa, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
  }, {
    0x00, 0x01, 0x02, 0xf3, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0xfb, 0x0c, 0x0d, 0x0e, 0x0f,
    0x10, 0x11, 0x12, 0x13, 0xf4, 0x15, 0x16, 0x17,
    0x18, 0x19, 0x1a, 0x1b, 0xfc, 0x1d, 0x1e, 0x1f,
  }, {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0xf6, 0x07,
    0x08, 0x09, 0x0a, 0x0b, 0x0c, 0xfd, 0x0e, 0x0f,
    0x10, 0x11, 0x12, 0x13, 0xf4, 0x15, 0x16, 0xf7,
    0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0xff,
  }
};

static uint8_t g_nonces[5][8] = {
  {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  }, {
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  }, {
    0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  }, {
    0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  }, {
    0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  }
};

static uint8_t g_golden_results[5][32] = {
  {
    0x46, 0x09, 0x92, 0xb9, 0xb4, 0x68, 0x5d, 0xd1, 0x3c, 0x62, 0x34, 0x9d, 0xcf, 0xa4, 0x78, 0xc3, 0xe0, 0xa8, 0x01, 0x02, 0x08, 0x66, 0x6e, 0x6b, 0x3b, 0x3a, 0x25, 0xb2, 0xb0, 0xda, 0xc3, 0x12,
  }, {
    0x21, 0xd7, 0xf7, 0xe2, 0x0b, 0x0d, 0x1e, 0x29, 0xf0, 0xc8, 0x8d, 0x4c, 0x57, 0x25, 0xc4, 0x37, 0x0f, 0x55, 0xef, 0x32, 0x1a, 0x9d, 0xad, 0x1b, 0x2a, 0x24, 0xb0, 0x77, 0x8f, 0x8b, 0x42, 0x77,
  }, {
    0xc7, 0x90, 0x41, 0xd8, 0x80, 0x82, 0x7f, 0xdd, 0x15, 0x10, 0x27, 0x00, 0x48, 0x02, 0x88, 0x6a, 0x03, 0x92, 0x85, 0xcb, 0x33, 0x6e, 0x53, 0x1c, 0xdc, 0x4a, 0x79, 0xb4, 0x98, 0xcb, 0x0c, 0x97,
  }, {
    0xc4, 0x38, 0x5d, 0x76, 0xfd, 0x11, 0x79, 0xb3, 0xc6, 0x5d, 0x6b, 0x38, 0xc2, 0xd7, 0x21, 0x08, 0x38, 0xe5, 0xd8, 0xd7, 0xa0, 0xce, 0x9b, 0xcb, 0x01, 0x3e, 0xa8, 0x10, 0xaf, 0xf5, 0xa6, 0xa2,
  }, {
    0xf5, 0x21, 0xe5, 0xdc, 0x47, 0xdb, 0x62, 0x69, 0x73, 0xee, 0x9d, 0x4d, 0x27, 0xec, 0xea, 0x00, 0x11, 0x94, 0x7f, 0xc6, 0xcb, 0xaf, 0x57, 0x61, 0x04, 0x7d, 0x1b, 0x31, 0x9c, 0x8e, 0xb2, 0x27,
  }
};

static void init_seed(Words32 &seed, uint32_t _seed[32])
{
  // for (int i = 0; i < 16; i++)
  //   seed.hi.w[i] = _seed[i];
  // for (int i = 0; i < 16; i++)
  //   seed.lo.w[i] = _seed[16 + i];
  for (int i = 0; i < 16; i++)
    seed.lo.w[i] = _seed[i];
  for (int i = 0; i < 16; i++)
    seed.hi.w[i] = _seed[16 + i];
}

int main(void)
{
  Words32 seed;

  cublasHandle_t handle;
  cublasStatus_t stat = cublasCreate(&handle);
  if (stat != CUBLAS_STATUS_SUCCESS){
    std::cerr<<"Fail to Create CuBlas Handle."<<std::endl;
    exit(EXIT_FAILURE);
  }

  std::cout << "init seed\t\t"; 
  init_seed(seed, g_seed);
  std::cout << "ok" << std::endl; 

  matList_int8=new BytomMatList;
  std::cout << "generate matrix\t\t"; 
  initMatVec(matList_int8->matVec, seed);
  std::cout << "ok" << std::endl;

  std::cout << "set matrix to gpu\t";
  matListGpu_float=new BytomMatListGpu;
  initMatVecGpu(matListGpu_float, matList_int8);
  std::cout << "ok" << std::endl;

  std::cout<<"Init finished"<<std::endl;

  clock_t start, end;
  start = clock();

  uint8_t results_gpu[5][32] = { 0 };
  for (int i = 0; i < 5; i++) {
    iter_mineBytom(g_msg[i], 32, g_nonces[i], results_gpu[i], true, handle);
  }

  end = clock();
  std::cout << "mul_gpu Time : " <<(double)(end - start) / CLOCKS_PER_SEC / 5 << "s" << std::endl; 
  std::cout << std::endl;

  for (int i = 0; i < 5; i++) {
    printf("\n%02x: ", i);
    for (int j = 0; j < 32; j++)
      printf("0x%02x, ",results_gpu[i][j]);
  }
  printf("\n\n");

  for (int i = 0; i < 5; i++)
    for (int j = 0; j < 32; j++)
      if (g_golden_results[i][j] != results_gpu[i][j]) {
        std::cerr<<"Result Compare Fail:"<<std::endl;
        std::cerr<<"\tg_golden_results["<<i<<"]["<<j<<"] = "<<g_golden_results[i][j]<<std::endl;
        std::cerr<<"\tresults_gpu["<<i<<"]["<<j<<"] = "<<results_gpu[i][j]<<std::endl;
        exit(EXIT_FAILURE);
      }

  printf("GPU result compare PASS\n");
  return 0;
}

// g++ byte_order.c sha3.c  test_BytomPoW.cpp -I /opt/OpenBLAS/include/ -L/opt/OpenBLAS/lib -lopenblas -lpthread
