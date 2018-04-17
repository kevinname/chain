#include "scrypt.h"
#include "sha3.h"
#include "BytomPoW.h"

static inline void core_mineBytom(
		std::vector<uint8_t> fourSeq[4],
		BytomMatList* matList_int8,
		uint32_t data[64]) {
  Mat256x256i8 *mat=new Mat256x256i8;
  Mat256x256i8 *tmp=new Mat256x256i8;
  Mat256x256i8 *res=new Mat256x256i8[4];

  for(int k=0; k<4; k++) {
    tmp->toIdentityMatrix();  //HKKUO: A). Memory Set
    for(int j=0; j<2; j++) {
      for(int i=0; i<32; i+=2) {
        mat->mul(*tmp, matList_int8->at(fourSeq[k][i]));   //HKKUO: B). General Matrix Multiplication (GEMM)
        tmp->mul(*mat, matList_int8->at(fourSeq[k][i+1])); //HKKUO: B). General Matrix Multiplication (GEMM)
      }
    }
    res[k].copyFrom(*tmp);
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

////////////////// Core Part Finishes Here /////////////////////////

void copyFrom_helper(Mat256x256i8* mat, LTCMemory& ltcMem, int offset) {
  for(int i=0; i<256; i++) {
    // 为什么不是顺序加入的？
    // 是考虑到内存性能的问题，这样可以充分利用内存。
    const Words32& lo=ltcMem.get(i*4+offset);
    const Words32& hi=ltcMem.get(i*4+2+offset);
    for(int j=0; j<64; j++) {
      uint32_t i32=j>=32?hi.get(j-32):lo.get(j);
      //此处按照列进行inner循环，相当于矩阵转秩了，是迁就硬件的考虑
      // 这里的转置是否需要在go里更改？
      mat->d[j*4+0][i]=(i32>>0)&0xFF;
      mat->d[j*4+1][i]=(i32>>8)&0xFF;
      mat->d[j*4+2][i]=(i32>>16)&0xFF;
      mat->d[j*4+3][i]=(i32>>24)&0xFF;
    }
  }
}
void copyFromEven(Mat256x256i8* mat, LTCMemory& ltcMem) {
  copyFrom_helper(mat, ltcMem, 0);
}
void copyFromOdd(Mat256x256i8* mat, LTCMemory& ltcMem) {
  copyFrom_helper(mat, ltcMem, 1);
}

void initMatVec(std::vector<Mat256x256i8*>& matVec, const Words32& X_in) {
  Words32 X=X_in;
  LTCMemory ltcMem;
  for(int i=0; i<128; i++) {
    ltcMem.scrypt(X);
    copyFromEven(matVec[2*i], ltcMem);
    copyFromOdd(matVec[2*i+1], ltcMem);
  }
}

void iter_mineBytom(
    const uint8_t *fixedMessage,
    uint32_t len,
    uint8_t nonce[8],
    uint8_t result[32],
    bool isGpu,
    cublasHandle_t handle)
{
  uint8_t sequence[32];
  sha3_ctx ctx;

  std::vector<uint8_t> fourSeq[4];
  for(int k=0; k<4; k++) { // The k-loop
    rhash_sha3_256_init(&ctx);
    rhash_sha3_update(&ctx, fixedMessage+(len*k/4),len/4);//分四轮消耗掉fixedMessage
    rhash_sha3_final(&ctx, sequence);
	for(int i=0; i<32; i++) fourSeq[k].push_back(sequence[i]);
  }
  uint32_t data[64];
  if (!isGpu)
    core_mineBytom(fourSeq, matList_int8, data);
  else
    core_mineBytom_gpu(fourSeq, matListGpu_int8, data, handle);

  rhash_sha3_256_init(&ctx);
  rhash_sha3_update(&ctx, (uint8_t*)data, 256);
  rhash_sha3_final(&ctx, result);
}

static inline void incrNonce(uint8_t nonce[32])
{
  for(int i=0; i<32; i++) {
    if(nonce[i]!=255) {
      nonce[i]++;
      break;
    } else {
      nonce[i]=0;
    }
  }
}

static inline int countLeadingZero(uint8_t result[32])
{
  int count=0;
  for (int i=31; i>=0; i--) { // NOTE: reverse
    if (result[i] < 1) {
      count+=8;
    } else if (result[i]<2)  {
      count+=7;
      break;
    } else if (result[i]<4)  {
      count+=6;
      break;
    } else if (result[i]<8)  {
      count+=5;
      break;
    } else if (result[i]<16) {
      count+=4;
      break;
    } else if (result[i]<32) {
      count+=3;
      break;
    } else if (result[i]<64) {
      count+=2;
      break;
    } else if (result[i]<128) {
      count+=1;
      break;
    }
  }
  return count;
}

// static inline int test_mineBytom(
//     const uint8_t *fixedMessage,
//     uint32_t len,
//     uint8_t nonce[32],
//     int count,
//     int leadingZeroThres)
// {
//   assert(len%4==0);
//   int step;
//   for(step=0; step<count; step++) {
//     uint8_t result[32];

//     //std::cerr<<"Mine step "<<step<<std::endl;
//     iter_mineBytom(fixedMessage,100,nonce,result);
//     std::cerr<<"Mine step "<<step<<std::endl;
//     for (int i = 0; i < 32; i++) {
//       printf("%02x ", result[i]);
//       if (i % 8 == 7)
//         printf("\n");
//     }
//     if (countLeadingZero(result) > leadingZeroThres)
//       return step;
//     incrNonce(nonce);
//   }
//   return step;
// }
