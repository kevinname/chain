#include "BytomPoW.h"
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <time.h>


typedef union {
  uint64_t  u64Val;
  uint8_t   u8Val[8];
} u64tou8_t;

u64tou8_t g_nonce;
uint8_t   g_u8Msg[136];
uint32_t  g_u32Diff = 8;
uint32_t  g_u32MsgId;
uint8_t   g_u8Start = 0;

BytomMatList* matList_int8 = NULL;
BytomMatListGpu* matListGpu_int8 = NULL;


static inline void extend( uint32_t* exted, uint8_t *g_seed){

  sha3_ctx *ctx = ( sha3_ctx*)calloc( 1, sizeof( *ctx));
  // uint8_t seedHash[ 4*32];
  uint8_t seedHash[ 4][ 32];

  // std::copy( g_seed, g_seed + 32, seedHash);
  std::copy( g_seed, g_seed + 32, seedHash[ 0]);

  for( int i = 0; i < 3; ++i) {

    rhash_sha3_256_init( ctx);
    rhash_sha3_update( ctx, seedHash[ i], 32);
    rhash_sha3_final( ctx, seedHash[ i+1]);
  }

  for( int i = 0; i < 32; ++i) {

    exted[ i] =  ( seedHash[ i/8][ ( i*4+3)%32]<<24) +
      ( seedHash[ i/8][ ( i*4+2)%32]<<16) +
      ( seedHash[ i/8][ ( i*4+1)%32]<<8) +
      seedHash[ i/8][ ( i*4)%32];
  }

  free( ctx);
}

static void init_seed(Words32 &seed, uint32_t _seed[32])
{
  for (int i = 0; i < 16; i++)
    seed.lo.w[i] = _seed[i];
  for (int i = 0; i < 16; i++)
    seed.hi.w[i] = _seed[16 + i];
}

void print_u8s( const char *info, uint8_t buf[ ], int n)
{
  printf( "%s:", info);
  for ( int i = 0; i < n; i++) {
    if ((i%8) == 0) printf("\n  ");
    printf( "0x%02x, ", (uint8_t)buf[ i]);
  }
  printf( "\n\n");
}

static int _get_leadingZeroCnt(uint8_t *result) {

  int count = 0;

  for (int i = 32 - 1; i >= 0; i--) {

    if (result[ i] < 1)        {count += 8;}
    else if (result[ i] < 2)   {count += 7; break;}
    else if (result[ i] < 4)   {count += 6; break;}
    else if (result[ i] < 8)   {count += 5; break;}
    else if (result[ i] < 16)  {count += 4; break;}
    else if (result[ i] < 32)  {count += 3; break;}
    else if (result[ i] < 64)  {count += 2; break;}
    else if (result[ i] < 128) {count += 1; break;}
    else break;
  }

  return count;
}

extern "C" void btm_generateMatrix(uint8_t *u8Seed) {
  Words32 seed;
  uint32_t exted[32];

  printf("%s\n", __FUNCTION__);

  extend(exted, u8Seed);
  init_seed(seed, exted);
  if (!matList_int8) matList_int8=new BytomMatList;
  initMatVec(matList_int8->matVec, seed);

  if (!matListGpu_int8) matListGpu_int8=new BytomMatListGpu;
  initMatVecGpu(matListGpu_int8, matList_int8);

  g_u8Start |= 2;
}

extern "C" void btm_setMsg(uint8_t *u8Header) {
  printf("%s\n", __FUNCTION__);
  g_u32MsgId = u8Header[0];
  memcpy(g_u8Msg, &u8Header[1], 136);
  memcpy(g_nonce.u8Val, u8Header+128, 8);
  g_u8Start |= 1;
}

extern "C" void btm_setDiff(uint32_t diff) {
  printf("%s\n", __FUNCTION__);
  g_u32Diff = diff;
}

extern "C" int btm_mine(uint8_t *u8Nonce) {
  sha3_ctx ctx;
  uint8_t u8MsgTmp[52] = {
    0x65, 0x6e, 0x74, 0x72, 0x79, 0x69, 0x64, 0x3a,
    0x62, 0x6c, 0x6f, 0x63, 0x6b, 0x68, 0x65, 0x61,
    0x64, 0x65, 0x72, 0x3a,
  };
  uint8_t u8Msg[32];
  uint8_t u8Res[32];

  if(g_u8Start != 3)
    return -1;

  cublasHandle_t handle;
  cublasStatus_t stat = cublasCreate(&handle);
  if (stat != CUBLAS_STATUS_SUCCESS) {
    printf("create cublas error!\n");
    return -1;
  }

  int msg_id = g_u32MsgId;
  do {
    memcpy(g_u8Msg + 128, g_nonce.u8Val, 8);

    rhash_sha3_256_init(&ctx);
    rhash_sha3_update(&ctx, g_u8Msg, 136);
    rhash_sha3_final(&ctx, u8MsgTmp + 20);

    print_u8s("first hash result", u8MsgTmp, 52);

    rhash_sha3_256_init(&ctx);
    rhash_sha3_update(&ctx, u8MsgTmp, 52);
    rhash_sha3_final(&ctx, u8Msg);

    iter_mineBytom(u8Msg, 32, u8Nonce, u8Res, true, handle);
    int diff = _get_leadingZeroCnt(u8Res);
    if (diff >= g_u32Diff) {
      memcpy(u8Nonce, g_nonce.u8Val, 8);
      cublasDestroy(handle);
      return msg_id;
    }

    g_nonce.u64Val += 1;
  } while(1);

  cublasDestroy(handle);
  return msg_id;
}


