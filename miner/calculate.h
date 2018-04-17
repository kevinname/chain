#ifndef __CALCULATE_H__
#define __CALCULATE_H__

#ifdef __cplusplus
extern "c" {
#endif

void btm_generateMatrix(uint8_t *u8Seed);
void btm_setMsg(uint8_t *u8Header);
void btm_setDiff(uint32_t diff);
int btm_mine(uint8_t *u8Nonce);

#ifdef __cplusplus
}
#endif

#endif
