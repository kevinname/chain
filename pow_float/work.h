#ifndef _WORK_H
#define _WORK_H

#include <stdint.h>

struct Transactionstatus {
	char *Bitmap;
};

struct BlockHeader {
	uint64_t Version;
	uint64_t Height;
	char *PreviousBlockId;
	char *Seed;
	uint64_t Timestamp;
	char *TransactionsRoot;
	char *AssetsRoot;
	struct Transactionstatus Status;
	uint64_t Nonce;
	uint64_t Bits;
};
#endif
