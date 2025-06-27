/*
 * fs.h
 *
 *  Created on: 06.01.2019
 *      Author: Erich Styger
 */

#ifndef SOURCES_FS_H_
#define SOURCES_FS_H_

#include <stdint.h>
#include <stdbool.h>
#include "W25QXX.h"

#ifdef __cplusplus
extern "C" {
#endif

// #include "CLS1.h"

// uint8_t FS_ParseCommand(const unsigned char* cmd, bool *handled, const CLS1_StdIOType *io);

uint8_t FS_Init(W25QXX_handle_t *handle);

uint8_t FS_Format(void);

uint8_t FS_Mount(void);

uint8_t FS_RunBenchmark(void);

uint8_t FS_PrintStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* SOURCES_FS_H_ */
