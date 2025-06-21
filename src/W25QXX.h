#ifndef __W25QXX_H__
#define __W25QXX_H__

#include "driver_w25qxx.h"
#include "driver_w25qxx_interface.h"

#define W25Q_BLOCK_SIZE_BYTES   65536UL   /* 64 KiB, fixed for every W25Qxx */
#define W25Q_SECTOR_SIZE_BYTES  4096UL    /* 4  KiB, fixed                 */
#define W25Q_PAGE_SIZE_BYTES    256UL     /* 256 B, fixed                  */

#ifdef __cplusplus
extern "C"{
#endif

typedef struct W25QXX_info_s {
    uint8_t  manufacturer_id;
    uint16_t jedec_id;
    uint32_t block_size;
    uint32_t block_count;
    uint32_t sector_size;
    uint32_t sectors_in_block;
    uint32_t page_size;
    uint32_t pages_in_sector;
} W25QXX_info_t;

typedef struct W25QXX_handle_s {
    W25QXX_info_t chip_info;
    w25qxx_handle_t w25qxx_hdl;
} W25QXX_handle_t;

typedef enum {
    W25QXX_Ok = 0,
    W25QXX_Err,
    W25QXX_Timeout
} W25QXX_err_t;

W25QXX_err_t W25QXX_init(W25QXX_handle_t *w25qxx, w25qxx_type_t type);
W25QXX_err_t W25QXX_deinit(W25QXX_handle_t *w25qxx);
W25QXX_err_t W25QXX_read(W25QXX_handle_t *w25qxx, uint32_t address, uint8_t *buf, uint32_t len);
W25QXX_err_t W25QXX_write(W25QXX_handle_t *w25qxx, uint32_t address, uint8_t *buf, uint32_t len);
W25QXX_err_t W25QXX_erase(W25QXX_handle_t *w25qxx, uint32_t address, uint32_t len);
W25QXX_err_t W25QXX_chip_erase(W25QXX_handle_t *w25qxx);

#ifdef __cplusplus
}
#endif

#endif /* __W25QXX_H__ */