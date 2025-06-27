/* MODULE CLS1. */
#include <ctype.h> /* for isalnum*/

#include "src/printf/printf.h"
#include "src/UTIL1/UTIL1.h"

#define std_printf printf_

/*
** ===================================================================
**     Method      :  PrintMemory (component Shell)
**
**     Description :
**         Prints a chunk of memory bytes in a formatted way.
**     Parameters  :
**         NAME            - DESCRIPTION
**       * hndl            - Pointer to 
**         startAddr       - Memory start address
**         endAddr         - Memory end address
**         addrSize        - Number of bytes for the address
**                           (1, 2, 3 or 4)
**         bytesPerLine    - Number of bytes per line
**         readfp          - Function pointer to read the memory.
**                           Returns error code, uses a device handle,
**                           32bit address with a pointer to a buffer
**                           and a buffer size.
**       * io              - Pointer to I/O to be used
**     Returns     :
**         ---             - Error code
** ===================================================================
*/
uint8_t CLS1_PrintMemory(void *hndl, uint32_t startAddr, uint32_t endAddr, uint8_t addrSize, uint8_t bytesPerLine, uint8_t (*readfp)(void *, uint32_t, uint8_t*, size_t))
{
  #define NOF_BYTES_PER_LINE 32 /* how many bytes are shown on a line. This defines as well the chunk size we read from memory */
  #define MAX_NOF_BYTES_PER_LINE 32
  uint8_t buf[MAX_NOF_BYTES_PER_LINE]; /* this is the chunk of data we get (per line in output) */
  uint8_t str[3*MAX_NOF_BYTES_PER_LINE+((MAX_NOF_BYTES_PER_LINE+1)/8)+1]; /* maximum string for output:
                                              - '3*' because each byte is 2 hex digits plus a space
                                              - '(NOF_BYTES_PER_LINE+1)/8' because we add a space between every 8 byte block
                                              - '+1' for the final zero byte */
  uint32_t addr;
  uint8_t res=0, j, bufSize;
  uint8_t ch;

  if (endAddr<startAddr) {
    std_printf("\r\n*** End address must be larger or equal than start address\r\n");
    return ERR_RANGE;
  }
  for(addr=startAddr; addr<=endAddr; /* nothing */ ) {
    if (endAddr-addr+1 >= bytesPerLine) { /* read only part of buffer */
      bufSize = bytesPerLine; /* read full buffer */
    } else {
      bufSize = (uint8_t)(endAddr-addr+1);
    }
    if (readfp(hndl, addr, buf, bufSize)!=ERR_OK) {
      stdErr("\r\n*** Read failed!\r\n");
      return ERR_FAILED;
    }
    if (res != ERR_OK) {
      std_printf("\r\n*** Failure reading memory block!\r\n");
      return ERR_FAULT;
    }
    /* write address */
    UTIL1_strcpy(str, sizeof(str), (unsigned char*)"0x");
    UTIL1_strcatNumHex(str, sizeof(str), addr, addrSize);
    UTIL1_chcat(str, sizeof(str), ':');
    std_printf(str);
    /* write data in hex */
    str[0] = '\0';
    for (j=0; j<bufSize; j++) {
      if ((j)==0) {
        UTIL1_chcat(str, sizeof(str), ' ');
      }
      UTIL1_strcatNum8Hex(str, sizeof(str), buf[j]);
      UTIL1_chcat(str, sizeof(str), ' ');
    }
    for (/*empty*/; j<bytesPerLine; j++) { /* fill up line */
      UTIL1_strcat(str, sizeof(str), (unsigned char*)"-- ");
    }
    std_printf(str);
    /* write in ASCII */
    std_printf(" ");
    for (j=0; j<bufSize; j++) {
      ch = buf[j];
      if (ch >= ' ' && ch <= 0x7f) {
        std_printf("%c", ch);
      } else {
        std_printf(".");
      }
    }
    for (/*empty*/; j<bytesPerLine; j++) { /* fill up line */
      UTIL1_strcat(str, sizeof(str), (unsigned char*)"-- ");
    }
    std_printf("\r\n");
    addr += bytesPerLine;
  }
  return ERR_OK;
}
