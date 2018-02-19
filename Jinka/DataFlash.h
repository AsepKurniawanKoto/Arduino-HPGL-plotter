#ifndef _DataFlash_H_
#define _DataFlash_H_

#include "Arduino.h"
#include <avr/pgmspace.h>

//DataFlash commands
#define FlashPageRead           0xD2    // Main memory page read
#define FlashToBuf1Transfer     0x53    // Main memory page to buffer 1 transfer
#define Buf1Read                0xD4    // Buffer 1 read
#define FlashToBuf2Transfer     0x55    // Main memory page to buffer 2 transfer
#define Buf2Read                0xD6    // Buffer 2 read
#define StatusReg               0xD7    // Status register
#define Buf1ToFlashWE           0x83    // Buffer 1 to main memory page program with built-in erase
#define Buf1Write               0x84    // Buffer 1 write
#define Buf2ToFlashWE           0x86    // Buffer 2 to main memory page program with built-in erase
#define Buf2Write               0x87    // Buffer 2 write
#define ReadMfgID               0x9F    // Read Manufacturer and Device ID

#define ContinuousArrayReadLegacy			0xE8
#define ContinuousArrayReadHighFrequency	0x0B
#define ContinuousArrayReadLowFrequency		0x03

class DataFlash {
public:
	uint16_t dflashPageSize;
	uint16_t currentPageInReadBuffer;
	

	DataFlash();
    void init(uint8_t ssPin, uint16_t pageSize);
    void Read_DF_ID(unsigned char* data);
    void Page_To_Buffer (unsigned char BufferNo, unsigned int PageAdr);
    unsigned char Buffer_Read_Byte (unsigned char BufferNo, unsigned int IntPageAdr);
    void Buffer_Write_Byte (unsigned char BufferNo, unsigned int IntPageAdr, unsigned char Data);
    void Buffer_To_Page (unsigned char BufferNo, unsigned int PageAdr);
	uint8_t readByte(uint32_t address);
	void startContinuousRead(uint32_t startAddress);
	uint8_t readContinuous();
	void stopContinuousRead();
	

private:
    unsigned char slave_select;
    unsigned char DF_SPI_RW (unsigned char output);
    unsigned char Read_DF_status (void);
    void DF_CS_inactive(void);
    void DF_CS_active(void);
};
#endif // _DataFlash_H_
