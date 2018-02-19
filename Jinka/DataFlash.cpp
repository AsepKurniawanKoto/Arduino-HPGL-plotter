#include "DataFlash.h"

#define PAGE_BITS 10

// configuration for the Atmel AT45DB161D device
DataFlash::DataFlash(void)
{
	currentPageInReadBuffer = 0xFFFF;
}

void DataFlash::init(unsigned char ssPin, uint16_t pageSize)
{
    char clr;
	
	dflashPageSize = pageSize;

    // setup the slave select pin
    slave_select = ssPin;
    pinMode (slave_select, OUTPUT);

    // setup the SPI pins
    pinMode(MOSI, OUTPUT); // MOSI
    pinMode(MISO, INPUT);  // MISO
    pinMode(SCK, OUTPUT); // SCK

    // disable the SPI device
    digitalWrite(slave_select, HIGH);

    // configure the SPI registers
    SPCR = (1<<SPE)|(1<<MSTR);
    SPSR = (1<<SPI2X);

    // clear the SPI registers
    clr = SPSR;
    clr = SPDR;
	
	// DF_CS_active();
	// for(int i=0;<100;i++)
	// {
		// Serial.println(DF_SPI_RW(0x00));
	// }
	// DF_CS_inactive();
}

unsigned char DataFlash::DF_SPI_RW(unsigned char output)
{
    SPDR = output;                 // Start the transmission
    while (!(SPSR & (1<<SPIF)))    // Wait the end of the transmission
    {
    };
    return SPDR;
}

unsigned char DataFlash::Read_DF_status (void)
{
    unsigned char result;

    DF_CS_active();
    result = DF_SPI_RW(StatusReg);
    result = DF_SPI_RW(0x00);
    DF_CS_inactive();

    return result;
}

void DataFlash::Read_DF_ID (unsigned char* data)
{
    DF_CS_active();
    DF_SPI_RW(ReadMfgID);
    data[0] = DF_SPI_RW(0x00);
    data[1] = DF_SPI_RW(0x00);
    data[2] = DF_SPI_RW(0x00);
    data[3] = DF_SPI_RW(0x00);
    DF_CS_inactive();
}

/*****************************************************************************
 *
 *  Function name : Page_To_Buffer
 *
 *  Returns :       None
 *  
 *	Parameters :	BufferNo	->	Decides usage of either buffer 1 or 2
 * 
 *			        PageAdr		->	Address of page to be transferred to buffer
 * 
 *	Purpose :	Transfers a page from flash to DataFlash SRAM buffer
 * 					
 * 
 ******************************************************************************/
void DataFlash::Page_To_Buffer (unsigned char BufferNo, unsigned int PageAdr)
{
    DF_CS_active();
    if (1 == BufferNo)                                              //transfer flash page to buffer 1
    {
        DF_SPI_RW(FlashToBuf1Transfer);
        DF_SPI_RW((unsigned char)(PageAdr >> (16 - PAGE_BITS)));    //upper part of page address
        DF_SPI_RW((unsigned char)(PageAdr << (PAGE_BITS - 8)));     //lower part of page address
        DF_SPI_RW(0x00);                                            //don't cares
    }
    else if (2 == BufferNo)                                         //transfer flash page to buffer 2
    {
        DF_SPI_RW(FlashToBuf2Transfer);
        DF_SPI_RW((unsigned char)(PageAdr >> (16 - PAGE_BITS)));	//upper part of page address
        DF_SPI_RW((unsigned char)(PageAdr << (PAGE_BITS - 8)));     //lower part of page address
        DF_SPI_RW(0x00);						                    //don't cares
    }
    DF_CS_inactive();

    while(!(Read_DF_status() & 0x80));      //monitor the status register, wait until busy-flag is high
}

/*****************************************************************************
 *  
 *	Function name : Buffer_Read_Byte
 *  
 *	Returns :		One read byte (any value)
 *
 *	Parameters :	BufferNo	->	Decides usage of either buffer 1 or 2
 * 
 *					IntPageAdr	->	Internal page address
 *  
 *	Purpose :		Reads one byte from one of the DataFlash
 * 
 *					internal SRAM buffers
 * 
 ******************************************************************************/
unsigned char DataFlash::Buffer_Read_Byte (unsigned char BufferNo, unsigned int IntPageAdr)
{
    unsigned char data;

    DF_CS_active();
    if (1 == BufferNo)      //read byte from buffer 1
    {
        DF_SPI_RW(Buf1Read);			//buffer 1 read op-code
        DF_SPI_RW(0x00);				//don't cares
        DF_SPI_RW((unsigned char)(IntPageAdr>>8));  //upper part of internal buffer address
        DF_SPI_RW((unsigned char)(IntPageAdr));     //lower part of internal buffer address
        DF_SPI_RW(0x00);				//don't cares
        data = DF_SPI_RW(0x00);			//read byte
    }
    else if (2 == BufferNo)  //read byte from buffer 2
    {
        DF_SPI_RW(Buf2Read);            //buffer 2 read op-code
        DF_SPI_RW(0x00);                //don't cares
        DF_SPI_RW((unsigned char)(IntPageAdr>>8));  //upper part of internal buffer address
        DF_SPI_RW((unsigned char)(IntPageAdr));     //lower part of internal buffer address
        DF_SPI_RW(0x00);                //don't cares
        data = DF_SPI_RW(0x00);         //read byte
    }
    DF_CS_inactive();

    return data;    //return the read data byte
}

/*****************************************************************************
 *  
 *	Function name : Buffer_Write_Byte
 * 
 *	Returns :		None
 *  
 *	Parameters :	IntPageAdr	->	Internal page address to write byte to
 * 
 *			BufferAdr	->	Decides usage of either buffer 1 or 2
 * 
 *			Data		->	Data byte to be written
 *  
 *	Purpose :		Writes one byte to one of the DataFlash
 * 
 *					internal SRAM buffers
 *
 ******************************************************************************/
void DataFlash::Buffer_Write_Byte (unsigned char BufferNo, unsigned int IntPageAdr, unsigned char Data)
{
    DF_CS_active();
    if (1 == BufferNo)              //write byte to buffer 1
    {
        DF_SPI_RW(Buf1Write);       //buffer 1 write op-code
        DF_SPI_RW(0x00);            //don't cares
        DF_SPI_RW((unsigned char)(IntPageAdr>>8));  //upper part of internal buffer address
        DF_SPI_RW((unsigned char)(IntPageAdr));     //lower part of internal buffer address
        DF_SPI_RW(Data);            //write data byte
    }
    else if (2 == BufferNo)         //write byte to buffer 2
    {
        DF_SPI_RW(Buf2Write);       //buffer 2 write op-code
        DF_SPI_RW(0x00);            //don't cares
        DF_SPI_RW((unsigned char)(IntPageAdr>>8));  //upper part of internal buffer address
        DF_SPI_RW((unsigned char)(IntPageAdr));     //lower part of internal buffer address
        DF_SPI_RW(Data);            //write data byte
    }
    DF_CS_inactive();
}

/*****************************************************************************
 * 
 * 
 *	Function name : Buffer_To_Page
 * 
 *	Returns :		None
 *  
 *	Parameters :	BufferAdr	->	Decides usage of either buffer 1 or 2
 * 
 *			PageAdr		->	Address of flash page to be programmed
 *  
 *	Purpose :	Transfers a page from DataFlash SRAM buffer to flash
 * 
 *			 
 ******************************************************************************/
void DataFlash::Buffer_To_Page (unsigned char BufferNo, unsigned int PageAdr)
{
    DF_CS_active();
    if (1 == BufferNo)
    {
        DF_SPI_RW(Buf1ToFlashWE);           //buffer 1 to flash with erase op-code
        DF_SPI_RW((unsigned char)(PageAdr >> (16 - PAGE_BITS))); //upper part of page address
        DF_SPI_RW((unsigned char)(PageAdr << (PAGE_BITS - 8)));  //lower part of page address
        DF_SPI_RW(0x00);                    //don't cares
    }
    else if (2 == BufferNo)
    {
        DF_SPI_RW(Buf2ToFlashWE);           //buffer 2 to flash with erase op-code
        DF_SPI_RW((unsigned char)(PageAdr >> (16 - PAGE_BITS)));    //upper part of page address
        DF_SPI_RW((unsigned char)(PageAdr << (PAGE_BITS - 8)));     //lower part of page address
        DF_SPI_RW(0x00);                    //don't cares
    }
    DF_CS_inactive();

    while(!(Read_DF_status() & 0x80));      //monitor the status register, wait until busy-flag is high
}

void DataFlash::DF_CS_inactive()
{
    digitalWrite(slave_select,HIGH);
}
void DataFlash::DF_CS_active()
{
    digitalWrite(slave_select,LOW);
}
uint8_t DataFlash::readByte(uint32_t address)
{
	uint16_t page = address / dflashPageSize;
	uint16_t reminder = address % dflashPageSize;
	
	
	if(currentPageInReadBuffer != page)
	{
		// Serial.print("currentPageInReadBuffer = ");
		// Serial.println(currentPageInReadBuffer);
		// Serial.print("page = ");
		// Serial.println(page);
		
		Page_To_Buffer (1, page);
		currentPageInReadBuffer = page;
	}
		// Serial.print("reminder = ");
		// Serial.println(reminder);

	return Buffer_Read_Byte(1, reminder);
}

void DataFlash::startContinuousRead(uint32_t startAddress)
{
	uint32_t pageComprise = (startAddress / dflashPageSize) << PAGE_BITS;
	pageComprise += startAddress % dflashPageSize;
	
	DF_CS_active();
	DF_SPI_RW(ContinuousArrayReadLowFrequency);
	DF_SPI_RW((unsigned char)(pageComprise >> 16));
	DF_SPI_RW((unsigned char)(pageComprise >> 8));
	DF_SPI_RW((unsigned char)(pageComprise >> 0));
}
uint8_t DataFlash::readContinuous()
{
	return DF_SPI_RW(0x00);
}
void DataFlash::stopContinuousRead()
{
	DF_CS_inactive();
}
