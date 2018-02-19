#ifndef _HPGL_H_
#define _HPGL_H_

#include "Arduino.h"
#include <avr/pgmspace.h>
#include "DataFlash.h"

#define penWidthUnit_metric 0
#define penWidthUnit_relative 1
#define pixelPlacement_GridIntersection 0
#define pixelPlacement_GridCentered 1
#define EncodeMode_64 0
#define EncodeMode_32 1

class HPGL
{
  public:
    uint32_t dataAddress;
    DataFlash *dflash;
    byte lastData = 0;
  
    String kindPictureName;
    byte kindNumberOfCopies;
    byte kindFileDispositionCode;
    byte kindRenderLastPlotIfUnfinished;
    byte kindAutorotation;
  
    byte qualityLevel;
  
    byte penActive;
    byte lineType;
    uint16_t rotate;
    long plotSizeLength;
    long plotSizeWidth;
    long anchorCornerX;
    long anchorCornerY;
    double scaleX;
    double scaleY;
    uint16_t penWidth[8];
    byte penWidthUnit;
    byte pixelPlacement;
    byte EncodeMode;
    uint16_t fractionalData = 0;
    byte fillType;
    uint16_t fillTypeOption1;
    uint16_t fillTypeOption2;
    uint8_t polylineMode;
    
    uint16_t polygonBufferPointer;
    int polygonBuffer[512];
  
    HPGL(DataFlash *dataFlash);
    byte ReadCommand();
    bool GetParameter(long *parameter);
  //  long GetEncodeParameter();
    bool isParameter(byte parameter);
    void GetString(String *HPGLString);
    bool GetPoint(long *x, long *y);
  
    byte BasicEncode();
    byte GetEncodePoint(long *x, long *y);
  
  private:
    bool GetEncode(long *Decoded);
};


#endif // _HPGL_H_
