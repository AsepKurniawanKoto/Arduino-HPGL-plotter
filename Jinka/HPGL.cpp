
// //IN;LT;SP1;..PU523,923;..PD523,5004;..PU1275,893;..PD1275,4396,4429,4396,4429,893,1275,893;..PU0,0;PG;

#include "HPGL.h"
#include "Arduino.h"
#include "DataFlash.h"

char HPGLCode[] = {
// 3.Setup of Plotter
/*0*/'I','N',//(Initialize)
/*1*/'D','F',//(Defalt)
/*2*/'I','P',//(Input P1 and P2)
/*3*/'S','C',//(Scale)
// 4. Basic Plot Command
/*4*/'S','P',//(Select Pen)
/*5*/'P','U',//(Pen Up)
/*6*/'P','D',//(Pen Down)
/*7*/'P','A',//(Plot Absolute)
/*8*/'P','R',//(Plot Relative)
/*9*/'L','T',//(Line Type)
// 5. Plot of Circle, Arc, and Polygon
/*10*/'C','T',//(Chord Tolerance)
/*11*/'C','I',//(Circle)
/*12*/'A','A',//(Arc Absolute)
/*13*/'A','R',//(Arc Relative)
/*14*/'F','T',//(Fill Type)
/*15*/'P','T',//(Pen Thickness)
/*16*/'W','G',//(Fill Wedge)
/*17*/'E','W',//(Edge Wedge)
/*18*/'R','A',//(Fill Absolute)
/*19*/'E','A',//(Edge Rectangle Absolute)
/*20*/'R','R',//(Fill Rectangle Relative)
/*21*/'E','R',//(Edge Rectangle Relative)
/*22*/'P','M',//(Polygon Mode)
/*23*/'E','P',//(Edge Polygon)
/*24*/'F','P',//(Fill Polygon)
// 6. Character Plot Command
/*25*/'L','B',//(Label)
/*26*/'D','T',//(Define Terminator)
/*27*/'S','I',//(Absolute Character Size)
/*28*/'S','R',//(Relative Character Size)
/*29*/'S','U',//(User Unit Character Size)
/*30*/'S','L',//(Character Slant)
/*31*/'D','I',//(Absolute Direction)
/*32*/'D','R',//(Relative Direction)
/*33*/'D','U',//(User Unit Direction)
/*34*/'D','V',//(Vertical Label Direction)
/*35*/'L','O',//(Label origin)
/*36*/'C','P',//(Character Plot)
/*37*/'E','S',//(Extra Space)
/*38*/'B','L',//(Buffer Label)
/*39*/'P','B',//(Print Bufferd Label)
/*40*/'C','S',//(Designate Standard Character Set)
/*41*/'C','A',//(Designate Alternate Character Set)
/*42*/'S','S',//(Select Standard Character set)
/*43*/'S','A',//(Select Alternate Character Set)
// 7. Change of Plot Area
/*44*/'I','W',//(Input window)
/*45*/'R','O',//(Rotate Coodinate System)
// 8. Plotter Control
/*46*/'P','G',//(Page)
/*47*/'A','F',//(Advance Full Page)
/*48*/'N','R',//(Not Ready)
/*49*/'P','S'};//(Paper Size)


char HPGLCode2[] = {

/* 0 */'A','A',//   |+  | Arc Absolute
/* 1 */'A','C',//   |+  | Anchor corner
/* 2 */'A','D',//   |*  | Alternate font Definition
/* 3 */'A','F',//   |+  | Advance Full page [same as PG]
/* 4 */'A','H',//   |+  | Advance Half page [same as PG]
/* 5 */'A','P',//   |  .| Automatic pen operations
/* 6 */'A','R',//   |+  | Arc Relative
/* 7 */'A','S',//   |  .| Acceleration select
/* 8 */'A','T',//   |+  | Arc through three points
/* 9 */'B','P',//   |+  | Begin Plot
/* 10 */'B','L',//   |+  | Buffer Label
/* 11 */'B','R',//   |+  | Bezier curve, Relative
/* 12 */'B','Z',//   |+  | Bezier curve, Absolute
/* 13 */'C','A',//   |+  | Designate alternate character set
/* 14 */'C','C',//   | - | Character Chord angle
/* 15 */'C','F',//   | - | Character Fill mode
/* 16 */'C','I',//   |+  | Circle
/* 17 */'C','M',//   |  .| Character selection mode
/* 18 */'C','O',//   |+  | File comment
/* 19 */'C','P',//   |+  | Character plot
/* 20 */'C','R',//   |+  | Color Range
/* 21 */'C','S',//   |+  | Designate standard character set
/* 22 */'C','T',//   |+  | Chord tolerance
/* 23 */'C','V',//   |  .| Curved line generator
/* 24 */'D','C',//   |  .| Digitize clear
/* 25 */'D','F',//   |+  | Default
/* 26 */'D','I',//   |+  | Absolute direction
/* 27 */'D','L',//   | - | Define downloadable character
/* 28 */'D','P',//   |  .| Digitize point
/* 29 */'D','R',//   |+  | Relative direction
/* 30 */'D','S',//   | - | Designate character into slot
/* 31 */'D','T',//   |+  | Define label terminator
/* 32 */'D','U',//   |2  | User Unit Direction
/* 33 */'D','V',//   |+  | text Direction Vertical
/* 34 */'E','A',//   |+  | Edge rectangle absolute
/* 35 */'E','C',//   |  .| Enable paper Cutter
/* 36 */'E','P',//   |+  | Edge polygon
/* 37 */'E','R',//   |+  | Edge rectangle relative
/* 38 */'E','S',//   |+  | Extra space
/* 39 */'E','W',//   |+  | Edge wedge
/* 40 */'F','I',//   | - | pcl Font ID
/* 41 */'F','N',//   | - | pcl secondary Font Number
/* 42 */'F','P',//   |+  | Fill polygon
/* 43 */'F','R',//   | - | FRame advance
/* 44 */'F','S',//   |  .| Force select
/* 45 */'F','T',//   |+  | Fill type
/* 46 */'G','C',//   |  .| Group count
/* 47 */'G','M',//   |  .| Graphics memory
/* 48 */'I','M',//   | - | Input error reporting mask
/* 49 */'I','N',//   |+  | Initialize
/* 50 */'I','P',//   |+  | Input P1 and P2
/* 51 */'I','R',//   |+  | Input Relative P1 and P2
/* 52 */'I','V',//   | - | Invoke character slot
/* 53 */'I','W',//   |+  | Input window
/* 54 */'K','Y',//   |  .| Define key
/* 55 */'L','A',//   |*  | Line Attributes
/* 56 */'L','B',//   |+  | Label
/* 57 */'L','M',//   | - | Label mode (for two-byte character sets)
/* 58 */'L','O',//   |+  | Label origin
/* 59 */'L','T',//   |+  | Line type
/* 60 */'M','C',//   | - | Merge Control
/* 61 */'M','G',//   |+  | Message [same as WD]
/* 62 */'M','T',//   |  .| Media Type
/* 63 */'N','P',//   |+  | Number of Pens
/* 64 */'N','R',//   |  .| Not ready (unload page and go offline)
/* 65 */'O','A',//   |  .| Output actual position and pen status
/* 66 */'O','C',//   |  .| Output commanded position and pen status
/* 67 */'O','D',//   |  .| Output digitized point and pen status
/* 68 */'O','E',//   | - | Output error
/* 69 */'O','F',//   | - | Output factors
/* 70 */'O','G',//   |  .| Output group count
/* 71 */'O','H',//   | - | Output hard-clip limits
/* 72 */'O','I',//   |  .| Output identification
/* 73 */'O','K',//   |  .| Output key
/* 74 */'O','L',//   | - | Output label length
/* 75 */'O','O',//   |  .| Output options
/* 76 */'O','P',//   |+  | Output P1 and P2
/* 77 */'O','S',//   | - | Output status
/* 78 */'O','T',//   |  .| Output carousel type
/* 79 */'O','W',//   |+  | Output window
/* 80 */'P','A',//   |+  | Plot absolute
/* 81 */'P','B',//   |+  | Print buffered label
/* 82 */'P','C',//   |+  | Pen Color
/* 83 */'P','D',//   |+  | Pen down
/* 84 */'P','E',//   |+  | Polyline Encoded
/* 85 */'P','G',//   |+  | Page feed
/* 86 */'P','M',//   | 1 | Polyline Mode
/* 87 */'P','P',//   | - | Pixel placement
/* 88 */'P','R',//   |+  | Plot relative
/* 89 */'P','S',//   |+  | Plot Size
/* 90 */'P','T',//   |+  | Pen thickness
/* 91 */'P','U',//   |+  | Pen up
/* 92 */'P','W',//   |+  | Pen Width
/* 93 */'Q','L',//   |  .| Quality Level 
/* 94 */'R','A',//   |+  | Fill rectangle absolute
/* 95 */'R','F',//   | - | Raster Fill pattern
/* 96 */'R','O',//   |+  | Rotate coordinate system
/* 97 */'R','P',//   | - | Replot
/* 98 */'R','R',//   |+  | Fill rectangle relative
/* 99 */'R','T',//   |+  | Relative arc through Three points
/* 100 */'S','A',//   |+  | Select alternate character set
/* 101 */'S','B',//   | - | Scalable or Bitmap font selection
/* 102 */'S','C',//   |+  | Scale
/* 103 */'S','D',//   |*  | Standard font attribute Definition
/* 104 */'S','I',//   |+  | Absolute character size
/* 105 */'S','L',//   |+  | Character slant
/* 106 */'S','M',//   |+  | Symbol mode
/* 107 */'S','P',//   |+  | Select pen
/* 108 */'S','R',//   |+  | Relative character size
/* 109 */'S','S',//   |+  | Select standard character set
/* 110 */'S','T',//   |  .| Sort vectors
/* 111 */'S','U',//   |  1| User Unit Character Size
/* 112 */'S','V',//   | - | Screened Vectors
/* 113 */'T','D',//   | - | Transparent Data
/* 114 */'T','L',//   |+  | Tick length
/* 115 */'T','R',//   | - | Transparency mode
/* 116 */'U','C',//   |+  | User-defined character
/* 117 */'U','F',//   | - | User-defined fill type
/* 118 */'U','L',//   |+  | User-defined line type
/* 119 */'V','S',//   |  .| Velocity select
/* 120 */'W','D',//   |+  | Write to display
/* 121 */'W','G',//   |+  | Fill wedge
/* 122 */'W','U',//   |+  | pen Width Unit
/* 123 */'X','T',//   |+  | X-Tick
/* 124 */'Y','T'};//  |+  | Y-Tick



HPGL::HPGL(DataFlash *dataFlash)
{
  dflash = dataFlash;
}

byte HPGL::ReadCommand()
{
  byte b;
  char commandBuffer[2];

//  Serial.println(char(lastData));

  if(lastData >= 'A')
  {
    commandBuffer[0] = lastData;
  }
  else
  {
    while(1)
    {
      commandBuffer[0] = dflash->readContinuous();
      dataAddress++;
      if(commandBuffer[0] >= 'A')
      {
        break;
      }
    }
  }
  commandBuffer[1] = dflash->readContinuous();
  dataAddress++;

  lastData = 0;

  for(int i=0;i<sizeof(HPGLCode2)/2;i++)
  {
    if((commandBuffer[0] == HPGLCode2[i*2]) && (commandBuffer[1] == HPGLCode2[(i*2)+1]))
    {
      Serial.println();
      Serial.print("Command = ");
      Serial.print(char(commandBuffer[0]));
      Serial.print(char(commandBuffer[1]));
      Serial.print(" ");
      return i;
    }
  }
  Serial.println();
  Serial.print("Unrecognized command = ");
  Serial.print(char(commandBuffer[0]));
  Serial.println(char(commandBuffer[1]));
}

bool HPGL::GetParameter(long *parameter)
{
  char c;
  char parameterBuffer[10];
  uint8_t parameterBufferCounter;

  if(lastData >= 'A')
  {
    return false;
  }

  for(int i=0;i<10;i++)
  {
    c = dflash->readContinuous();
    dataAddress++;
//    Serial.println();
//    Serial.print(c);
//    Serial.println();
    if((c >= '0') && (c <= '9'))
    {
      parameterBuffer[i] = c;
    }
    else
    {
      if(i == 0)
      {
        lastData = c;
        return false;
      }
      parameterBufferCounter = i;
      break;
    }
  }

  uint32_t mul10 = 1;
  *parameter = 0;
  for(int8_t i=parameterBufferCounter-1;i >= 0;i--)
  {
    *parameter += (parameterBuffer[i] - '0') * mul10;
    mul10 *= 10;
  }

  //Serial.print("parameter = ");
  Serial.print(*parameter);
  Serial.print(" ");
//  Serial.print("c = ");
//  Serial.println(c);
//  if(c == ',')
//  {
//    return true;
//  }
//  else
//  {
    lastData = c;
//    return false;
//  }
  return true;
  
}

void HPGL::GetString(String *HPGLString)
{
  char c;
  *HPGLString = "";

  while(1)
  {
    c = dflash->readContinuous();
    dataAddress++;
    if((c == ';') || (c == ','))
    {
      *HPGLString += c;
    }
    else
    {
      break;
    }
  }
}
bool HPGL::GetPoint(long *x, long *y)
{
  if(GetParameter(x))
  {
    return GetParameter(y);
  }
  return false;
}
bool HPGL::GetEncode(long *Decoded)
{
  uint32_t base64 = 1;
  uint32_t base32 = 1;
  //long Value = 0;
  byte Data;
  *Decoded = 0;
  
  while(1)
  {
    if(EncodeMode == EncodeMode_64)
    {
      Data = dflash->readContinuous();
      dataAddress++;
      
//      Serial.print("Data = ");
//      Serial.println(Data);
      if((Data >= 63) && (Data <= 126))
      {
        *Decoded += (Data - 63) * base64;
//        Serial.print("Decoded = ");
//        Serial.println(*Decoded);
      }
      else if((Data >= 191) && (Data <= 254))
      {
        *Decoded += (Data - 191) * base64;
//        Serial.print("Decoded = ");
//        Serial.println(*Decoded);
        break;
      }
      else
      {
        lastData = Data;
        return false;
      }
      base64 *= 64;
    }
    else if(EncodeMode == EncodeMode_32)
    {
      Data = dflash->readContinuous();
      dataAddress++;
      if((Data >= 63) && (Data <= 94))
      {
        *Decoded += (Data - 63) * base32;
      }
      else if((Data >= 95) && (Data <= 126))
      {
        *Decoded += (Data - 95) * base32;
        break;
      }
      else
      {
        lastData = Data;
        return 1;
      }
      base32 *= 32;
    }
  }
  if(*Decoded % 2 == 0)
  {
    *Decoded /= 2;  
  }
  else
  {
    *Decoded /= -2;
  }
  *Decoded /= (pow(2,fractionalData));

  return true;
}
byte HPGL::GetEncodePoint(long *x, long *y)
{
  byte Data;
  if(GetEncode(x))
  {
    GetEncode(y);
//    Serial.print("Data EncodeX = ");
//    Serial.println(*x);
//    Serial.print("Data EncodeY = ");
//    Serial.println(*y);
    return 0;
  }
  return lastData;
//
//  return true;
}
byte HPGL::BasicEncode()
{
  if(EncodeMode == EncodeMode_64)
  {
    dataAddress++;
    return dflash->readContinuous() - 191;
    
  }
  else if(EncodeMode == EncodeMode_32)
  {
    dataAddress++;    
    return dflash->readContinuous() - 95;
  }
  
}


bool HPGL::isParameter(byte parameter)
{
  if(((parameter >= '0') && (parameter <= '9')) || (parameter == ','))
  {
    return true;
  }
  return false;
}


