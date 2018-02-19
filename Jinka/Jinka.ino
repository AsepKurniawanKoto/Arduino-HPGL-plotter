#include <LiquidCrystal.h>
#include <EEPROM.h>
#include "DataFlash.h"

#define numberOfPen 8
#include "HPGL.h"
#include "PWM.h"

//0  f+ offline
//1  v+ test
//2  offline setup test origin reset
//3  reset ----> vcc
//4*** -ledoffline
//5*** -ledreset
//6
//7  v- setup
//8  f- origin
//9 v- f- v+ f+
//10*** led+
//11*** -ledsetup
//12*** -ledreset
//13
//Pin
LiquidCrystal lcd(23, 25, 27, 29, 31, 33, 35, 37, 39, 41, 43);
int button[] = {22, 24, 26, 28, 30, 32, 34,  36, 38, 40, 42, 44, 46, 48};
long baudRate[] = {9600, 115200};
int PosX0Sensor = 45;
int PosX0SensorPU = 47;
int XStep = 3;
int YStep = 2;
int XDir = 6;
int YDir = 5;
int stepperEnable = 8;
int penA = 10;
int penB = 9;

long timeJobStart;
long watchDog;
int delaySpeed;

#define Reset 1
#define Offline 2
#define Setup 3
#define Test 4
#define Origin 5
#define Fplus 6
#define Fminus 7
#define Vplus 8
#define Vminus 9

#define FplusLong 10
#define FminusLong 11
#define VplusLong 12
#define VminusLong 13

#define menuReset 0
#define menuOffline 1
//#define menuSetup 2
#define menuOrigin 5

#define minSpeed 1
#define maxSpeed 30
#define minForce 0
#define maxForce 255

//#define mineSpeed 0
//#define maxeSpeed 10
//#define minBaud 0
//#define maxBaud 255

#define maxX 4800*4
//#define mmPerStep 0.254
//#define mmPerStep 25.4/200
#define mmPerStep 25.4/800

#define scale 1.5748031496062992125984251968504

#define dflashPageSize 528

byte menu = 0;
double scalePosX;
double scalePosY;

int16_t PosX;
int32_t PosY;

int16_t UserPosX;
int32_t UserPosY;

bool penDownStatus = false;

//double scaleX = 1;
//double scaleY = 1;
byte PolylineMode = 0;

long pointX, pointY;
bool PolylineEncode;
bool execute = false;
int longPressCounter = 0;
byte longPressStatus;

struct settings_t
{
  int Speed;
  int Force;
  uint32_t dataSizeinMemory;
} settings;

DataFlash dflash;
HPGL hpgl = HPGL(&dflash);

uint16_t dflashTransferBufferCounter = 0;
uint8_t dflashCurrentTransferBuffer = 1;
bool serialDataTransfer = false;
uint32_t dflashSerialInputCounter = 0;

byte cm2;
byte cm1;
byte c;

#define bufferStartAddress 0

void setup() {
  digitalWrite(stepperEnable, HIGH);
  pinMode(stepperEnable, OUTPUT);

  Serial.begin(115200);
  Serial1.begin(9600);
  Serial.println("Initialize...");

  dflash.init(SS,dflashPageSize);

  pinMode(button[2], OUTPUT);
  pinMode(button[9], OUTPUT);
  pinMode(button[10], OUTPUT);
  pinMode(button[4], OUTPUT);
  pinMode(button[5], OUTPUT);
  pinMode(button[11], OUTPUT);

  pinMode(penA, OUTPUT);
  pinMode(penB, OUTPUT);

  analogWrite(penA, 0);
  analogWrite(penB, 0);

  digitalWrite(button[4], HIGH);
  digitalWrite(button[5], HIGH);
  digitalWrite(button[10], HIGH);
  digitalWrite(button[11], HIGH);

  pinMode(button[0], INPUT_PULLUP);
  pinMode(button[1], INPUT_PULLUP);
  pinMode(button[3], INPUT_PULLUP);
  pinMode(button[7], INPUT_PULLUP);
  pinMode(button[8], INPUT_PULLUP);

  pinMode(PosX0Sensor, INPUT_PULLUP);
  pinMode(PosX0SensorPU, OUTPUT);

  pinMode(XStep, OUTPUT);
  pinMode(YStep, OUTPUT);
  pinMode(XDir, OUTPUT);
  pinMode(YDir, OUTPUT);

  lcd.begin(16, 2);
  lcd.print("Initialize...   ");

  unsigned char data[4];
  dflash.Read_DF_ID(data);

  if(data[0] != 0x1F)
  {
    lcd.setCursor(0, 1);
    lcd.print("Memory Error!!! ");
    Serial.print("Memory Error!!! ");
    Serial.println(data[0]);
    while(1);
  }

  digitalWrite(stepperEnable, LOW);
  digitalWrite(XDir, LOW); 

  delaySpeed = toDelaySpeed(20);

  long startZero = millis();
  while(digitalRead(PosX0Sensor))
  {
    XMove(-1);
    if(millis() - startZero > 30000)
    {
      lcd.setCursor(0, 1);
      lcd.print("X-axis Error!!! ");
      Serial.print("X-axis Error!!! ");
      while(1);
    }
  }
  digitalWrite(stepperEnable, HIGH);

  eeprom_read_block((void*)&settings, (void*)0, sizeof(settings));
  delaySpeed = toDelaySpeed(settings.Speed);

  PosX = 0;
  PosY = 0;
  UserPosX = 0;
  UserPosY = 0;
  scalePosX = 0;
  scalePosY = 0;

  displayMenu();
  Serial.println("System Begin");
}


void loop() {
  while (Serial1.available())
  {
    cm2 = cm1;
    cm1 = c;
    c = Serial1.read();

//    Serial.print(char(c));
    //Serial.print(c,HEX);

    if (cm1 == 0x03)//start job
    {
      execute = false;
      serialDataTransfer = true;
      dflashSerialInputCounter = 0;

      if(menu == menuOffline)
      {
        digitalWrite(stepperEnable, HIGH);
        menu = menuReset;
        displayMenu();
      }

      Serial.println("Begin transfer data");
    }

    //Serial.println(dflashTransferBufferCounter,HEX);
    dflash.Buffer_Write_Byte(dflashCurrentTransferBuffer, dflashTransferBufferCounter, c);

    dflashTransferBufferCounter++;
    
    if ((c == ';') && (cm2 == 'P') && (cm1 == 'G'))//EndPage
    {
      execute = true;
    }

    if((dflashTransferBufferCounter == dflashPageSize) || execute)
    {

      dflash.Buffer_To_Page(dflashCurrentTransferBuffer, dflashSerialInputCounter/dflashPageSize);

      dflashSerialInputCounter += dflashTransferBufferCounter;

      if(dflashCurrentTransferBuffer == 1)
      {
        dflashCurrentTransferBuffer = 2;
      }
      else
      {
        dflashCurrentTransferBuffer = 1;
      }
      dflashTransferBufferCounter = 0;
    }
    if (execute)
    {
      hpgl.dataAddress = 0;
      serialDataTransfer = false;
      settings.dataSizeinMemory = dflashSerialInputCounter;
      eeprom_write_block((const void*)&settings, (void*)0, sizeof(settings));
      Serial.println("Finish transfer data");
    }
  }

  byte HPGLState = 0;
  uint16_t commandPointer = 0;
  byte commandCount = 0;
  long t1, t2, t3, t4, t5, t6, t7, t8;
  byte b1, b2, lastData;

  if(execute)
  {
    digitalWrite(stepperEnable, LOW);
    hpgl.dataAddress = 0;
    dflash.startContinuousRead(0);
    long timeJobStart = millis();
    
    while (execute)
    {
      byte progress = (byte)((100.0*hpgl.dataAddress)/settings.dataSizeinMemory);
      lcd.setCursor(0,1);
      lcd.print("                ");
      lcd.setCursor(0,1);
      lcd.print(progress);
      lcd.print("% ");
      lcd.print((int)((millis() - timeJobStart)/1000));
      lcd.print("dtk");

      switch (hpgl.ReadCommand())
      {
        case  0 : //AA   |+  | Arc Absolute
          Serial.println("--- AA");
          break;
        case  1 : //AC   |+  | Anchor corner
          if(hpgl.GetParameter(&hpgl.anchorCornerX))
          {
            hpgl.GetParameter(&hpgl.anchorCornerY);
          }
          break;
        case  2 : //AD   |*  | Alternate font Definition
          Serial.println("--- AD");
          break;
        case  3 : //AF   |+  | Advance Full page [same as PG]
          Serial.println("--- AF");
          break;
        case  4 : //AH   |+  | Advance Half page [same as PG]
          Serial.println("--- AH");
          break;
        case  5 : //AP   |  .| Automatic pen operations
          Serial.println("--- AP");
          break;
        case  6 : //AR   |+  | Arc Relative
          Serial.println("--- AR");
          break;
        case  7 : //AS   |  .| Acceleration select
          Serial.println("--- AS");
          break;
        case  8 : //AT   |+  | Arc through three points
          Serial.println("--- AT");
          break;
        case  9 : //BP   |+  | Begin Plot
          if(hpgl.GetParameter(&t1))
          {
            switch (t1)
            {
              case 1:// HPGLKind_PictureName[20];
                hpgl.GetString(&hpgl.kindPictureName);
                break;
              case 2:// HPGLKind_NumberOfCopies;
                if(hpgl.GetParameter(&t1))
                {
                  hpgl.kindNumberOfCopies = (byte)t1;
                }
                break;
              case 3:// HPGLKind_FileDispositionCode;
                if(hpgl.GetParameter(&t1))
                {
                  hpgl.kindFileDispositionCode = (byte)t1;
                }
                break;
              case 4:// HPGLKind_RenderLastPlotIfUnfinished;
                if(hpgl.GetParameter(&t1))
                {
                  hpgl.kindRenderLastPlotIfUnfinished = (byte)t1;
                }
                break;
              case 5:// HPGLKind_Autorotation;
                if(hpgl.GetParameter(&t1))
                {
                  hpgl.kindAutorotation = (byte)t1;
                }
                break;
            }
          }
          break;
        case  10 : //BL   |+  | Buffer Label
          Serial.println("--- BL");
          break;
        case  11 : //BR   |+  | Bezier curve, Relative
          Serial.println("--- BR");
          break;
        case  12 : //BZ   |+  | Bezier curve, Absolute
          Serial.println("--- BZ");
          break;
        case  13 : //CA   |+  | Designate alternate character set
          Serial.println("--- CA");
          break;
        case  14 : //CC   | - | Character Chord angle
          Serial.println("--- CC");
          break;
        case  15 : //CF   | - | Character Fill mode
          Serial.println("--- CF");
          break;
        case  16 : //CI   |+  | Circle
          Serial.println("--- CI");
          break;
        case  17 : //CM   |  .| Character selection mode
          Serial.println("--- CM");
          break;
        case  18 : //CO   |+  | File comment
          Serial.println("--- CO");
          break;
        case  19 : //CP   |+  | Character plot
          Serial.println("--- CP");
          break;
        case  20 : //CR   |+  | Color Range
          Serial.println("--- CR");
          break;
        case  21 : //CS   |+  | Designate standard character set
          Serial.println("--- CS");
          break;
        case  22 : //CT   |+  | Chord tolerance
          Serial.println("--- CT");
          break;
        case  23 : //CV   |  .| Curved line generator
          Serial.println("--- CV");
          break;
        case  24 : //DC   |  .| Digitize clear
          Serial.println("--- DC");
          break;
        case  25 : //DF   |+  | Default
          Serial.println("--- DF");
          break;
        case  26 : //DI   |+  | Absolute direction
          Serial.println("--- DI");
          break;
        case  27 : //DL   | - | Define downloadable character
          Serial.println("--- DL");
          break;
        case  28 : //DP   |  .| Digitize point
          Serial.println("--- DP");
          break;
        case  29 : //DR   |+  | Relative direction
          Serial.println("--- DR");
          break;
        case  30 : //DS   | - | Designate character into slot
          Serial.println("--- DS");
          break;
        case  31 : //DT   |+  | Define label terminator
          Serial.println("--- DT");
          break;
        case  32 : //DU   |2  | User Unit Direction
          Serial.println("--- DU");
          break;
        case  33 : //DV   |+  | text Direction Vertical
          Serial.println("--- DV");
          break;
        case  34 : //EA   |+  | Edge rectangle absolute
          Serial.println("--- EA");
          break;
        case  35 : //EC   |  .| Enable paper Cutter
          Serial.println("--- EC");
          break;
        case  36 : //EP   |+  | Edge polygon
          //diabaikan
          break;
        case  37 : //ER   |+  | Edge rectangle relative
          Serial.println("--- ER");
          break;
        case  38 : //ES   |+  | Extra space
          Serial.println("--- ES");
          break;
        case  39 : //EW   |+  | Edge wedge
          Serial.println("--- EW");
          break;
        case  40 : //FI   | - | pcl Font ID
          Serial.println("--- FI");
          break;
        case  41 : //FN   | - | pcl secondary Font Number
          Serial.println("--- FN");
          break;
        case  42 : //FP   |+  | Fill polygon
          Serial.println("--- FP");
          break;
        case  43 : //FR   | - | FRame advance
          Serial.println("--- FR");
          break;
        case  44 : //FS   |  .| Force select
          Serial.println("--- FS");
          break;
        case  45 : //FT   |+  | Fill type
           if(hpgl.GetParameter(&t1))
           {
             hpgl.fillType = (byte)t1;
             if(hpgl.GetParameter(&t1))
             {
               hpgl.fillTypeOption1 = (uint16_t)t1;
               if(hpgl.GetParameter(&t1))
               {
                 hpgl.fillTypeOption2 = (uint16_t)t1;
               }
             }
           }
          break;
        case  46 : //GC   |  .| Group count
          Serial.println("--- GC");
          break;
        case  47 : //GM   |  .| Graphics memory
          Serial.println("--- GM");
          break;
        case  48 : //IM   | - | Input error reporting mask
          Serial.println("--- IM");
          break;
        case  49 : //IN   |+  | Initialize
          penUp();
          break;
        case  50 : //IP   |+  | Input P1 and P2
          if(hpgl.GetParameter(&t1)) //x1
          {
            hpgl.GetParameter(&t2); //y1
            hpgl.GetParameter(&t3); //x2
            hpgl.GetParameter(&t4); //y2
          }
          break;
        case  51 : //IR   |+  | Input Relative P1 and P2
          Serial.println("--- IR");
          break;
        case  52 : //IV   | - | Invoke character slot
          Serial.println("--- IV");
          break;
        case  53 : //IW   |+  | Input window
          if(hpgl.GetParameter(&t1)) //Xll   //diabaikan
          {
            hpgl.GetParameter(&t2); //Xll
            hpgl.GetParameter(&t3); //Xur
            hpgl.GetParameter(&t4); //Yur
          }
          break;
        case  54 : //KY   |  .| Define key
          Serial.println("--- KY");
          break;
        case  55 : //LA   |*  | Line Attributes
          Serial.println("--- LA");
          break;
        case  56 : //LB   |+  | Label
          Serial.println("--- LB");
          break;
        case  57 : //LM   | - | Label mode (for two-byte character sets)
          Serial.println("--- LM");
          break;
        case  58 : //LO   |+  | Label origin
          Serial.println("--- LO");
          break;
        case  59 : //LT   |+  | Line type
          if(hpgl.GetParameter(&t1))
          {
            hpgl.lineType = (byte)t1;
          }
          break;
        case  60 : //MC   | - | Merge Control
          Serial.println("--- MC");
          break;
        case  61 : //MG   |+  | Message [same as WD]
          Serial.println("--- MG");
          break;
        case  62 : //MT   |  .| Media Type
          Serial.println("--- MT");
          break;
        case  63 : //NP   |+  | Number of Pens
          Serial.println("--- NP");
          break;
        case  64 : //NR   |  .| Not ready (unload page and go offline)
          Serial.println("--- NR");
          break;
        case  65 : //OA   |  .| Output actual position and pen status
          Serial.println("--- OA");
          break;
        case  66 : //OC   |  .| Output commanded position and pen status
          Serial.println("--- OC");
          break;
        case  67 : //OD   |  .| Output digitized point and pen status
          Serial.println("--- OD");
          break;
        case  68 : //OE   | - | Output error
          Serial.println("--- OE");
          break;
        case  69 : //OF   | - | Output factors
          Serial.println("--- OF");
          break;
        case  70 : //OG   |  .| Output group count
          Serial.println("--- OG");
          break;
        case  71 : //OH   | - | Output hard-clip limits
          Serial.println("--- OH");
          break;
        case  72 : //OI   |  .| Output identification
          Serial.println("--- OI");
          break;
        case  73 : //OK   |  .| Output key
          Serial.println("--- OK");
          break;
        case  74 : //OL   | - | Output label length
          Serial.println("--- OL");
          break;
        case  75 : //OO   |  .| Output options
          Serial.println("--- OO");
          break;
        case  76 : //OP   |+  | Output P1 and P2
          Serial.println("---OP ");
          break;
        case  77 : //OS   | - | Output status
          Serial.println("--- OS");
          break;
        case  78 : //OT   |  .| Output carousel type
          Serial.println("--- OT");
          break;
        case  79 : //OW   |+  | Output window
          Serial.println("--- OW");
          break;
        case  80 : //PA   |+  | Plot absolute
          //hpgl.GetPoint(&t1,&t2);
          gotoPolyLine();
  
  
          break;
        case  81 : //PB   |+  | Print buffered label
          Serial.println("--- PB");
          break;
        case  82 : //PC   |+  | Pen Color
          Serial.println("--- PC");
          break;
        case  83 : //PD   |+  | Pen down
          penDown();
          gotoPolyLine();
          break;
        case  84 : //PE   |+  | Polyline Encoded
          PolylineEncode = true;
          while (PolylineEncode)
          {
            lastData = hpgl.GetEncodePoint(&t1, &t2);
            if(lastData == 0)
            {
              penDown();
              gotoXYRelative(t1, t2);
            }
            else
            {
              switch (lastData)
              {
                case ':'://Select Pen
                  hpgl.penActive = hpgl.BasicEncode();
                  break;
                case '<'://Pen Up
                  penUp();
                  break;
                case '>'://Fractional Data
                  hpgl.fractionalData = hpgl.BasicEncode();
                  break;
                case '='://Absolute
                  while (!hpgl.GetEncodePoint(&t1, &t2))
                  {
                    gotoXYAbsolute(t1, t2);
                  }
                  PolylineEncode = false;
                  break;
                case '7'://7-bit mode
                  hpgl.EncodeMode = EncodeMode_32;
                  break;
                case ';':
                  PolylineEncode = false;
                  break;
              }
            }
          }
          break;
        case  85 : //PG   |+  | Page feed
          gotoXYAbsolute(0,hpgl.plotSizeLength);
          //gotoXYAbsolute(0,0);
          UserPosX = PosX;
          UserPosY = PosY;
          scalePosX = 0;
          scalePosY = 0;

          execute = false;

          progress = (byte)((100.0*hpgl.dataAddress)/settings.dataSizeinMemory);
          lcd.setCursor(0,1);
          lcd.print("                ");
          lcd.setCursor(0,1);
          lcd.print(progress);
          lcd.print("% ");
          lcd.print((int)((millis() - timeJobStart)/1000));
          lcd.print("dtk");
          
          Serial.println();
          Serial.println("Page plot finish");
          break;
        case  86 : //PM   | 2 | Polyline Mode
          if(hpgl.GetParameter(&t1))
          {
            switch(t1)
            {
              case 0:
              case '0':
                hpgl.polylineMode = true;
                break;
              case '1':
                break;
              case '2':
                hpgl.polylineMode = false;
                break;
            }
          }
          
         break;
        case  87 : //PP   | - | Pixel placement
           if(hpgl.GetParameter(&t1))
           {
             hpgl.pixelPlacement = (byte)t1;
           }
          break;
        case  88 : //PR   |+  | Plot relative
          Serial.println("--- PR");
          break;
        case  89 : //PS   |+  | Plot Size
           if(hpgl.GetParameter(&hpgl.plotSizeLength))
           {
             hpgl.GetParameter(&hpgl.plotSizeWidth);
           }
          break;
        case  90 : //PT   |+  | Pen thickness
          Serial.println("--- PT");
          break;
        case  91 : //PU   |+  | Pen up
          penUp();
          gotoPolyLine();
          break;
        case  92 : //PW   |+  | Pen Width
          if(hpgl.GetParameter(&t1))
          {
            if (hpgl.GetParameter(&t2))
            {
              hpgl.penWidth[t2] = t1;
            }
            else
            {
              hpgl.penWidth[hpgl.penActive] = t1;
            }
          }
          break;
        case  93 : //QL   |  .| Quality Level
          if(hpgl.GetParameter(&t1))
          {
            hpgl.qualityLevel = t1;
          }
          break;
        case  94 : //RA   |+  | Fill rectangle absolute
          Serial.println("--- RA");
          break;
        case  95 : //RF   | - | Raster Fill pattern
          Serial.println("--- RF");
          break;
        case  96 : //RO   |+  | Rotate coordinate system
          if(hpgl.GetParameter(&t1))
          {
            hpgl.rotate = (uint16_t)t1;
          }
          break;
        case  97 : //RP   | - | Replot
          Serial.println("--- R");
          break;
        case  98 : //RR   |+  | Fill rectangle relative
          if(hpgl.GetPoint(&t1, &t2))
          {
            penDown();
            gotoXYRelative(t1, 0);
            gotoXYRelative(0, t2);
            gotoXYRelative(-t1, 0);
            gotoXYRelative(0, -t2);
          }
          break;
        case  99 : //RT   |+  | Relative arc through Three points
          Serial.println("--- RT");
          break;
        case  100 : //SA   |+  | Select alternate character set
          Serial.println("--- SA");
          break;
        case  101 : //SB   | - | Scalable or Bitmap font selection
          Serial.println("--- SB");
          break;
        case  102 : //SC   |+  | Scale
          if(hpgl.GetParameter(&t5)) //xmin
          {
            hpgl.GetParameter(&t6); //xmax
            hpgl.GetParameter(&t7); //ymin
            hpgl.GetParameter(&t8); //ymax
          }
  
//          hpgl.scaleX = (1.0*(t6 - t5)) / (t3 - t1);
//          hpgl.scaleY = (-1.0*(t8 - t7)) / (t4 - t2);

          hpgl.scaleX = scale;
          hpgl.scaleY = scale;
//          Serial.println();
//          Serial.print("scaleX = ");
//          Serial.println(hpgl.scaleX);
//          Serial.print("scaleY = ");
//          Serial.println(hpgl.scaleY);
          break;
        case  103 : //SD   |*  | Standard font attribute Definition
          Serial.println("--- SD");
          break;
        case  104 : //SI   |+  | Absolute character size
          Serial.println("--- SI");
          break;
        case  105 : //SL   |+  | Character slant
          Serial.println("--- SL");
          break;
        case  106 : //SM   |+  | Symbol mode
          Serial.println("--- SM");
          break;
        case  107 : //SP   |+  | Select pen
          if(hpgl.GetParameter(&t1))
          {
            hpgl.penActive = (byte)hpgl.penActive;
          }
          break;
        case  108 : //SR   |+  | Relative character size
          Serial.println("--- SR");
          break;
        case  109 : //SS   |+  | Select standard character set
          Serial.println("--- S");
          break;
        case  110 : //ST   |  .| Sort vectors
          Serial.println("--- ST");
          break;
        case  111 : //SU   |  2| User Unit Character Size
          Serial.println("--- SU");
          break;
        case  112 : //SV   | - | Screened Vectors
          Serial.println("--- SV");
          break;
        case  113 : //TD   | - | Transparent Data
          Serial.println("--- TD");
          break;
        case  114 : //TL   |+  | Tick length
          Serial.println("--- TL");
          break;
        case  115 : //TR   | - | Transparency mode
          Serial.println("--- TR");
          break;
        case  116 : //UC   |+  | User-defined character
          Serial.println("--- UC");
          break;
        case  117 : //UF   | - | User-defined fill type
          Serial.println("--- UF");
          break;
        case  118 : //UL   |+  | User-defined line type
          Serial.println("--- UL");
          break;
        case  119 : //VS   |  .| Velocity select
          Serial.println("--- VS");
          break;
        case  120 : //WD   |+  | Write to display
          Serial.println("--- WD");
          break;
        case  121 : //WG   |+  | Fill wedge
          Serial.println("--- WG");
          break;
        case  122 : //WU   |+  | pen Width Unit
          if(hpgl.GetParameter(&t1))          {
            hpgl.penWidthUnit = (byte)t1;
          }
          break;
        case  123 : //XT   |+  | X-Tick
          Serial.println("--- XT");
          break;
        case  124 : //YT  |+  | Y-Tick
          Serial.println("--- YT");
          break;
      }
      if(getButton() == Test)
      {
        penUp();
        execute = false;
      }
    }
    dflash.stopContinuousRead();
    digitalWrite(stepperEnable, HIGH);
  }

  if((menu == menuOffline) || (menu == menuOrigin))
  {
    if(millis() - watchDog > 15000)
    {
      penUp();
      digitalWrite(stepperEnable, HIGH);
      menu = menuReset;
      displayMenu();
    }
  }
  checkButton();
}

void checkButton()
{  
  switch (getButton())
  {
    case Reset:
      if (menu != menuReset)
      {
        menu = menuReset;
        displayMenu();
      }
      break;
    case Offline:
      if (menu != menuOffline)
      {
        menu = menuOffline;
        displayMenu();
        watchDog = millis();
        digitalWrite(stepperEnable, LOW);
      }
      else
      {
        digitalWrite(stepperEnable, HIGH);
        menu = menuReset;
        displayMenu();
      }
      break;
    case Setup:
      execute = true;
      break;
    case Origin:
      menu = menuOrigin;

      if(penDownStatus)
      {
        penUp();
      }
      else
      {
        penDown();
      }
      watchDog = millis();
      
//      XMove(PosX - UserPosX);
//      YMove(PosY - UserPosY);
//      updateValue();
      break;
    case Fplus:
      if (menu == menuReset)
      {
        if (settings.Force < maxForce)
        {
          settings.Force += 1;
        }
      }
      if (menu == menuOffline)
      {
        if (PosX < (maxX-10))
        {
          XMove(10);
        }
      }
      updateValue();
      break;
    case Fminus:
      if (menu == menuReset)
      {
        if (settings.Force != minForce)
        {
          settings.Force -= 1;
        }
      }
      if (menu == menuOffline)
      {
        if (PosX >= 10)
        {
          XMove(-10);
        }
      }
      updateValue();
      break;
    case Vplus:
      if (menu == menuReset)
      {
        if (settings.Speed < maxSpeed)
        {
          settings.Speed += 1;
        }
      }
      if (menu == menuOffline)
      {
        YMove(-10);
      }
      updateValue();
      break;
    case Vminus:
      if (menu == menuReset)
      {
        if (settings.Speed != minSpeed)
        {
          settings.Speed -= 1;
        }
      }
      if (menu == menuOffline)
      {
        YMove(10);
      }
      updateValue();
      break;


    case FplusLong:
      while (longPressStatus = longPress(0))
      {
        if (menu == menuReset)
        {
          if (settings.Force <= (maxForce - 10))
          {
            settings.Force += 10;
          }
        }
        if (menu == menuOffline)
        {
          while((digitalRead(button[0]) == LOW) && (PosX < maxX))
          {
            XMove(1);
          }
        }
        updateValue();
      }
      break;
    case FminusLong:
      while (longPressStatus = longPress(8))
      {
        if (menu == menuReset)
        {
          if (settings.Force >= 10)
          {
            settings.Force -= 10;
          }
        }
        if (menu == menuOffline)
        {
          while((digitalRead(button[8]) == LOW) && (PosX != 0))
          {
            XMove(-1);
          }
        }
        updateValue();
      }
      break;
    case VplusLong:
      while (longPressStatus = longPress(1))
      {
        if (menu == menuReset)
        {
          if (settings.Speed <= (maxSpeed - 10))
          {
            settings.Speed += 10;
          }
        }
        if (menu == menuOffline)
        {
          while(digitalRead(button[1]) == LOW)
          {
            YMove(-1);
          }
        }
        updateValue();
      }
      break;
    case VminusLong:
      while (longPressStatus = longPress(7))
      {
        if (menu == menuReset)
        {
          if (settings.Speed >= 10)
          {
            settings.Speed -= 10;
          }
        }
        if (menu == menuOffline)
        {
          while(digitalRead(button[7]) == LOW)
          {
            YMove(1);
          }

        }
        updateValue();
      }
      break;
  }
}

void updateValue()
{
  switch (menu)
  {
    case menuReset:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Speed :         ");
      lcd.setCursor(0, 1);
      lcd.print("Force :         ");
      digitalWrite(button[4], HIGH);
      digitalWrite(button[5], LOW);
      digitalWrite(button[11], HIGH);

      lcd.setCursor(7, 0);
      lcd.print(settings.Speed * mmPerStep * 10);
      lcd.print("cm/s  ");
      lcd.setCursor(7, 1);
      lcd.print(settings.Force);
      lcd.print("gr  ");
      break;
    case menuOffline:
      lcd.setCursor(7, 0);
      lcd.print(PosX * mmPerStep);
      lcd.print("mm   ");
      lcd.setCursor(7, 1);
      lcd.print(PosY * mmPerStep);
      lcd.print("mm    ");
      UserPosX = PosX;
      UserPosY = PosY;
      scalePosX = 0;
      scalePosY = 0;
      watchDog = millis();

      break;
  }
  delaySpeed = toDelaySpeed(settings.Speed);

  eeprom_write_block((const void*)&settings, (void*)0, sizeof(settings));
}

void displayMenu()
{
  switch (menu)
  {
    case menuReset:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Speed :         ");
      lcd.setCursor(0, 1);
      lcd.print("Force :         ");
      digitalWrite(button[4], HIGH);
      digitalWrite(button[5], LOW);
      digitalWrite(button[11], HIGH);
      break;
    case menuOffline:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Move X:       mm");
      lcd.setCursor(0, 1);
      lcd.print("     Y:       mm");
      digitalWrite(button[4], LOW);
      digitalWrite(button[5], HIGH);
      digitalWrite(button[11], HIGH);
      break;
  }
  updateValue();
}

int getButton()
{

  byte pressedbutton = 0;

  longPressCounter = 0;

  digitalWrite(button[2], LOW);
  digitalWrite(button[9], HIGH);

  if (deBounce(0))
  {
    pressedbutton = Offline;
  }
  else if (deBounce(1))
  {
    pressedbutton = Test;
  }
  else if (deBounce(7))
  {
    pressedbutton = Setup;
  }
  else if (deBounce(8))
  {
    pressedbutton = Origin;
  }
  else if (deBounce(3))
  {
    pressedbutton = Reset;
  }

  digitalWrite(button[2], HIGH);
  digitalWrite(button[9], LOW);

  if (deBounce(0) == 1)
  {
    pressedbutton = Fplus;
  }
  else if (deBounce(0) == 2)
  {
    pressedbutton = FplusLong;
  }
  else if (deBounce(1) == 1)
  {
    pressedbutton = Vplus;
  }
  else if (deBounce(1) == 2)
  {
    pressedbutton = VplusLong;
  }
  else if (deBounce(7) == 1)
  {
    pressedbutton = Vminus;
  }
  else if (deBounce(7) == 2)
  {
    pressedbutton = VminusLong;
  }
  else if (deBounce(8) == 1)
  {
    pressedbutton = Fminus;
  }
  else if (deBounce(8) == 2)
  {
    pressedbutton = FminusLong;
  }
  return pressedbutton;
}

byte deBounce(int pin)
{
  if (digitalRead(button[pin]) == LOW)
  {
    delay(50);
    if (digitalRead(button[pin]) == LOW)
    {
      //while(digitalRead(button[pin]) == LOW);
      for (int i = 0; i < 300; i++)
      {
        if (digitalRead(button[pin]) == HIGH)
        {
          delay(50);
          if (digitalRead(button[pin]) == HIGH)
          {
            return 1;
          }
        }
        delay(1);
      }
      return 2;//long press
    }
  }
  return 0;
}


byte longPress(int pin)
{
  delay(300);
  if (digitalRead(button[pin]) == LOW)
  {
    if (digitalRead(button[pin]) == LOW)
    {
      longPressCounter++;
      if (longPressCounter <= 5)
      {
        return 1;
      }
      else
      {
        return 2;
      }
    }
  }
  return 0;
}

bool gotoXYAbsolute(long x, long y)
{
  Serial.println();
  Serial.print("goto absoulute(");
  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.print(")");

  if(settings.Speed > 25)
  {
    delaySpeed = toDelaySpeed(settings.Speed);
  }
  else
  {
    delaySpeed = toDelaySpeed(25);
  }

  YMove(((hpgl.scaleY * y) - PosY) + UserPosY);
  XMove(((hpgl.scaleX * x) - PosX) + UserPosX);

  scalePosX = hpgl.scaleX * x;
  scalePosY = hpgl.scaleY * y;
}

bool gotoXYRelative(long x, long y)
{
  Serial.println();
  Serial.print("goto relative(");
  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.print(")");

  double lastScalePosX = scalePosX;
  double lastScalePosY = scalePosY;

  scalePosX += hpgl.scaleX * x;
  scalePosY += hpgl.scaleY * y;

  long aX = (long)scalePosX - (long)lastScalePosX;
  long aY = (long)scalePosY - (long)lastScalePosY;

  

  const float pi = atan2(0, -100);
  const float pi2 = atan2(100, 0);
  const float pi3 = atan2(-100, 0);
  float angle = atan2(aY , aX);

  if((angle == pi) || (angle == 0))
  {
    XMove(aX);
  }
  else if((angle == pi2) || (angle == pi3))
  {
    YMove(aY);
  }
  else 
  {

    delaySpeed = toDelaySpeed(10);

    (aX>0)?digitalWrite(XDir, HIGH):digitalWrite(XDir, LOW);
    (aY>0)?digitalWrite(YDir, HIGH):digitalWrite(YDir, LOW);
    
    double m = (1.0*aY)/aX;
    long absX = abs(aX);
    long absY = abs(aY);
    int8_t xStep = x>0?1:-1;
    int8_t yStep = y>0?1:-1;
    
    if(absY >= absX)
    {
      double p = (1.0*absY)/absX;
      double q = (1.0*absY)/(absX*2);
      double xMoving = q;

      for(int i=0;i<absY;i++)
      {
        if(i == 40)
        {
          delaySpeed = toDelaySpeed(settings.Speed);
        }
        if(i > xMoving)
        {
          for(int j=0;j<4;j++)
          {
            digitalWrite(YStep, HIGH);
            digitalWrite(XStep, HIGH);
            delayMicroseconds(delaySpeed);
            digitalWrite(YStep, LOW);
            digitalWrite(XStep, LOW);
            delayMicroseconds(delaySpeed);
          }
          PosX += xStep;
          PosY += yStep;
          xMoving += p;
       }
        else
        {
          for(int j=0;j<4;j++)
          {
            digitalWrite(YStep, HIGH);
            delayMicroseconds(delaySpeed);
            digitalWrite(YStep, LOW);
            delayMicroseconds(delaySpeed);
          }
          PosY += yStep;
        }
      }
    }
    else
    {
      double p = (1.0*absX)/absY;
      double q = (1.0*absX)/(absY*2);
      double yMoving = q;

      for(int i=0;i<absX;i++)
      {
        if(i == 40)
        {
          delaySpeed = toDelaySpeed(settings.Speed);
        }
        if(i > yMoving)
        {
          for(int j=0;j<4;j++)
          {
            digitalWrite(YStep, HIGH);
            digitalWrite(XStep, HIGH);
            delayMicroseconds(delaySpeed);
            digitalWrite(YStep, LOW);
            digitalWrite(XStep, LOW);
            delayMicroseconds(delaySpeed);
          }
          PosX += xStep;
          PosY += yStep;
          yMoving += p;
        }
        else
        {
          for(int j=0;j<4;j++)
          {
            digitalWrite(XStep, HIGH);
            delayMicroseconds(delaySpeed);
            digitalWrite(XStep, LOW);
            delayMicroseconds(delaySpeed);
          }
          PosX += xStep;
        }
      }
    }
  }

}
bool penUp()
{
  Serial.println();
  Serial.print("Pen Up");
  
  analogWrite(penB, 0);
  analogWrite(penA, 255);

  delay(50);

  analogWrite(penB, 0);
  analogWrite(penA, 0);

  penDownStatus = false;
}
bool penDown()
{
  Serial.println();
  Serial.print("Pen Down");

  analogWrite(penA, 0);
  analogWrite(penB, settings.Force);

  penDownStatus = true;
}
bool gotoPolyLine()
{
  long x, y;
  bool next = true;
  while(1)
  {
    if(hpgl.GetPoint(&x, &y))
    {
      gotoXYAbsolute(x, y);
    }
    else
    {
      break;
    }
  }
}
bool XMove(long steps)
{
  if(steps>0)
  {
    digitalWrite(XDir, HIGH);
  }
  else
  {
    digitalWrite(XDir, LOW);
  }
  
  for(long i=0;i<abs(steps);i++)
  {
    for(int j=0;j<4;j++)
    {
      digitalWrite(XStep, HIGH);
      delayMicroseconds(delaySpeed);
      digitalWrite(XStep, LOW);
      delayMicroseconds(delaySpeed);
    }
  }
  PosX += steps;
}

bool YMove(long steps)
{
  if(steps>0)
  {
    digitalWrite(YDir, HIGH);
  }
  else
  {
    digitalWrite(YDir, LOW);
  }
  for(long i=0;i<abs(steps);i++)
  {
    for(int j=0;j<4;j++)
    {
      digitalWrite(YStep, HIGH);
      delayMicroseconds(delaySpeed);
      digitalWrite(YStep, LOW);
      delayMicroseconds(delaySpeed);
    }
  }
  PosY += steps;
}
int toDelaySpeed(int speed)
{
  return (int)((1000000 * mmPerStep/(2.54 * speed))/32);// mmPerStep (mm/step) / settings.Speed (mm/s) = (s/step)
}

