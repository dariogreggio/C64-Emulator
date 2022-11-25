//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------

#ifndef _C64_PIC_H
#define _C64_PIC_H


/* check if build is for a real debug tool */
#if defined(__DEBUG) && !defined(__MPLAB_ICD2_) && !defined(__MPLAB_ICD3_) && \
   !defined(__MPLAB_PICKIT2__) && !defined(__MPLAB_PICKIT3__) && \
   !defined(__MPLAB_REALICE__) && \
   !defined(__MPLAB_DEBUGGER_REAL_ICE) && \
   !defined(__MPLAB_DEBUGGER_ICD3) && \
   !defined(__MPLAB_DEBUGGER_PK3) && \
   !defined(__MPLAB_DEBUGGER_PICKIT2) && \
   !defined(__MPLAB_DEBUGGER_PIC32MXSK)
    #warning Debug with broken MPLAB simulator
    #define USING_SIMULATOR
#endif


//#define COMMODORE64 1
#define APPLE2 1
//#define AMICO2000

#ifdef COMMODORE64
#if defined(__PIC32MM__)
#ifdef USING_SIMULATOR
#define MAX_RAM 0x6800
#else
#define MAX_RAM 0x6d00
#endif
#else
#define MAX_RAM 0x10000
#endif
#endif
#ifdef AMICO2000
#define MAX_RAM 2048
#endif
#ifdef APPLE2
#define MAX_RAM 0x8000      // diciamo...
#endif

#ifdef COMMODORE64
//https://www.c64-wiki.com/wiki/raster_time
//#define REAL_SIZE    
#define MIN_RASTER 0        // noi visualizziamo da 48 a 248
#define MAX_RASTER 311
#define HORIZ_SIZE 320      // 504 pixel compresi bordi, dice...
#define VERT_SIZE 200
#define HORIZ_OFFSCREEN 0
#ifdef ST7735
#ifdef REAL_SIZE    
#define VERT_OFFSCREEN 28      // 
#else
#define VERT_OFFSCREEN 22       // sarebbe ok 28 (128-100) ma preferisco leggere meglio su display piccolo! 25*5+3
#endif
#endif
#ifdef ILI9341
#define VERT_OFFSCREEN 18       // (240-200) 
#endif
#if defined(__PIC32MM__)
#define VERT_OFFSCREEN 18       // (240-200) 
#endif
#endif
#ifdef APPLE2
#define HORIZ_SIZE (40*8)      // 
#define VERT_SIZE (24*8)
#endif
#ifdef AMICO2000
#endif


#if defined(__PIC32MM__)
#define FCY 32000000ul    //24000000ul    //Oscillator frequency; 
#warning vERIFICARE 2022
#else
#define FCY 205000000ul    //Oscillator frequency; ricontrollato con baud rate, pare giusto così!
#endif

#define CPU_CLOCK_HZ             (FCY)    // CPU Clock Speed in Hz
#define CPU_CT_HZ            (CPU_CLOCK_HZ/2)    // CPU CoreTimer   in Hz
#define PERIPHERAL_CLOCK_HZ      (FCY/2 /*100000000UL*/)    // Peripheral Bus  in Hz
#define GetSystemClock()         (FCY)    // CPU Clock Speed in Hz
#define GetPeripheralClock()     (PERIPHERAL_CLOCK_HZ)    // Peripheral Bus  in Hz
#define FOSC 8000000ul

#define US_TO_CT_TICKS  (CPU_CT_HZ/1000000UL)    // uS to CoreTimer Ticks
    
#define VERNUML 11
#define VERNUMH 1


typedef char BOOL;
typedef unsigned char UINT8;
typedef unsigned char BYTE;
typedef signed char INT8;
typedef unsigned int WORD;      // più veloce int che short int :) circa da 6 a 5.5uS ciclo macchina, 16/6/21
typedef unsigned int SWORD;
typedef unsigned long UINT32;
typedef unsigned long DWORD;
typedef signed long INT32;
typedef unsigned short int UINT16;
typedef signed int INT16;

typedef DWORD COLORREF;

#define RGB(r,g,b)      ((COLORREF)(((BYTE)(r)|((WORD)((BYTE)(g))<<8))|(((DWORD)(BYTE)(b))<<16)))


#define TRUE 1
#define FALSE 0


#ifdef ST7735
#define _TFTWIDTH  		160     //the REAL W resolution of the TFT
#define _TFTHEIGHT 		128     //the REAL H resolution of the TFT
typedef signed char GRAPH_COORD_T;
typedef unsigned char UGRAPH_COORD_T;
#endif
#ifdef ILI9341
#define _TFTWIDTH  		320     //the REAL W resolution of the TFT
#define _TFTHEIGHT 		240     //the REAL H resolution of the TFT
typedef signed short int GRAPH_COORD_T;
typedef unsigned short int UGRAPH_COORD_T;
#endif
typedef WORD GFX_COLOR;
#if defined(__PIC32MM__)
typedef signed short int GRAPH_COORD_T;
typedef unsigned short int UGRAPH_COORD_T;
#endif
typedef struct tagRECT {
  short int left;
  short int top;
  short int right;
  short int bottom;
  } RECT;
#define MAKEWORD(a, b)   ((WORD) (((BYTE) (a)) | ((WORD) ((BYTE) (b))) << 8)) 
#define MAKELONG(a, b)   ((unsigned long) (((WORD) (a)) | ((DWORD) ((WORD) (b))) << 16)) 
#define HIBYTE(w)   ((BYTE) ((((WORD) (w)) >> 8) /* & 0xFF*/)) 
//#define HIBYTE(w)   ((BYTE) (*((char *)&w+1)))		// molto meglio :)
#define HIWORD(l)   ((WORD) (((DWORD) (l) >> 16) & 0xFFFF)) 
#define LOBYTE(w)   ((BYTE) (w)) 
#define LOWORD(l)   ((WORD) (l)) 



#ifdef __PIC32
void mySYSTEMConfigPerformance(void);
void myINTEnableSystemMultiVectoredInt(void);

#define ReadCoreTimer()                  _CP0_GET_COUNT()           // Read the MIPS Core Timer

void ShortDelay(DWORD DelayCount);
//#define __delay_ms(n) ShortDelay(n*100000UL)

#define ClrWdt() { WDTCONbits.WDTCLRKEY=0x5743; }
#endif

#ifdef COMMODORE64
extern const unsigned char C64kern[];
extern const unsigned char C64basic[];
extern const unsigned char C64char[];
extern const unsigned char C64cartridge[];
#endif
#ifdef APPLE2
extern const unsigned char AppleROM_D0[],AppleROM_E0[],AppleROM_E8[],AppleROM_F0[],AppleROM_F8[];
extern const unsigned char AppleChar[];
#endif
#ifdef AMICO2000
extern const unsigned char Amico2000_rom1[],Amico2000_rom2[];
extern const unsigned char rom_asteroidi[];
extern const unsigned char rom_labirinto[];
extern const unsigned char rom_atterraggiolunare[];
#endif

void Timer_Init(void);
void PWM_Init(void);
void UART_Init(DWORD);
void putsUART1(unsigned int *buffer);

int decodeKBD(int, long, BOOL);
BYTE GetValue(SWORD);
SWORD GetIntValue(SWORD);
void PutValue(SWORD, BYTE);
int Emulate(int);

void __delay_ms(unsigned int);

int UpdateScreen(SWORD rowIni, SWORD rowFin);

#ifdef ST7735           // ST7735 160x128 su Arduino con dsPIC
#define LED1 LATEbits.LATE2
#define LED2 LATEbits.LATE3
#define LED3 LATEbits.LATE4
#define SW1  PORTDbits.RD2
#define SW2  PORTDbits.RD3


// pcb SDRradio 2019
#define	SPISDITris 0		// niente qua
#define	SPISDOTris TRISGbits.TRISG8				// SDO
#define	SPISCKTris TRISGbits.TRISG6				// SCK
#define	SPICSTris  TRISGbits.TRISG7				// CS
#define	LCDDCTris  TRISEbits.TRISE7				// DC che su questo LCD è "A0" per motivi ignoti
//#define	LCDRSTTris TRISBbits.TRISB7
	
#define	m_SPISCKBit LATGbits.LATG6		// pin 
#define	m_SPISDOBit LATGbits.LATG8		// pin 
#define	m_SPISDIBit 0
#define	m_SPICSBit  LATGbits.LATG7		// pin 
#define	m_LCDDCBit  LATEbits.LATE7 		// pin 
//#define	m_LCDRSTBit LATBbits.LATB7 //FARE
//#define	m_LCDBLBit  LATBbits.LATB12
#endif

#if defined(__PIC32MM__)
#define LED1 LATBbits.LATB2         //
#define LED2 LATBbits.LATB4
#define LED3 LATBbits.LATB9
#define SW1  PORTBbits.RB3          // 
#define SW2  PORTAbits.RA0          // 
#endif

#ifdef ILI9341

#define LED1 LATEbits.LATE4
#define LED2 LATDbits.LATD0
#define LED3 LATDbits.LATD11
#define SW1  PORTFbits.RF0
#define SW2  PORTBbits.RB0          // bah uso AREF tanto per...

#define	LCDDCTris  TRISBbits.TRISB3				// http://attach01.oss-us-west-1.aliyuncs.com/IC/Datasheet/11009.zip?spm=a2g0o.detail.1000023.9.70352ae94rI9S1&file=11009.zip
#define	LCDRSTTris TRISBbits.TRISB10

#define	LCDRDTris  TRISBbits.TRISB5          // 
#define	LCDWRTris  TRISBbits.TRISB4          // WR per LCD parallelo
#define	LCDSTRTris  TRISBbits.TRISB4         // Strobe per LCD parallelo A3_TRIS (in pratica Write...)

#define	LCDCSTris  TRISBbits.TRISB2

#define	m_LCDDCBit  LATBbits.LATB3 		// 
#define	m_LCDRSTBit LATBbits.LATB10
//#define	m_LCDBLBit  LATBbits.LATB12

#define	m_LCDRDBit  LATBbits.LATB5 		// 
#define	m_LCDWRBit  LATBbits.LATB4 		// per LCD parallelo ILI
#define	m_LCDSTRBit LATBbits.LATB4        // non è chiaro... m_A3_out; in pratica è WRITE

#define	m_LCDCSBit  LATBbits.LATB2
#endif

#endif
