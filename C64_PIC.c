// Local Header Files
#include <stdlib.h>
#include <string.h>
#include <xc.h>
#include "c64_pic.h"

#ifdef __PIC32
#include <sys/attribs.h>
#include <sys/kmem.h>
#else
#include <libpic30.h>
#endif

#ifdef ST7735
#include "Adafruit_ST77xx.h"
#include "Adafruit_ST7735.h"
#include "adafruit_gfx.h"
#endif
#ifdef ILI9341
#include "Adafruit_ILI9341.h"
#include "adafruit_gfx.h"
#endif
#if defined(__PIC32MM__)
#include "adafruit_gfx.h"
#endif


#ifdef __PIC32

#if defined(__PIC32MM__)

// PIC32MM0256GPM028 Configuration Bit Settings
// 'C' source line config statements

// FDEVOPT
#pragma config SOSCHP = OFF             // Secondary Oscillator High Power Enable bit (SOSC oprerates in normal power mode.)
#pragma config ALTI2C = OFF             // Alternate I2C1 Pins Location Enable bit (Primary I2C1 pins are used)
#pragma config FUSBIDIO = ON           // USBID pin control (USBID pin is available)
#pragma config FVBUSIO = ON            // VBUS Pin Control (VBUS pin is available)
#pragma config USERID = 0x4443          // User ID bits (Enter Hexadecimal value)

// FICD
#pragma config JTAGEN = OFF             // JTAG Enable bit (JTAG is enabled)
#pragma config ICS = PGx1               // ICE/ICD Communication Channel Selection bits (Communicate on PGEC1/PGED1)

// FPOR
#pragma config BOREN = BOR3             // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware; SBOREN bit disabled)
#pragma config RETVR = OFF              // Retention Voltage Regulator Enable bit (Retention regulator is disabled)
#pragma config LPBOREN = ON             // Downside Voltage Protection Enable bit (Low power BOR is enabled, when main BOR is disabled)

// FWDT
#pragma config SWDTPS = PS1048576       // Sleep Mode Watchdog Timer Postscale Selection bits (1:1048576)
#pragma config FWDTWINSZ = PS25_0       // Watchdog Timer Window Size bits (Watchdog timer window size is 25%)
#pragma config WINDIS = OFF             // Windowed Watchdog Timer Disable bit (Watchdog timer is in non-window mode)
#pragma config RWDTPS = PS1048576       // Run Mode Watchdog Timer Postscale Selection bits (1:1048576)
#pragma config RCLKSEL = LPRC           // Run Mode Watchdog Timer Clock Source Selection bits (Clock source is LPRC (same as for sleep mode))
#pragma config FWDTEN = ON              // Watchdog Timer Enable bit (WDT is enabled)

// FOSCSEL
#pragma config FNOSC = FRCDIV           // Oscillator Selection bits (Fast RC oscillator (FRC) with divide-by-N)
#pragma config PLLSRC = FRC             // System PLL Input Clock Selection bit (FRC oscillator is selected as PLL reference input on device reset)
#pragma config SOSCEN = OFF             // Secondary Oscillator Enable bit (Secondary oscillator (SOSC) is enabled)
#pragma config IESO = OFF               // Two Speed Startup Enable bit (Two speed startup is disabled)
#pragma config POSCMOD = OFF            // Primary Oscillator Selection bit (Primary oscillator is disabled)
#pragma config OSCIOFNC = OFF           // System Clock on CLKO Pin Enable bit (OSCO pin operates as a normal I/O)
#pragma config SOSCSEL = ON            // Secondary Oscillator External Clock Enable bit (Crystal is used (RA4 and RB4 are controlled by SOSC))
#pragma config FCKSM = CSECME           // Clock Switching and Fail-Safe Clock Monitor Enable bits (Clock switching is enabled; Fail-safe clock monitor is enabled)

// FSEC
#pragma config CP = OFF                 // Code Protection Enable bit (Code protection is disabled)

#else

// PIC32MZ1024EFE064 Configuration Bit Settings

// 'C' source line config statements

// DEVCFG3
// USERID = No Setting
#pragma config FMIIEN = OFF             // Ethernet RMII/MII Enable (RMII Enabled)
#pragma config FETHIO = OFF             // Ethernet I/O Pin Select (Alternate Ethernet I/O)
#pragma config PGL1WAY = ON             // Permission Group Lock One Way Configuration (Allow only one reconfiguration)
#pragma config PMDL1WAY = ON            // Peripheral Module Disable Configuration (Allow only one reconfiguration)
#pragma config IOL1WAY = ON             // Peripheral Pin Select Configuration (Allow only one reconfiguration)
#pragma config FUSBIDIO = ON            // USB USBID Selection (Controlled by the USB Module)

// DEVCFG2
/* Default SYSCLK = 200 MHz (8MHz FRC / FPLLIDIV * FPLLMUL / FPLLODIV) */
//#pragma config FPLLIDIV = DIV_1, FPLLMULT = MUL_50, FPLLODIV = DIV_2
#pragma config FPLLIDIV = DIV_2         // System PLL Input Divider (8x Divider)
#pragma config FPLLRNG = RANGE_34_68_MHZ// System PLL Input Range (34-68 MHz Input)
#pragma config FPLLICLK = PLL_FRC       // System PLL Input Clock Selection (FRC is input to the System PLL)
#pragma config FPLLMULT = MUL_50       // System PLL Multiplier (PLL Multiply by 128)
#pragma config FPLLODIV = DIV_2        // System PLL Output Clock Divider (32x Divider)
#pragma config UPLLFSEL = FREQ_24MHZ    // USB PLL Input Frequency Selection (USB PLL input is 24 MHz)

// DEVCFG1
#pragma config FNOSC = FRCDIV           // Oscillator Selection Bits (Fast RC Osc w/Div-by-N (FRCDIV))
#pragma config DMTINTV = WIN_127_128    // DMT Count Window Interval (Window/Interval value is 127/128 counter value)
#pragma config FSOSCEN = ON             // Secondary Oscillator Enable (Enable SOSC)
#pragma config IESO = ON                // Internal/External Switch Over (Enabled)
#pragma config POSCMOD = OFF            // Primary Oscillator Configuration (Primary osc disabled)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FCKSM = CSECME           // Clock Switching and Monitor Selection (Clock Switch Enabled, FSCM Enabled)
#pragma config WDTPS = PS16384          // Watchdog Timer Postscaler (1:16384)
  // circa 6-7 secondi, 24.7.19
#pragma config WDTSPGM = STOP           // Watchdog Timer Stop During Flash Programming (WDT stops during Flash programming)
#pragma config WINDIS = NORMAL          // Watchdog Timer Window Mode (Watchdog Timer is in non-Window mode)
#pragma config FWDTEN = ON             // Watchdog Timer Enable (WDT Enabled)
#pragma config FWDTWINSZ = WINSZ_25     // Watchdog Timer Window Size (Window size is 25%)
#pragma config DMTCNT = DMT31           // Deadman Timer Count Selection (2^31 (2147483648))
#pragma config FDMTEN = OFF             // Deadman Timer Enable (Deadman Timer is disabled)

// DEVCFG0
#pragma config DEBUG = OFF              // Background Debugger Enable (Debugger is disabled)
#pragma config JTAGEN = OFF             // JTAG Enable (JTAG Disabled)
#pragma config ICESEL = ICS_PGx2        // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
#pragma config TRCEN = OFF              // Trace Enable (Trace features in the CPU are disabled)
#pragma config BOOTISA = MIPS32         // Boot ISA Selection (Boot code and Exception code is MIPS32)
#pragma config FECCCON = OFF_UNLOCKED   // Dynamic Flash ECC Configuration (ECC and Dynamic ECC are disabled (ECCCON bits are writable))
#pragma config FSLEEP = OFF             // Flash Sleep Mode (Flash is powered down when the device is in Sleep mode)
#pragma config DBGPER = PG_ALL          // Debug Mode CPU Access Permission (Allow CPU access to all permission regions)
#pragma config SMCLR = MCLR_NORM        // Soft Master Clear Enable bit (MCLR pin generates a normal system Reset)
#pragma config SOSCGAIN = GAIN_2X       // Secondary Oscillator Gain Control bits (2x gain setting)
#pragma config SOSCBOOST = ON           // Secondary Oscillator Boost Kick Start Enable bit (Boost the kick start of the oscillator)
#pragma config POSCGAIN = GAIN_2X       // Primary Oscillator Gain Control bits (2x gain setting)
#pragma config POSCBOOST = ON           // Primary Oscillator Boost Kick Start Enable bit (Boost the kick start of the oscillator)
#pragma config EJTAGBEN = NORMAL        // EJTAG Boot (Normal EJTAG functionality)

// DEVCP0
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

// SEQ3

// DEVADC0

// DEVADC1

// DEVADC2

// DEVADC3

// DEVADC4

// DEVADC7

#endif


#else
// DSPIC33CH128MP202/502 Configuration Bit Settings

// 'C' source line config statements
#if defined(__dsPIC33CH__)
// FSEC
#pragma config BWRP = OFF               // Boot Segment Write-Protect bit (Boot Segment may be written)
#pragma config BSS = DISABLED           // Boot Segment Code-Protect Level bits (No Protection (other than BWRP))
#pragma config BSEN = OFF               // Boot Segment Control bit (No Boot Segment)
#pragma config GWRP = OFF               // General Segment Write-Protect bit (General Segment may be written)
#pragma config GSS = DISABLED           // General Segment Code-Protect Level bits (No Protection (other than GWRP))
#pragma config CWRP = OFF               // Configuration Segment Write-Protect bit (Configuration Segment may be written)
#pragma config CSS = DISABLED           // Configuration Segment Code-Protect Level bits (No Protection (other than CWRP))
#pragma config AIVTDIS = OFF            // Alternate Interrupt Vector Table bit (Disabled AIVT)

// FBSLIM
#pragma config BSLIM = 0x1FFF           // Boot Segment Flash Page Address Limit bits (Boot Segment Flash page address  limit)

// FSIGN

// FOSCSEL
#pragma config FNOSC = FRCPLL    //FRCDIVN          // Oscillator Source Selection (Internal Fast RC (FRC) Oscillator with postscaler)
#pragma config IESO = ON                // Two-speed Oscillator Start-up Enable bit (Start up device with FRC, then switch to user-selected oscillator source)

// FOSC
#pragma config POSCMD = NONE            // Primary Oscillator Mode Select bits (Primary Oscillator disabled)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function bit (OSC2 is clock output)
#pragma config FCKSM = CSDCMD           // Clock Switching Mode bits (Both Clock switching and Fail-safe Clock Monitor are disabled)
#pragma config PLLKEN = PLLKEN_ON       // PLLKEN (PLLKEN_ON)
#pragma config XTCFG = G3               // XT Config (24-32 MHz crystals)
#pragma config XTBST = ENABLE           // XT Boost (Boost the kick-start)

// FWDT
#pragma config RWDTPS = PS8192       // Run Mode Watchdog Timer Post Scaler select bits (1:8192)
  // 8 secondi, 17/12/19
#pragma config RCLKSEL = LPRC           // Watchdog Timer Clock Select bits (Always use LPRC)
#pragma config WINDIS = ON              // Watchdog Timer Window Enable bit (Watchdog Timer operates in Non-Window mode)
#pragma config WDTWIN = WIN25           // Watchdog Timer Window Select bits (WDT Window is 25% of WDT period)
#pragma config SWDTPS = PS8192       // Sleep Mode Watchdog Timer Post Scaler select bits (1:8192)
#pragma config FWDTEN = ON              // Watchdog Timer Enable bit (WDT enabled in hardware)

// FICD
#pragma config ICS = PGD1               // ICD Communication Channel Select bits (Communicate on PGEC1 and PGED1)
#pragma config JTAGEN = OFF             // JTAG Enable bit (JTAG is disabled)

// FDMTIVTL
#pragma config DMTIVTL = 0xFFFF         // Dead Man Timer Interval low word (Lower 16 bits of 32 bitDMT window interval (0-0xFFFF))

// FDMTIVTH
#pragma config DMTIVTH = 0xFFFF         // Dead Man Timer Interval high word (Uper 16 bits of 32 bitDMT window interval (0-0xFFFF))

// FDMTCNTL
#pragma config DMTCNTL = 0xFFFF         // Lower 16 bits of 32 bit DMT instruction count time-out value (0-0xFFFF) (Lower 16 bits of 32 bit DMT instruction count time-out value (0-0xFFFF))

// FDMTCNTH
#pragma config DMTCNTH = 0xFFFF         // Upper 16 bits of 32 bit DMT instruction count time-out value (0-0xFFFF) (Upper 16 bits of 32 bit DMT instruction count time-out value (0-0xFFFF))

// FDMT
#pragma config DMTDIS = OFF             // Dead Man Timer Disable bit (Dead Man Timer is Disabled and can be enabled by software)

// FDEVOPT
#pragma config ALTI2C1 = OFF            // Alternate I2C1 Pin bit (I2C1 mapped to SDA1/SCL1 pins)
#pragma config ALTI2C2 = OFF            // Alternate I2C2 Pin bit (I2C2 mapped to SDA2/SCL2 pins)
#pragma config SMBEN = SMBUS            // SM Bus Enable (SMBus input threshold is enabled)
#pragma config SPI2PIN = PPS            // SPI2 Pin Select bit (SPI2 uses I/O remap (PPS) pins)

// FALTREG
#pragma config CTXT1 = OFF              // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 1 bits (Not Assigned)
#pragma config CTXT2 = OFF              // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 2 bits (Not Assigned)
#pragma config CTXT3 = OFF              // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 3 bits (Not Assigned)
#pragma config CTXT4 = OFF              // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 4 bits (Not Assigned)

// FMBXM
#pragma config MBXM0 = M2S              // Mailbox 0 data direction (Mailbox register configured for Master data read (Slave to Master data transfer))
#pragma config MBXM1 = M2S              // Mailbox 1 data direction (Mailbox register configured for Master data read (Slave to Master data transfer))
#pragma config MBXM2 = M2S              // Mailbox 2 data direction (Mailbox register configured for Master data read (Slave to Master data transfer))
#pragma config MBXM3 = M2S              // Mailbox 3 data direction (Mailbox register configured for Master data read (Slave to Master data transfer))
#pragma config MBXM4 = M2S              // Mailbox 4 data direction (Mailbox register configured for Master data read (Slave to Master data transfer))
#pragma config MBXM5 = M2S              // Mailbox 5 data direction (Mailbox register configured for Master data read (Slave to Master data transfer))
#pragma config MBXM6 = M2S              // Mailbox 6 data direction (Mailbox register configured for Master data read (Slave to Master data transfer))
#pragma config MBXM7 = M2S              // Mailbox 7 data direction (Mailbox register configured for Master data read (Slave to Master data transfer))
#pragma config MBXM8 = S2M              // Mailbox 8 data direction (Mailbox register configured for Master data read (Slave to Master data transfer))
#pragma config MBXM9 = S2M              // Mailbox 9 data direction (Mailbox register configured for Master data read (Slave to Master data transfer))
#pragma config MBXM10 = S2M             // Mailbox 10 data direction (Mailbox register configured for Master data read (Slave to Master data transfer))
#pragma config MBXM11 = S2M             // Mailbox 11 data direction (Mailbox register configured for Master data read (Slave to Master data transfer))
#pragma config MBXM12 = S2M             // Mailbox 12 data direction (Mailbox register configured for Master data read (Slave to Master data transfer))
#pragma config MBXM13 = S2M             // Mailbox 13 data direction (Mailbox register configured for Master data read (Slave to Master data transfer))
#pragma config MBXM14 = S2M             // Mailbox 14 data direction (Mailbox register configured for Master data read (Slave to Master data transfer))
#pragma config MBXM15 = S2M             // Mailbox 15 data direction (Mailbox register configured for Master data read (Slave to Master data transfer))

// FMBXHS1
#pragma config MBXHSA = MBX15           // Mailbox handshake protocol block A register assignment (MSIxMBXD15 assigned to mailbox handshake protocol block A)
#pragma config MBXHSB = MBX15           // Mailbox handshake protocol block B register assignment (MSIxMBXD15 assigned to mailbox handshake protocol block B)
#pragma config MBXHSC = MBX15           // Mailbox handshake protocol block C register assignment (MSIxMBXD15 assigned to mailbox handshake protocol block C)
#pragma config MBXHSD = MBX15           // Mailbox handshake protocol block D register assignment (MSIxMBXD15 assigned to mailbox handshake protocol block D)

// FMBXHS2
#pragma config MBXHSE = MBX15           // Mailbox handshake protocol block E register assignment (MSIxMBXD15 assigned to mailbox handshake protocol block E)
#pragma config MBXHSF = MBX15           // Mailbox handshake protocol block F register assignment (MSIxMBXD15 assigned to mailbox handshake protocol block F)
#pragma config MBXHSG = MBX15           // Mailbox handshake protocol block G register assignment (MSIxMBXD15 assigned to mailbox handshake protocol block G)
#pragma config MBXHSH = MBX15           // Mailbox handshake protocol block H register assignment (MSIxMBXD15 assigned to mailbox handshake protocol block H)

// FMBXHSEN
#pragma config HSAEN = OFF              // Mailbox A data flow control protocol block enable (Mailbox data flow control handshake protocol block disabled.)
#pragma config HSBEN = OFF              // Mailbox B data flow control protocol block enable (Mailbox data flow control handshake protocol block disabled.)
#pragma config HSCEN = OFF              // Mailbox C data flow control protocol block enable (Mailbox data flow control handshake protocol block disabled.)
#pragma config HSDEN = OFF              // Mailbox D data flow control protocol block enable (Mailbox data flow control handshake protocol block disabled.)
#pragma config HSEEN = OFF              // Mailbox E data flow control protocol block enable (Mailbox data flow control handshake protocol block disabled.)
#pragma config HSFEN = OFF              // Mailbox F data flow control protocol block enable (Mailbox data flow control handshake protocol block disabled.)
#pragma config HSGEN = OFF              // Mailbox G data flow control protocol block enable (Mailbox data flow control handshake protocol block disabled.)
#pragma config HSHEN = OFF              // Mailbox H data flow control protocol block enable (Mailbox data flow control handshake protocol block disabled.)

// FCFGPRA0
#pragma config CPRA0 = MSTR             // Pin RA0 Ownership Bits (Master core owns pin.)
#pragma config CPRA1 = MSTR             // Pin RA1 Ownership Bits (Master core owns pin.)
#pragma config CPRA2 = MSTR             // Pin RA2 Ownership Bits (Master core owns pin.)
#pragma config CPRA3 = MSTR             // Pin RA3 Ownership Bits (Master core owns pin.)
#pragma config CPRA4 = MSTR             // Pin RA4 Ownership Bits (Master core owns pin.)

// FCFGPRB0
#pragma config CPRB0 = MSTR             // Pin RB0 Ownership Bits (Master core owns pin.)
#pragma config CPRB1 = MSTR             // Pin RB1 Ownership Bits (Master core owns pin.)
#pragma config CPRB2 = MSTR             // Pin RB2 Ownership Bits (Master core owns pin.)
#pragma config CPRB3 = MSTR             // Pin RB3 Ownership Bits (Master core owns pin.)
#pragma config CPRB4 = MSTR             // Pin RB4 Ownership Bits (Master core owns pin.)
#pragma config CPRB5 = MSTR             // Pin RB5 Ownership Bits (Master core owns pin.)
#pragma config CPRB6 = MSTR             // Pin RB6 Ownership Bits (Master core owns pin.)
#pragma config CPRB7 = MSTR             // Pin RB7 Ownership Bits (Master core owns pin.)
#pragma config CPRB8 = MSTR             // Pin RB8 Ownership Bits (Master core owns pin.)
#pragma config CPRB9 = MSTR             // Pin RB9 Ownership Bits (Master core owns pin.)
#pragma config CPRB10 = MSTR            // Pin RB10 Ownership Bits (Master core owns pin.)
#pragma config CPRB11 = MSTR            // Pin RB11 Ownership Bits (Master core owns pin.)
#pragma config CPRB12 = MSTR            // Pin RB12 Ownership Bits (Master core owns pin.)
#pragma config CPRB13 = MSTR            // Pin RB13 Ownership Bits (Master core owns pin.)
#pragma config CPRB14 = MSTR            // Pin RB14 Ownership Bits (Master core owns pin.)
#pragma config CPRB15 = MSTR            // Pin RB15 Ownership Bits (Master core owns pin.)

// FS1OSCSEL
#pragma config S1FNOSC = FRCPLL  //FRCDIVN        // Oscillator Source Selection (Internal Fast RC (FRC) Oscillator with postscaler)
#pragma config S1IESO = ON              // Two-speed Oscillator Start-up Enable bit (Start up device with FRC, then switch to user-selected oscillator source)

// FS1OSC
#pragma config S1OSCIOFNC = OFF         // Slave OSC2 Pin Function bit (OSC2 is clock output)
#pragma config S1FCKSM = CSDCMD         // Clock Switching Mode bits (Both Clock switching and Fail-safe Clock Monitor are disabled)
#pragma config S1PLLKEN = S1PLLKEN_ON   // S1PLLKEN (S1PLLKEN_ON)

// FS1WDT
#pragma config S1RWDTPS = PS8192     // Run Mode Watchdog Timer Post Scaler select bits (1:8192)
#pragma config S1RCLKSEL = LPRC         // Watchdog Timer Clock Select bits (Always use LPRC)
#pragma config S1WINDIS = ON            // Watchdog Timer Window Enable bit (Watchdog Timer operates in Non-Window mode)
#pragma config S1WDTWIN = WIN25         // Watchdog Timer Window Select bits (WDT Window is 25% of WDT period)
#pragma config S1SWDTPS = PS8192     // Sleep Mode Watchdog Timer Post Scaler select bits (1:8192)
#pragma config S1FWDTEN = ON_SW           // Watchdog Timer Enable bit (WDT enabled in hardware)

// FS1ICD
#pragma config S1ICS = PGD1             // ICD Communication Channel Select bits (Communicate on PGEC1 and PGED1)
#pragma config S1ISOLAT = ON            // Isolate the Slave core subsystem from the master subsystem during Debug (The slave can operate (in debug mode) even if the SLVEN bit in the MSI is zero.)
#pragma config S1NOBTSWP = OFF          // BOOTSWP Instruction Enable/Disable bit (BOOTSWP instruction is disabled)

// FS1DEVOPT
#pragma config S1ALTI2C1 = OFF          // Alternate I2C1 Pin bit (I2C1 mapped to SDA1/SCL1 pins)
#pragma config S1SPI1PIN = PPS          // S1 SPI1 Pin Select bit (Slave SPI1 uses I/O remap (PPS) pins)
#pragma config S1SSRE = ON              // Slave Slave Reset Enable (Slave generated resets will reset the Slave Enable Bit in the MSI module)
#pragma config S1MSRE = ON              // Master Slave Reset Enable (The master software oriented RESET events (RESET Op-Code, Watchdog timeout, TRAP reset, illegalInstruction) will also cause the slave subsystem to reset.)

// FS1ALTREG
#pragma config S1CTXT1 = OFF            // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 1 bits (Not Assigned)
#pragma config S1CTXT2 = OFF            // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 2 bits (Not Assigned)
#pragma config S1CTXT3 = OFF            // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 3 bits (Not Assigned)
#pragma config S1CTXT4 = OFF            // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 4 bits (Not Assigned)
#endif
#endif



const char CopyrightString[]= {'C','6','4',' ','E','m','u','l','a','t','o','r',' ','v',
	VERNUMH+'0','.',VERNUML/10+'0',(VERNUML % 10)+'0',' ','-',' ', '0','6','/','1','3','/','2','2', 0 };

const char Copyr1[]="(C) Dario's Automation 2019-2021 - G.Dar\xd\xa\x0";



// Global Variables:
BOOL fExit,debug;
extern BYTE DoIRQ,DoNMI,DoHalt,DoReset,ColdReset;
#if defined(__PIC32MM__)
extern BYTE ram_seg[MAX_RAM],*stack_seg;
extern BYTE ColorRAM[1024];
#else
extern BYTE ram_seg[0x10000],*stack_seg;
extern BYTE ColorRAM[1024],VideoHIRAM[((HORIZ_SIZE/2)+(HORIZ_OFFSCREEN*2))*(MAX_RASTER-MIN_RASTER+1) /* /8 FARE */];
#endif
extern BYTE VICReg[64];
extern SWORD VICRaster;
extern BYTE SIDReg[32];
extern BYTE CIA1RegR[16],CIA1RegW[16];
extern BYTE CIA2RegR[16],CIA2RegW[16];
extern BYTE CPUIOReg[2];
extern BYTE PLAReg[1];
extern BYTE Keyboard[8];
extern volatile BYTE CIA1IRQ,CIA2IRQ,VICIRQ;

const char keysFeed[]="10 PRINT TI:?\r20 POKE 53281,2\rPOKE 54273,100\rLIST\rRUN\r";
volatile BYTE *keysFeedPtr=NULL;
static WORD dividerEmulKbd=0;

#if defined(__PIC32MM__)
#include "C:\Users\dario\MPLABXProjects\WiFi_PenUSB.X\at_winc1500.h"
BYTE myRSSI;
uint8 internetBuffer[100];     // 
WORD internetBufferLen;   //
SOCKET TCPlistenSocket=INVALID_SOCKET, TCPacceptedSocket=INVALID_SOCKET;
uint8 rxBuffer[1536]; // 
uint8 bIsfinished=0;
sint16 sendEx(SOCKET, void *, uint16);
void tcpStartServer(uint16 u16ServerPort);
void clientSocketEventHandler(SOCKET sock, uint8 u8Msg, void *pvMsg);
enum VNC_SECURITY_TYPES {
	VNC_SECURITY_TYPE_INVALID       =  0,
	VNC_SECURITY_TYPE_NONE          =  1,
	VNC_SECURITY_TYPE_VNC           =  2,
	VNC_SECURITY_TYPE_RA2           =  5,
	VNC_SECURITY_TYPE_RA2ne         =  6,
	VNC_SECURITY_TYPE_TIGHT         = 16,
	VNC_SECURITY_TYPE_ULTRA         = 17,
	VNC_SECURITY_TYPE_TLS           = 18,
	VNC_SECURITY_TYPE_VENCRYPT      = 19,
	VNC_SECURITY_TYPE_GTK_VNC_SASL  = 20,
	VNC_SECURITY_TYPE_MD5_HASH_AUTH = 21,
	VNC_SECURITY_TYPE_XVP           = 22,
	VNC_SECURITY_TYPE_ARD           = 30,
	VNC_TIGHT_AUTH_TGHT_ULGNAUTH	= 119,
	VNC_TIGHT_AUTH_TGHT_XTRNAUTH	= 130
  };
enum VNC_CLIENT_MESSAGE_TYPES {
	VNC_CLIENT_MESSAGE_TYPE_SET_PIXEL_FORMAT	= 0,
	VNC_CLIENT_MESSAGE_TYPE_SET_ENCODING		= 2,
	VNC_CLIENT_MESSAGE_TYPE_FRAMEBUF_UPDATE_REQ	= 3,
	VNC_CLIENT_MESSAGE_TYPE_KEY_EVENT		= 4,
	VNC_CLIENT_MESSAGE_TYPE_POINTER_EVENT		= 5,
	VNC_CLIENT_MESSAGE_TYPE_CLIENT_CUT_TEXT		= 6,
	VNC_CLIENT_MESSAGE_TYPE_MIRRORLINK		= 128
  };
enum VNC_SERVER_MESSAGES_TYPES {
	VNC_SERVER_MESSAGE_TYPE_FRAMEBUFFER_UPDATE   = 0,
	VNC_SERVER_MESSAGE_TYPE_SET_COLORMAP_ENTRIES = 1,
	VNC_SERVER_MESSAGE_TYPE_RING_BELL            = 2,
	VNC_SERVER_MESSAGE_TYPE_CUT_TEXT             = 3,
	VNC_SERVER_MESSAGE_TYPE_MIRRORLINK           = 128
  };
#define VNC_ENCODING_TYPE_DESKTOP_SIZE       0xFFFFFF21
#define VNC_ENCODING_TYPE_LAST_RECT          0xFFFFFF20
#define VNC_ENCODING_TYPE_POINTER_POS        0xFFFFFF18
#define VNC_ENCODING_TYPE_RICH_CURSOR        0xFFFFFF11
#define VNC_ENCODING_TYPE_X_CURSOR           0xFFFFFF10
#define VNC_ENCODING_TYPE_RAW                0
#define VNC_ENCODING_TYPE_COPY_RECT          1
#define VNC_ENCODING_TYPE_RRE                2
#define VNC_ENCODING_TYPE_CORRE              4
#define VNC_ENCODING_TYPE_HEXTILE            5
#define VNC_ENCODING_TYPE_ZLIB	             6
#define VNC_ENCODING_TYPE_TIGHT	             7
#define VNC_ENCODING_TYPE_ZLIBHEX            8
#define VNC_ENCODING_TYPE_ULTRA	             9
#define VNC_ENCODING_TYPE_TRLE	             15
#define VNC_ENCODING_TYPE_RLE	             16
#define VNC_ENCODING_TYPE_HITACHI_ZYWRLE     17
#define VNC_ENCODING_TYPE_JPEG_0             -32
#define VNC_ENCODING_TYPE_JPEG_1             -31
#define VNC_ENCODING_TYPE_JPEG_2             -30
#define VNC_ENCODING_TYPE_JPEG_3             -29
#define VNC_ENCODING_TYPE_JPEG_4             -28
#define VNC_ENCODING_TYPE_JPEG_5             -27
#define VNC_ENCODING_TYPE_JPEG_6             -26
#define VNC_ENCODING_TYPE_JPEG_7             -25
#define VNC_ENCODING_TYPE_JPEG_8             -24
#define VNC_ENCODING_TYPE_JPEG_9             -23
#define VNC_ENCODING_TYPE_COMPRESSION_0      0xFFFFFF00
#define VNC_ENCODING_TYPE_COMPRESSION_1      0xFFFFFF01
#define VNC_ENCODING_TYPE_COMPRESSION_2      0xFFFFFF02
#define VNC_ENCODING_TYPE_COMPRESSION_3      0xFFFFFF03
#define VNC_ENCODING_TYPE_COMPRESSION_4      0xFFFFFF04
#define VNC_ENCODING_TYPE_COMPRESSION_5      0xFFFFFF05
#define VNC_ENCODING_TYPE_COMPRESSION_6      0xFFFFFF06
#define VNC_ENCODING_TYPE_COMPRESSION_7      0xFFFFFF07
#define VNC_ENCODING_TYPE_COMPRESSION_8      0xFFFFFF08
#define VNC_ENCODING_TYPE_COMPRESSION_9      0xFFFFFF09
#define VNC_ENCODING_TYPE_WMVi               0x574D5669
#define VNC_ENCODING_TYPE_CACHE              0xFFFF0000
#define VNC_ENCODING_TYPE_CACHE_ENABLE       0xFFFF0001
#define VNC_ENCODING_TYPE_XOR_ZLIB           0xFFFF0002
#define VNC_ENCODING_TYPE_XOR_MONO_ZLIB      0xFFFF0003
#define VNC_ENCODING_TYPE_XOR_MULTI_ZLIB     0xFFFF0004
#define VNC_ENCODING_TYPE_SOLID_COLOR        0xFFFF0005
#define VNC_ENCODING_TYPE_XOR_ENABLE         0xFFFF0006
#define VNC_ENCODING_TYPE_CACHE_ZIP          0xFFFF0007
#define VNC_ENCODING_TYPE_SOL_MONO_ZIP       0xFFFF0008
#define VNC_ENCODING_TYPE_ULTRA_ZIP          0xFFFF0009
#define VNC_ENCODING_TYPE_SERVER_STATE       0xFFFF8000
#define VNC_ENCODING_TYPE_ENABLE_KEEP_ALIVE  0xFFFF8001
#define VNC_ENCODING_TYPE_FTP_PROTO_VER      0xFFFF8002
#define VNC_ENCODING_TYPE_POINTER_CHANGE     -257
#define VNC_ENCODING_TYPE_EXT_KEY_EVENT      -258
#define VNC_ENCODING_TYPE_AUDIO               259
#define VNC_ENCODING_TYPE_DESKTOP_NAME       -307
#define VNC_ENCODING_TYPE_EXTENDED_DESK_SIZE -308
#define VNC_ENCODING_TYPE_KEYBOARD_LED_STATE 0XFFFE0000
#define VNC_ENCODING_TYPE_SUPPORTED_MESSAGES 0XFFFE0001
#define VNC_ENCODING_TYPE_SUPPORTED_ENCODINGS 0XFFFE0002
#define VNC_ENCODING_TYPE_SERVER_IDENTITY    0XFFFE0003
#define VNC_ENCODING_TYPE_MIRRORLINK         0xFFFFFDF5
#define VNC_ENCODING_TYPE_CONTEXT_INFORMATION 0xFFFFFDF4
#define VNC_ENCODING_TYPE_SLRLE              0xFFFFFDF3
#define VNC_ENCODING_TYPE_TRANSFORM          0xFFFFFDF2
#define VNC_ENCODING_TYPE_HSML               0xFFFFFDF1
#define VNC_ENCODING_TYPE_H264               0X48323634
#pragma scalar_storage_order big-endian
struct __attribute__((packed)) RFB_PIXEL_FORMAT {
  uint8_t bitsPerPixel;       /* 8,16,32 only */
  uint8_t depth;              /* 8 to 32 */
  uint8_t bigEndian;          /* True if multi-byte pixels are interpreted
                                  as big endian, or if single-bit-per-pixel
                                  has most significant bit of the byte
                                  corresponding to first (leftmost) pixel. Of
                                  course this is meaningless for 8 bits/pix */
  uint8_t trueColour;         /* If false then we need a "colour map" to
                                  convert pixels to RGB.  If true, xxxMax and
                                  xxxShift specify bits used for red, green
                                  and blue */
  /* the following fields are only meaningful if trueColour is true */
  uint16_t redMax;            /* maximum red value (= 2^n - 1 where n is the
                                  number of bits used for red). Note this
                                  value is always in big endian order. */
  uint16_t greenMax;          /* similar for green */
  uint16_t blueMax;           /* and blue */
  uint8_t redShift;           /* number of shifts needed to get the red
                                  value in a pixel to the least significant
                                  bit. To find the red value from a given
                                  pixel, do the following:
                                  1) Swap pixel value according to bigEndian
                                     (e.g. if bigEndian is false and host byte
                                     order is big endian, then swap).
                                  2) Shift right by redShift.
                                  3) AND with redMax (in host byte order).
                                  4) You now have the red value between 0 and
                                     redMax. */
  uint8_t greenShift;         /* similar for green */
  uint8_t blueShift;          /* and blue */
  uint8_t pad1;
  uint16_t pad2;
  };
struct __attribute__((packed,scalar_storage_order("big-endian"))) RFB_FRAMEBUFFER_UPDATE_MSG {
  uint8_t type;                       /* always rfbFramebufferUpdate */
  uint8_t pad;
  uint16_t nRects;
  /* followed by nRects rectangles */
  };
struct __attribute__((packed,scalar_storage_order("big-endian"))) RFB_FIX_COLOUR_MAP_ENTRIES_MSG {
  uint8_t type;                       /* always rfbFixColourMapEntries */
  uint8_t pad;
  uint16_t firstColour;
  uint16_t nColours;
  /* Followed by nColours * 3 * uint16_t
     r1, g1, b1, r2, g2, b2, r3, g3, b3, ..., rn, bn, gn */
  };
struct __attribute__((packed,scalar_storage_order("big-endian"))) RFB_FRAMEBUFFER_UPDATE_RECT_HEADER {
  RECT r;
  uint32_t encoding;  /* one of the encoding types rfbEncoding... */
  };
struct __attribute__((packed)) RFB_SET_PIXEL_FORMAT_MSG {
  uint8_t type;                       /* always rfbSetPixelFormat */
  uint8_t pad1;
  uint16_t pad2;
  struct RFB_PIXEL_FORMAT pixelFormat;
  };
struct __attribute__((packed,scalar_storage_order("big-endian"))) RFB_SET_ENCODINGS_MSG {
  uint8_t type;                       /* always rfbSetEncodings */
  uint8_t pad;
  uint16_t nEncodings;
  /* followed by nEncodings * uint32_t encoding types */
  };
struct __attribute__((packed,scalar_storage_order("big-endian"))) RFB_FRAMEBUFFER_UPDATE_REQUEST_MSG {
  uint8_t type;                       /* always rfbFramebufferUpdateRequest */
  uint8_t incremental;
  uint16_t x;
  uint16_t y;
  uint16_t w;
  uint16_t h;
  };
struct __attribute__((packed)) RFB_KEY_EVENT_MSG {
  uint8_t type;                       /* always rfbKeyEvent */
  uint8_t down;                       /* true if down (press), false if up */
  uint16_t pad;
  union {
    uint32_t key;                       /* key is specified as an X keysym */
    uint8_t keys[4];
    };
  };
struct __attribute__((packed,scalar_storage_order("big-endian"))) RFB_POINTER_EVENT_MSG {
  uint8_t type;                       /* always rfbPointerEvent */
  uint8_t buttonMask;         /* bits 0-7 are buttons 1-8, 0=up, 1=down */
  uint16_t x;
  uint16_t y;
  };
struct __attribute__((packed,scalar_storage_order("big-endian"))) RFB_CLIENT_CUT_TEXT_MSG {
  uint8_t type;                       /* always rfbClientCutText */
  uint8_t pad1;
  uint16_t pad2;
  uint32_t length;
  /* followed by char text[length] */
  };
struct __attribute__((packed)) RFB_RRE_HEADER {
  uint32_t nSubrects;
  };
#define VNC_DESKTOPNAME "C64"
struct __attribute__((packed,scalar_storage_order("big-endian"))) RFB_SERVER_INIT_MSG {
  uint16_t framebufferWidth;
  uint16_t framebufferHeight;
  struct RFB_PIXEL_FORMAT format;      /* the server's preferred pixel format */
  uint32_t nameLength;
  /* followed by char name[nameLength] */
  char name[sizeof(VNC_DESKTOPNAME)-1];
  };
#pragma scalar_storage_order little-endian
enum VNC_STATE {
  VNC_IDLE=0,
  VNC_PROTOCOL,
  VNC_SECURITY,
  VNC_AUTH,
  VNC_AUTH_CONFIRMED,
  VNC_CLIENT_INIT,
  VNC_RUNNING, 
  VNC_RUNNING_WAIT0,
  VNC_RUNNING_WAIT1,
  VNC_RUNNING_WAIT2,
  VNC_RUNNING_WAIT22,
  VNC_RUNNING_WAIT3,
  VNC_RUNNING_WAIT4,
  VNC_RUNNING_WAIT5,
  VNC_RUNNING_WAIT6,
  VNC_RUNNING_WAIT66,
  VNC_RUNNING_WAIT7,
  VNC_RUNNING_WAIT8,
  VNC_RUNNING_WAIT9
  } VNCState;
RECT VNCArea={0,0,0,0};
BYTE VNCEncoding;
struct RFB_PIXEL_FORMAT VNCFormat;

#endif


const WORD textColors[16]={BLACK,WHITE,RED,CYAN,MAGENTA,GREEN,BLUE,YELLOW,
	ORANGE,BROWN,BRIGHTRED,DARKGRAY,GRAY128,LIGHTGREEN,BRIGHTCYAN,LIGHTGRAY};
/*COLORREF Colori[16]={
	RGB(0,0,0),						 // nero
	RGB(0xff,0xff,0xff),	 // bianco
	RGB(0x80,0x00,0x00),	 // rosso
	RGB(0x00,0x80,0x80),	 // azzurro
	RGB(0x80,0x00,0x80),	 // porpora
	RGB(0x00,0x80,0x00),	 // verde
	RGB(0x00,0x00,0x80),	 // blu
	RGB(0x80,0x80,0x00),	 // giallo
	
	RGB(0xff,0x80,0x40),	 // arancio
	RGB(0x80,0x40,0x40),	 // marrone
	RGB(0xff,0x80,0x80),	 // rosso chiaro
	RGB(0x20,0x20,0x20),	 // grigio 1
	RGB(0x54,0x54,0x54),	 // grigio 2
	RGB(0xc0,0xc0,0xc0),	 // grigio chiaro
	RGB(0x80,0x80,0xff),	 // blu chiaro
	RGB(0xa8,0xa8,0xa8)		 // grigio 3
	};*/



#if defined(__PIC32MM__)
#define ROWINI_OFFSET 0
//#define USA_COMPRESSION 1
BYTE myRowIni=0;

int UpdateScreen(SWORD rowIni, SWORD rowFin) {
	register int i,j,k;
	int y1,y2,x1,x2,row1,row2;
	register BYTE *p,*p1;
	BYTE *psc,*psc2;
  BYTE ch;
  BYTE *rowData=&rxBuffer;

  // ci mette circa ??mS ogni passata... (viene chiamata circa 1 volte al secondo , / /21)
  
      LED3 = 1;
      

 
    if(dividerEmulKbd)    // se no disturba...
      goto skippa;
      
    //25mS 15/6/21
    if(VNCState==VNC_RUNNING) {     
      
      if(VNCArea.bottom && VNCArea.right) {

        myRowIni++;

        switch(VNCFormat.bitsPerPixel) {
          case 8:
            if(myRowIni==1) {
      struct RFB_FRAMEBUFFER_UPDATE_MSG frameBufferUpdate;
      struct RFB_FRAMEBUFFER_UPDATE_RECT_HEADER frameBufferUpdateRect;
        
#ifdef USA_COMPRESSION
//      PROVo Encoding = 2 (RRE) e così mando i due bordi compressi
        
        // NON va.. sembrano sfalsati v. sotto i colori vs. #num bits...
        
      rowData[0]=VNC_SERVER_MESSAGE_TYPE_FRAMEBUFFER_UPDATE; rowData[1]=0;
      rowData[2]=0; rowData[3]=3;   // 3 rectangle
      // BIG ENDIAN!
      rowData[4]=rowData[5]=0; rowData[6]=rowData[7]=0;      // VNCArea.top ecc?
      rowData[8]=HIBYTE(VNCArea.right); rowData[9]=LOBYTE(VNCArea.right);
      rowData[10]=0; rowData[11]=20;
      rowData[12]=rowData[13]=rowData[14]=0; rowData[15]=VNC_ENCODING_TYPE_RRE;    // VNCEncoding
#else
      frameBufferUpdate.type=VNC_SERVER_MESSAGE_TYPE_FRAMEBUFFER_UPDATE;
      frameBufferUpdate.nRects=htons(1);
      
      
      rowData[0]=VNC_SERVER_MESSAGE_TYPE_FRAMEBUFFER_UPDATE; rowData[1]=0;
      rowData[2]=0; rowData[3]=1;   // 1 rectangle

      // BIG ENDIAN!
      rowData[4]=rowData[5]=0; rowData[6]=rowData[7]=0;      // VNCArea.top ecc?
      rowData[8]=HIBYTE(VNCArea.right); rowData[9]=LOBYTE(VNCArea.right);
      rowData[10]=HIBYTE(VNCArea.bottom); rowData[11]=LOBYTE(VNCArea.bottom);
      rowData[12]=rowData[13]=rowData[14]=0; rowData[15]=VNC_ENCODING_TYPE_RAW;    // VNCEncoding
#endif
      
      sendEx(TCPacceptedSocket,rowData,16);     // FrameUpdate

      rowData[0]=VNC_SERVER_MESSAGE_TYPE_SET_COLORMAP_ENTRIES; rowData[1]=0;
      // BIG ENDIAN!
      rowData[2]=0; rowData[3]=0;
      rowData[4]=0; rowData[5]=16;
      for(i=0; i<16; i++) {
        rowData[6+i*6]=ColorR(textColors[i]); rowData[6+1+i*6]=0;
        rowData[6+2+i*6]=ColorG(textColors[i]); rowData[6+3+i*6]=0;
        rowData[6+4+i*6]=ColorB(textColors[i]); rowData[6+5+i*6]=0;
        }
//      sendEx(TCPacceptedSocket,rowData,6+6*16);     // SetColorMap
      
#ifdef USA_COMPRESSION
            rowData[0]=0; rowData[1]=0; rowData[2]=0; rowData[3]=1;   // # subrectangles
            rowData[4]=0; rowData[5]=Color565To222(textColors[VICReg[0x20]]);
            rowData[6]=0; rowData[7]=0; // boh padding necessario...
            rowData[8]=0; rowData[9]=0; rowData[10]=0; rowData[11]=0;
            rowData[12]=HIBYTE(VNCArea.right); rowData[13]=LOBYTE(VNCArea.right);
            rowData[14]=0; rowData[15]=20;
            sendEx(TCPacceptedSocket,rowData,16);     // 
#else
            for(j=0; j<20/4; j++) {
              for(i=0; i<320*4; i++) {
                if(VNCFormat.redShift==4)
                  // 4=2-2-2; 5=3-3-2; ci sarebbe anche 2 x 8 colori/1 bit/pixel, e altre variazioni in TightVNC... tipo 0-3-6
                  rowData[i]=Color565To222(textColors[VICReg[0x20]]);
                else
                  rowData[i]=Color565To332(textColors[VICReg[0x20]]);
                }
              sendEx(TCPacceptedSocket,rowData,320*4);
              }
#endif
            }
            else if(myRowIni>1 && myRowIni<27) {
              //if((rowIni & 8)) {   // volevo fare blocchi da 16, per una futura compressione TRLE... 
#ifdef USA_COMPRESSION
            rowData[0]=rowData[1]=0; rowData[2]=0; rowData[3]=20;      // VNCArea.top
            rowData[4]=HIBYTE(VNCArea.right); rowData[5]=LOBYTE(VNCArea.right);
            rowData[6]=0; rowData[7]=200;
            rowData[8]=rowData[9]=rowData[10]=0; rowData[11]=VNC_ENCODING_TYPE_RAW;    // VNCEncoding
            sendEx(TCPacceptedSocket,rowData,12);
#endif
            if(VICReg[0x11] & 0x10) {      // video disabilitato
              if(VICReg[0x11] & 0x20) {			 // bitmap (BMM)
                if(VICReg[0x16] & 0x10) {			 // multicolor (MCM)
                  p1=ram_seg+((CIA2RegR[0] & 0x3) << 14)+i*40;
                  psc=ram_seg+((VICReg[0x18] & 0xf0) << 6)+(i*40/8);
                  psc2=ColorRAM+(i*40/8);
                  
              for(j=0; j<8; j++) {//FINIRE
                for(i=0; i<320*4; i++) {
                  rowData[i]=VICReg[0x20];
                  }
                sendEx(TCPacceptedSocket,rowData,320*4);
                }
                  }
                else {
                  p1=ram_seg+((CIA2RegR[0] & 0x3) << 14)+(myRowIni-2)*40;
                  psc=ram_seg+((VICReg[0x18] & 0xf0) << 6)+(myRowIni-2)*40;
                  for(j=0; j<8; ) {
                    BYTE j1;
                    WORD i1;
                    for(j1=0,i1=0; j1<4; j1++,j++) {
                      for(i=0; i<320; ) {
                        for(k=0x80; k; k>>=1) {
                          if(*p1 & k) {
                            if(VNCFormat.redShift==4)
                              rowData[i+i1]=Color565To222(textColors[*(psc+i/8) & 0xf]);
                            else
                              rowData[i+i1]=Color565To332(textColors[*(psc+i/8) & 0xf]);
                            }
                          else {
                            if(VNCFormat.redShift==4)
                              rowData[i+i1]=Color565To222(textColors[*(psc+i/8) >> 4]);
                            else
                              rowData[i+i1]=Color565To332(textColors[*(psc+i/8) >> 4]);
                            }
                          i++;
                          }
                        p1++;
                        }
                      i1+=320;
                      }
                    sendEx(TCPacceptedSocket,rowData,320*4);
                    if(!(j & 7))
                      psc+=40;
                    }
                  }
                }
              else {							// text mode
                p1=ram_seg+((VICReg[0x18] & 0xf0) << 6) +  (myRowIni-2)*40;
                psc=ColorRAM +(myRowIni-2)*40;

                for(j=0; j<8; ) {
                  BYTE j1;
                  WORD i1;
                  for(j1=0,i1=0; j1<4; j1++,j++) {
                    for(i=0; i<320; ) {
                      p=&C64char;
                      ch=*(p1+i/8);
                      if(VICReg[0x18] & 2) {       
                        p+=0x0800;
                        }
                    //	else {
                    //		p+=0x0000;
                    //		}
                      p+= ch << 3;
                      p+= j & 7;
                      for(k=0x80; k; k>>=1) {
                        if(*p & k) {
                          if(VNCFormat.redShift==4)
                            rowData[i+i1]=Color565To222(textColors[*(psc+i/8) & 0xf]);
                          else
                            rowData[i+i1]=Color565To332(textColors[*(psc+i/8) & 0xf]);
                          }
                        else {
                          if(VNCFormat.redShift==4)
                            rowData[i+i1]=Color565To222(textColors[VICReg[0x21] & 0xf]);
                          else
                            rowData[i+i1]=Color565To332(textColors[VICReg[0x21] & 0xf]);
                          }
                        i++;
                        }
                      }
                    i1+=320;
                    }
                  sendEx(TCPacceptedSocket,rowData,320*4);
                  if(!(j & 7)) {
                    p1+=40;
                    psc+=40;
                    }
                  }
                }
              }
            else {
              for(j=0; j<2; j++) {
                for(i=0; i<320*4; i++) {
                  if(VNCFormat.redShift==4)
                    rowData[i]=Color565To222(textColors[VICReg[0x20]]);
                  else
                    rowData[i]=Color565To332(textColors[VICReg[0x20]]);
                  }
                sendEx(TCPacceptedSocket,rowData,320*4);
                }
              }
            }
            else if(myRowIni==27) {   // 
#ifdef USA_COMPRESSION
            rowData[0]=rowData[1]=0; rowData[2]=0; rowData[3]=220;      // 
            rowData[4]=HIBYTE(VNCArea.right); rowData[5]=LOBYTE(VNCArea.right);
            rowData[6]=0; rowData[7]=20;
            rowData[8]=rowData[9]=rowData[10]=0; rowData[11]=VNC_ENCODING_TYPE_RRE;    // VNCEncoding
            sendEx(TCPacceptedSocket,rowData,12);     // 
            rowData[0]=0; rowData[1]=0; rowData[2]=0; rowData[3]=1;   // # subrectangles
            rowData[4]=0; rowData[5]=Color565To222(textColors[VICReg[0x20]]);
            rowData[6]=0; rowData[7]=0; // boh padding necessario...
            rowData[8]=0; rowData[9]=0; rowData[10]=0; rowData[11]=0;
            rowData[12]=HIBYTE(VNCArea.right); rowData[13]=LOBYTE(VNCArea.right);
            rowData[14]=0; rowData[15]=20;
            sendEx(TCPacceptedSocket,rowData,16);     // 
#else
            for(j=0; j<20/4; j++) {
              for(i=0; i<320*4; i++) {
                if(VNCFormat.redShift==4)
                  rowData[i]=Color565To222(textColors[VICReg[0x20]]);
                else
                  rowData[i]=Color565To332(textColors[VICReg[0x20]]);
                }
              sendEx(TCPacceptedSocket,rowData,320*4);
              }
#endif
            myRowIni=0;
            VNCArea.bottom=VNCArea.right=0;
            }
            break;
            
          case 32:
            if(myRowIni==1) {
      struct RFB_FRAMEBUFFER_UPDATE_MSG frameBufferUpdate;
      struct RFB_FRAMEBUFFER_UPDATE_RECT_HEADER frameBufferUpdateRect;
        
#ifdef USA_COMPRESSION
//      PROVo Encoding = 2 (RRE) e così mando i due bordi compressi
        
        // NON va.. sembrano sfalsati v. sotto i colori vs. #num bits...
        
      rowData[0]=VNC_SERVER_MESSAGE_TYPE_FRAMEBUFFER_UPDATE; rowData[1]=0;
      rowData[2]=0; rowData[3]=3;   // 3 rectangle
      // BIG ENDIAN!
      rowData[4]=rowData[5]=0; rowData[6]=rowData[7]=0;      // VNCArea.top ecc?
      rowData[8]=HIBYTE(VNCArea.right); rowData[9]=LOBYTE(VNCArea.right);
      rowData[10]=0; rowData[11]=20;
      rowData[12]=rowData[13]=rowData[14]=0; rowData[15]=VNC_ENCODING_TYPE_RRE;    // VNCEncoding
#else
      frameBufferUpdate.type=VNC_SERVER_MESSAGE_TYPE_FRAMEBUFFER_UPDATE;
      frameBufferUpdate.nRects=htons(1);
      
      
      rowData[0]=VNC_SERVER_MESSAGE_TYPE_FRAMEBUFFER_UPDATE; rowData[1]=0;
      rowData[2]=0; rowData[3]=1;   // 1 rectangle

      // BIG ENDIAN!
      rowData[4]=rowData[5]=0; rowData[6]=rowData[7]=0;      // VNCArea.top ecc?
      rowData[8]=HIBYTE(VNCArea.right); rowData[9]=LOBYTE(VNCArea.right);
      rowData[10]=HIBYTE(VNCArea.bottom); rowData[11]=LOBYTE(VNCArea.bottom);
      rowData[12]=rowData[13]=rowData[14]=0; rowData[15]=VNC_ENCODING_TYPE_RAW;    // VNCEncoding
#endif
      
      sendEx(TCPacceptedSocket,rowData,16);     // FrameUpdate
      
#ifdef USA_COMPRESSION
            rowData[0]=0; rowData[1]=0; rowData[2]=0; rowData[3]=1;   // # subrectangles
            rowData[4]=0; rowData[5]=Color565To222(textColors[VICReg[0x20]]);
            rowData[6]=0; rowData[7]=0; // boh padding necessario...
            rowData[8]=0; rowData[9]=0; rowData[10]=0; rowData[11]=0;
            rowData[12]=HIBYTE(VNCArea.right); rowData[13]=LOBYTE(VNCArea.right);
            rowData[14]=0; rowData[15]=20;
            sendEx(TCPacceptedSocket,rowData,16);     // 
#else
            for(j=0; j<20; j++) {
              for(i=0; i<320*4; i+=4) {
                WORD t=textColors[VICReg[0x20]];
                rowData[i]=ColorB(t);
                rowData[i+1]=ColorG(t);
                rowData[i+2]=ColorR(t);
                rowData[i+3]=0;
                }
              sendEx(TCPacceptedSocket,rowData,320*4);
              }
#endif
              }
            else if(myRowIni>1 && myRowIni<27) {
            if(VICReg[0x11] & 0x10) {      // video disabilitato
              if(VICReg[0x11] & 0x20) {			 // bitmap (BMM)
                if(VICReg[0x16] & 0x10) {			 // multicolor (MCM)
                  p1=ram_seg+((CIA2RegR[0] & 0x3) << 14)+(myRowIni-2)*40;
                  psc=ram_seg+((VICReg[0x18] & 0xf0) << 6)+(myRowIni-2)*40;
                  psc2=ColorRAM+(myRowIni-2)*40;

                  for(j=0; j<200; j++) {
                    for(i=0; i<320*4; ) {
                      WORD t;
                      switch(*p1 & 0b11000000) {
                        case 0:
                          t=textColors[VICReg[0x21] & 0xf];
                          rowData[i]=ColorB(t);
                          rowData[i+1]=ColorG(t);
                          rowData[i+2]=ColorR(t);
                          break;
                        case 1:
                          t=textColors[*(psc+i/(4*8)) & 0xf];
                          rowData[i]=ColorB(t);
                          rowData[i+1]=ColorG(t);
                          rowData[i+2]=ColorR(t);
                          break;
                        case 2:
                          t=textColors[*(psc+i/(4*8)) >> 4];
                          rowData[i]=ColorB(t);
                          rowData[i+1]=ColorG(t);
                          rowData[i+2]=ColorR(t);
                          break;
                        case 3:
                          t=textColors[*(psc2+i/(4*8)) & 0xf];
                          rowData[i]=ColorB(t);
                          rowData[i+1]=ColorG(t);
                          rowData[i+2]=ColorR(t);
                          break;
                        }
                      rowData[i+3]=0;
                      i+=4;
                      switch(*p1 & 0b00110000) {
                        case 0:
                          t=textColors[VICReg[0x21] & 0xf];
                          rowData[i]=ColorB(t);
                          rowData[i+1]=ColorG(t);
                          rowData[i+2]=ColorR(t);
                          break;
                        case 1:
                          t=textColors[*(psc+i/(4*8)) & 0xf];
                          rowData[i]=ColorB(t);
                          rowData[i+1]=ColorG(t);
                          rowData[i+2]=ColorR(t);
                          break;
                        case 2:
                          t=textColors[*(psc+i/(4*8)) >> 4];
                          rowData[i]=ColorB(t);
                          rowData[i+1]=ColorG(t);
                          rowData[i+2]=ColorR(t);
                          break;
                        case 3:
                          t=textColors[*(psc2+i/(4*8)) & 0xf];
                          rowData[i]=ColorB(t);
                          rowData[i+1]=ColorG(t);
                          rowData[i+2]=ColorR(t);
                          break;
                        }
                      rowData[i+3]=0;
                      i+=4;
                      switch(*p1 & 0b00001100) {
                        case 0:
                          t=textColors[VICReg[0x21] & 0xf];
                          rowData[i]=ColorB(t);
                          rowData[i+1]=ColorG(t);
                          rowData[i+2]=ColorR(t);
                          break;
                        case 1:
                          t=textColors[*(psc+i/(4*8)) & 0xf];
                          rowData[i]=ColorB(t);
                          rowData[i+1]=ColorG(t);
                          rowData[i+2]=ColorR(t);
                          break;
                        case 2:
                          t=textColors[*(psc+i/(4*8)) >> 4];
                          rowData[i]=ColorB(t);
                          rowData[i+1]=ColorG(t);
                          rowData[i+2]=ColorR(t);
                          break;
                        case 3:
                          t=textColors[*(psc2+i/(4*8)) & 0xf];
                          rowData[i]=ColorB(t);
                          rowData[i+1]=ColorG(t);
                          rowData[i+2]=ColorR(t);
                          break;
                        }
                      rowData[i+3]=0;
                      i+=4;
                      switch(*p1 & 0b00000011) {
                        case 0:
                          t=textColors[VICReg[0x21] & 0xf];
                          rowData[i]=ColorB(t);
                          rowData[i+1]=ColorG(t);
                          rowData[i+2]=ColorR(t);
                          break;
                        case 1:
                          t=textColors[*(psc+i/(4*8)) & 0xf];
                          rowData[i]=ColorB(t);
                          rowData[i+1]=ColorG(t);
                          rowData[i+2]=ColorR(t);
                          break;
                        case 2:
                          t=textColors[*(psc+i/(4*8)) >> 4];
                          rowData[i]=ColorB(t);
                          rowData[i+1]=ColorG(t);
                          rowData[i+2]=ColorR(t);
                          break;
                        case 3:
                          t=textColors[*(psc2+i/(4*8)) & 0xf];
                          rowData[i]=ColorB(t);
                          rowData[i+1]=ColorG(t);
                          rowData[i+2]=ColorR(t);
                          break;
                        }
                      rowData[i+3]=0;
                      i+=4;

                      p1++;
                      }
                    sendEx(TCPacceptedSocket,rowData,320*4);
                    if((j & 7) == 7)
                      psc+=40;
                    }
                  }
                else {
                  p1=ram_seg+((CIA2RegR[0] & 0x3) << 14)+(myRowIni-2)*40;
                  psc=ram_seg+((VICReg[0x18] & 0xf0) << 6)+(myRowIni-2)*40;
                  for(j=0; j<8; j++) {
                    for(i=0; i<320*4; ) {
                      for(k=0x80; k; k>>=1) {
                        if(*p1 & k) {
                          WORD t=textColors[*(psc+i/(4*8)) & 0xf];
                          rowData[i]=ColorB(t);
                          rowData[i+1]=ColorG(t);
                          rowData[i+2]=ColorR(t);
                          }
                        else {
                          WORD t=textColors[*(psc+i/(4*8)) >> 4];
                          rowData[i]=ColorB(t);
                          rowData[i+1]=ColorG(t);
                          rowData[i+2]=ColorR(t);
                          }
                        rowData[i+3]=0;
                        i+=4;
                        }
                      p1++;
                      }
                    sendEx(TCPacceptedSocket,rowData,320*4);
                    if((j & 7) == 7)
                      psc+=40;
                    }
                  }
                }
              else {							// text mode
                p1=ram_seg+((VICReg[0x18] & 0xf0) << 6)+(myRowIni-2)*40;
                psc=ColorRAM+(myRowIni-2)*40;

                for(j=0; j<8; j++) {
                  for(i=0; i<320*4; ) {
                    p=&C64char;
                    ch=*(p1+i/(4*8));
                    if(VICReg[0x18] & 2) {       
                      p+=0x0800;
                      }
                  //	else {
                  //		p+=0x0000;
                  //		}
                    p+= ch << 3;
                    p+= j & 7;
                    for(k=0x80; k; k>>=1) {
                      if(*p & k) {
                        WORD t=textColors[*(psc+i/(4*8)) & 0xf];
                        rowData[i]=ColorB(t);
                        rowData[i+1]=ColorG(t);
                        rowData[i+2]=ColorR(t);
                        }
                      else {
                        WORD t=textColors[VICReg[0x21] & 0xf];
                        rowData[i]=ColorB(t);
                        rowData[i+1]=ColorG(t);
                        rowData[i+2]=ColorR(t);
                        }
                      rowData[i+3]=0;
                      i+=4;
                      }
                    }
                  sendEx(TCPacceptedSocket,rowData,320*4);
                  if((j & 7) == 7) {
                    p1+=40;
                    psc+=40;
                    }
                  }

                }
              }
            else {
              for(j=0; j<8; j++) {
                for(i=0; i<320*4; i+=4) {
                  WORD t=textColors[VICReg[0x20]];
                  rowData[i]=ColorB(t);
                  rowData[i+1]=ColorG(t);
                  rowData[i+2]=ColorR(t);
                  rowData[i+3]=0;
                  }
                sendEx(TCPacceptedSocket,rowData,320*4);
                }
              }
              }
            else if(myRowIni==27) {   // 
            for(j=0; j<20; j++) {
              for(i=0; i<320*4; i+=4) {
                WORD t=textColors[VICReg[0x20]];
                rowData[i]=ColorB(t);
                rowData[i+1]=ColorG(t);
                rowData[i+2]=ColorR(t);
                rowData[i+3]=0;
                }
              sendEx(TCPacceptedSocket,rowData,320*4);
              }
              myRowIni=0;
              VNCArea.bottom=VNCArea.right=0;
              }
            break;
          }
        }
      

      }
//    }

skippa:
    if(!SW2) {
      if(!myRowIni)   // per non disturbare flusso...
        sendEx(TCPacceptedSocket,"\x2",1);      // prova Bell :) VNC_SERVER_MESSAGE_TYPE_RING_BELL
      }
		m2m_wifi_handle_events(NULL);

    
    LED3 = 0;

  
	}

#else

int PlotChar(DWORD pos,WORD ch,BYTE c) {
	BYTE *p,*p1;
  BYTE c1h,c1l,c2h,c2l;
	register int i,j,k;

	p=&C64char;
	if(VICReg[0x18] & 2) {       
		p+=0x0800;
		}
//	else {
//		p+=0x0000;
//		}
	p+= ch << 3;
	p1=VideoHIRAM+pos;
  
  c1h=(c & 0xf) << 4;
  c1l=c & 0xf;
  c2h=(VICReg[0x21] & 0xf) << 4;
  c2l=VICReg[0x21] & 0xf;
	for(i=0; i<8; i++) {
		k=*p++;
		*p1=k & 0x80 ? c1h : c2h;
		*p1 |= k & 0x40 ? c1l : c2l;
		p1++;
		*p1=k & 0x20 ? c1h : c2h;
		*p1 |= k & 0x10 ? c1l : c2l;
		p1++;
		*p1=k & 0x8 ? c1h : c2h;
		*p1 |= k & 0x4 ? c1l : c2l;
		p1++;
		*p1=k & 0x2 ? c1h : c2h;
		*p1 |= k & 0x1 ? c1l : c2l;

		p1+=((HORIZ_SIZE/2)-3)+(HORIZ_OFFSCREEN*2);
		}

	}

int Plot1Byte(DWORD pos,BYTE k,BYTE c) {
	BYTE *p1;
  BYTE c1h,c1l,c2h,c2l;

	p1=VideoHIRAM+pos;
  
  c1h=(c & 0xf) << 4;
  c1l=c & 0xf;
  c2h=c & 0xf0;
  c2l=(c & 0xf0) >> 4;
  *p1=k & 0x80 ? c1h : c2h;
  *p1 |= k & 0x40 ? c1l : c2l;
  p1++;
  *p1=k & 0x20 ? c1h : c2h;
  *p1 |= k & 0x10 ? c1l : c2l;
  p1++;
  *p1=k & 0x8 ? c1h : c2h;
  *p1 |= k & 0x4 ? c1l : c2l;
  p1++;
  *p1=k & 0x2 ? c1h : c2h;
  *p1 |= k & 0x1 ? c1l : c2l;

	}

int Plot1ByteMC(DWORD pos,BYTE k,BYTE c1,BYTE c2) {
	BYTE *p1;
  BYTE c1h,c1l,c2h,c2l,c3h,c3l;

	p1=VideoHIRAM+pos;
  
  c1h=(c1 & 0xf) << 4;
  c1l=c1 & 0xf;
  c2h=c1 & 0xf0;
  c2l=c1 >> 4;
  c3h=(c2 & 0xf) << 4;
  c3l=c2 & 0xf;
  switch(k & 0b11000000) {
    case 0:
      *p1=(VICReg[0x21] & 0xf) << 4;
      break;
    case 1:
      *p1=c1h;
      break;
    case 2:
      *p1=c2h;
      break;
    case 3:
      *p1=c3h;
      break;
    }
  switch(k & 0b00110000) {
    case 0:
      *p1 |=(VICReg[0x21] & 0xf);
      break;
    case 1:
      *p1 |= c1l;
      break;
    case 2:
      *p1 |= c2l;
      break;
    case 3:
      *p1 |= c3l;
      break;
    }
  p1++;
  switch(k & 0b00001100) {
    case 0:
      *p1=(VICReg[0x21] & 0xf) << 4;
      break;
    case 1:
      *p1=c1h;
      break;
    case 2:
      *p1=c2h;
      break;
    case 3:
      *p1=c3h;
      break;
    }
  switch(k & 0b00000011) {
    case 0:
      *p1 |= (VICReg[0x21] & 0xf);
      break;
    case 1:
      *p1 |= c1l;
      break;
    case 2:
      *p1 |= c2l;
      break;
    case 3:
      *p1 |= c3l;
      break;
    }
	}

#ifdef ST7735
#define ROWINI_OFFSET 32
#endif
#ifdef ILI9341
#define ROWINI_OFFSET 32
#endif
int UpdateScreen(SWORD rowIni, SWORD rowFin) {
	register int i,j;
	int k,y1,y2,x1,x2,row1,row2;
	register BYTE *p,*p1;
	BYTE *psc,*psc2;
  BYTE ch;

  // ci mette circa 1mS ogni passata... dovrebbe essere la metà... (viene chiamata circa 25-30 volte al secondo e ora siamo a 40mS invece di 20, 8/11/19)
  
	// per SPI DMA https://www.microchip.com/forums/m1110777.aspx#1110777

	if(rowIni>=ROWINI_OFFSET && rowIni<ROWINI_OFFSET +16) {
    row1=rowIni-ROWINI_OFFSET;
    row2=rowFin-ROWINI_OFFSET;
    p1=VideoHIRAM+row1*((HORIZ_SIZE/2)+(HORIZ_OFFSCREEN*2));
    memset(p1,VICReg[0x20] | (VICReg[0x20] << 4),((HORIZ_SIZE/2)+(HORIZ_OFFSCREEN*2))*VERT_OFFSCREEN);
    }
  else if(rowIni>=ROWINI_OFFSET+16 && rowIni<=200+ROWINI_OFFSET+16) {
    
      LED3 = 1;
      
    if(VICReg[0x11] & 0x10) {      // video disabilitato
			row1=rowIni-ROWINI_OFFSET;
			row2=rowFin-ROWINI_OFFSET;
#if HORIZ_OFFSCREEN>0
			for(i=row1-2; i<row2-2; i++) {
				p1=VideoHIRAM+(i*((HORIZ_SIZE/2)+(HORIZ_OFFSCREEN*2)));
				*p1=*(p1+1)=*(p1+2)=*(p1+3)=*(p1+4)=*(p1+5)=VICReg[0x20] | (VICReg[0x20] << 4);
				*(p1+(HORIZ_SIZE/2)+6)=*(p1+(HORIZ_SIZE/2)+7)=*(p1+(HORIZ_SIZE/2)+8)=*(p1+(HORIZ_SIZE/2)+9)=*(p1+(HORIZ_SIZE/2)+10)=*(p1+(HORIZ_SIZE/2)+11)=VICReg[0x20] | (VICReg[0x20] << 4);
				}
#endif
			row1-=16;
			row2-=16;
      if(VICReg[0x11] & 0x20) {			 // bitmap (BMM)
        if(VICReg[0x16] & 0x10) {			 // multicolor (MCM)
          y1=row1;
          y2=row2;
          x1=0;
          x2=320/8;
          for(i=y1; i<y2; i++) {
            k = HORIZ_OFFSCREEN+ (VERT_OFFSCREEN+(VICReg[0x11] & 7))*((HORIZ_SIZE/2)+(HORIZ_OFFSCREEN*2)) 
              + i*((HORIZ_SIZE/2)+(HORIZ_OFFSCREEN*2));
            p1=ram_seg+((CIA2RegR[0] & 0x3) << 14)+i*40;
            psc=ram_seg+((VICReg[0x18] & 0xf0) << 6)+(i*40/8);
            psc2=ColorRAM+(i*40/8);
            for(j=x1; j<x2; j++) {
              Plot1ByteMC(k++,*p1++,*psc++,*psc2++);
              }
            }
          }
        else {
          y1=row1;
          y2=row2;
          x1=0;
          x2=320/8;
          for(i=y1; i<y2; i++) {
            k = HORIZ_OFFSCREEN+ (VERT_OFFSCREEN+(VICReg[0x11] & 7))*((HORIZ_SIZE/2)+(HORIZ_OFFSCREEN*2)) 
              + i*((HORIZ_SIZE/2)+(HORIZ_OFFSCREEN*2));
            p1=ram_seg+((CIA2RegR[0] & 0x3) << 14)+i*40;
            psc=ram_seg+((VICReg[0x18] & 0xf0) << 6)+(i*40/8);
            for(j=x1; j<x2; j++) {
              Plot1Byte(k++,*p1++,*psc++);
              }
            }
          }
        }
      else {							// text mode
        y1=row1/8;
//        y1=max(y1,0);
        y2=row2/8;
        y2=min(y2,(VICReg[0x11] & 8 ? 25 : 24));
        x1=VICReg[0x16] & 8 ? 0 : 1;
        x2=VICReg[0x16] & 8 ? 40 : 39;
        if(VICReg[0x16] & 0x10) {			 // multicolor (MCM)) FARE!
          }
        for(i=y1; i<y2; i++) {
    // usare VIC[0x11] 0..2 per yscroll e VIC[0x16] per xscroll
					k = HORIZ_OFFSCREEN+ (VERT_OFFSCREEN+(VICReg[0x11] & 7))*((HORIZ_SIZE/2)+(HORIZ_OFFSCREEN*2)) 
            + i*(((HORIZ_SIZE/2)+(HORIZ_OFFSCREEN*2))*8);
          k += x1*4;
          p1=ram_seg+((VICReg[0x18] & 0xf0) << 6)+i*40;
          p1+=x1;
          psc=ColorRAM+i*40;
          psc+=x1;
          for(j=x1; j<x2; j++) {
//  					k = 6+ 18*(160+6+6) + i*((160+6+6)*8) + j*4;
//  					k = 22*((HORIZ_SIZE/2)) + i*(((HORIZ_SIZE/2))*8) + j*4;
            PlotChar(k,*p1++,*psc++);
            k+=4;
            }
          }
        }
// poi ci sarebbe anche      if(VICReg[0x11] & 0x40) {			 // char with extended background (ECM)

//			wsprintf(myBuf,"row: %d, (%d-%d)",rowIni,y1,y2);
//				SetWindowText(hStatusWnd,myBuf);
//	wsprintf(myBuf,"stretch bits: %d (%d), %d",i,GDI_ERROR,GetLastError());
//				SetWindowText(hStatusWnd,myBuf);
      }
    }
  else if(rowIni>248) {
    row1=rowIni-ROWINI_OFFSET;
    row2=rowFin-ROWINI_OFFSET;
    p1=VideoHIRAM+row1*((HORIZ_SIZE/2)+(HORIZ_OFFSCREEN*2));
    memset(p1,VICReg[0x20] | (VICReg[0x20] << 4),((HORIZ_SIZE/2)+(HORIZ_OFFSCREEN*2))*VERT_OFFSCREEN);
    }

  
	UINT16 px,py;

  
  if(rowIni>=ROWINI_OFFSET && rowIni<256+ROWINI_OFFSET) {
    
//  #define REAL_SIZE    1      // diciamo :)

#ifdef ST7735
#ifdef REAL_SIZE    
    rowIni-=32;
    rowIni/=2;
    rowFin-=32;
    rowFin/=2;

    START_WRITE();
    setAddrWindow(0,rowIni ,_width,(rowFin-rowIni));
    p1=VideoHIRAM + (rowIni*(HORIZ_SIZE/2));
    for(py=rowIni; py<rowFin /*_height*/; py+=1) {    // 200 linee 
      for(px=0; px<HORIZ_SIZE /*_width*/; px+=4) {         // 320 pixel (160byte) 
        ch=*p1++;
        writedata16(textColors[ch >> 4]);     // FINIRE 
        writedata16(textColors[ch & 0xf]);    // ogni byte contiene 2 pixel
        }
#ifdef USA_SPI_HW
      ClrWdt();
#endif
      p1+=(HORIZ_SIZE/2)/2;
      }
    END_WRITE();
  //	writecommand(CMD_NOP);

#else
    rowIni-=32;
    rowFin-=32;

    START_WRITE();
    setAddrWindow(0,rowIni/2,_width,(rowFin-rowIni)/2);
    p1=VideoHIRAM + (rowIni*(HORIZ_SIZE/2));
    if(rowIni)
      p1 -= (HORIZ_SIZE/2);     // truschino per plottare la prima riga dei caratteri, + bella!
    for(py=rowIni; py<rowFin /*_height*/; py+=2) {    // 200 linee diventa 100 O MEGLIO 125
      for(px=0; px<HORIZ_SIZE /*_width*/; px+=4) {         // 320 pixel (160byte) diventano 160)
        ch=*p1;
        p1+=2;
        writedata16x2(textColors[ch >> 4],textColors[ch & 0xf]);     // 
//        writedata16(textColors[ch >> 4]);     // FINIRE 
//        writedata16(textColors[ch & 0xf]);     // 
        // ogni byte contiene 2 pixel
        }
#ifdef USA_SPI_HW
      ClrWdt();
#endif
      p1+=(HORIZ_SIZE/2);
      }
    END_WRITE();
  //	writecommand(CMD_NOP);
#endif
#endif

#ifdef ILI9341
    rowIni-=ROWINI_OFFSET;
    rowFin-=ROWINI_OFFSET;

    START_WRITE();
    setAddrWindow(0,rowIni,_width,(rowFin-rowIni));
    p1=VideoHIRAM + (rowIni*(HORIZ_SIZE/2));
    for(py=rowIni; py<rowFin /*_height*/; py++) {       // 200 linee 
      for(px=0; px<HORIZ_SIZE /*_width*/; px+=2) {         // 320 pixel (160byte) 
        ch=*p1++;
        writedata16x2(textColors[ch >> 4],textColors[ch & 0xf]);     // 
        }
#ifdef USA_SPI_HW
      ClrWdt();
#endif
      }
    END_WRITE();
  //	writecommand(CMD_NOP);
#endif
    
#if defined(__PIC32MM__)
		m2m_wifi_handle_events(NULL);
#endif

    
    LED3 = 0;

    }
  
	}

#endif


int main(void) {

  // disable JTAG port
//  DDPCONbits.JTAGEN = 0;
  
#if defined(__PIC32MM__)

  ANSELA=0x0000;
  ANSELB=0x0000;
  ANSELC=0x0000;


  SYSKEY = 0xAA996655;
  SYSKEY = 0x556699AA;

  OSCCONCLR = _OSCCON_FRCDIV_MASK | _OSCCON_NOSC_MASK;
  OSCCONSET = _OSCCON_OSWEN_MASK;
  while(OSCCONbits.OSWEN == 1)
      Nop();

  SPLLCON = 0x02050080;
  OSCCONSET = ( 0b001 << _OSCCON_NOSC_POSITION ) | _OSCCON_OSWEN_MASK;
  while(OSCCONbits.OSWEN == 1)
      Nop();

  OSCTUN = 0x0000001f;        // forzo al max :) verso i 25MHz
  CFGCONbits.BMXARB = 0b10;
  SYSKEY = 0;


    
// pps
  SYSKEY = 0x00000000;
  SYSKEY = 0xAA996655;    
  SYSKEY = 0x556699AA;
  RPCONbits.IOLOCK = 0;      // PPS Unlock
//  RPB9Rbits.RPB9R = 1;        // Assign RPB9 as U3TX
//  U4RXRbits.U4RXR = 2;      // Assign RPB8 as U4RX
  RPOR2bits.RP12R = 11;       // Assign RP12/RPB7 as OC4, pin 16
  RPOR4bits.RP17R = 8;       // Assign RP17/RPB15 as SDO2, pin 26
  RPINR11bits.SDI2R = 0b10000;       // Assign RP16/RPB14 as SDI2, pin 25
  RPOR3bits.RP15R = 9;       // Assign RP15/RB13 as SCK2, pin 24
  RPINR11bits.SCK2INR = 0b01111;       // SERVE??  Assign RP15/RPB13 as SCK2, pin 24


#ifdef DEBUG_TESTREFCLK
// test REFCLK
  RPB0Rbits.RPB0R = 15;        // RefClk3 su pin 16 (RB0, AREF)
	REFO3CONbits.RSLP=1;
	REFO3CONbits.ROSEL=1;
	REFO3CONbits.RODIV=1;        // ok 50MHz 27/7/20
	REFO3CONbits.OE=1;
	REFO3CONbits.ON=1;
	TRISBbits.TRISB0=1;
#endif
  RPCONbits.IOLOCK = 1;      // PPS Lock
  SYSKEY = 0x00000000;

    
/*    while(1) {
  TRISB=0;
    LATB^=0xffff;
    __delay_us(100);    //100uS su PIC32MM 14/4/21
    ClrWdt();
    }*/
    
    
  CNPUAbits.CNPUA0=1;   // pulsanti
  CNPUAbits.CNPUA1=1;   // 
  CNPUBbits.CNPUB3=1;   // pulsanti
  CNPUBbits.CNPUB8=1;   // WINC IRQ


#else
  CFGCONbits.IOLOCK = 0;      // PPS Unlock
#ifdef ST7735
  RPB15Rbits.RPB15R = 4;        // Assign RPB15 as U6TX, pin 30
  U6RXRbits.U6RXR = 2;      // Assign RPB14 as U6RX, pin 29 
  
#ifdef USA_SPI_HW
  RPG8Rbits.RPG8R = 6;        // Assign RPG8 as SDO2, pin 6
//  SDI2Rbits.SDI2R = 1;        // Assign RPG7 as SDI2, pin 5
#endif
  RPD5Rbits.RPD5R = 12;        // Assign RPD5 as OC1, pin 53; vaga uscita audio :)
#endif
#ifdef ILI9341
//  RPB9Rbits.RPB9R = 1;        // Assign RPB9 as U3TX, pin 22 IO1
//  U4RXRbits.U4RXR = 2;      // Assign RPB8 as U4RX, pin 21 IO0 NON VA come coppia sulla stessa UART... :( se serve...

  RPB1Rbits.RPB1R = 11;       // Assign RPB1 as OC4, pin 15
//  PPSOutput(4,RPC4,OC1);   //buzzer 4KHz , qua rimappabile 
#endif

  CFGCONbits.IOLOCK = 1;      // PPS Lock


#ifdef DEBUG_TESTREFCLK
// test REFCLK
  PPSOutput(4,RPC4,REFCLKO2);   // RefClk su pin 1 (RG15, buzzer)
	REFOCONbits.ROSSLP=1;
	REFOCONbits.ROSEL=1;
	REFOCONbits.RODIV=0;
	REFOCONbits.ROON=1;
	TRISFbits.TRISF3=1;
#endif

//	PPSLock;

   // Disable all Interrupts
  __builtin_disable_interrupts();
  
//  SPLLCONbits.PLLMULT=10;
  
  OSCTUN=0;
  OSCCONbits.FRCDIV=0;
  
  // Switch to FRCDIV, SYSCLK=8MHz
  SYSKEY=0xAA996655;
  SYSKEY=0x556699AA;
  OSCCONbits.NOSC=0x00; // FRC
  OSCCONbits.OSWEN=1;
  SYSKEY=0x33333333;
  while(OSCCONbits.OSWEN) {
    Nop();
    }
    // At this point, SYSCLK is ~8MHz derived directly from FRC
 //http://www.microchip.com/forums/m840347.aspx
  // Switch back to FRCPLL, SYSCLK=200MHz
  SYSKEY=0xAA996655;
  SYSKEY=0x556699AA;
  OSCCONbits.NOSC=0x01; // SPLL
  OSCCONbits.OSWEN=1;
  SYSKEY=0x33333333;
  while(OSCCONbits.OSWEN) {
    Nop();
    }
  // At this point, SYSCLK is ~200MHz derived from FRC+PLL
//***
  mySYSTEMConfigPerformance();
  //myINTEnableSystemMultiVectoredInt(();
#endif
  
    
#if defined(__PIC32MM__)
  TRISA=0b0000000000000011;   // (SD_CD); WincReset, sw2
  TRISB=0b0100000100001000;   // SDI; sw1; IRQ; PWR_CTL; WincWake
  TRISC=0b0000000000000000;   // CS2
 
  LATA=0b0000000000000000;      // reset winc;
  LATB=0b0000001000010000;      // pwr_ctl; CS1; wake
  LATC=0b0000001000000000;      // CS2
#else
#ifdef ST7735
	TRISB=0b0000000000110000;			// AN4,5 (rb4..5)
	TRISC=0b0000000000000000;
	TRISD=0b0000000000001100;			// 2 pulsanti
	TRISE=0b0000000000000000;			// 3 led
	TRISF=0b0000000000000000;			// 
	TRISG=0b0000000000000000;			// SPI2 (rg6..8)

  ANSELB=0;
  ANSELE=0;
  ANSELG=0;

  CNPUDbits.CNPUD2=1;   // switch/pulsanti
  CNPUDbits.CNPUD3=1;
  CNPUGbits.CNPUG6=1;   // I2C tanto per
  CNPUGbits.CNPUG8=1;  
#endif
#ifdef ILI9341

	TRISB=0b0000000000000001;			// pulsante; [ AN ?? ]
	TRISC=0b0000000000000000;
	TRISD=0b0000000000000000;			// 2led
	TRISE=0b0000000000000000;			// led
	TRISF=0b0000000000000001;			// pulsante
	TRISG=0b0000000000000000;			// SPI2 (rg6..8)

  ANSELB=0;
  ANSELE=0;
  ANSELG=0;

  CNPUFbits.CNPUF0=1;   // switch/pulsanti
  CNPUBbits.CNPUB0=1;
  CNPUDbits.CNPUD9=1;   // I2C tanto per
  CNPUDbits.CNPUD10=1;  
#endif
#endif

      
  
  Timer_Init();
  PWM_Init();
  UART_Init(/*230400L*/ 115200L);

  myINTEnableSystemMultiVectoredInt();
  ShortDelay(50000); 

  
//    	ColdReset=0;    Emulate(0);

#if defined(__PIC32MM__)
  __delay_ms(200);
#else
//#ifndef __DEBUG
#ifdef ST7735
  Adafruit_ST7735_1(0,0,0,0,-1);
  Adafruit_ST7735_initR(INITR_BLACKTAB);
#endif
#ifdef ILI9341
  Adafruit_ILI9341_8(8, 9, 10, 11, 12, 13, 14);
	begin(0);
  __delay_ms(200);
#endif
  
//  displayInit(NULL);
  
#ifdef m_LCDBLBit
  m_LCDBLBit=1;
#endif
  
//	begin();
	clearScreen();

// init done
	setTextWrap(1);
//	setTextColor2(WHITE, BLACK);

	drawBG();
  
  __delay_ms(200);
#endif

//#endif

  if(!SW2)      //
    PLAReg[0]=0;    // cartridge a 8000-bfff, kernel al suo posto
//    memcpy(&ram_seg[0x8000],&C64cartridge,0x4000);
  
	ColdReset=0;

  
#if defined(__PIC32MM__)
	tstrWifiInitParam param;
  void wifi_cb(uint8_t u8MsgType, void *pvMsg);
	nm_bsp_init();
   
      m_Led0Bit = 1;
	m2m_memset((uint8*)&param, 0, sizeof(param));
	param.pfAppWifiCb = wifi_cb;
	/*initialize the WINC Driver*/
		M2M_INFO("Init WiFi...\r\n");
	int ret = m2m_wifi_init(&param);
	if(M2M_SUCCESS != ret){
		M2M_ERR("Driver Init Failed <%d>\r\n",ret);
		while(1) {
      m_Led0Bit^=1;
      ClrWdt();
      }
		}
  
  // Initialize the socket layer.
  socketInit();

  m2m_wifi_connect("wlan_greggio",12,M2M_WIFI_SEC_WPA_PSK,
            "dariog20",M2M_WIFI_CH_ALL /*7*/);
    
	// Handle the app state machine plus the WINC event handler 
		while(m2m_wifi_handle_events(NULL) != M2M_SUCCESS) {
      m_Led0Bit^=1;
      ClrWdt();
			}
  
#endif
  

  Emulate(0);

  }


#define FOSC 8000000UL    //Oscillator frequency

void mySYSTEMConfigPerformance(void) {
  unsigned PLLIDIV;
  unsigned PLLMUL;
  unsigned PLLODIV;
  float CLK2USEC;
  unsigned int SYSCLK;
  char PLLODIVVAL[]={
    2,2,4,8,16,32,32,32
    };
  unsigned int cp0;

#if defined(__PIC32MM__)
#else
  PLLIDIV=SPLLCONbits.PLLIDIV+1;
  PLLMUL=SPLLCONbits.PLLMULT+1;
  PLLODIV=PLLODIVVAL[SPLLCONbits.PLLODIV];

  SYSCLK=(FOSC*PLLMUL)/(PLLIDIV*PLLODIV);
  CLK2USEC=SYSCLK/1000000.0f;

  SYSKEY = 0x0;
  SYSKEY = 0xAA996655;
  SYSKEY = 0x556699AA;

  if(SYSCLK<=60000000)
    PRECONbits.PFMWS=0;
  else if(SYSCLK<=120000000)
    PRECONbits.PFMWS=1;
  else if(SYSCLK<=210000000)		// per overclock :)
    PRECONbits.PFMWS=2;
  else if(SYSCLK<=252000000)
    PRECONbits.PFMWS=4;
  else
    PRECONbits.PFMWS=7;

  PRECONbits.PFMSECEN=0;    // non c'è nella versione "2019" ...
  PRECONbits.PREFEN=0x1;
#endif

  SYSKEY = 0x0;

	  // Set up caching
  cp0 = _mfc0(16, 0);
  cp0 &= ~0x07;
  cp0 |= 0b011; // K0 = Cacheable, non-coherent, write-back, write allocate
  _mtc0(16, 0, cp0);  

  }

void myINTEnableSystemMultiVectoredInt(void) {

  PRISS = 0x76543210;
  INTCONSET = _INTCON_MVEC_MASK /*0x1000*/;    //MVEC
  asm volatile ("ei");
  //__builtin_enable_interrupts();
  }

/* CP0.Count counts at half the CPU rate */
#define TICK_HZ (CPU_HZ / 2)

/* wait at least usec microseconds */
#if 0
void delay_usec(unsigned long usec) {
unsigned long start, stop;

  /* get start ticks */
  start = readCP0Count();

  /* calculate number of ticks for the given number of microseconds */
  stop = (usec * 1000000) / TICK_HZ;

  /* add start value */
  stop += start;

  /* wait till Count reaches the stop value */
  while (readCP0Count() < stop)
    ;
  }
#endif

void xdelay_us(uint32_t us) {
  
  if(us == 0) {
    return;
    }
  unsigned long start_count = ReadCoreTimer /*_CP0_GET_COUNT*/();
  unsigned long now_count;
  long cycles = ((GetSystemClock() + 1000000U) / 2000000U) * us;
  do {
    now_count = ReadCoreTimer /*_CP0_GET_COUNT*/();
    } while ((unsigned long)(now_count-start_count) < cycles);
  }

void __delay_us(unsigned int usec) {
  unsigned int tWait, tStart;

  tWait=(GetSystemClock()/2000000UL)*usec;
  tStart=_mfc0(9,0);
  while((_mfc0(9,0)-tStart)<tWait)
    ClrWdt();        // wait for the time to pass
  }

void __delay_ms(unsigned int ms) {
  
  for(;ms;ms--)
    __delay_us(1000);
  }

// ===========================================================================
// ShortDelay - Delays (blocking) for a very short period (in CoreTimer Ticks)
// ---------------------------------------------------------------------------
// The DelayCount is specified in Core-Timer Ticks.
// This function uses the CoreTimer to determine the length of the delay.
// The CoreTimer runs at half the system clock. 100MHz
// If CPU_CLOCK_HZ is defined as 80000000UL, 80MHz/2 = 40MHz or 1LSB = 25nS).
// Use US_TO_CT_TICKS to convert from uS to CoreTimer Ticks.
// ---------------------------------------------------------------------------

void ShortDelay(                       // Short Delay
  DWORD DelayCount)                   // Delay Time (CoreTimer Ticks)
{
  DWORD StartTime;                    // Start Time
  StartTime = ReadCoreTimer();         // Get CoreTimer value for StartTime
  while ( (DWORD )(ReadCoreTimer() - StartTime) < DelayCount ) 
    ClrWdt();
  }
 

void Timer_Init(void) {

  T2CON=0;
  T2CONbits.TCS = 0;                  // clock from peripheral clock
  T2CONbits.TCKPS = 7;                // 1:256 prescaler (pwm clock=390625Hz)
  T2CONbits.T32 = 0;                  // 16bit
//  PR2 = 2000;                         // rollover every n clocks; 2000 = 50KHz
  PR2 = 65535;                         // per ora faccio solo onda quadra, v. SID
  T2CONbits.TON = 1;                  // start timer per PWM
  
  // TIMER 3 INITIALIZATION (TIMER IS USED AS A TRIGGER SOURCE FOR ALL CHANNELS).
  T3CON=0;
  T3CONbits.TCS = 0;                  // clock from peripheral clock
#if defined(__PIC32MM__)
  T3CONbits.TCKPS = 4;                // 1:16 prescaler CAMBIARE? :) per precisione
#else
  T3CONbits.TCKPS = 4;                // 1:16 prescaler
#endif
  PR3 = FCY/51203L /*3906 @200MHz*/;   // rollover every n clocks; 
  T3CONbits.TON = 1;                  // start timer 

#if defined(__PIC32MM__)
  IPC4bits.T3IP=4;            // set IPL 4, sub-priority 2??
  IPC4bits.T3IS=0;
#else
  IPC3bits.T3IP=4;            // set IPL 4, sub-priority 2??
  IPC3bits.T3IS=0;
#endif
  IEC0bits.T3IE=1;             // enable Timer 3 interrupt se si vuole

	}

void PWM_Init(void) {

#ifdef __PIC32

#if defined(__PIC32MM__)
#else
  CFGCONbits.OCACLK=0;      // sceglie timer per PWM
#endif
  
#ifdef ST7735
  OC1CON = 0x0006;      // TimerX ossia Timer2; PWM mode no fault; Timer 16bit, TimerX
//  OC1R    = 500;		 // su PIC32 è read-only!
//  OC1RS   = 1000;   // 50%, relativo a PR2 del Timer2
  OC1R    = 32768;		 // su PIC32 è read-only!
  OC1RS   = 0;        // per ora faccio solo onda quadra, v. SID reg. 0-1
  OC1CONbits.ON = 1;   // on
#endif
#ifdef ILI9341
  OC4CON = 0x0006;      // TimerX ossia Timer2; PWM mode no fault; Timer 16bit, TimerX
//  OC1R    = 500;		 // su PIC32 è read-only!
//  OC1RS   = 1000;   // 50%, relativo a PR2 del Timer2
  OC4R    = 32768;		 // su PIC32 è read-only!
  OC4RS   = 0;        // per ora faccio solo onda quadra, v. SID reg. 0-1
  OC4CONbits.ON = 1;   // on
#endif

#if defined(__PIC32MM__)
  CCP4CON1=0;

  CCP4CON1bits.CLKSEL=0;     //Fosc/2
  CCP4CON1bits.CCSEL=0;
  CCP4CON1bits.T32=0;
  CCP4CON1bits.MOD=0b0101;    // PWM
  CCP4CON1bits.TMRPS=0b10;   //16
  CCP4CON1bits.SYNC=0;
  CCP4CON2=0;
  CCP4CON2bits.OCAEN=1;
  CCP4CON3=0;
//  CCP4TMR=375;
  CCP4PR=/*GetPeripheralClock() / */ 375;              // 24000000/16/375 = 4000
  CCP4RA=375/2;
//  CCP4RB=350;
//  CCP4CON1bits.ON=1;
#endif

#else
#endif

  }

void UART_Init(DWORD baudRate) {
  
#ifdef __PIC32
  
#if defined(__PIC32MM__)
  U1MODE=0b0000000000001000;    // BRGH=1
  U1STA= 0b0000010000000000;    // TXEN
  DWORD baudRateDivider = ((GetPeripheralClock()/(4*baudRate))-1);
  U1BRG=baudRateDivider;
  U1MODEbits.ON=1;
#else
  U6MODE=0b0000000000001000;    // BRGH=1
  U6STA= 0b0000010000000000;    // TXEN
  DWORD baudRateDivider = ((GetPeripheralClock()/(4*baudRate))-1);
  U6BRG=baudRateDivider;
  U6MODEbits.ON=1;
#endif
  
#if 0
  ANSELDCLR = 0xFFFF;
  CFGCONbits.IOLOCK = 0;      // PPS Unlock
  RPD11Rbits.RPD11R = 3;        // Assign RPD11 as U1TX
  U1RXRbits.U1RXR = 3;      // Assign RPD10 as U1RX
  CFGCONbits.IOLOCK = 1;      // PPS Lock

  // Baud related stuffs.
  U1MODEbits.BRGH = 1;      // Setup High baud rates.
  unsigned long int baudRateDivider = ((GetSystemClock()/(4*baudRate))-1);
  U1BRG = baudRateDivider;  // set BRG

  // UART Configuration
  U1MODEbits.ON = 1;    // UART1 module is Enabled
  U1STAbits.UTXEN = 1;  // TX is enabled
  U1STAbits.URXEN = 1;  // RX is enabled

  // UART Rx interrupt configuration.
  IFS1bits.U1RXIF = 0;  // Clear the interrupt flag
  IFS1bits.U1TXIF = 0;  // Clear the interrupt flag

  INTCONbits.MVEC = 1;  // Multi vector interrupts.

  IEC1bits.U1RXIE = 1;  // Rx interrupt enable
  IEC1bits.U1EIE = 1;
  IPC7bits.U1IP = 7;    // Rx Interrupt priority level
  IPC7bits.U1IS = 3;    // Rx Interrupt sub priority level
#endif
  
#else
  U1MODE=0b0000000010110000;    // TX,RX; 8bit async; BRGH=1
  U1STA= 0b0000000000000000;    // 
  DWORD baudRateDivider = ((GetPeripheralClock()/(2*baudRate))-1);    //v. anche BCLKSEL in U1MODEH
  U1BRG=baudRateDivider;
  TRISBbits.TRISB4=1;
  TRISBbits.TRISB5=0;
//  U1MODEbits.UARTEN=1;
  
//  IEC0bits.U1RXIE=1;
#endif
  
  }

char BusyUART1(void) {
  
#ifdef __PIC32
#if defined(__PIC32MM__)
  return(!U1STAbits.TRMT);
#else
  return(!U6STAbits.TRMT);
#endif
#else
  return(!U1STAbits.TRMT);
#endif
  }

char DataRdyUART1(void) {
  
#ifdef __PIC32
#if defined(__PIC32MM__)
  return(U1STAbits.URXDA);
#else
  return(U6STAbits.URXDA);
#endif
#else
#if defined(__dsPIC33CH__)
  return(!U1STAHbits.URXBE);
#endif
#endif
  }

void putsUART1(unsigned int *buffer) {
  char *temp_ptr = (char *)buffer;

    // transmit till NULL character is encountered 
#ifdef __PIC32
#if defined(__PIC32MM__)
  if(U1MODEbits.PDSEL == 3)        /* check if TX is 8bits or 9bits */
    {
        while(*buffer) {
            while(U1STAbits.UTXBF); /* wait if the buffer is full */
            U1TXREG = *buffer++;    /* transfer data word to TX reg */
        }
    }
  else {
        while(*temp_ptr) {
            while(U1STAbits.UTXBF);  /* wait if the buffer is full */
            U1TXREG = *temp_ptr++;   /* transfer data byte to TX reg */
        }
    }
#else
  if(U6MODEbits.PDSEL == 3)        /* check if TX is 8bits or 9bits */
    {
        while(*buffer) {
            while(U6STAbits.UTXBF); /* wait if the buffer is full */
            U6TXREG = *buffer++;    /* transfer data word to TX reg */
        }
    }
  else {
        while(*temp_ptr) {
            while(U6STAbits.UTXBF);  /* wait if the buffer is full */
            U6TXREG = *temp_ptr++;   /* transfer data byte to TX reg */
        }
    }
#endif
#else

    // transmit till NULL character is encountered 
#if defined(__dsPIC33CH__)
  if(U1MODEbits.MOD == 3)        /* VERIFICARE! check if TX is 8bits or 9bits */
    {
        while(*buffer) {
            while(U1STAHbits.UTXBF); /* wait if the buffer is full */
            U1TXREG = *buffer++;    /* transfer data word to TX reg */
        }
    }
  else {
        while(*temp_ptr) {
            while(U1STAHbits.UTXBF);  /* wait if the buffer is full */
            U1TXREG = *temp_ptr++;   /* transfer data byte to TX reg */
        }
    }
#endif
#endif
  }

unsigned int ReadUART1(void) {
  
#ifdef __PIC32
#if defined(__PIC32MM__)
#else
  if(U6MODEbits.PDSEL == 3)
    return (U6RXREG);
  else
    return (U6RXREG & 0xFF);
#endif
#else
#if defined(__dsPIC33CH__)
    if(U1MODEbits.MOD == 3)     // VERIFICARE
    return (U1RXREG);
  else
    return (U1RXREG & 0xFF);
#endif
#endif
  }

void WriteUART1(unsigned int data) {
  
#ifdef __PIC32
#if defined(__PIC32MM__)
#else
  if(U6MODEbits.PDSEL == 3)
    U6TXREG = data;
  else
    U6TXREG = data & 0xFF;
#endif
#else
#if defined(__dsPIC33CH__)
  if(U1MODEbits.MOD == 3)
    U1TXREG = data;
  else
    U1TXREG = data & 0xFF;
#endif
#endif
  }


#ifdef __PIC32
void __ISR(_UART1_RX_VECTOR) UART1_ISR(void) {

#if defined(__PIC32MM__)
  IFS1bits.U1RXIF = 0;  // Clear the interrupt flag!
#else
  LATDbits.LATD4 ^= 1;    // LED to indicate the ISR.
  char curChar = U1RXREG;
  IFS3bits.U1RXIF = 0;  // Clear the interrupt flag!
#endif
  }
#endif

#if defined(__PIC32MM__)
void __ISR ( _CHANGE_NOTICE_B_VECTOR, IPL5AUTO ) CNBInt(void) {
  
  extern tpfNmBspIsr gpfIsr;
	if(gpfIsr) {
		gpfIsr();
		}

  CNFB=0;
  IFS0bits.CNBIF=0;
  }
#endif



BYTE isCtrlAltCanc=0;
int emulateKBD(unsigned char ch) {
  int i;
// https://www.c64-wiki.com/wiki/Keyboard
	switch(ch) {
    case 0:
      for(i=0; i<8; i++)
        Keyboard[i]=0xff;
      // FORSE non dovremmo rilasciare i modifier, qua...
      isCtrlAltCanc=0;
      break;
		case ' ':
			Keyboard[7] &= 0xef;
			break;
		case 'A':
			Keyboard[1] &= 0xfb;
			break;
		case 'B':
			Keyboard[3] &= 0xef;
			break;
		case 'C':
			Keyboard[2] &= 0xef;
			break;
		case 'D':
			Keyboard[2] &= 0xfb;
			break;
		case 'E':
			Keyboard[1] &= 0xbf;
			break;
		case 'F':
			Keyboard[2] &= 0xdf;
			break;
		case 'G':
			Keyboard[3] &= 0xfb;
			break;
		case 'H':
			Keyboard[3] &= 0xdf;
			break;
		case 'I':
			Keyboard[4] &= 0xfd;
			break;
		case 'J':
			Keyboard[4] &= 0xfb;
			break;
		case 'K':
			Keyboard[4] &= 0xdf;
			break;
		case 'L':
			Keyboard[5] &= 0xfb;
			break;
		case 'M':
			Keyboard[4] &= 0xef;
			break;
		case 'N':
			Keyboard[4] &= 0x7f;
			break;
		case 'O':
			Keyboard[4] &= 0xbf;
			break;
		case 'P':
			Keyboard[5] &= 0xfd;
			break;
		case 'Q':
			Keyboard[7] &= 0xbf;
			break;
		case 'R':
			Keyboard[2] &= 0xfd;
			break;
		case 'S':
			Keyboard[1] &= 0xdf;
			break;
		case 'T':
			Keyboard[2] &= 0xbf;
			break;
		case 'U':
			Keyboard[3] &= 0xbf;
			break;
		case 'V':
			Keyboard[3] &= 0x7f;
			break;
		case 'W':
			Keyboard[1] &= 0xfd;
			break;
		case 'X':
			Keyboard[2] &= 0x7f;
			break;
		case 'Y':
			Keyboard[3] &= 0xfd;
			break;
		case 'Z':
			Keyboard[1] &= 0xef;
			break;
		case '0':
			Keyboard[4] &= 0xf7;
			break;
		case '!':
			Keyboard[6] &= 0xef;
		case '1':
			Keyboard[7] &= 0xfe;
			break;
		case '\"':
			Keyboard[6] &= 0xef;
		case '2':
			Keyboard[7] &= 0xf7;
			break;
		case '#':
			Keyboard[6] &= 0xef;
		case '3':
			Keyboard[1] &= 0xfe;
			break;
		case '$':
			Keyboard[6] &= 0xef;
		case '4':
			Keyboard[1] &= 0xf7;
			break;
		case '%':
			Keyboard[6] &= 0xef;
		case '5':
			Keyboard[2] &= 0xfe;
			break;
		case '&':
			Keyboard[6] &= 0xef;
		case '6':
			Keyboard[2] &= 0xf7;
			break;
		case '\'':
			Keyboard[6] &= 0xef;
		case '7':
			Keyboard[3] &= 0xfe;
			break;
		case '(':
			Keyboard[6] &= 0xef;
		case '8':
			Keyboard[3] &= 0xf7;
			break;
		case ')':
			Keyboard[6] &= 0xef;
		case '9':
			Keyboard[4] &= 0xfe;
			break;
		case '<':
			Keyboard[6] &= 0xef;
		case '.':
			Keyboard[5] &= 0xef;
			break;
		case '>':
			Keyboard[6] &= 0xef;
		case ',':		// ,
			Keyboard[5] &= 0x7f;		 // ,
			break;
		case '£':
			Keyboard[6] &= 0xfe;
			break;
		case '@':
			Keyboard[5] &= 0xbf;
			break;
		case '=':
			Keyboard[6] &= 0xdf;
			break;
		case '-':
			Keyboard[5] &= 0xf7;
			break;
		case '+':
			Keyboard[5] &= 0xfe;		 // 
			break;
		case ']':
			Keyboard[6] &= 0xef;
		case ';':
			Keyboard[6] &= 0xbf;		 // 
			break;
		case '[':
			Keyboard[6] &= 0xef;
		case ':':
			Keyboard[5] &= 0xdf;		 // 
			break;
		case '?':
			Keyboard[6] &= 0xef;
		case '/':
			Keyboard[6] &= 0x7f;
			break;
		case '*':
			Keyboard[6] &= 0xfd;		 // 
			break;
		case '\r':
			Keyboard[0] &= 0xfd;
			break;
		case 0x1:     // home
			Keyboard[6] &= 0xf7;
			break;
		case 0x2:     // CRSR right
			Keyboard[0] &= 0xfb;
			break;
		case 0x3:     // CRSR down
			Keyboard[0] &= 0x7f;
			break;
		case 0x4:     // DEL
			Keyboard[0] &= 0xfe;
			break;
		case 0x1f:    // LShift (dice che shift-lock è pure qua...)
			Keyboard[1] &= 0x7f;
			break;
		case 0x1e:    // RShift
			Keyboard[6] &= 0xef;
			break;
		case 0x1d:    // Ctrl
			Keyboard[7] &= 0xfb;
			break;
		case 0x8:    // backspace
			Keyboard[0] &= 0xfe;
			break;
		case 0x1b:     // ESC , run/stop...
			Keyboard[7] &= 0x7f;
			break;
		case 0x1c:     // commodore
			Keyboard[7] &= 0xdf;
			break;
		case 0xf1:    // F1
			Keyboard[0] &= 0xef;
			break;
		case 0xf3:    // F3
			Keyboard[0] &= 0xdf;
			break;
		case 0xf5:    // F5
			Keyboard[0] &= 0xbf;
			break;
		case 0xf7:    // F7
			Keyboard[0] &= 0xf7;
			break;
		}
  }

#ifdef __PIC32
void __ISR(_TIMER_3_VECTOR,ipl4AUTO) TMR_ISR(void) {
// https://www.microchip.com/forums/m842396.aspx per IRQ priority ecc
  static BYTE divider,dividerVICpatch;
  static BYTE keysFeedPhase=0;

//  LED2 ^= 1;      // check timing: 1600Hz, 9/11/19 (fuck berlin day))
  
  divider++;
  if(divider>=32) {   // 50 Hz per TOD
    divider=0;
    CIA1IRQ=1;
    }
//  CIA2IRQ=1; fare...
  dividerVICpatch++;
  if(dividerVICpatch>1) {    // troppo lento il display...
    dividerVICpatch=0;
    VICIRQ=1;       // refresh screen in 256/8=32 passate, 50 volte al secondo
    }
// v.  CIA1RegR[0xe];

  if(keysFeedPtr) {
    if(!*keysFeedPtr) {
      keysFeedPtr=NULL;
      goto fine_tasti;
      }
    dividerEmulKbd++;
#if defined(__PIC32MM__)
    if(dividerEmulKbd>=500) {   // ~.5Hz per emulazione tastiera! 
#else
    if(dividerEmulKbd>=300) {   // ~.2Hz per emulazione tastiera! (più veloce di tot non va...))
#endif
      dividerEmulKbd=0;
      if(!keysFeedPhase) {
        keysFeedPhase=1;
        emulateKBD(*keysFeedPtr);
        }
      else {
        dividerEmulKbd=0;
        keysFeedPtr++;
fine_tasti:
        keysFeedPhase=0;
        emulateKBD(NULL);
        }
      }
    }
    
  IFS0CLR = _IFS0_T3IF_MASK;
  }

#endif

#ifdef __PIC32

// ---------------------------------------------------------------------------------------
// declared static in case exception condition would prevent
// auto variable being created
static enum {
	EXCEP_IRQ = 0,			// interrupt
	EXCEP_AdEL = 4,			// address error exception (load or ifetch)
	EXCEP_AdES,				// address error exception (store)
	EXCEP_IBE,				// bus error (ifetch)
	EXCEP_DBE,				// bus error (load/store)
	EXCEP_Sys,				// syscall
	EXCEP_Bp,				// breakpoint
	EXCEP_RI,				// reserved instruction
	EXCEP_CpU,				// coprocessor unusable
	EXCEP_Overflow,			// arithmetic overflow
	EXCEP_Trap,				// trap (possible divide by zero)
	EXCEP_IS1 = 16,			// implementation specfic 1
	EXCEP_CEU,				// CorExtend Unuseable
	EXCEP_C2E				// coprocessor 2
  } _excep_code;

static unsigned int _epc_code;
static unsigned int _excep_addr;

void __attribute__((weak)) _general_exception_handler(uint32_t __attribute__((unused)) code, uint32_t __attribute__((unused)) address) {
  }

void __attribute__((nomips16,used)) _general_exception_handler_entry(void) {
  
	asm volatile("mfc0 %0,$13" : "=r" (_epc_code));
	asm volatile("mfc0 %0,$14" : "=r" (_excep_addr));

	_excep_code = (_epc_code & 0x0000007C) >> 2;

  _general_exception_handler(_excep_code, _excep_addr);

	while (1)	{
		// Examine _excep_code to identify the type of exception
		// Examine _excep_addr to find the address that caused the exception
    }
  }

#else

void _ISR __attribute__((__no_auto_psv__)) _AddressError(void) {

	Nop();
	Nop();
	}

void _ISR __attribute__((__no_auto_psv__)) _StackError(void) {

	Nop();
	Nop();
	}


#if defined(__dsPIC33CH__)
// prove...
void _ISR __attribute__ ((no_auto_psv)) _MSIAInterrupt(void){
  uint16_t temp1;
  // bah IFS8bits.MSIAIF=0; 
  //Read the data from slave
  temp1=MSI1MBX6D;
  // set the flag for the next round of data transfer
  IFS8bits.MSIAIF=0;
  //Flag=1;
  }

void _ISR __attribute__ ((no_auto_psv)) _MSIS1Interrupt(void){

  // set the flag for the next round of data transfer
  IFS8bits.MSIS1IF=0;
  }
#endif

#endif


#if defined(__PIC32MM__)
// https://datatracker.ietf.org/doc/html/rfc6143

void wifi_cb(uint8_t u8MsgType, void *pvMsg) {
  
  switch(u8MsgType) {
    case M2M_WIFI_REQ_DHCP_CONF:
      {
        m2m_wifi_get_connection_info();
        uint8 *pu8IPAddress = (uint8*)pvMsg;
//        wifi_connected = 1;
        /* Turn LED0 on to declare that IP address received. */
        m2m_periph_gpio_set_dir(M2M_PERIPH_GPIO16,1);
        m2m_periph_gpio_set_val(M2M_PERIPH_GPIO16,0);
//        port_pin_set_output_level(LED_0_PIN, false);
//        printf("m2m_wifi_state: M2M_WIFI_REQ_DHCP_CONF: IP is %u.%u.%u.%u\n",
//          pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
        /*TODO: add socket initialization here. */
        registerSocketCallback(clientSocketEventHandler, NULL);
        tcpStartServer(5900);
      }
        break;
      case M2M_WIFI_RESP_CON_STATE_CHANGED:   // https://www.avrfreaks.net/forum/winc1500-connecting-wifi-disconnects-immediately-error-code-1
      {
        tstrM2mWifiStateChanged *strState=(tstrM2mWifiStateChanged *)pvMsg;
        switch(strState->u8CurrState) {
          case M2M_WIFI_DISCONNECTED:
            m2m_periph_gpio_set_val(M2M_PERIPH_GPIO18,1);
            close(TCPlistenSocket);   // in effetti, chissà se serve...
            close(TCPacceptedSocket);
            TCPacceptedSocket=TCPlistenSocket=INVALID_SOCKET;
            bIsfinished = 0;
// fare...            reconnectTimeout=0;
            break;
          case M2M_WIFI_CONNECTED:
            // messo sopra...
            m2m_periph_gpio_set_val(M2M_PERIPH_GPIO18,0);
            m2m_wifi_req_curr_rssi();
            break;
          case M2M_WIFI_ROAMED:
            break;
          case M2M_WIFI_UNDEF:
            break;
          }

      }
        break;
    case M2M_WIFI_RESP_CONN_INFO:
      {
        tstrM2MConnInfo *pstrConnInfo = (tstrM2MConnInfo*)pvMsg;
        char buf[128];

        m2m_periph_gpio_set_dir(M2M_PERIPH_GPIO18,1);    // GP18 su pdf schema FROCI! opp. 6 merdeee get_gpio_idx
        m2m_periph_gpio_set_dir(M2M_PERIPH_GPIO15,1);
        m2m_periph_gpio_set_dir(M2M_PERIPH_GPIO16,1);
        m2m_periph_gpio_set_val(M2M_PERIPH_GPIO18,0);
        m2m_periph_gpio_set_val(M2M_PERIPH_GPIO15,1);
        m2m_periph_gpio_set_val(M2M_PERIPH_GPIO16,1);

      }
      
//        ftpServerStart(&FTPcontext);

        break;
    case M2M_WIFI_RESP_GET_SYS_TIME: 
      { 
      tstrSystemTime *mytime = (tstrSystemTime *)pvMsg; 
//      printf("wifi_cb:M2M_WIFI_RESP_GET_SYS_TIME \r\n"); 
//      printf("My hour %d \r\n",mytime->u8Hour); 
//      printf("My min %d \r\n",mytime->u8Minute); 
//      printf("My sec %d \r\n",mytime->u8Second); 
      uint32_t t;
//https://www.oryx-embedded.com/doc/date__time_8c_source.html
      
      //prendo hours, minutes and seconds
      t = (3600 * mytime->u8Hour) + (60 * mytime->u8Minute) + mytime->u8Second;
      // e li metto in TI/TI$ (che conta 60mi) :)
      t *= 60;
      ram_seg[0xa0]=LOBYTE(LOWORD(t)); //https://sta.c64.org/cbm64mem.html
      ram_seg[0xa1]=HIBYTE(LOWORD(t));
      ram_seg[0xa2]=LOBYTE(HIWORD(t));

      } 
      break; 
    case M2M_WIFI_RESP_CURRENT_RSSI:
      myRSSI=*(BYTE *)pvMsg;
      break; 
      default:
      {
      }
        break;
    }
  }


/* Socket event handler.
*/


/* This function needs to be called from main function. For the callbacks to be invoked correctly, the API
  m2m_wifi_handle_events should be called continuously from main. */
void tcpStartServer(uint16 u16ServerPort) {
  struct sockaddr_in strAddr;
  WORD tOut;
  
  // Register socket application callbacks.
//  registerSocketCallback(tcpServerSocketEventHandler, NULL);
  // Create the server listen socket.
  TCPlistenSocket = socket(AF_INET, SOCK_STREAM, 0);
  if(TCPlistenSocket >= 0) {
    strAddr.sin_family = AF_INET;
    strAddr.sin_port = _htons(u16ServerPort);
    strAddr.sin_addr.s_addr = nmi_inet_addr(INADDR_ANY);
    tOut=0;
    *(unsigned long*)internetBuffer=0;
    bind(TCPlistenSocket, (struct sockaddr*)&strAddr, sizeof(struct sockaddr_in));
    while((*(unsigned long*)internetBuffer) != 2 && tOut<2000) {    // DEVO aspettare qua, perché altrimenti si mischiano i socket # con FTP server!
      // no, non è vero...
      m2m_wifi_handle_events(NULL);
      tOut++;
      __delay_ms(1);
      }
    }
  }

sint16 sendEx(SOCKET sock, void *pvSendBuffer, uint16 u16SendLength) {
  uint16 n,n2;
//  WORD tOut;
  int i;
//  uint32_t u32EnableCallbacks=1;

// 	setsockopt(sock, SOL_SOCKET, SO_SET_UDP_SEND_CALLBACK, &u32EnableCallbacks, 4);
#if 0     // non va.. è molto lento (le callback arrivano male, pare un bug) e spesso con -1... 
  do {
    n=min(u16SendLength,SOCKET_BUFFER_MAX_LENGTH);
    if(send(sock, pvSendBuffer, n, 0) == SOCK_ERR_NO_ERROR) {
      tOut=0;
      internetBufferLen=0;
      while(!internetBufferLen && tOut<1000) {
        while(m2m_wifi_handle_events(NULL) != M2M_SUCCESS)
          ClrWdt();
        tOut++;
        __delay_ms(1);
        }
      n=internetBufferLen;
      }
    else
      break;
    pvSendBuffer+=n;
    u16SendLength-=n;
    } while(u16SendLength);
#endif

  n2=0;
  do {
    n=min(u16SendLength,SOCKET_BUFFER_MAX_LENGTH);
rifo:
    if((i=send(sock, pvSendBuffer, n, 0)) == SOCK_ERR_NO_ERROR) {
      // alle volte arriva SOCK_ERR_INVALID -9 in callback... checcazz'è?? parrebbe invalid ovvero magari socket chiuso... ma cmq non uso + la callback
      while(m2m_wifi_handle_events(NULL) != M2M_SUCCESS) {
        ClrWdt();
        }
      }
    else {
//      printf("sendex: %d\r\n",i);
      while(m2m_wifi_handle_events(NULL) != M2M_SUCCESS) {
        ClrWdt();
        }
      switch(i) {
        case SOCK_ERR_BUFFER_FULL:
          goto rifo;
          break;
//        case -3:
        // arriva -3=SOCK_ERR_MAX_TCP_SOCK che non ha senso... di tanto, v. di là
//          goto rifo;
//          break;
        case SOCK_ERR_INVALID_ARG:    // per socket chiuso... ma il close non arriva cmq se interrompo un download lungo (wireshark dice di sì)
          break;
        default:
          goto fine;
          break;
        }
      }
    pvSendBuffer+=n;
    n2+=n;
    u16SendLength-=n;
    } while(u16SendLength);
    
fine:

  return n2;
  }

/* Socket event handler */
void clientSocketEventHandler(SOCKET sock, uint8 u8Msg, void *pvMsg) {
  int i,ret;
  char *p;
  
//        printf("Event: socket=%d, msg=%u\r\n",sock, u8Msg);
        
  if(sock==TCPlistenSocket)
  switch(u8Msg) {
    case SOCKET_MSG_BIND:
      {
      tstrSocketBindMsg *pstrBind = (tstrSocketBindMsg*)pvMsg;
      *(unsigned long*)internetBuffer=1;
      if(pstrBind->status == 0) {
        listen(TCPlistenSocket, 1);
        }
      else {
        printf("Bind Failed\r\n");
        }
      }
      break;
    case SOCKET_MSG_LISTEN:
      {
      tstrSocketListenMsg *pstrListen = (tstrSocketListenMsg*)pvMsg;
      *(unsigned long*)internetBuffer=2;
      if(pstrListen->status != 0) {
        M2M_INFO("listen Failed\r\n");
        }
      }
      break;
    case SOCKET_MSG_ACCEPT:
      {
      // New Socket is accepted.
      tstrSocketAcceptMsg *pstrAccept = (tstrSocketAcceptMsg *)pvMsg;
      if(pstrAccept->sock >= 0) {
        // Get the accepted socket.
        if(TCPacceptedSocket != INVALID_SOCKET) {   // 
          M2M_INFO("Accept rejected\r\n");
          return;
          }

      #if defined(__PIC32MM__)
        myRowIni=0;
      #endif

        TCPacceptedSocket = pstrAccept->sock;
//      M2M_INFO("  accept www %d\r\n",TCPacceptedSocket);
// ??        recv(TCPacceptedSocket, rxBuffer, 12, 0);		// sembra NECESSARIO per l'effettiva accept! ????
        
        sendEx(TCPacceptedSocket,"RFB 003.008\n",12);
        VNCState=VNC_PROTOCOL;
        recv(TCPacceptedSocket,rxBuffer,12,0);		// 
        }
      else {
        M2M_INFO("Accept fail\r\n");
        }
      }
      break;
    case SOCKET_MSG_RECV:
      break;
    case SOCKET_MSG_SEND:
    {
      int sentBytes = *((sint16*)pvMsg);
      internetBufferLen=sentBytes;
    }
      break;
    case SOCKET_MSG_CLOSE:
      bIsfinished=TRUE;
      close(TCPlistenSocket);   // in effetti, chissà se serve...
      close(TCPacceptedSocket);
      TCPacceptedSocket=TCPlistenSocket=INVALID_SOCKET;
      #if defined(__PIC32MM__)
        myRowIni=0;
      #endif
      VNCState=VNC_IDLE;
      break;
      
    default:
      break;
    }
  else if(sock==TCPacceptedSocket)
  switch(u8Msg) {
    case SOCKET_MSG_RECV:
      {
      tstrSocketRecvMsg *pstrRecvMsg = (tstrSocketRecvMsg*)pvMsg;
      if((pstrRecvMsg->pu8Buffer) && (pstrRecvMsg->s16BufferSize > 0)) {
        // Process the received message
        // Perform data exchange
        // Fill in the acSendBuffer with some data here
        // Send some data.
        switch(VNCState) {
          case VNC_IDLE:
            break;
          case VNC_PROTOCOL:
            i=atoi(&pstrRecvMsg->pu8Buffer[4]);   //3
            if(i>3)
              goto close;
            i=atoi(&pstrRecvMsg->pu8Buffer[8]);   //8
            if(i>8)
              goto close;
            VNCState=VNC_SECURITY;
            VNCState=VNC_AUTH;
            if(i==8)
              sendEx(sock,"\x1\x1",2);     // tipi auth: solo no password VNC_SECURITY_TYPE_NONE
            else {
              sendEx(sock,"\x0\x0\x0\x1",4);   // per 3.3: https://web.archive.org/web/20140921005313/http://grox.net/doc/apps/vnc/rfbproto.pdf
              VNCState=VNC_AUTH_CONFIRMED;
              VNCState=VNC_CLIENT_INIT;
              }
            recv(sock,rxBuffer,1,0);
            break;
          case VNC_SECURITY:
            break;
          case VNC_AUTH:
            if(pstrRecvMsg->pu8Buffer[0] == 1) {
              }
            VNCState=VNC_AUTH_CONFIRMED;
            VNCState=VNC_CLIENT_INIT;
            sendEx(sock,"\x0\x0\x0\x0",4);     // handshake ok
            recv(sock,rxBuffer,1,0);
            break;
          case VNC_AUTH_CONFIRMED:
            break;
          case VNC_CLIENT_INIT:
          {
//            if(pstrRecvMsg->pu8Buffer[0] == 0)    // share desktop
            // BIG ENDIAN!
            struct RFB_SERVER_INIT_MSG initMsg;
#warning USARE buffer diversi rx e tx! per non sporcare
            initMsg.framebufferWidth=htons(HORIZ_SIZE);
            initMsg.framebufferHeight=htons(VERT_SIZE+40);
            initMsg.format.bigEndian=0;
            initMsg.format.bitsPerPixel=8;
            initMsg.format.depth=8;
            initMsg.format.trueColour=1;
            initMsg.format.redMax=htons(3);
            initMsg.format.greenMax=htons(3);
            initMsg.format.blueMax=htons(3);
            initMsg.format.redShift=4;
            initMsg.format.greenShift=2;
            initMsg.format.blueShift=0;
            initMsg.nameLength=htonl(sizeof(VNC_DESKTOPNAME)-1);
            memcpy(initMsg.name,VNC_DESKTOPNAME,sizeof(VNC_DESKTOPNAME)-1);
//            sendEx(sock,&initMsg "C64",27);     // e la stringa...?
            //sendEx(sock,"\x1\x40\x0\xf0" "\x20\x18\x0\x1" "\x0\xff\x0\xff\x0\xff\x10\x8\x0\x0\x0\x0" "\x0\x0\x0\x3" "C64",27);     // ServerInit
//            sendEx(sock,"\x1\x40\x0\xf0" "\x8\x8\x0\x1" "\x0\x7\x0\x7\x0\x3\x6\x3\x0\x0\x0\x0" "\x0\x0\x0\x3" "C64",27);     // ServerInit PROVARE 8bit 332
//            sendEx(sock,"\x1\x40\x0\xf0" "\x8\x8\x0\x1" "\x0\x3\x0\x3\x0\x3\x4\x2\x0\x0\x0\x0" "\x0\x0\x0\x3" "C64",27);     // ServerInit PROVARE 8bit 332
            sendEx(sock,&initMsg,sizeof(initMsg));
vncstate_running:            
            VNCState=VNC_RUNNING;
            recv(sock,rxBuffer,1,0);     // 
          }
            break;
          case VNC_RUNNING:
            switch(pstrRecvMsg->pu8Buffer[0]) {
              case VNC_CLIENT_MESSAGE_TYPE_SET_PIXEL_FORMAT:     // SetPixelFormat 
                // 3 byte padding, poi 16 PIXEL_FORMAT
                recv(sock,rxBuffer,sizeof(struct RFB_SET_PIXEL_FORMAT_MSG)-1,0);     // aspetto 
                VNCState=VNC_RUNNING_WAIT0;
                break;
              case VNC_CLIENT_MESSAGE_TYPE_SET_ENCODING:     // SetEncodings
                // 1 padding, 2 number-of-encodings; poi seguono n 4-byte encoding-type
                recv(sock,rxBuffer,3,0);     // aspetto #
                VNCState=VNC_RUNNING_WAIT2;
                break;
              case VNC_CLIENT_MESSAGE_TYPE_FRAMEBUF_UPDATE_REQ:     // FramebufferUpdateRequest
                // incremental, 2 xPos, 2 yPos, 2 width, 2 height BIG ENDIAN!
                recv(sock,rxBuffer,sizeof(struct RFB_FRAMEBUFFER_UPDATE_REQUEST_MSG)-1,0);     // aspetto 
                VNCState=VNC_RUNNING_WAIT3;
                break;
              case VNC_CLIENT_MESSAGE_TYPE_KEY_EVENT:     // KeyEvent
                // down-flag, 2 padding, 4 key
                recv(sock,rxBuffer,sizeof(struct RFB_KEY_EVENT_MSG)-1,0);     // aspetto 
                VNCState=VNC_RUNNING_WAIT4;
                break;
              case VNC_CLIENT_MESSAGE_TYPE_POINTER_EVENT:     // PointerEvent
                // button-mask, 2 xPos, 2 yPos 
                recv(sock,rxBuffer,sizeof(struct RFB_POINTER_EVENT_MSG)-1,0);     // aspetto 
                VNCState=VNC_RUNNING_WAIT5;
                break;
              case VNC_CLIENT_MESSAGE_TYPE_CLIENT_CUT_TEXT:     // ClientCutText
                // 3 padding, 4 length, text
                recv(sock,rxBuffer,sizeof(struct RFB_CLIENT_CUT_TEXT_MSG)-1,0);     // aspetto 
//            len=MAKELONG(MAKEWORD(pstrRecvMsg->pu8Buffer[6],pstrRecvMsg->pu8Buffer[5]),
//              MAKEWORD(pstrRecvMsg->pu8Buffer[4],pstrRecvMsg->pu8Buffer[3]));
                VNCState=VNC_RUNNING_WAIT6;
                break;
              }
            break;
          case VNC_RUNNING_WAIT0:
          {
            struct RFB_SET_PIXEL_FORMAT_MSG *pPixelFormatMsg = (struct RFB_SET_PIXEL_FORMAT_MSG *)(pstrRecvMsg->pu8Buffer-1);
            VNCFormat=pPixelFormatMsg->pixelFormat;
//            VNCFormat=pstrRecvMsg->pu8Buffer[3];      //bitsPerPixel
//            VNCFormat=32;      //bitsPerPixel PATCH**********
            goto vncstate_running;
          }
            break;
          case VNC_RUNNING_WAIT2:
          {
            struct RFB_SET_ENCODINGS_MSG *pSetEncodingsMsg = (struct RFB_SET_ENCODINGS_MSG *)(pstrRecvMsg->pu8Buffer-1);
                // in effetti sono MAKEWORD(pstrRecvMsg->pu8Buffer[2],pstrRecvMsg->pu8Buffer[1]) * 4 ...
            recv(sock,rxBuffer,4* htons(pSetEncodingsMsg->nEncodings) /*BIG_ENDIAN*/ 
//                 MAKEWORD(pstrRecvMsg->pu8Buffer[2],pstrRecvMsg->pu8Buffer[1])
              ,0);     // riaspetto 
            VNCState=VNC_RUNNING_WAIT22;
          }
            break;
          case VNC_RUNNING_WAIT22:
          {
            DWORD *pSetEncodingsMsg = (DWORD *)(pstrRecvMsg->pu8Buffer-4);
                // in effetti sono MAKEWORD(pstrRecvMsg->pu8Buffer[2],pstrRecvMsg->pu8Buffer[1]) * 4 ...
            VNCEncoding=htonl(pSetEncodingsMsg[0]);      /// usare..
//                    MAKELONG(MAKEWORD(pstrRecvMsg->pu8Buffer[3],pstrRecvMsg->pu8Buffer[2]),
  //            MAKEWORD(pstrRecvMsg->pu8Buffer[1],pstrRecvMsg->pu8Buffer[0]));
            // VNC client 3.3.8 supporta: -239 -223 16 1 5 2 0
            // TigerVNC ne ha alcune altre (18 in tutto)
            goto vncstate_running;
            }
            break;
          case VNC_RUNNING_WAIT3:
          {
            struct RFB_FRAMEBUFFER_UPDATE_REQUEST_MSG *pFrameBufferUpdateRequest = (struct RFB_FRAMEBUFFER_UPDATE_REQUEST_MSG *)(pstrRecvMsg->pu8Buffer-1);
						VNCArea.left=htons(pFrameBufferUpdateRequest->x);
						VNCArea.top=htons(pFrameBufferUpdateRequest->y);
						VNCArea.right=htons(pFrameBufferUpdateRequest->w);
						VNCArea.bottom=htons(pFrameBufferUpdateRequest->h);
//            VNCArea.left=MAKEWORD(pstrRecvMsg->pu8Buffer[2],pstrRecvMsg->pu8Buffer[1]);
//            VNCArea.top=MAKEWORD(pstrRecvMsg->pu8Buffer[4],pstrRecvMsg->pu8Buffer[3]);
//            VNCArea.right=MAKEWORD(pstrRecvMsg->pu8Buffer[6],pstrRecvMsg->pu8Buffer[5]);
//            VNCArea.bottom=MAKEWORD(pstrRecvMsg->pu8Buffer[8],pstrRecvMsg->pu8Buffer[7]);
            goto vncstate_running;
          }
            break;
          case VNC_RUNNING_WAIT4:
          {
            struct RFB_KEY_EVENT_MSG *pKeyEventMsg = (struct RFB_KEY_EVENT_MSG *)(pstrRecvMsg->pu8Buffer-1);
            if(pKeyEventMsg->down) {
              if(pKeyEventMsg->keys[2] == 0xff) {
                dividerEmulKbd=1;
                switch(pKeyEventMsg->keys[3]) {
                  case 0xb0:    // NUM0
                  case 0xb1:
                  case 0xb2:
                  case 0xb3:
                  case 0xb4:
                  case 0xb5:
                  case 0xb6:
                  case 0xb7:
                  case 0xb8:
                  case 0xb9:    // NUM9
                  case 0xaa:    // NUM *
                  case 0xab:    // NUM +
                  case 0xad:    // NUM -
                  case 0xae:    // NUM .
                  case 0xaf:    // NUM /
                    emulateKBD(pKeyEventMsg->keys[3] & 0x7f);
                    break;
                  case 0xc9:    // F12
                    DoNMI=1;    // Restore... 
                    break;
                  case 0xc8:    // F11
                    break;
                  case 0xbe:    // F1..F7
                  case 0xc0:
                  case 0xc2:
                  case 0xc4:
                    emulateKBD(0xf0-(pKeyEventMsg->keys[3]-0xbd));    // 
                    break;
                  case 0xbf:    // F2..F8
                  case 0xc1:
                  case 0xc3:
                  case 0xc5:
                    emulateKBD(0x1e); emulateKBD(0xf0-(pKeyEventMsg->keys[3]-0xbe));    // emulo pure shift per gli altri
                    break;
                  case 0x1b:    // ESC=run/stop
                    emulateKBD(0x1b);    // 
                    break;
// NON E' VERO!                  case 0xe7:    // L-Win = Commodore
                  case 0xeb:    // L-Win = Commodore
                  case 0xec:    // R-Win idem
                    emulateKBD(0x1c);    // 
                    break;
                  case 0xe9:    // ALT
                  case 0xea:    // 
                    isCtrlAltCanc|=2;
                    goto key_default;
                    break;
                  case 0x8:    // 
                  case 0x9:
                  case 0xd:
                  case 0x8d:   // NUM ENTER
                    emulateKBD(pKeyEventMsg->keys[3] & 0x7f);
                    break;
                  case 0x51:    // left
										emulateKBD(0x1e); emulateKBD(0x2);    // right+rshift
                    break;
                  case 0x52:    // up
                    emulateKBD(0x1e); emulateKBD(0x3);
                    break;
                  case 0x53:    // crsr right
                    emulateKBD(0x2);    // 
                    break;
                  case 0x54:    // crsr down
                    emulateKBD(0x3);    // 
                    break;
                  case 0x50:    // home
                  case 0x95:    // NUM home
                    emulateKBD(0x1);
                    break;
                  case 0xe1:    // L-shift
                    emulateKBD(0x1f);    // 
                    break;
                  case 0xe2:    // R-shift
                    emulateKBD(0x1e);    // 
                    break;
                  case 0xe3:    // L-CTRL
                  case 0xe4:    // R-CTRL
                    isCtrlAltCanc|=1;
                    emulateKBD(0x1d);    // 
                    break;
                  case 0xff:    // DEL
                  case 0x9f:    // NUM DEL
                    if(isCtrlAltCanc==3) {
                      DoReset=1;
                      goto key_default;
                      }
                    else
                      emulateKBD(0x4);    // 
                    break;
                  case 0x67:    // menu
                  {
										struct RFB_CLIENT_CUT_TEXT_MSG testclip;			// prova... :) copio in clipboard tutto lo schermo!
										testclip.type=3;
										testclip.length=htonl(1024);
                    if(!myRowIni) {  // per non disturbare flusso...
                      sendEx(sock,&testclip,sizeof(testclip));
//											if(ram_seg[i]<32)
//												ram_seg[i]+=64; // bisognerebbe convertire... ascii ecc !
                      sendEx(sock,&ram_seg[1024],1024);  
                      }
                  }
// prosegue e pulisce                    break;
                  default:
key_default:
                    dividerEmulKbd=0;
                    break;
                  }
                }
              else {
                emulateKBD(toupper(pKeyEventMsg->keys[3]));    // solo maiuscole...
                dividerEmulKbd=1;
                }
              }
            else {
              emulateKBD(NULL);
              dividerEmulKbd=0;
              }
            goto vncstate_running;
          }
            break;
          case VNC_RUNNING_WAIT5:
          {
            struct RFB_POINTER_EVENT_MSG *pPointerEventMsg = (struct RFB_POINTER_EVENT_MSG *)(pstrRecvMsg->pu8Buffer-1);
            goto vncstate_running;
          }
            break;
          case VNC_RUNNING_WAIT6:
          {
            struct RFB_CLIENT_CUT_TEXT_MSG *ClientCutTextMsg = (struct RFB_CLIENT_CUT_TEXT_MSG *)(pstrRecvMsg->pu8Buffer-1);
//            len=MAKELONG(MAKEWORD(pstrRecvMsg->pu8Buffer[6],pstrRecvMsg->pu8Buffer[5]),
//              MAKEWORD(pstrRecvMsg->pu8Buffer[4],pstrRecvMsg->pu8Buffer[3]));
            recv(sock,rxBuffer,htonl(ClientCutTextMsg->length)
//              MAKELONG(MAKEWORD(pstrRecvMsg->pu8Buffer[6],pstrRecvMsg->pu8Buffer[5]),
//              MAKEWORD(pstrRecvMsg->pu8Buffer[4],pstrRecvMsg->pu8Buffer[3]))
                ,0);     // riaspetto 
            VNCState=VNC_RUNNING_WAIT66;
          }
            break;
          case VNC_RUNNING_WAIT66:
          {
            struct RFB_CLIENT_CUT_TEXT_MSG *ClientCutTextMsg = (struct RFB_CLIENT_CUT_TEXT_MSG *)(pstrRecvMsg->pu8Buffer-8);
            // dati per clipboard...
//            for(i=0; i<pstrRecvMsg->s16BufferSize; i++)
						emulateKBD(pstrRecvMsg->pu8Buffer[0]);		// bah ok, FARE RILASCIO! farli tutti :)
						// keysFeed[keysFeedPtr]
            goto vncstate_running;
          }
            break;
          }
        
        }
      }
      break;
    case SOCKET_MSG_SEND:
    {
      int sentBytes = *((sint16*)pvMsg);
      internetBufferLen=sentBytes;
//        printf("  Send: %u\r\n",sentBytes);
    }
      break;
    case SOCKET_MSG_CLOSE:
close:
      bIsfinished=TRUE;
      #if defined(__PIC32MM__)
        myRowIni=0;
      #endif
      VNCState=VNC_IDLE;
      VNCArea.left=VNCArea.top=VNCArea.right=VNCArea.bottom=0;
      close(TCPacceptedSocket);
      TCPacceptedSocket=INVALID_SOCKET;
      break;
    default:
//      bIsfinished=TRUE;
      break;
    }
      
  }


void advertise(SOCKET sock) {
  // per rendere pubblico l'IP... per una lista
  }

#endif 

