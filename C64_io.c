#include <ctype.h>
#include <stdlib.h>
#ifdef __PIC32
#include <conio.h>
#endif
#include <xc.h>
#include "c64_pic.h"

#ifdef ST7735
#include "Adafruit_ST77xx.h"
#include "Adafruit_ST7735.h"
#include "adafruit_gfx.h"
#endif
#ifdef ILI9341
#include "Adafruit_ILI9341.h"
#include "adafruit_gfx.h"
#endif



extern BOOL debug;
BYTE ram_seg[MAX_RAM];

BYTE *stack_seg=ram_seg+0x100;
#warning anche gli accessi allo stack forse dovrebbero passare da Get/PutValue...? 2022
extern BYTE DoIRQ,DoNMI,DoReset,DoHalt;
extern BYTE ColdReset;


#ifdef COMMODORE64
//http://sta.c64.org/cbm64mem.html
//https://www.pagetable.com/c64disasm/#FDDD

BYTE CPUIOReg[2] = { 0x2F, 0x37 };    // bit 0..2 memory bank switching; 3..5 datassette
BYTE PLAReg[1] = { 0x03 };    // emulo EXROM(1) e GAME(0) della cartridge
BYTE VICReg[64] = {
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x1B,0x00,0x00,0x00,0x00,0xC8,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
  };
SWORD VICRaster=MIN_RASTER;
BYTE SIDReg[32] = {
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
  };
BYTE CIA1RegR[16]  = {
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
  },
  CIA1RegW[16] = {
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
  };
BYTE CIA2RegR[16] = {
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
  },
  CIA2RegW[16] = {
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
  };
#if defined(__PIC32MM__)
BYTE ColorRAM[1024];
#else
BYTE ColorRAM[1024],VideoHIRAM[((HORIZ_SIZE/2)+(HORIZ_OFFSCREEN*2))*(MAX_RASTER-MIN_RASTER+1) /* /8 FARE */];
#endif
BYTE Keyboard[8]={255,255,255,255,255,255,255,255};
extern volatile BYTE CIA1IRQ,CIA2IRQ,VICIRQ;
#endif

#ifdef COMMODOREVIC20
BYTE VICReg[16] = {
  0x00,0x00,0x80,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
  };
SWORD VICRaster=MIN_RASTER;
BYTE SIDReg[32] = {
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
  };
BYTE VIA1RegR[16]  = {
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
  },
  VIA1RegW[16] = {
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
  };
BYTE VIA2RegR[16] = {
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
  },
  VIA2RegW[16] = {
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
  };
#if defined(__PIC32MM__)
BYTE ColorRAM[1024];
#else
BYTE ColorRAM[1024];
#endif
BYTE Keyboard[8]={255,255,255,255,255,255,255,255};
extern volatile BYTE VIA1IRQ,VIA2IRQ;
#endif

#ifdef AMICO2000
BYTE CIA8255RegR[4]  = {
  0x00,0x00,0x00,0x00
  },
  CIA8255RegW[4] = {
  0x00,0x00,0x00,0x00
  };
BYTE DisplayRAM[7];
BYTE Keyboard[4]={255,255,255,255};     // solo 3 usati, v.schema
#endif

#ifdef APPLE2
//https://www.kreativekorp.com/miscpages/a2info/memorymap.shtml
//http://apple2.guidero.us/doku.php/mg_notes/general/mem_map
//https://mirrors.apple2.org.za/apple.cabi.net/Languages.Programming/MemoryMap.IIe.64K.128K.txt
BYTE Keyboard[1]={0};
BYTE memSelector[8]={0,0,0,0,0,0,0,0};
BYTE LoHiRes=0;
#endif


BYTE Pipe1;
union __attribute__((__packed__)) PIPE {
	SWORD word;
	BYTE b[2];
	struct __attribute__((__packed__)) {
		BYTE byte1;
		BYTE byte2;
		} bytes;
	} Pipe2;
union __attribute__((__packed__)) OPERAND {
  BYTE *reg8;
  WORD *reg16;
  SWORD mem;
  };
union __attribute__((__packed__)) RESULT {
  struct __attribute__((__packed__)) {
    BYTE l;
    BYTE h;
    } b;
  WORD w;
  };

extern volatile BYTE *keysFeedPtr;
extern const char keysFeed[];

#ifdef CPU_KIMKLONE   // https://laughtonelectronics.com/Arcana/KimKlone/Kimklone_intro.html
extern BYTE _K[4];
extern BYTE *theK;
#endif


BYTE GetValue(SWORD t) {
	register SWORD i,j,k;
//	char myBuf[128];

#ifdef COMMODORE64
  // https://www.c64-wiki.com/wiki/Bank_Switching
  // a 0x8000 ci sono le catridge ... (e v. bit 0-1 di PLAReg[1]))
	if(t >= 0x8000) {
    if(t <= 0x9fff) {
      if(!(PLAReg[0] & 3)) {    // non è perfetto ma per ora ok
  			if((CPUIOReg[1] & 3) == 3) {
#ifdef USA_CARTRIDGE
          t &= 0x3fff;
  				i=C64cartridge[t];
#endif
          }
        else {
#if !defined(__PIC32MM__)
  				i=ram_seg[t];
#endif
          }
        }
      else {
#if !defined(__PIC32MM__)
				i=ram_seg[t];
#endif
        }
      }
    else if(t <= 0xbfff) {
      if(!(PLAReg[0] & 3)) {    // non è perfetto ma per ora ok
  			if(CPUIOReg[1] & 2) {
#ifdef USA_CARTRIDGE
          t &= 0x3fff;
  				i=C64cartridge[t];
#endif
          }
        else {
#if !defined(__PIC32MM__)
  				i=ram_seg[t];
#endif
          }
        }
			else if(CPUIOReg[1] & 1) {
        t &= 0x1fff;
				i=C64basic[t];
				}
			else {
#if !defined(__PIC32MM__)
				i=ram_seg[t];
#endif
				}
			}
		else if(t >= 0xe000) {
			if(CPUIOReg[1] & 2) {
        t &= 0x1fff;
				i=C64kern[t];
				}
			else {
#if !defined(__PIC32MM__)
				i=ram_seg[t];
#endif
				}
			}
		else if(t >= 0xc000 && t < 0xd000) {
#if defined(__PIC32MM__)
//			i=ram_seg[t-0xc000 + 0x3000];
#else
			i=ram_seg[t];
#endif
			}
		else {
			if(!(CPUIOReg[1] & 3)) {
#if !defined(__PIC32MM__)
				i=ram_seg[t];
#endif
        }
			else if(!(CPUIOReg[1] & 4)) {
				i=C64char[t-0xd000];
        }
      else {
				if((t >= 0xd000) && (t <= 0xd3ff)) {
					t &= 0x3f;
					switch(t) {
						case 0x11:                                 //MSB raster
						case 0x12:                                 //raster
							VICReg[0x12]=VICRaster /*& 0xfc*/;
							VICReg[0x11] &= 0x7f;
							VICReg[0x11] |= (VICRaster & 0x100) >> 1;
							break;
						case 0x13:                                 //light pen x
							break;
						case 0x14:                                 //light pen y
							break;
						}
					i=VICReg[t];
					}
				else if((t >= 0xd400) && (t <= 0xd7ff)) {
					t &= 0x1f;
          switch(t) {
            case 0x19:
#if defined(__PIC32MM__)
    					SIDReg[t] = ADC1BUF4;     // PADDLE, tanto per...
#else
    					SIDReg[t] = ADCDATA4;     // PADDLE, tanto per...
#endif
              break;
            case 0x1a:
#if defined(__PIC32MM__)
    					SIDReg[t] = ADC1BUF0;
#else
    					SIDReg[t] = ADCDATA0;
#endif
              break;
            }
					i=SIDReg[t];
					}
				else if((t >= 0xd800) && (t <= 0xdbff)) {
          t &= 0x3ff;
					i=ColorRAM[t];
					}
				else if((t >= 0xdc00) && (t <= 0xdcff)) {
	//    printf("Leggo a %04x: %02x\n",t,CIA1Reg[t & 0xf]);
					t &= 0xf;
					switch(t) {
/*Write to Port A($DC00)row
Read from Port B($DC01)column

      7   6   5   4   3   2   1   0
    --------------------------------
  7| STP  /   ,   N   V   X  LSH CDN    STP=RUN/STOP, LSH=Left SHIFT, CDN=CRSR-Down
   |
  6|  Q  UPA  @   O   U   T   E  F5     UPA=Up Arrow
   |
  5| CBM  =   :   K   H   F   S  F3     CBM=Commodore logo
   |
  4| SPC RSH  .   M   B   C   Z  F1     RSH=Right SHIFT
   |
  3|  2  HOM  -   0   8   6   4  F7     HOM=CLR/HOME
   |
  2| CTL  ;   L   J   G   D   A  CRT    CTL=CTRL, CRT=CRSR-Right
   |
  1| LFA  *   P   I   Y   R   W  RET    LFA=Left Arrow, RET=RETURN
   |
  0|  1  BP   +   9   7   5   3  DEL    BP=British Pound/Lira 
*/  				case 1:
							i=0xff;
							for(j=0,k=1; j<8; j++,k <<= 1) {
								if(!(CIA1RegR[0] & k)) {
									i &= Keyboard[j];
									}
								}
							break;
						case 0xd:
              CIA1RegR[0xd] |= U1STA & 1 /*URXDA*/ ? 0x10 : 0;			// FLAG, ser. in
            // FINIRE se si vuole!
							i=CIA1RegR[0xd];
              CIA1RegR[0xd] = 0;
							break;
						default:
							i=CIA1RegR[t];
							break;
						}
					}
				else if((t >= 0xdd00) && (t <= 0xddff)) {
					t &= 0xf;
					switch(t) {
            case 0:
#if defined(__PIC32MM__)
              CIA2RegR[0] = (CIA2RegR[0] & 0x3f) | (0 & 0x8000 ? 0x40 : 0) | (0 & 0x8000 ? 0x80 : 0);
#else
              CIA2RegR[0] = (CIA2RegR[0] & 0x3f) | (PMSTAT & 0x8000 ? 0x40 : 0) | (PMSTAT & 0x8000 ? 0x80 : 0);
#endif
            // FINIRE se si vuole!
    					i=CIA2RegR[0];
              break;
						case 1:
              CIA2RegR[1] = PORTB & 0xff;
    					i=CIA2RegR[1];
							break;
						case 0xd:
							i=CIA2RegR[0xd];
              CIA2RegR[0xd] = 0;
							break;
						default:
    					i=CIA2RegR[t];
							break;
						}
					}
				}
			}
		}
//	else if(t == 0x2000) {
//		i=0;
//		}
	else if(t <= 1) {
#if defined(__PIC32MM__)
		CPUIOReg[1] |= 0 & 0x8000 ? 0x10 : 0;
#else
		CPUIOReg[1] |= PMSTAT & 0x8000 ? 0x10 : 0;
#endif
		i=CPUIOReg[t];
		}
	else {
#if defined(__PIC32MM__)
    if(t < MAX_RAM) 
#endif
      i=ram_seg[t];
		}
#endif
  
#ifdef COMMODOREVIC20
	if(t >= 0x8000) {
    if(t <= 0x8fff) {
      t &= 0xfff;
			i=V20char[t];
      }
    else if(t <= 0x9fff) {
      if((t >= 0x9000) && (t <= 0x900f)) {
        t &= 0xf;
        switch(t) {
          case 0x3:                                 //MSB raster
          case 0x4:                                 //raster
            VICReg[0x4]=VICRaster >> 1;
            VICReg[0x3] &= 0x7f;
            VICReg[0x3] |= (VICRaster & 0x1) << 7;
            break;
          case 0x6:                                 //light pen x
            // da qualche parte dice che b0=1 fisso e il valroe è shiftato sx di 1..
            break;
          case 0x7:                                 //light pen y
            break;
          case 0x8:
#if defined(__PIC32MM__)
            VICReg[t] = ADC1BUF4;     // PADDLE, tanto per...
#else
            VICReg[t] = ADCDATA4;     // PADDLE, tanto per...
#endif
            break;
          case 0x9:
#if defined(__PIC32MM__)
            VICReg[t] = ADC1BUF0;
#else
            VICReg[t] = ADCDATA0;
#endif
            break;
          case 0xa:   // audio-alto
            break;
          case 0xb:   // audio-tenor
            break;
          case 0xc:   // audio-soprano
            break;
          case 0xd:   // audio-noise
            break;
  				case 0xe:   // audio-volume, multicolor
    				break;
          }
        i=VICReg[t];
        }
      else if((t >= 0x9110) && (t <= 0x911f)) {
//http://www.zimmers.net/anonftp/pub/cbm/maps/Vic20.MemoryMap.txt
        t &= 0xf;
        switch(t) {
          case 0:   // ORB
#if defined(__PIC32MM__)
            VIA1RegR[0] = (VIA1RegR[0] & 0x3f) | (0 & 0x8000 ? 0x40 : 0) | (0 & 0x8000 ? 0x80 : 0);
#else
            VIA1RegR[0] = (VIA1RegR[0] & 0x3f) | (PMSTAT & 0x8000 ? 0x40 : 0) | (PMSTAT & 0x8000 ? 0x80 : 0);
#endif
          // FINIRE se si vuole!
            VIA1RegR[0xd] &= ~0x10;
            if(1)   // v. doc...
              VIA1RegR[0xd] &= ~0x8;
            i=VIA1RegR[0];
            break;
          case 1:   // ORA
            VIA1RegR[1] = PORTB & 0xff;
            VIA1RegR[0xd] &= ~0x2;
            if(1)   // v. doc...
              VIA1RegR[0xd] &= ~0x1;
            i=VIA1RegR[1];
            break;
          case 0x04:  // T1 low
            VIA1RegR[0xd] &= ~0x40;
            i=VIA1RegR[4];
            break;
          case 0x08:  // T2 low
            VIA1RegR[0xd] &= ~0x20;
            i=VIA1RegR[8];
            break;
          case 0x0a:  // SR
            VIA1RegR[0xd] &= ~0x4;
            i=VIA1RegR[10];
            break;
          case 0xd:
            i=VIA1RegR[0xd];
//            VIA1RegR[0xd] = 0;
            break;
          case 0xe:
            i=VIA1RegR[0xe] | 0x80;
            break;
          default:
            i=VIA1RegR[t];
            break;
          }
        }
      else if((t >= 0x9120) && (t <= 0x912f)) {
        t &= 0xf;
        switch(t) {
          case 0:   // ORB
            VIA2RegR[0xd] &= ~0x10;
            if(1)   // v. doc...
              VIA2RegR[0xd] &= ~0x8;
            i=VIA2RegR[0];
            break;
/*Write to Port B($9120)column
Read from Port A($9121)row

     7   6   5   4   3   2   1   0
    --------------------------------
  7| F7  F5  F3  F1  CDN CRT RET DEL    CRT=Cursor-Right, CDN=Cursor-Down
   |
  6| HOM UA  =   RSH /   ;   *   BP     BP=British Pound, RSH=Should be Right-SHIFT,
   |                                    UA=Up Arrow
  5| -   @   :   .   ,   L   P   +
   |
  4| 0   O   K   M   N   J   I   9
   |
  3| 8   U   H   B   V   G   Y   7
   |
  2| 6   T   F   C   X   D   R   5
   |
  1| 4   E   S   Z   LSH A   W   3      LSH=Should be Left-SHIFT
   |
  0| 2   Q   CBM SPC STP CTL LA  1      LA=Left Arrow, CTL=Should be CTRL, STP=RUN/STOP
   |                                    CBM=Commodore key 
*/        case 0x1:   // ORA
            i=0xff;
            for(j=0,k=1; j<8; j++,k <<= 1) {
              if(!(VIA2RegR[0] & k)) {
                i &= Keyboard[j];
                }
              }
            VIA2RegR[0xd] &= ~0x2;
            if(1)   // v. doc...
              VIA2RegR[0xd] &= ~0x1;
            break;
          case 0x04:  // T1 low
            VIA2RegR[0xd] &= ~0x40;
            i=VIA2RegR[4];
            break;
          case 0x08:  // T2 low
            VIA2RegR[0xd] &= ~0x20;
            i=VIA2RegR[8];
            break;
          case 0x0a:  // SR
            VIA2RegR[0xd] &= ~0x4;
            i=VIA2RegR[10];
            break;
          case 0xd:
            VIA2RegR[0xd] |= U1STA & 1 /*URXDA*/ ? 0x10 : 0;			// FLAG, ser. in
          // FINIRE se si vuole!
            i=VIA2RegR[0xd];
//            VIA2RegR[0xd] = 0;
            break;
          case 0xe:
            i=VIA2RegR[0xe] | 0x80;
            break;
          default:
            i=VIA2RegR[t];
            break;
          }
        }
      else if((t >= (0x9400 | ((WORD)VICReg[2] & 0x80)<<2)) && 
        (t <= (0x95ff | ((WORD)VICReg[2] & 0x80)<<2))) {
        t &= 0x1ff;
        i=ColorRAM[t];
        }
      }
    else if(t <= 0xbfff) {
      t &= 0x1fff;
#ifdef USA_CARTRIDGE
      if(SW2)      //
        i=V20cartridge[t];
#endif
			}
		else if(t <= 0xdfff) {
      t &= 0x1fff;
			i=V20basic[t];
			}
		else {
      t &= 0x1fff;
  		i=V20kern[t];
			}
		}
	else {
    if(t < MAX_RAM) 
      i=ram_seg[t];
		}
#endif

#ifdef AMICO2000
	if(t >= 0xfb00) {
		if(t <= 0xfcff) {
			i=Amico2000_rom2[t-0xfb00];
			}
		else if(t >= 0xfe00) {
			i=Amico2000_rom1[t-0xfe00];
			}
  	else if(t >= 0xfd00 && t <= 0xfd03) {
			t &= 0x3;
	//    printf("Leggo a %04x: %02x\n",t,CIA1Reg[t & 0xf]);
      switch(t) {
        case 0:
          if(CIA8255RegW[3] & 0b00010000) {   // se in lettura
            j=(CIA8255RegW[1] >> 1) & 3;    // posizione mux, 0..2 (però CIA è X2, B1..B4)
            i = Keyboard[j];
//            }
//          else {
//            i=CIA8255RegR[t];     // DisplayRAM...
//            }
            }
          else {
            j=CIA8255RegW[0];
            }
          // pin 18 (PB0, output) e 37 (PA7, direi input) del CIA sono usati per le cassette (v.schema))
          if(!(CIA8255RegW[1] & 0b00010000)) {   // inoltre, se PB7=0 4 led vengono pilotati da PA0..3 (invertiti))

            }
          break;
        case 1:
          i=CIA8255RegR[1];
          break;
        case 2:       // user port
          if(CIA8255RegW[3] & 0b10000000) {   // se non modalità single bit IN LETTURA INFLUISCE O NO?? diciamo di sì :)
            if(CIA8255RegW[3] & 0b00000001)    // se è in input, nibble basso
              i=PORTB & 0x0f;
            else
              i=CIA8255RegW[2] & 0x0f;
            if(CIA8255RegW[3] & 0b00001000)    // idem, nibble alto
              i |= PORTB & 0xf0;
            else
              i |= CIA8255RegW[2] & 0xf0;
            }
          else {
            i = LATB & 0xff;
            
            LED3 = LATBbits.LATB0;    // tanto per :)
          
            }
          CIA8255RegR[2]=i;
          break;
        default:      // control byte
          i=CIA8255RegR[t];
          break;
        }
			}
		}
	else {
    if(t < MAX_RAM)
      i=ram_seg[t];
    // else random :)
		}
#endif
  
#ifdef APPLE2
	if(t >= 0xf800) {
    t-=0xf800;
    if(memSelector[0]) {
      }
    i=AppleROM_F8[t];
		}
	else if(t >= 0xf000) {
    t-=0xf000;
    if(memSelector[1]) {
      }
    i=AppleROM_F0[t];
		}
	else if(t >= 0xe800) {
    t-=0xe800;
    if(memSelector[2]) {
      }
    i=AppleROM_E8[t];
		}
	else if(t >= 0xe000) {
    t-=0xe000;
    if(memSelector[3]) {
      }
    i=AppleROM_E0[t];
		}
	else if(t >= 0xd000) {
    t-=0xd000;
    if(memSelector[5]) {
      }
    i=AppleROM_D0[t];
		}
	else if(t >= 0xc000 && t<0xd000) {
    switch(t & 0xfff) {
      case 0x000:
        i=Keyboard[0];
        break;
      case 0x010:
// strobe...    
				Keyboard[0] &= 0x7f;
        break;
      case 0x020:
 //TAPEOUT    =($C02X); Cassette Data Out: Digital to Analog Audio 
        break;
      case 0x030:
// speaker;
        break;
      case 0x040:   // tutte e 16, fare
      case 0x04f:
//          STROBE     =($C04X); Outputs Strobe Pulse to Game I/O Connector;
        break;
      case 0x050:
        LoHiRes &= ~1;  //  TXTCLR     Sets Graphics Mode without Clearing Screen;
                        // Resets from Text Mode.
        break;
      case 0x051:
        LoHiRes |= 1;   //  TXTSET     Sets Text Mode without Resetting Scrolling Window;
                        //             Resets from Graphics Mode.
        break;
      case 0x052:
        LoHiRes &= ~2;  //  MIXCLR     Sets Full-Screen Graphics Mode;
                        // Resets from Mixed Graphics Mode
                        // (with 4 lines of text at the bottom of the screen).
        break;
      case 0x053:
        LoHiRes |= 2;   // MIXSET     Sets Mixed Text & Graphics Mode
                        // (with 4 lines of text at the bottom of the screen).
        break;
      case 0x054:
        LoHiRes &= ~4;  // LOWSCR     Displays Page 1 without Clearing the Screen.
        break;
      case 0x055:
        LoHiRes |= 4;   // HISCR      Displays Page 2 without Clearing the Screen.
        break;
      case 0x056:
        LoHiRes &= ~8;  // LORES      Resets Page from Hi-Res to Lo-Res/Text Mode.
        break;
      case 0x057:
        LoHiRes |= 8;   // HIRES      Resets Page from Lo-Res/Text to Hi-Res Mode.
        break;
      case 0x061:
//PB0	$C061
        break;
      case 0x069:
//PB0_X	$C069
        break;
      case 0x062:
//PB1	$C062
        break;
      case 0x06a:
//PB1_X	$C06A
        break;
      case 0x063:
//PB2	$C063
        break;
      case 0x06b:
//PB2_X	$C06B        
        break;
      case 0x064:
//PADDL0	$C064
        break;
      case 0x06c:
//PADDL0_X	$C06C
        break;
      case 0x065:
//PADDL1	$C065
        break;
      case 0x06d:
//PADDL1_X	$C06D
        break;
      case 0x066:
//PADDL2	$C066
        break;
      case 0x06e:
//PADDL2_X	$C06E
        break;
      case 0x067:
//PADDL3	$C067
        break;
      case 0x06f:
//PADDL3_X	$C06F
        break;
      case 0x080:
      case 0x081:
      case 0x082:
      case 0x083:
      case 0x084:
      case 0x085:
      case 0x086:
      case 0x087:
      case 0x088:
      case 0x089:
      case 0x08a:
      case 0x08b:
      case 0x08c:
      case 0x08d:
      case 0x08e:
      case 0x08f:
// soft switch
        memSelector;
        break;
/*IWMMOTORON	$C089
IWMPH0OFF	$C080
IWMPH0ON	$C081
IWMQ6OFF	$C08C
IWMQ7OFF	$C08E
IWMSELDRV1	$C08A*/        
      default:
        //creare una I/O area da 0x100 byte...?
        break;
      }
		}
	else {
#if defined(__PIC32MM__)
    if(t < MAX_RAM) 
#endif
      i=ram_seg[t];
		}
#endif

//		wsprintf(myBuf,"read8 a %04x, %02x\n",t1,i);
//		_lwrite(spoolFile,myBuf,strlen(myBuf));
	return i;
	}

#ifdef KIMKLONE 
BYTE GetValueExt(DWORD t) {
  return GetValue(t);
	}
#endif

SWORD GetIntValue(SWORD t) {
	register SWORD i;
	char myBuf[128];

#ifdef COMMODORE64
	if(t >= 0x8000) {
    if(t <= 0x9fff) {
      if(!(PLAReg[0] & 3)) {    // non è perfetto ma per ora ok
  			if((CPUIOReg[1] & 3) == 3) {
          t &= 0x3fff;
#ifdef USA_CARTRIDGE
          i=MAKEWORD(C64cartridge[t],C64cartridge[t+1]);
#endif
          }
        else {
#if !defined(__PIC32MM__)
          i=MAKEWORD(ram_seg[t],ram_seg[t+1]);
#endif
          }
        }
      else {
#if !defined(__PIC32MM__)
				i=MAKEWORD(ram_seg[t],ram_seg[t+1]);
#endif
        }
      }
    else if(t <= 0xbfff) {
      if(!(PLAReg[0] & 3)) {    // non è perfetto ma per ora ok
  			if(CPUIOReg[1] & 2) {
          t &= 0x3fff;
#ifdef USA_CARTRIDGE
          i=MAKEWORD(C64cartridge[t],C64cartridge[t+1]);
#endif
          }
        else {
#if !defined(__PIC32MM__)
          i=MAKEWORD(ram_seg[t],ram_seg[t+1]);
#endif
          }
        }
			else if(CPUIOReg[1] & 1) {
        t &= 0x1fff;
				i=MAKEWORD(C64basic[t],C64basic[t+1]);
				}
			else {
#if !defined(__PIC32MM__)
				i=MAKEWORD(ram_seg[t],ram_seg[t+1]);
#endif
				}
			}
		else if(t >= 0xe000) {
			if(CPUIOReg[1] & 2) {
        t &= 0x1fff;
				i=MAKEWORD(C64kern[t],C64kern[t+1]);
				}
			else {
#if !defined(__PIC32MM__)
				i=MAKEWORD(ram_seg[t],ram_seg[t+1]);
#endif
				}
			}
		else {     // è lento, ma si gestisce tutto il resto :)
  		i=MAKEWORD(GetValue(t),GetValue(t+1));
			}
		}
	else {
#if defined(__PIC32MM__)
    if(t < MAX_RAM) 
#endif
    	i=MAKEWORD(ram_seg[t],ram_seg[t+1]);
		}
#endif
  
#ifdef COMMODOREVIC20
	if(t >= 0x8000) {
    if(t <= 0x8fff) {
      t &= 0xfff;
      i=MAKEWORD(V20char[t],V20char[t+1]);
      }
    else if(t <= 0x9fff) {  // è lento, ma si gestisce tutto il resto :)
  		i=MAKEWORD(GetValue(t),GetValue(t+1));
			}
    else if(t <= 0xbfff) {
      t &= 0x1fff;
#ifdef USA_CARTRIDGE
      if(SW2)      //
  			i=MAKEWORD(V20cartridge[t],V20cartridge[t+1]);
#endif
			}
    else if(t <= 0xdfff) {
      t &= 0x1fff;
			i=MAKEWORD(V20basic[t],V20basic[t+1]);
			}
		else {
      t &= 0x1fff;
			i=MAKEWORD(V20kern[t],V20kern[t+1]);
			}
		}
	else {
    if(t < MAX_RAM) 
    	i=MAKEWORD(ram_seg[t],ram_seg[t+1]);
		}
#endif

#ifdef AMICO2000
	if(t >= 0xfb00) {
		if(t <= 0xfcff) {
			i=MAKEWORD(Amico2000_rom2[t-0xfe00],Amico2000_rom2[t-0xfe00+1]);
			}
		else if(t >= 0xfe00) {
			i=MAKEWORD(Amico2000_rom1[t-0xfe00],Amico2000_rom1[t-0xfe00+1]);
			}
		else {     // è lento, ma si gestisce tutto il resto :)
  		i=MAKEWORD(GetValue(t),GetValue(t+1));
			}
		}
	else {
    if(t < MAX_RAM)
  		i=MAKEWORD(ram_seg[t],ram_seg[t+1]);
		}
#endif
  
#ifdef APPLE2
	if(t >= 0xf800) {
    t-=0xf800;
    if(memSelector[0]) {
      }
    i=MAKEWORD(AppleROM_F8[t],AppleROM_F8[t+1]);
		}
	else if(t >= 0xf000) {
    t-=0xf000;
    if(memSelector[1]) {
      }
    i=MAKEWORD(AppleROM_F0[t],AppleROM_F0[t+1]);
		}
	else if(t >= 0xe800) {
    t-=0xe800;
    if(memSelector[2]) {
      }
    i=MAKEWORD(AppleROM_E8[t],AppleROM_E8[t+1]);
		}
	else if(t >= 0xe000) {
    t-=0xe000;
    if(memSelector[3]) {
      }
    i=MAKEWORD(AppleROM_E0[t],AppleROM_E0[t+1]);
		}
	else if(t >= 0xd000) {
    t-=0xd000;
    if(memSelector[5]) {
      }
    i=MAKEWORD(AppleROM_D0[t],AppleROM_D0[t+1]);
		}
	else if(t >= 0xc000 && t<0xd000) {
    switch(t & 0xfff) {
      case 0x000:
        i=Keyboard[0];			// QUA? boh
        break;
      case 0x010:
// strobe...  
        break;
      case 0x020:
 //TAPEOUT    =($C02X); Cassette Data Out: Digital to Analog Audio 
        break;
      case 0x030:
// speaker;
        break;
      case 0x040:   // tutte e 16, fare
      case 0x04f:
//          STROBE     =($C04X); Outputs Strobe Pulse to Game I/O Connector;
        break;
      case 0x080:
      case 0x081:
      case 0x082:
      case 0x083:
      case 0x084:
      case 0x085:
      case 0x086:
      case 0x087:
      case 0x088:
      case 0x089:
      case 0x08a:
      case 0x08b:
      case 0x08c:
      case 0x08d:
      case 0x08e:
      case 0x08f:
// soft switch
        memSelector;
        break;
      default:
        //creare una I/O area da 0x100 byte...?
        break;
      }
		}
	else {
#if defined(__PIC32MM__)
    if(t < MAX_RAM) 
#endif
      i=MAKEWORD(ram_seg[t],ram_seg[t+1]);
		}
#endif


//			wsprintf(myBuf,"read16 a %04x, %02x\n",t1,i);
//		_lwrite(spoolFile,myBuf,strlen(myBuf));
	return i;
	}

#ifdef KIMKLONE 
SWORD GetIntValueExt(DWORD t) {
  return GetIntValue(t);
	}
#endif

BYTE GetPipe(SWORD t) {

#ifdef COMMODORE64
	if(t >= 0x8000) {
    if(t <= 0x9fff) {
      if(!(PLAReg[0] & 3)) {    // non è perfetto ma per ora ok
  			if((CPUIOReg[1] & 3) == 3) {
          t &= 0x3fff;
#ifdef USA_CARTRIDGE
          Pipe1=C64cartridge[t++];
          Pipe2.bytes.byte1=C64cartridge[t++];
          Pipe2.bytes.byte2=C64cartridge[t];
#endif
          }
        else {
          Pipe1=ram_seg[t++];
          Pipe2.bytes.byte1=ram_seg[t++];
          Pipe2.bytes.byte2=ram_seg[t];
          }
				}
			else {
#if !defined(__PIC32MM__)
				Pipe1=ram_seg[t++];
				Pipe2.bytes.byte1=ram_seg[t++];
				Pipe2.bytes.byte2=ram_seg[t];
#endif
				}
      }
    else if(t <= 0xbfff) {
      if(!(PLAReg[0] & 3)) {    // non è perfetto ma per ora ok
  			if(CPUIOReg[1] & 2) {
          t &= 0x3fff;
#ifdef USA_CARTRIDGE
          Pipe1=C64cartridge[t++];
          Pipe2.bytes.byte1=C64cartridge[t++];
          Pipe2.bytes.byte2=C64cartridge[t];
#endif
          }
        else {
#if !defined(__PIC32MM__)
          Pipe1=ram_seg[t++];
          Pipe2.bytes.byte1=ram_seg[t++];
          Pipe2.bytes.byte2=ram_seg[t];
#endif
          }
        }
			else if(CPUIOReg[1] & 1) {
        t &= 0x1fff;
				Pipe1=C64basic[t++];
				Pipe2.bytes.byte1=C64basic[t++];
				Pipe2.bytes.byte2=C64basic[t];
				}
			else {
#if !defined(__PIC32MM__)
				Pipe1=ram_seg[t++];
				Pipe2.bytes.byte1=ram_seg[t++];
				Pipe2.bytes.byte2=ram_seg[t];
#endif
				}
			}
		else if(t >= 0xe000) {
			if(CPUIOReg[1] & 2) {
        t &= 0x1fff;
				Pipe1=C64kern[t++];
				Pipe2.bytes.byte1=C64kern[t++];
				Pipe2.bytes.byte2=C64kern[t];
				}
			else {
#if !defined(__PIC32MM__)
				Pipe1=ram_seg[t++];
				Pipe2.bytes.byte1=ram_seg[t++];
				Pipe2.bytes.byte2=ram_seg[t];
#endif
				}
			}
		else {     // questo gestisce tutto il resto
			Pipe1=GetValue(t++);
			Pipe2.word=GetIntValue(t);
			}
		}
	else {
#if defined(__PIC32MM__)
    if(t < MAX_RAM) {
#endif
      Pipe1=ram_seg[t++];
      Pipe2.bytes.byte1=ram_seg[t++];
      Pipe2.bytes.byte2=ram_seg[t];
#if defined(__PIC32MM__)
    }
#endif
		}
#endif
  
#ifdef COMMODOREVIC20
	if(t >= 0x8000) {
    if(t <= 0xbfff) {
      t &= 0x1fff;
#ifdef USA_CARTRIDGE
      if(SW2) {     //
        Pipe1=V20cartridge[t++];
        Pipe2.bytes.byte1=V20cartridge[t++];
        Pipe2.bytes.byte2=V20cartridge[t];
        }
#endif
      }
    else if(t <= 0xdfff) {
      t &= 0x1fff;
      Pipe1=V20basic[t++];
      Pipe2.bytes.byte1=V20basic[t++];
      Pipe2.bytes.byte2=V20basic[t];
			}
		else {
      t &= 0x1fff;
      Pipe1=V20kern[t++];
      Pipe2.bytes.byte1=V20kern[t++];
      Pipe2.bytes.byte2=V20kern[t];
			}
		}
	else {
    if(t < MAX_RAM) {
      Pipe1=ram_seg[t++];
      Pipe2.bytes.byte1=ram_seg[t++];
      Pipe2.bytes.byte2=ram_seg[t];
      }
		}
#endif

#ifdef AMICO2000
	if(t >= 0xfb00) {
		if(t <= 0xfcff) {
      t-=0xfb00;
      Pipe1=Amico2000_rom2[t++];
      Pipe2.bytes.byte1=Amico2000_rom2[t++];
      Pipe2.bytes.byte2=Amico2000_rom2[t];
			}
		else if(t >= 0xfe00) {
      t-=0xfe00;
      Pipe1=Amico2000_rom1[t++];
      Pipe2.bytes.byte1=Amico2000_rom1[t++];
      Pipe2.bytes.byte2=Amico2000_rom1[t];
			}
		else {     // questo gestisce tutto il resto
			Pipe1=GetValue(t++);
			Pipe2.word=GetIntValue(t);
			}
		}
	else {
    if(t<MAX_RAM) {
      Pipe1=ram_seg[t++];
      Pipe2.bytes.byte1=ram_seg[t++];
      Pipe2.bytes.byte2=ram_seg[t];
      }
		}
#endif
  
#ifdef APPLE2
	if(t >= 0xf800) {
    t-=0xf800;
    if(memSelector[0]) {
      }
		Pipe1=AppleROM_F8[t++];
		Pipe2.bytes.byte1=AppleROM_F8[t++];
		Pipe2.bytes.byte2=AppleROM_F8[t];
		}
	else if(t >= 0xf000) {
    t-=0xf000;
    if(memSelector[1]) {
      }
		Pipe1=AppleROM_F0[t++];
		Pipe2.bytes.byte1=AppleROM_F0[t++];
		Pipe2.bytes.byte2=AppleROM_F0[t];
		}
	else if(t >= 0xe800) {
    t-=0xe800;
    if(memSelector[2]) {
      }
		Pipe1=AppleROM_E8[t++];
		Pipe2.bytes.byte1=AppleROM_E8[t++];
		Pipe2.bytes.byte2=AppleROM_E8[t];
		}
	else if(t >= 0xe000) {
    t-=0xe000;
    if(memSelector[3]) {
      }
		Pipe1=AppleROM_E0[t++];
		Pipe2.bytes.byte1=AppleROM_E0[t++];
		Pipe2.bytes.byte2=AppleROM_E0[t];
		}
	else if(t >= 0xd000) {
    t-=0xd000;
    if(memSelector[5]) {
      }
		Pipe1=AppleROM_D0[t++];
		Pipe2.bytes.byte1=AppleROM_D0[t++];
		Pipe2.bytes.byte2=AppleROM_D0[t];
		}
	else {
#if defined(__PIC32MM__)
    if(t < MAX_RAM) {
#endif
      Pipe1=ram_seg[t++];
      Pipe2.bytes.byte1=ram_seg[t++];
      Pipe2.bytes.byte2=ram_seg[t];
#if defined(__PIC32MM__)
    }
#endif
    }
#endif

	return Pipe1;
	}

#ifdef KIMKLONE 
BYTE GetPipeExt(DWORD t) {
  return GetPipe(t);
	}
#endif

void PutValue(SWORD t,BYTE t1) {
	register SWORD i;
	BYTE *video;
	char myBuf[128];
	SWORD j;

//			wsprintf(myBuf,"store a %04x, %02x\n",t,t1);
//		_lwrite(spoolFile,myBuf,strlen(myBuf));
  
#ifdef COMMODORE64
	if((t >= 0xd000) && (t <= 0xdfff)) {
//    printf("Store a %04x = %02x\n",t,t1);
		if(t <= 0xd3ff) {
			t &= 0x3f;
			VICReg[t]=t1;
			switch(t) {
				case 0x19:
          /// IRQ
					break;
				case 0x20:
				case 0x21:
					break;
				}
			}
		else if((t >= 0xd400) && (t <= 0xd7ff)) {
//https://www.c64-wiki.com/wiki/SID
      t &= 0x1f;
			SIDReg[t]=t1;
			switch(t) {
        case 0:
        case 1:
  #define PAL_PHI 985248L
	#define NTSC_PHI 1022727L //This is for machines with 6567R8 VIC. 6567R56A is slightly different.
	#define CONSTANT ((256UL*256UL*256UL) / PAL_PHI) //Select the constant appropriate for your machine (PAL vs NTSC).
	j =  MAKEWORD(SIDReg[0],SIDReg[1]) / CONSTANT /*SID_FREQ*/; //Calculate SID freq for a certain note (specified in Hz).
  
  if(!j)
    j=1;
  j=(390625 /*v. timer2*/ )/j;
          PR2 = j;		 // 
#if defined(__PIC32MM__)
          CCP4RA=j/2;
#else
          OC1RS = j/2;		 // 
#endif
          break;
  			}
			}
		else if((t >= 0xd800) && (t <= 0xdbff)) {
      t &= 0x3ff;
			ColorRAM[t]=t1;
			}
		else if((t >= 0xdc00) && (t <= 0xdcff)) {
//printf("Scrivo a %04x: %02x\n",t,t1);
      t &= 0xf;
      switch(t) {
        case 0xd:
    			CIA1RegW[t]=t1;
          break;
        default:
    			CIA1RegR[t]=CIA1RegW[t]=t1;
          break;
        }
			}
		else if((t >= 0xdd00) && (t <= 0xddff)) {
      t &= 0xf;
      switch(t) {
        case 0:
    			CIA2RegW[t]=t1;
#if defined(__PIC32MM__)
  				(t1 & 0x10 ? 2 : 0) | (t1 & 0x20 ? 8 : 0) | (t1 & 0x8 ? 1 : 0);    // FINIRE!
#else
  				PMMODE=(t1 & 0x10 ? 2 : 0) | (t1 & 0x20 ? 8 : 0) | (t1 & 0x8 ? 1 : 0);    // FINIRE!
#endif
          break;
        case 1:
    			CIA2RegW[t]=t1;
  				LATB = (LATB & 0xff00) | t1;
          break;
        case 0xd:
    			CIA2RegW[t]=t1;
          break;
        default:
    			CIA2RegR[t]=CIA2RegW[t]=t1;
          break;
        }
			}
		}
	else if(t <= 1) {
		ram_seg[t]=CPUIOReg[t]=t1;      // per sicurezza... specie le istruzioni zero-page
		}
	else {
#if defined(__PIC32MM__)
    if(t < MAX_RAM)
#endif
      ram_seg[t]=t1;
		}
#endif

#ifdef COMMODOREVIC20
	if((t >= 0x9000) && (t <= 0x9fff)) {
		if(t <= 0x900f) {
      t &= 0xf;
			VICReg[t]=t1;
			switch(t) {
				case 0xa:   // audio-alto
#define PAL_PHI (4433618UL/4)
#define NTSC_PHI (14318181UL/14) 
          if(VICReg[0xa] & 0x80) {   //b7=on-off
            if(VICReg[0xa]<255) {   //
              j=PAL_PHI/256/(255-VICReg[0xa]);
              if(!j)
                j=1;
              j=(390625 /*v. timer2*/ )/j;
              PR2 = j;		 // 
#if defined(__PIC32MM__)
              CCP4RA=j/2;
#else
              OC1RS = j/2;		 // 
#endif
              }
            }
          else {
#if defined(__PIC32MM__)
            CCP4RA=0;
#else
            OC1RS =0;		 // 
#endif
            }
          break;
				case 0xb:   // audio-tenor
          if(VICReg[0xb] & 0x80) {   //b7=on-off
            if(VICReg[0xb]<255) {
              j=PAL_PHI/128/(255-VICReg[0xb]);
              if(!j)
                j=1;
              j=(390625 /*v. timer2*/ )/j;
              PR2 = j;		 // 
#if defined(__PIC32MM__)
              CCP4RA=j/2;
#else
              OC1RS = j/2;		 // 
#endif
              }
            }
          else {
#if defined(__PIC32MM__)
            CCP4RA=0;
#else
            OC1RS =0;		 // 
#endif
            }
					break;
				case 0xc:   // audio-soprano
          if(VICReg[0xc] & 0x80) {   //b7=on-off
            if(VICReg[0xc]<255) {
              j=PAL_PHI/64/(255-VICReg[0xc]);
              if(!j)
                j=1;
              j=(390625 /*v. timer2*/ )/j;
              PR2 = j;		 // 
#if defined(__PIC32MM__)
              CCP4RA=j/2;
#else
              OC1RS =j/2;		 // 
#endif
              }
            }
          else {
#if defined(__PIC32MM__)
            CCP4RA=0;
#else
            OC1RS =0;		 // 
#endif
            }
					break;
				case 0xd:   // audio-noise
          if(VICReg[0xd] & 0x80) {   //b7=on-off
            if(VICReg[0xd]<255) { 
              j=PAL_PHI/32/(255-VICReg[0xd]);
              if(!j)
                j=1;
              j=(390625 /*v. timer2*/ )/j;
              PR2 = j;		 // 
#if defined(__PIC32MM__)
              CCP4RA=j/2;
#else
              OC1RS = j/2;		 // 
#endif
              }
            }
          else {
#if defined(__PIC32MM__)
            CCP4RA=0;
#else
            OC1RS =0;		 // 
#endif
            }
					break;
				case 0xe:   // audio-volume, multicolor
					break;
				}
			}
		else if((t >= 0x9110) && (t <= 0x911f)) {
//printf("Scrivo a %04x: %02x\n",t,t1);
      t &= 0xf;
      switch(t) {
        case 0:   // ORB
          VIA1RegR[0xd] &= ~0x10;
          if(1)   // v. doc...
            VIA1RegR[0xd] &= ~0x8;
    			VIA1RegR[0]=VIA1RegW[0]=t1;
  				LATB = (LATB & 0xff00) | t1;
          break;
        case 1:   // ORA
          VIA1RegR[0xd] &= ~0x2;
          if(1)   // v. doc...
            VIA1RegR[0xd] &= ~0x1;
    			VIA1RegR[1]=VIA1RegW[1]=t1;
#if defined(__PIC32MM__)
  				(t1 & 0x10 ? 2 : 0) | (t1 & 0x20 ? 8 : 0) | (t1 & 0x8 ? 1 : 0);    // FINIRE!
#else
  				PMMODE=(t1 & 0x10 ? 2 : 0) | (t1 & 0x20 ? 8 : 0) | (t1 & 0x8 ? 1 : 0);    // FINIRE!
#endif
          break;
        case 0x05:  // T1 high
          VIA1RegR[0xd] &= ~0x40;
    			VIA1RegW[5]=t1;
          break;
        case 0x09:  // T2 high
          VIA1RegR[0xd] &= ~0x20;
    			VIA1RegW[9]=t1;
          break;
        case 0x0a:  // SR
          VIA1RegR[0xd] &= ~0x4;
    			VIA1RegR[10]=VIA1RegW[10]=t1;
          break;
        case 0xd:   // IFR
    			VIA1RegW[0xd] = t1;   // 
    			VIA1RegR[0xd] &= ~(t1 & 0x7f);   // 
          break;
        case 0xe:   // IER
          if(t1 & 0x80)
            VIA1RegW[0xe] |= t1 & 0x7f;   //
          else
            VIA1RegW[0xe] &= ~(t1 & 0x7f);   //
          break;
        default:
    			VIA1RegR[t]=VIA1RegW[t]=t1;
          break;
        }
      if(VIA1RegR[0xd] & 0x7f)
        VIA1RegR[0xd] |= 0x80;
      else
        VIA1RegR[0xd] &= ~0x80;
			}
		else if((t >= 0x9120) && (t <= 0x912f)) {
      t &= 0xf;
      switch(t) {
        case 0:
          VIA2RegR[0xd] &= ~0x10;
          if(1)   // v. doc...
            VIA2RegR[0xd] &= ~0x8;
    			VIA2RegR[0]=VIA2RegW[0]=t1;
          break;
        case 1:
          VIA2RegR[0xd] &= ~0x2;
          if(1)   // v. doc...
            VIA2RegR[0xd] &= ~0x1;
    			VIA2RegR[1]=VIA2RegW[1]=t1;
          break;
        case 0x05:  // T1 high
          VIA2RegR[0xd] &= ~0x40;
    			VIA2RegW[5]=t1;
          break;
        case 0x09:  // T2 high
          VIA2RegR[0xd] &= ~0x20;
    			VIA2RegW[9]=t1;
          break;
        case 0x0a:  // SR
          VIA2RegR[0xd] &= ~0x4;
    			VIA2RegR[10]=VIA1RegW[10]=t1;
          break;
        case 0xd:   // IFR
    			VIA2RegW[0xd] = t1;   // 
    			VIA2RegR[0xd] &= ~(t1 & 0x7f);   // 
          break;
        case 0xe:   // IER
          if(t1 & 0x80)
            VIA2RegW[0xe] |= t1 & 0x7f;   //
          else
            VIA2RegW[0xe] &= ~(t1 & 0x7f);   //
          break;
        default:
    			VIA2RegR[t]=VIA2RegW[t]=t1;
          break;
        }
      if(VIA2RegR[0xd] & 0x7f)
        VIA2RegR[0xd] |= 0x80;
      else
        VIA2RegR[0xd] &= ~0x80;
			}
    else if((t >= (0x9400 | ((WORD)VICReg[2] & 0x80)<<2)) && 
      (t <= (0x95ff | ((WORD)VICReg[2] & 0x80)<<2))) {
      t &= 0x1ff;
      ColorRAM[t]=t1;
      }
		}
	else {
    if(t < MAX_RAM)
      ram_seg[t]=t1;
		}
#endif

#ifdef AMICO2000
	if(t >= 0xfd00 && t <= 0xfd03) {
//printf("Scrivo a %04x: %02x\n",t,t1);
    t &= 0x3;
    switch(t) {
      case 0x0:
        CIA8255RegR[0]=CIA8255RegW[0]=t1;
        if(!(CIA8255RegW[3] & 0b00010000)) {   // se in scrittura
          if(CIA8255RegW[1]>=8) {
      LED3 ^= 1;      // test timing, refresh completo ogni ~400uS, 13/11/19
            i=(CIA8255RegW[1] >> 1) & 15;    // posizione mux, 4..9 in uscita dal CIA che però è x2 (B1..B4)
            i-=4;
            i=min(5,i);
            if(t1) {      // qua non è mai 0... così uso sto trucco per emulare il MUX!
              t1 &= 0x7f;
              DisplayRAM[i]=t1;
#ifndef USING_SIMULATOR
              PlotDisplay(i,t1,1);
#endif
              }
            }
          }
        if(!(CIA8255RegW[1] & 0b00010000)) {   // inoltre, se PB7=0 4 led vengono pilotati da PA0..3 (invertiti))
          
          }
        break;
      case 0x1:
        CIA8255RegR[1]=CIA8255RegW[1]=t1;
        break;
      case 0x2:     // user port
        if(CIA8255RegW[3] & 0b10000000) {   // se non modalità single bit
          if(!(CIA8255RegW[3] & 0b00000001))    // se è in output, nibble basso
            LATB = (LATB & 0xfff0) | (t1 & 0xf);
          if(!(CIA8255RegW[3] & 0b00001000))    // idem, nibble alto
            LATB = (LATB & 0xff0f) | ((t1 & 0xf) << 4);
          CIA8255RegR[2]=CIA8255RegW[2]=t1;
          }
        break;
      default:      // control byte
        CIA8255RegR[t]=CIA8255RegW[t]=t1;
        if(!(CIA8255RegW[3] & 0b10000000)) {   // se modalità single bit
          i=(CIA8255RegW[3] >> 1) & 0x7;
          i = 1 << i;
          if(!(CIA8255RegW[3] & 0b00000001))
            LATB &= ~i;
          else
            LATB |= i;
          }
        break;
      }
		}
	else {
		ram_seg[t]=t1;
		}
#endif
  
#ifdef APPLE2
	if(t >= 0xd000) {
		}
	else if(t >= 0xc000 && t<0xd000) {
    switch(t & 0xfff) {
      case 0x000:
//        i=Keyboard[0];
        break;
      case 0x010:
// strobe...  opp 80col video
        break;
      case 0x030:
// speaker;
        break;
      case 0x050:      // CREDO anche in write, anche se non è chiaro..
        LoHiRes &= ~1;  //  TXTCLR     Sets Graphics Mode without Clearing Screen;
                        // Resets from Text Mode.
        break;
      case 0x051:
        LoHiRes |= 1;   //  TXTSET     Sets Text Mode without Resetting Scrolling Window;
                        //             Resets from Graphics Mode.
        break;
      case 0x052:
        LoHiRes &= ~2;  //  MIXCLR     Sets Full-Screen Graphics Mode;
                        // Resets from Mixed Graphics Mode
                        // (with 4 lines of text at the bottom of the screen).
        break;
      case 0x053:
        LoHiRes |= 2;   // MIXSET     Sets Mixed Text & Graphics Mode
                        // (with 4 lines of text at the bottom of the screen).
        break;
      case 0x054:
        LoHiRes &= ~4;  // LOWSCR     Displays Page 1 without Clearing the Screen.
        break;
      case 0x055:
        LoHiRes |= 4;   // HISCR      Displays Page 2 without Clearing the Screen.
        break;
      case 0x056:
        LoHiRes &= ~8;  // LORES      Resets Page from Hi-Res to Lo-Res/Text Mode.
        break;
      case 0x057:
        LoHiRes |= 8;   // HIRES      Resets Page from Lo-Res/Text to Hi-Res Mode.
        break;
      case 0x061:
//PB0	$C061
        break;
      case 0x069:
//PB0_X	$C069
        break;
      case 0x062:
//PB1	$C062
        break;
      case 0x06a:
//PB1_X	$C06A
        break;
      case 0x063:
//PB2	$C063
        break;
      case 0x06b:
//PB2_X	$C06B        
        break;
      case 0x064:
//PADDL0	$C064
        break;
      case 0x06c:
//PADDL0_X	$C06C
        break;
      case 0x065:
//PADDL1	$C065
        break;
      case 0x06d:
//PADDL1_X	$C06D
        break;
      case 0x066:
//PADDL2	$C066
        break;
      case 0x06e:
//PADDL2_X	$C06E
        break;
      case 0x067:
//PADDL3	$C067
        break;
      case 0x06f:
//PADDL3_X	$C06F
        break;
      case 0x080:
      case 0x081:
      case 0x082:
      case 0x083:
      case 0x084:
      case 0x085:
      case 0x086:
      case 0x087:
      case 0x088:
      case 0x089:
      case 0x08a:
      case 0x08b:
      case 0x08c:
      case 0x08d:
      case 0x08e:
      case 0x08f:
// soft switch
//https://mirrors.apple2.org.za/apple.cabi.net/Languages.Programming/MemoryMap.IIe.64K.128K.txt
        memSelector;
        break;
/*IWMMOTORON	$C089
IWMPH0OFF	$C080
IWMPH0ON	$C081
IWMQ6OFF	$C08C
IWMQ7OFF	$C08E
IWMSELDRV1	$C08A*/        
      default:
        //creare una I/O area da 0x100 byte...?
        break;
      }
		}
	else {
#if defined(__PIC32MM__)
    if(t < MAX_RAM) 
#endif
      ram_seg[t]=t1;
		}
#endif
  
	}

#ifdef KIMKLONE 
void PutValueExt(DWORD t,BYTE t1) {
  
  PutValue(t,t1);
	}
#endif
