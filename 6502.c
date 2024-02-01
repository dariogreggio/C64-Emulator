#include <ctype.h>
#include <stdlib.h>
#ifdef __PIC32
#include <conio.h>
#endif
#include <xc.h>
#include "c64_pic.h"


//v. per Ovf metodo usato su 68000 senza byte alto... provare

//https://www.masswerk.at/6502/6502_instruction_set.html
#undef CPU_65C02
#define CPU_KIMKLONE

extern BOOL debug;
extern BYTE ram_seg[MAX_RAM];
extern BYTE *stack_seg;
#ifdef COMMODORE64
extern BYTE VICReg[64];
extern SWORD VICRaster;
extern BYTE CIA1RegR[16], CIA1RegW[16];
extern BYTE CIA2RegR[16], CIA2RegW[16];
#endif
#ifdef COMMODOREVIC20
extern BYTE VICReg[16];
extern SWORD VICRaster;
extern BYTE VIA1RegR[16], VIA1RegW[16];
extern BYTE VIA2RegR[16], VIA2RegW[16];
#endif
#ifdef CPU_KIMKLONE   // https://laughtonelectronics.com/Arcana/KimKlone/Kimklone_intro.html
BYTE K[4];
BYTE IP,W;
#endif

BYTE DoIRQ=0,DoNMI=0,DoReset=1,DoHalt=0;
BYTE ColdReset=1;

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

#ifdef COMMODORE64
extern volatile BYTE *keysFeedPtr;
extern const char keysFeed[];
volatile BYTE CIA1IRQ,CIA2IRQ,VICIRQ;
#endif
#ifdef COMMODOREVIC20
extern volatile BYTE *keysFeedPtr;
extern const char keysFeed[];
volatile BYTE VIA1IRQ,VIA2IRQ,VICrfsh;
#endif

#ifdef AMICO2000
extern volatile BYTE keysFeedPtr;
#endif

#ifdef APPLE2
extern volatile BYTE keysFeedPtr;
extern const char keysFeed[];
#endif


int Emulate(int mode) {
	register SWORD _pc=0;
	register BYTE _a=0;
	register BYTE _x=0,_y=0;
	register BYTE _s=0;
#define ID_CARRY 0x1
#define ID_ZERO 0x2
#define ID_INTERRUPT 0x4
#define ID_DECIMAL 0x8
#define ID_BREAK 0x10
#define ID_OVERFLOW 0x40
#define ID_MINUS 0x80
	union __attribute__((aligned(1), packed)) REGISTRO_P {
    BYTE b;
		struct __attribute__((__packed__)) {
      unsigned int Carry: 1;
      unsigned int Zero: 1;
      unsigned int Interrupt: 1;
      unsigned int Decimal: 1;
      unsigned int Break: 1;
      unsigned int unused: 1;
      unsigned int Overflow: 1;
      unsigned int Minus: 1;
      };
		} _pIRQ;
// http://www.righto.com/2012/12/the-6502-overflow-flag-explained.html
// per overflow e halfcarry https://stackoverflow.com/questions/8034566/overflow-and-carry-flags-on-z80
	register union REGISTRO_P _p;
	union REGISTRO_P _p1;
  register union OPERAND op1,op2;
  register union RESULT res1,res2,res3;
	register SWORD i;
	int c=0;
	
  
	while(TRUE) {

		c++;
		if(!(c & 4095)) {
      ClrWdt();
// yield()
    	}
		if(ColdReset) {
      DoReset=1;
      ColdReset=0;
			continue;
      }
		/*
		if((_pc >= 0xa000) && (_pc <= 0xbfff)) {
			printf("%04x    %02x\n",_pc,GetValue(_pc));
			}
			*/
		if(debug) {
//			wsprintf(myBuf,"PC: %04x    %02x",_pc,GetValue(_pc));
//			SetWindowText(hStatusWnd,myBuf);
			}
/*		if(_pc>0xa000 && _pc<0xc000) {
		wsprintf(myBuf,"PC: %04x    %02x, stack: %02x\n",_pc,GetValue(_pc),_s);
		_lwrite(spoolFile,myBuf,strlen(myBuf));
		}*/

//		_lwrite(spoolFile,ram_seg+0x400,0x400);
/*		if(kbhit()) {
			getch();
			printf("%04x    %02x\n",_pc,GetValue(_pc));
			printf("281-284: %02x %02x %02x %02x\n",*(p1+0x281),*(p1+0x282),*(p1+0x283),*(p1+0x284));
			printf("2b-2c: %02x %02x\n",*(p1+0x2b),*(p1+0x2c));
			printf("33-34: %02x %02x\n",*(p1+0x33),*(p1+0x34));
			printf("37-38: %02x %02x\n",*(p1+0x37),*(p1+0x38));
			}*/
		if(DoReset) {
#ifdef COMMODORE64
			PutValue(0,0x2f);
			PutValue(1,0x37);
			VICReg[0x11]=0x10;
#endif
			_pc=GetIntValue(0xfffc);
			_p.Interrupt=1;
      _p.unused=1;
      _pIRQ.b=0;
      _p1.b=0;
      _s=0;     // dicono...
			DoReset=0;
			}
    if(DoNMI) {
			DoNMI=0;
			stack_seg[_s--]=HIBYTE(_pc);
			stack_seg[_s--]=LOBYTE(_pc);
			_pIRQ=_p;
// flag BRK: https://wiki.nesdev.com/w/index.php/Status_flags#The_B_flag
			_pIRQ.Break=0;
			stack_seg[_s--]=_pIRQ.b;
			_pc=GetIntValue(0xfffa);
			_p.Interrupt=1;
//	    LED1 ^= 1;      // test NMI, 15/11/2019
			}
		if(DoIRQ) {
			if(!_p.Interrupt) {
				DoIRQ=0;
				stack_seg[_s--]=HIBYTE(_pc);
				stack_seg[_s--]=LOBYTE(_pc);
				_pIRQ=_p;
				_pIRQ.Break=0;
				stack_seg[_s--]=_pIRQ.b;
				_pc=GetIntValue(0xfffe);
				_p.Interrupt=1;
				}
			}
//printf("Pipe1: %02x, Pipe2w: %04x, Pipe2b1: %02x,%02x\n",Pipe1,Pipe2.word,Pipe2.bytes.byte1,Pipe2.bytes.byte2);

#ifdef COMMODORE64
      if(VICIRQ) {
        static BYTE oldSW2;
        VICIRQ=0;
  
        i=VICReg[0x12];
        i |= VICReg[0x11] & 0x80 ? 0x100 : 0;
        VICRaster+=8;					 	 // raster pos count, 200 al sec...
        if(VICRaster >= MAX_RASTER) {		 // 
          VICRaster=MIN_RASTER;
          LED2 ^= 1;      // 50Hz 8/11/19; 70mS su ILI 320x240, 7/8/20; 25mS PIC32MM 17/6/21
          }
        if(VICReg[0x1a] & 1) {
          if((VICRaster & 0xf7) == (i & 0xf7)) {
            VICReg[0x19] |= 0x81;
            DoIRQ=1;
            }
          }

        
#ifdef ILI9341
        static BYTE divider;
        divider++;
        if(!(divider & 3))
#endif
#if defined(__PIC32MM__)
        static BYTE divider;
        divider++;
        if(divider > 24) {
          divider=0;
          UpdateScreen(0,240);
          }
#else
        UpdateScreen(VICRaster,VICRaster+8);
#endif
  //      LED3 ^= 1;
        
        
        /* if(!SW1)        // VK_RETURN: sta bene qua :)
          Keyboard[0] &= 0xfd;
        else
          Keyboard[0] |= 0x02;
        */
        if(!SW1)        // test tastiera, me ne frego del repeat/rientro :)
          keysFeedPtr=&keysFeed;
        
        if(!SW2) {
          if(oldSW2) {
            DoNMI=1;    // solo sul fronte! o si blocca/sovraccarica
            oldSW2=0;
            }
          }
        else
          oldSW2=1;
  
        }
#endif
#ifdef COMMODOREVIC20
      if(VICrfsh) {
        static BYTE oldSW2;
        VICrfsh=0;
  
        i=VICReg[0x3] << 1;
        i |= VICReg[0x4] & 0x1;
        VICRaster+=8;					 	 // raster pos count, 200 al sec...
        if(VICRaster >= MAX_RASTER /*((VICReg[3] >> 1) & 0x3f)*8*/) {		 // 
          VICRaster=MIN_RASTER;
          LED2 ^= 1;      // 50Hz 8/11/19; 70mS su ILI 320x240, 7/8/20; 25mS PIC32MM 17/6/21
          }
        
#ifdef ILI9341
        static BYTE divider;
        divider++;
        if(!(divider & 3))
#endif
#if defined(__PIC32MM__)
        static BYTE divider;
        divider++;
        if(divider > 24) {
          divider=0;
          UpdateScreen(0,240);
          }
#else
        UpdateScreen(VICRaster,VICRaster+8);
#endif
  //      LED3 ^= 1;
        
        if(!SW1)        // test tastiera, me ne frego del repeat/rientro :)
          keysFeedPtr=&keysFeed;
        
        if(!SW2) {
          if(oldSW2) {
            DoNMI=1;    // solo sul fronte! o si blocca/sovraccarica
            oldSW2=0;
            }
          }
        else
          oldSW2=1;
  
        }
#endif
    
#ifdef AMICO2000
      if(!SW1)        // test tastiera
        keysFeedPtr=0;
    
      if(!SW2) {
  //      DoNMI=1;
          emulateKBD('+');
  //        emulateKBD(0);
  //      DoHalt=1;
        }
      else
        DoHalt=0;
#endif
    
#ifdef APPLE2
        static BYTE oldSW2;
        if(!SW1) {       // test tastiera, me ne frego del repeat/rientro :)
//          keysFeedPtr=&keysFeed;
          if(keysFeedPtr==255)
            keysFeedPtr=254;
          }
    
        static int divider;
        divider++;
        if(divider > 100000) {      // ~100mS
          divider=0;
#ifndef USING_SIMULATOR
          UpdateScreen(0,24*8);
#endif
          }

        if(!SW2) {
          if(oldSW2) {
            DoNMI=1;    // solo sul fronte! o si blocca/sovraccarica
            oldSW2=0;
            }
          }
        else
          oldSW2=1;
#endif
    

#ifdef COMMODORE64
//Timers
//http://www.oxyron.de/html/registers_cia.html
    if(CIA1IRQ) {       // TOD
      CIA1IRQ=0;
   		CIA1RegR[8]++;
   		if(CIA1RegR[8] >= 10) {
     		CIA1RegR[8]=0;
     		CIA1RegR[9]++;
     		if(CIA1RegR[9] >= 60) {
       		CIA1RegR[9]=0;
       		CIA1RegR[10]++;
          if(CIA1RegR[10] >= 60) {
            CIA1RegR[10]=0;
            CIA1RegR[11]++;
            if(CIA1RegR[11] >= 24) {
              CIA1RegR[11]=0;
              }
            }
          }
        }
      }
		if(CIA1RegR[0xe] & 1) {   // questo gira a 1MHz...
			if(!(CIA1RegR[0xe] & 4)) {
				if(!MAKEWORD(CIA1RegR[4],CIA1RegR[5])) {
          if(CIA1RegW[0xd] & 1) {
            CIA1RegR[0xd] |= 0x81;
            DoIRQ=1;
            }
					CIA1RegR[4]=CIA1RegW[4];
					CIA1RegR[5]=CIA1RegW[5];
          // e prescaler?
          }
				else {
					if(!CIA1RegR[4])
            CIA1RegR[5]--;
					CIA1RegR[4]--;
//					if(!*(short int *)(CIA1RegR+4)) {
//						T1++;
//				wsprintf(myBuf,"Timer 1A: %04x",T1);
//				SetWindowText(hStatusWnd,myBuf);
//					  }

					}
				}
		  }
		if(CIA2RegR[0xe] & 1) {
			if(!(CIA2RegR[0xe] & 4)) {
				if(!MAKEWORD(CIA2RegR[4],CIA2RegR[5])) {
          if(CIA2RegW[0xd] & 1) {
            CIA2RegR[0xd] |= 0x81;
            DoNMI=1;
            }
					CIA2RegR[4]=CIA2RegW[4];
					CIA2RegR[5]=CIA2RegW[5];
          // e prescaler?
          }
				else {
					if(!CIA2RegR[4])
            CIA2RegR[5]--;
					CIA2RegR[4]--;
					}
				}
		  }
		if(CIA1RegR[0xf] & 1) {   // timer B che fa??
  		switch(CIA1RegR[0xf] & 0b01100000) {
        case 0b00000000:     // conta cycles
          break;
        case 0b00100000:     // conta pulses su CNT-pin
          break;
        case 0b01000000:     // conta underflow di A... FINIRE
/*			if(!(CIA1RegR[0xf] & 4)) {
				if(!MAKEWORD(CIA1RegR[6],CIA1RegR[6])) {
          if(CIA1RegW[0xd] & 2) {
            CIA1RegR[0xd] |= 0x82;
            DoIRQ=1;
            }
					CIA1RegR[6]=CIA1RegW[6];
					CIA1RegR[7]=CIA1RegW[7];
          }
				else {
					if(!CIA1RegR[6])
            CIA1RegR[7]--;
					CIA1RegR[6]--;
					}
				}*/
          break;
        case 0b01100000:     // conta underflow A se CNT-pin
          break;
  		  }
		  }
		if(CIA2RegR[0xf] & 1) {
/*			if(!(CIA1RegR[0xf] & 4)) {
				if(!MAKEWORD(CIA1RegR[6],CIA1RegR[6])) {
          if(CIA1RegW[0xd] & 2) {
            CIA1RegR[0xd] |= 0x82;
            DoIRQ=1;
            }
					CIA1RegR[6]=CIA1RegW[6];
					CIA1RegR[7]=CIA1RegW[7];
          }
				else {
					if(!CIA1RegR[6])
            CIA1RegR[7]--;
					CIA1RegR[6]--;
					}
				}*/
		  }
#endif    

#ifdef COMMODOREVIC20
//Timers
//https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&ved=2ahUKEwiDgPDZ9uf7AhUFY8AKHfH5AW4QFnoECBQQAQ&url=http%3A%2F%2Farchive.6502.org%2Fdatasheets%2Fmos_6522_preliminary_nov_1977.pdf&usg=AOvVaw0IYOXNYjVcLjosryYWFgpa
    if(VIA1IRQ) {       // 
      }
		// questo gira a 1MHz...
    if((VIA2RegR[0xb] & 0x40)) {   // finire, capire..
      if(!MAKEWORD(VIA2RegR[4],VIA2RegR[5])) {
        if(VIA2RegW[0xe] & 0x40) {
          VIA2RegR[0xd] |= 0xc0;
          }
        if(VIA2RegR[0xb] & 0x80) {
          // toggle PB7
          }
        VIA2RegR[4]=VIA2RegW[4];
        VIA2RegR[5]=VIA2RegW[5];
        }
      else {
        if(!VIA2RegR[4])
          VIA2RegR[5]--;
        VIA2RegR[4]--;
        }
      }
    if(!(VIA2RegR[0xb] & 0x20)) {   // finire, capire..
      if(!MAKEWORD(VIA2RegR[8],VIA2RegR[9])) {
        if(VIA2RegW[0xe] & 0x20) {
          VIA2RegR[0xd] |= 0xa0;
          }
        VIA2RegR[8]=VIA2RegW[8];
        VIA2RegR[9]=VIA2RegW[9];
        }
      else {
        if(!VIA2RegR[8])
          VIA2RegR[9]--;
        VIA2RegR[8]--;
        }
      }
		switch(VIA2RegR[0xb] & 0b00011100) {
      case 0:
        break;
      default:    // FARE!!! finire
    		VIA2RegR[0xa] <<= 1;
        break;
      }
    if((VIA1RegR[0xb] & 0x40)) {   // 
      if(!MAKEWORD(VIA1RegR[4],VIA1RegR[5])) {
        if(VIA1RegW[0xe] & 0x40) {
          VIA1RegR[0xd] |= 0xc0;
          DoNMI=1;
          }
        if(VIA1RegR[0xb] & 0x80) {
          // toggle PB7
          }
        VIA1RegR[4]=VIA1RegW[4];
        VIA1RegR[5]=VIA1RegW[5];
        }
      else {
        if(!VIA1RegR[4])
          VIA1RegR[5]--;
        VIA1RegR[4]--;
        }
      }
    if(!(VIA1RegR[0xb] & 0x20)) {   // finire, capire..
      if(!MAKEWORD(VIA1RegR[8],VIA1RegR[9])) {
        if(VIA1RegW[0xe] & 0x20) {
          VIA1RegR[0xd] |= 0xa0;
          }
        VIA1RegR[8]=VIA1RegW[8];
        VIA1RegR[9]=VIA1RegW[9];
        }
      else {
        if(!VIA1RegR[8])
          VIA1RegR[9]--;
        VIA1RegR[8]--;
        }
      }
		switch(VIA1RegR[0xb] & 0b00011100) {
      case 0:
        break;
      default:    // FARE!!! finire
    		VIA1RegR[0xa] <<= 1;
        break;
      }
    
    if(VIA2RegR[0xd] & 0x80)
      DoIRQ=1;
    if(VIA1RegR[0xd] & 0x80)
      DoIRQ=1;
#endif    
    
    LED1 ^= 1;      // ~1uS (CHE CULO!) 8/11/19 ; ~0.75 con ottimizzazione=1 PIC32MZ
    // in effetti la maggior parte delle istruzioni impiega 2-3 cicli, quindi siamo troppo veloci... diciamo di 3x
    // 6uS su PIC32MM @25MHz, 15/6/21
      
		if(debug) {
      
      ram_seg[0x403]=(_pc & 0xf) + 0x30;
      if(ram_seg[0x403] >= 0x3a)
        ram_seg[0x403]-=0x39;
      ram_seg[0x402]=((_pc/16) & 0xf) + 0x30;
      if(ram_seg[0x402] >= 0x3a)
        ram_seg[0x402]-=0x39;
      ram_seg[0x401]=((_pc/256) & 0xf) + 0x30;
      if(ram_seg[0x401] >= 0x3a)
        ram_seg[0x401]-=0x39;
      ram_seg[0x400]=((_pc/4096) & 0xf) + 0x30;
      if(ram_seg[0x400] >= 0x3a)
        ram_seg[0x400]-=0x39;
      }       
//    ram_seg[0xa000]=0;    // *** CI DEV'ESSERE QUALCHE PROBLEMA nel banking, senza una così si pianta... al boot

    
		    
    if(DoHalt) {    // 
      __delay_us(1);     // tanto per :)
      continue;   // chissà se dovrebbe saltare NMI/IRQ o no... boh
      }
      
		switch(GetPipe(_pc++)) {
			case 0:
				stack_seg[_s--]=HIBYTE(_pc);
				stack_seg[_s--]=LOBYTE(_pc);
				_pIRQ=_p;
				_pIRQ.Break=1;
				stack_seg[_s--]=_pIRQ.b;
//		_lwrite(spoolFile,ram_seg,0x400);
        
//		wsprintf(myBuf,"Break a: %04x",_pc);
//		SetWindowText(hStatusWnd,myBuf);
    
				_pc=GetIntValue(0xfffa);
				_p.Interrupt=1;
				break;

			case 1:
				_a |= GetValue(MAKEWORD(ram_seg[Pipe2.bytes.byte1+_x],ram_seg[Pipe2.bytes.byte1+_x+1]));
//				_a |= GetValue(*(SWORD *)(ram_seg+Pipe2.bytes.byte1+_x));
				_pc++;
				goto aggFlagA;
				break;

			case 2:     // halt and catch fire :D opp. SINC
#ifdef CPU_KIMKLONE
        IP++;
#else
//				wsprintf(myBuf,"Istruzione HALT AND CATCH FIRE a %04x: %02x",_pc-1,GetValue(_pc-1));
//				SetWindowText(hStatusWnd,myBuf);
#endif
				break;
        
#ifdef CPU_KIMKLONE
			case 0x3:    // NOP implied 1 cycle
				break;
#endif

#ifdef CPU_65C02
			case 0x04:
				res2.b.l= _a | ram_seg[Pipe2.bytes.byte1];		// verificare...
				ram_seg[Pipe2.bytes.byte1]=i;
				_pc++;
				goto aggTrb;
#endif

			case 5:
				_a |= ram_seg[Pipe2.bytes.byte1];
				_pc++;
				goto aggFlagA;
				break;

			case 6:
				res3.b.l=ram_seg[Pipe2.bytes.byte1];
				res3.w <<= 1;
				_p.Carry=res3.b.h & 1;
				ram_seg[Pipe2.bytes.byte1]=res3.b.l;
				_pc++;
				goto aggFlagI;
				break;

#ifdef CPU_65C02		// rockwell
			case 7:
				ram_seg[Pipe2.bytes.byte1] &= ~0x1;
				_pc++;
				break;
#endif

			case 8:
				_pIRQ=_p;
				_pIRQ.Break=0;
				stack_seg[_s--]=_pIRQ.b;
				break;

			case 9:
				_a |= Pipe2.bytes.byte1;
				_pc++;
				goto aggFlagA;
				break;

			case 0xa:
				if(_a & 0x80)
					_p.Carry=1;
				else
					_p.Carry=0;
				_a <<= 1;
				goto aggFlagA;
				break;

#ifdef CPU_KIMKLONE
			case 0x0b:    // LDK2
				break;
#endif

#ifdef CPU_65C02
			case 0x0c:
				res2.b.l= _a | GetValue(Pipe2.word);		// verificare...
				PutValue(Pipe2.word,res2.b.l);
				_pc+=2;
				goto aggTrb;
#endif

			case 0xd:
				_a |= GetValue(Pipe2.word);
				_pc+=2;
				goto aggFlagA;
				break;

			case 0xe:
				res3.b.l=GetValue(Pipe2.word);
				res3.w <<= 1;
				_p.Carry=res3.b.h & 1;
				PutValue(Pipe2.word,res3.b.l);
				_pc+=2;
				goto aggFlagI;
				break;

#ifdef CPU_65C02		// rockwell
			case 0x0f:
				if(!(ram_seg[Pipe2.bytes.byte1] & 0x1))
					_pc+=(signed char)Pipe2.bytes.byte2;
				_pc=2;
				break;
#endif

			case 0x10:
				if(!_p.Minus)
					_pc+=(signed char)Pipe2.bytes.byte1;
				_pc++;
				break;

			case 0x11:
				_a |= GetValue(MAKEWORD(ram_seg[Pipe2.bytes.byte1],ram_seg[Pipe2.bytes.byte1+1])+_y);
//				_a |= GetValue(*(SWORD *)(ram_seg+Pipe2.bytes.byte1)+_y);
				_pc++;
				goto aggFlagA;
				break;

#ifdef CPU_65C02
			case 0x12:
				_a |= GetValue(MAKEWORD(ram_seg[Pipe2.bytes.byte1],ram_seg[Pipe2.bytes.byte1+1]));
				_pc++;
				goto aggFlagA;
				break;

			case 0x14:
				res2.b.l= (_a ^ 0xff) & ram_seg[Pipe2.bytes.byte1];		// verificare...
				ram_seg[Pipe2.bytes.byte1] = i;
				_pc++;
aggTrb:
				_p.Zero=!res2.b.l;
				break;
#endif
#ifdef CPU_KIMKLONE    //https://laughtonelectronics.com/Arcana/KimKlone/KK%20Instruction%20List.html
			case 0x13:    // JMP_K3
				break;
#endif

			case 0x15:
				_a |= ram_seg[Pipe2.bytes.byte1+_x];
				_pc++;
				goto aggFlagA;
				break;

			case 0x16:
				res3.b.l=ram_seg[Pipe2.bytes.byte1+_x];
				res3.w <<= 1;
				_p.Carry=res3.b.h & 1;
				ram_seg[Pipe2.bytes.byte1+_x]=res3.b.l;
				_pc++;
				goto aggFlagI;
				break;

#ifdef CPU_65C02		// rockwell
			case 0x17:
				ram_seg[Pipe2.bytes.byte1] &= ~0x2;
				_pc++;
				break;
#endif

			case 0x18:
				_p.Carry=0;
				break;

			case 0x19:
				_a |= GetValue(Pipe2.word+_y);
				_pc+=2;
				goto aggFlagA;
				break;

#ifdef CPU_65C02
			case 0x1a:
				_a++;
				goto aggFlagA;
				break;

			case 0x1c:
				res2.b.l= (_a ^ 0xff) & GetValue(Pipe2.word);		// verificare...
				PutValue(Pipe2.word,res2.b.l);
				_pc++;
				goto aggTrb;
				break;
#endif

#ifdef CPU_KIMKLONE
			case 0x1b:    // PLYIPL
				break;
#endif

			case 0x1d:
				_a |= GetValue(Pipe2.word+_x);
				_pc+=2;
				goto aggFlagA;
				break;

			case 0x1e:
				res3.b.l=GetValue(Pipe2.word+_x);
				res3.w <<= 1;
				_p.Carry=res3.b.h & 1;
				PutValue(Pipe2.word+_x,res3.b.l);
				_pc+=2;
				goto aggFlagI;
				break;

#ifdef CPU_65C02		// rockwell
			case 0x1f:
				if(!(ram_seg[Pipe2.bytes.byte1] & 0x2))
					_pc+=(signed char)Pipe2.bytes.byte2;
				_pc=2;
				break;
#endif

			case 0x20:
				stack_seg[_s--]=HIBYTE(++_pc);      // v. RTS...
				stack_seg[_s--]=LOBYTE(_pc);
				_pc=Pipe2.word;
				break;

			case 0x21:
				_a &= GetValue(MAKEWORD(ram_seg[Pipe2.bytes.byte1+_x],ram_seg[Pipe2.bytes.byte1+_x+1]));
//				_a &= GetValue(*(SWORD *)(ram_seg+Pipe2.bytes.byte1+_x));
    		_pc++;
				goto aggFlagA;
				break;

#ifdef CPU_KIMKLONE    //https://laughtonelectronics.com/Arcana/KimKlone/KK%20Instruction%20List.html
      case 0x22:    // DINC
        IP+=2;
        break;
        
			case 0x23:    // JSR_K3
				break;
#endif

			case 0x24:    // BIT zp
				res3.b.l = ram_seg[Pipe2.bytes.byte1];
				_pc++;
aggBit:
				_p.Minus=res3.b.l & 0x80 ? 1 : 0;
				_p.Overflow=res3.b.l & 0x40 ? 1 : 0;
				_p.Zero=!res3.b.l;
				break;

			case 0x25:
				_a &= ram_seg[Pipe2.bytes.byte1];
				_pc++;
				goto aggFlagA;
				break;

			case 0x26:
				res3.b.l=ram_seg[Pipe2.bytes.byte1];
				_p1=_p;
				res3.w <<= 1;
				_p.Carry=res3.b.h & 1;
				res3.b.l |= _p1.Carry;
				ram_seg[Pipe2.bytes.byte1]=res3.b.l;
				_pc++;
				goto aggFlagI;
				break;

#ifdef CPU_65C02		// rockwell
			case 27:
				ram_seg[Pipe2.bytes.byte1] &= ~0x4;
				_pc++;
				break;
#endif

			case 0x28:
				_p.b=stack_seg[++_s];
				break;

			case 0x29:
				_a &= Pipe2.bytes.byte1;
				_pc++;
				goto aggFlagA;
				break;

			case 0x2a:
				_p1=_p;
				if(_a & 0x80)
					_p.Carry=1;
				else
					_p.Carry=0;
				_a <<= 1;
				_a |= _p1.Carry;
				goto aggFlagA;
				break;

#ifdef CPU_KIMKLONE
			case 0x2b:    // PLYIPH
				break;
#endif

			case 0x2c:    // BIT abs
				res3.b.l = GetValue(Pipe2.word);
				_pc+=2;
				goto aggBit;
				break;

			case 0x2d:
				_a &= GetValue(Pipe2.word);
				_pc+=2;
				goto aggFlagA;
				break;

			case 0x2e:
				_p1=_p;
				res3.b.l=GetValue(Pipe2.word);
				res3.w <<= 1;
				_p.Carry=res3.b.h & 1;
				res3.b.l |= _p1.Carry;
				PutValue(Pipe2.word,res3.b.l);
				_pc+=2;
				goto aggFlagI;
				break;

#ifdef CPU_65C02		// rockwell
			case 0x2f:
				if(!(ram_seg[Pipe2.bytes.byte1] & 0x4))
					_pc+=(signed char)Pipe2.bytes.byte2;
				_pc=2;
				break;
#endif

			case 0x30:
				if(_p.Minus)
					_pc+=(signed char)Pipe2.bytes.byte1;
				_pc++;
				break;

			case 0x31:
				_a &= GetValue(MAKEWORD(ram_seg[Pipe2.bytes.byte1],ram_seg[Pipe2.bytes.byte1+1])+_y);
				_pc++;
				goto aggFlagA;
				break;

#ifdef CPU_65C02
			case 0x32:
				_a &= GetValue(MAKEWORD(ram_seg[Pipe2.bytes.byte1],ram_seg[Pipe2.bytes.byte1+1]));
				_pc++;
				goto aggFlagA;
				break;

			case 0x34:
				res3.b.l = ram_seg[Pipe2.bytes.byte1+_x];
				_pc++;
				goto aggBit;
				break;
#endif

#ifdef CPU_KIMKLONE
			case 0x33:    // SCAN_K3
				break;
#endif

			case 0x35:
				_a &= ram_seg[Pipe2.bytes.byte1+_x];
				_pc++;
				goto aggFlagA;
				break;

			case 0x36:
				_p1=_p;
				res3.b.l=ram_seg[Pipe2.bytes.byte1+_x];
				res3.w <<= 1;
				_p.Carry=res3.b.h & 1;
				res3.b.l |= _p1.Carry;
				ram_seg[Pipe2.bytes.byte1+_x]=res3.b.l;
				_pc++;
				goto aggFlagI;
				break;

#ifdef CPU_65C02		// rockwell
			case 0x37:
				ram_seg[Pipe2.bytes.byte1] &= ~0x8;
				_pc++;
				break;
#endif

			case 0x38:
				_p.Carry=1;
				break;

			case 0x39:
				_a &= GetValue(Pipe2.word+_y);
				_pc+=2;
				goto aggFlagA;
				break;

#ifdef CPU_65C02
			case 0x3a:
				_a--;
				goto aggFlagA;
				break;

			case 0x3c:
				res3.b.l = GetValue(Pipe2.word+_x);
				_pc+=2;
				goto aggBit;
				break;
#endif
        
#ifdef CPU_KIMKLONE
			case 0x3b:    // NEXT
				break;
#endif


			case 0x3d:
				_a &= GetValue(Pipe2.word+_x);
				_pc+=2;
				goto aggFlagA;
				break;

			case 0x3e:
				_p1=_p;
				res3.b.l=GetValue(Pipe2.word+_x);
				res3.w <<= 1;
				_p.Carry=res3.b.h & 1;
				res3.b.l |= _p1.Carry;
				PutValue(Pipe2.word+_x,res3.b.l);
				_pc+=2;
				goto aggFlagI;
				break;

#ifdef CPU_65C02		// rockwell
			case 0x3f:
				if(!(ram_seg[Pipe2.bytes.byte1] & 0x8))
					_pc+=(signed char)Pipe2.bytes.byte2;
				_pc=2;
				break;
#endif

			case 0x40:
				_p.b = stack_seg[++_s];
				_pc=stack_seg[++_s];
				_pc |= ((short int)stack_seg[++_s]) << 8;
				_p.Break=0; _p.unused=1;    // meglio, viste le specifiche :)
				break;

			case 0x41:
				_a ^= GetValue(MAKEWORD(ram_seg[Pipe2.bytes.byte1+_x],ram_seg[Pipe2.bytes.byte1+_x+1]));
				_pc++;
				goto aggFlagA;
				break;

#ifdef CPU_KIMKLONE
			case 0x42:    // LDK1
				break;

			case 0x43:    // K3_
				break;

			case 0x44:    // LDK2
				break;
#endif

			case 0x45:
				_a ^= ram_seg[Pipe2.bytes.byte1];
				_pc++;
				goto aggFlagA;
				break;

			case 0x46:
				res3.b.l=ram_seg[Pipe2.bytes.byte1];
				_p.Carry=res3.b.l & 1;
				res3.b.l >>= 1;
				ram_seg[Pipe2.bytes.byte1]=res3.b.l;
				_pc++;
				goto aggFlagI;
				break;

#ifdef CPU_65C02		// rockwell
			case 0x47:
				ram_seg[Pipe2.bytes.byte1] &= ~0x10;
				_pc++;
				break;
#endif

			case 0x48:
				stack_seg[_s--]=_a;
				break;

			case 0x49:
				_a ^= Pipe2.bytes.byte1;
				_pc++;
				goto aggFlagA;
				break;

			case 0x4a:
				_p.Carry=_a & 0x1;
				_a >>= 1;
				goto aggFlagA;
				break;

#ifdef CPU_KIMKLONE
			case 0x4b:    // PUSHK0
				break;
#endif

			case 0x4c:
				_pc=Pipe2.word;
				break;

			case 0x4d:
				_a ^= GetValue(Pipe2.word);
				_pc+=2;
				goto aggFlagA;
				break;

			case 0x4e:
				res3.b.l=GetValue(Pipe2.word);
				_p.Carry=res3.b.l & 1;
				res3.b.l >>= 1;
				PutValue(Pipe2.word,res3.b.l);
				_pc+=2;
				goto aggFlagI;
				break;

#ifdef CPU_65C02		// rockwell
			case 0x4f:
				if(!(ram_seg[Pipe2.bytes.byte1] & 0x10))
					_pc+=(signed char)Pipe2.bytes.byte2;
				_pc=2;
				break;
#endif

			case 0x50:
				if(!_p.Overflow)
					_pc+=(signed char)Pipe2.bytes.byte1;
				_pc++;
				break;

			case 0x51:
				_a ^= GetValue(MAKEWORD(ram_seg[Pipe2.bytes.byte1],ram_seg[Pipe2.bytes.byte1+1])+_y);
				_pc++;
				goto aggFlagA;
				break;

#ifdef CPU_65C02
			case 0x52:
				_a ^= GetValue(MAKEWORD(ram_seg[Pipe2.bytes.byte1],ram_seg[Pipe2.bytes.byte1+1]));
				_pc++;
				goto aggFlagA;
				break;
#endif

#ifdef CPU_KIMKLONE
			case 0x53:    // PLYK3
				break;
			case 0x54:    // LDK1
				break;
#endif

			case 0x55:
				_a ^= ram_seg[Pipe2.bytes.byte1+_x];
				_pc++;
				goto aggFlagA;
				break;

			case 0x56:
				res3.b.l=ram_seg[Pipe2.bytes.byte1+_x];
				_p.Carry=res3.b.l & 1;
				res3.b.l >>= 1;
				ram_seg[Pipe2.bytes.byte1+_x]=res3.b.l;
				_pc++;
				goto aggFlagI;
				break;

#ifdef CPU_65C02		// rockwell
			case 0x57:
				ram_seg[Pipe2.bytes.byte1] &= ~0x20;
				_pc++;
				break;
#endif

			case 0x58:
				_p.Interrupt=0;
				break;

			case 0x59:
				_a ^= GetValue(Pipe2.word+_y);
				_pc+=2;
				goto aggFlagA;
				break;

#ifdef CPU_65C02
			case 0x5a:
				stack_seg[_s--]=_y;
				break;
#endif

#ifdef CPU_KIMKLONE
			case 0x5b:    // PUSHK3
				break;
#endif

#ifdef CPU_KIMKLONE
			case 0x5c:    // K0<>K3
				break;
#endif

			case 0x5d:
				_a ^= GetValue(Pipe2.word+_x);
				_pc+=2;
				goto aggFlagA;
				break;

			case 0x5e:
				res3.b.l=GetValue(Pipe2.word+_x);
				_p.Carry=res3.b.l & 1;
				res3.b.l >>= 1;
				PutValue(Pipe2.word+_x,res3.b.l);
				_pc+=2;
				goto aggFlagI;
				break;

#ifdef CPU_65C02		// rockwell
			case 0x5f:
				if(!(ram_seg[Pipe2.bytes.byte1] & 0x20))
					_pc+=(signed char)Pipe2.bytes.byte2;
				_pc=2;
				break;
#endif

			case 0x60:
				_pc=stack_seg[++_s];
				_pc |= ((short int)stack_seg[++_s]) << 8;
				_pc++;
				break;

			case 0x61:
				res2.b.l=GetValue(MAKEWORD(ram_seg[Pipe2.bytes.byte1+_x],ram_seg[Pipe2.bytes.byte1+_x+1]));
				_pc++;
				goto aggSomma;
				break;

#ifdef CPU_KIMKLONE
			case 0x63:    // PLYK1
				break;
#endif

#ifdef CPU_65C02
			case 0x64:
				PutValue(Pipe2.bytes.byte1,0);
				_pc++;
				break;
#endif

			case 0x65:
				res2.b.l=ram_seg[Pipe2.bytes.byte1];
//				res3.w = (SWORD)_a+ram_seg[Pipe2.bytes.byte1];
				_pc++;
				goto aggSomma;
				break;

			case 0x66:
				res3.b.l=ram_seg[Pipe2.bytes.byte1];
				_p1=_p;
				_p.Carry=res3.b.l & 1;
				res3.b.l >>= 1;
				if(_p1.Carry)
					res3.b.l |= 0x80;
				else
					res3.b.l &= 0x7f;
				ram_seg[Pipe2.bytes.byte1]=res3.b.l;
				_pc++;
				goto aggFlagI;
				break;

#ifdef CPU_65C02		// rockwell
			case 0x67:
				ram_seg[Pipe2.bytes.byte1] &= ~0x40;
				_pc++;
				break;
#endif

			case 0x68:
				_a=stack_seg[++_s];
				goto aggFlagA;
				break;

			case 0x69:
				res2.b.l=Pipe2.bytes.byte1;
				_pc++;
        
aggSomma:
				res1.b.l=_a;
        res1.b.h=res2.b.h=0;
				res3.w = res1.w+res2.w;
#warning SOMMARE CARRY direttamente??
				if(_p.Carry)
					res3.w++;
				_a=res3.b.l;
				_p.Carry=res3.b.h & 1;
//				_p.Overflow=((res1.b.l ^ res2.b.l) & 0x80) ? 1 : 0;		
//        _p.Overflow = !!(res1.b.l & 0x40 + res2.b.l & 0x40) != !!(res1.b.l & 0x80 + res2.b.l & 0x80);
//        _p.Overflow = ((res1.b.l ^ res3.b.l) & (res2.b.l ^ res3.b.l) & 0x80) ? 1 : 0;
//        _p.Overflow = !!(((res1.b.l & 0x40) + (res2.b.l & 0x40)) & 0x80) != !!(((res1.w & 0x80) + (res2.w & 0x80)) & 0x100);
				/*For addition, operands with different signs never cause overflow. When adding operands
with similar signs and the result contains a different sign, the Overflow Flag is set, as
shown in the following example.
The two numbers added together result in a number that exceeds +127 and the two posi-
tive operands result in a negative number (–95), which is incorrect. The Overflow Flag is
therefore set.
For subtraction, overflow can occur for operands of unalike signs. Operands of alike signs
never cause overflow, as shown in the following example.
The minuend sign has changed from a positive to a negative, resulting in an incorrect dif-
ference; the Overflow Flag is set.*/
				/*Overflow can be computed simply in C++ from the inputs and the result. 
				Overflow occurs if (M^result)&(N^result)&0x80 is nonzero. 
				That is, if the sign of both inputs is different from the sign of the result. 
				(Anding with 0x80 extracts just the sign bit from the result.) 
				Another C++ formula is !((M^N) & 0x80) && ((M^result) & 0x80). 
				This means there is overflow if the inputs do not have different signs and the input sign is different from the output sign */ 
        
        _p.Overflow = !!res3.b.h != !!((res3.b.l & 0x80) ^ (res1.b.l & 0x80) ^ (res2.b.l & 0x80));
        
aggFlagA:
				_p.Zero=_a ? 0 : 1;
				_p.Minus=_a & 0x80 ? 1 : 0;

				break;

			case 0x6a:
				_p1=_p;
				_p.Carry=_a & 0x1;
				_a >>= 1;
				if(_p1.Carry)
					_a |= 0x80;
				else
					_a &= 0x7f;
				goto aggFlagA;
				break;

#ifdef CPU_KIMKLONE
			case 0x6b:    // PUSHK1
				break;
#endif

			case 0x6c:
				_pc=GetIntValue(Pipe2.word);
//      printf("JMP (%04x)=%04x\n",i,_pc);
				break;

			case 0x6d:
				res2.b.l=GetValue(Pipe2.word);
				_pc+=2;
				goto aggSomma;
				break;

			case 0x6e:
				_p1=_p;
				res3.b.l=GetValue(Pipe2.word);
				_p.Carry=res3.b.l & 0x1;
				res3.b.l >>= 1;
				if(_p1.Carry)
					res3.b.l |= 0x80;
				else
					res3.b.l &= 0x7f;
				PutValue(Pipe2.word,res3.b.l);
				_pc+=2;
				goto aggFlagI;
				break;

#ifdef CPU_65C02		// rockwell
			case 0x6f:
				if(!(ram_seg[Pipe2.bytes.byte1] & 0x40))
					_pc+=(signed char)Pipe2.bytes.byte2;
				_pc=2;
				break;
#endif

			case 0x70:
				if(_p.Overflow)
					_pc+=(signed char)Pipe2.bytes.byte1;
				_pc++;
				break;

			case 0x71:
				res2.b.l=GetValue(MAKEWORD(ram_seg[Pipe2.bytes.byte1],ram_seg[Pipe2.bytes.byte1+1])+_y);
				_pc++;
				goto aggSomma;
				break;

#ifdef CPU_65C02
			case 0x72:
				res2.b.l=_a + GetValue(MAKEWORD(ram_seg[Pipe2.bytes.byte1],ram_seg[Pipe2.bytes.byte1+1]));
				_pc++;
				goto aggSomma;
				break;

			case 0x74:
				ram_seg[Pipe2.bytes.byte1+_x]=0;
				_pc++;
				break;
#endif

#ifdef CPU_KIMKLONE
			case 0x73:    // PLYK2
				break;
#endif

			case 0x75:
				res2.b.l=ram_seg[Pipe2.bytes.byte1+_x];
				_pc++;
				goto aggSomma;
				break;

			case 0x76:
				_p1=_p;
				res3.b.l=ram_seg[Pipe2.bytes.byte1+_x];
				_p.Carry=res3.b.l & 0x1;
				res3.b.l >>= 1;
				if(_p1.Carry)
					res3.b.l |= 0x80;
				else
					res3.b.l &= 0x7f;
				ram_seg[Pipe2.bytes.byte1+_x]=res3.b.l;
				_pc++;
				goto aggFlagI;
				break;

#ifdef CPU_65C02		// rockwell
			case 0x77:
				ram_seg[Pipe2.bytes.byte1] &= ~0x80;
				_pc++;
				break;
#endif

			case 0x78:
				_p.Interrupt=1;
				break;

			case 0x79:
				res2.b.l=GetValue(Pipe2.word+_y);
				_pc+=2;
				goto aggSomma;
				break;

#ifdef CPU_65C02
			case 0x7a:
				_y=stack_seg[++_s];
				goto aggFlagY;
				break;

			case 0x7c:
				_pc=GetIntValue(Pipe2.word+_x);
				break;
#endif

#ifdef CPU_KIMKLONE
			case 0x7b:    // PUSHK2
				break;
#endif

			case 0x7d:
				res2.b.l=GetValue(Pipe2.word+_x);
				_pc+=2;
				goto aggSomma;
				break;

			case 0x7e:
				_p1=_p;
				res3.b.l=GetValue(Pipe2.word+_x);
				_p.Carry=res3.b.l & 0x1;
				res3.b.l >>= 1;
				if(_p1.Carry)
					res3.b.l |= 0x80;
				else
					res3.b.l &= 0x7f;
				PutValue(Pipe2.word+_x,res3.b.l);
				_pc+=2;
				goto aggFlagI;
				break;

#ifdef CPU_65C02
			// rockwell
			case 0x7f:
				if(!(ram_seg[Pipe2.bytes.byte1] & 0x80))
					_pc+=(signed char)Pipe2.bytes.byte2;
				_pc=2;
				break;

			case 0x80:
				_pc+=(signed char)Pipe2.bytes.byte1+2;
				break;
#endif

			case 0x81:
				PutValue(MAKEWORD(ram_seg[Pipe2.bytes.byte1+_x],ram_seg[Pipe2.bytes.byte1+_x+1]),_a);
				_pc++;
				break;

#ifdef CPU_KIMKLONE
			case 0x83:    // K1_
				break;
#endif

			case 0x84:
				ram_seg[Pipe2.bytes.byte1]=_y;
				_pc++;
				break;

			case 0x85:
				ram_seg[Pipe2.bytes.byte1]=_a;
				_pc++;
				break;

			case 0x86:
				ram_seg[Pipe2.bytes.byte1]=_x;
				_pc++;
				break;

#ifdef CPU_65C02		// rockwell
			case 0x87:
				ram_seg[Pipe2.bytes.byte1] |= 0x1;
				_pc++;
				break;
#endif

			case 0x88:
				_y--;
				goto aggFlagY;
				break;

#ifdef CPU_65C02
			case 0x89:
				res3.b.l = Pipe2.bytes.byte1;
				_pc++;
				goto aggBit;		// OCCHIO AI flag, dice qua http://6502.org/tutorials/65c02opcodes.html
				break;
#endif

			case 0x8a:
				_a=_x;
				goto aggFlagA;
				break;

#ifdef CPU_KIMKLONE
			case 0x8b:    // TSB_K1
				break;
#endif

			case 0x8c:
				PutValue(Pipe2.word,_y);
				_pc+=2;
				break;

			case 0x8d:
				PutValue(Pipe2.word,_a);
				_pc+=2;
				break;

			case 0x8e:
				PutValue(Pipe2.word,_x);
				_pc+=2;
				break;

#ifdef CPU_65C02		// rockwell
			case 0x8f:
				if(ram_seg[Pipe2.bytes.byte1] & 0x1)
					_pc+=(signed char)Pipe2.bytes.byte2;
				_pc=2;
				break;
#endif

			case 0x90:
				if(!_p.Carry)
					_pc+=(signed char)Pipe2.bytes.byte1;
				_pc++;
				break;

			case 0x91:
				PutValue(MAKEWORD(ram_seg[Pipe2.bytes.byte1],ram_seg[Pipe2.bytes.byte1+1])+_y,_a);
				_pc++;
				break;

#ifdef CPU_65C02
			case 0x92:
				PutValue(MAKEWORD(ram_seg[Pipe2.bytes.byte1],ram_seg[Pipe2.bytes.byte1+1]),_a);
				_pc++;
				break;
#endif

#ifdef CPU_KIMKLONE
			case 0x93:    // STA_K1
				break;
#endif

			case 0x94:
				ram_seg[Pipe2.bytes.byte1+_x]=_y;
				_pc++;
				break;

			case 0x95:
				ram_seg[Pipe2.bytes.byte1+_x]=_a;
				_pc++;
				break;

			case 0x96:
				ram_seg[Pipe2.bytes.byte1+_y]=_x;
				_pc++;
				break;

#ifdef CPU_65C02		// rockwell
			case 0x97:
				ram_seg[Pipe2.bytes.byte1] |= 0x2;
				_pc++;
				break;
#endif

			case 0x98:
				_a=_y;
				goto aggFlagA;
				break;

			case 0x99:
				PutValue(Pipe2.word+_y,_a);
				_pc+=2;
				break;

			case 0x9a:
				_s=_x;
				break;

#ifdef CPU_KIMKLONE
			case 0x9b:    // TRB_K1
				break;
#endif

#ifdef CPU_65C02
			case 0x9c:
				PutValue(Pipe2.word,0);
				_pc+=2;
				break;
#endif

			case 0x9d:
				PutValue(Pipe2.word+_x,_a);
				_pc+=2;
				break;

#ifdef CPU_65C02
			case 0x9e:
				PutValue(Pipe2.word+_x,0);
				_pc+=2;
				break;

			// rockwell
			case 0x9f:
				if(ram_seg[Pipe2.bytes.byte1] & 0x2)
					_pc+=(signed char)Pipe2.bytes.byte2;
				_pc=2;
				break;
#endif

			case 0xa0:
				_y=Pipe2.bytes.byte1;
				_pc++;
				goto aggFlagY;
				break;

			case 0xa1:
				_a = GetValue(MAKEWORD(ram_seg[Pipe2.bytes.byte1+_x],ram_seg[Pipe2.bytes.byte1+_x+1]));
				_pc++;
				goto aggFlagA;
				break;

			case 0xa2:
				_x=Pipe2.bytes.byte1;
				_pc++;
				goto aggFlagX;
				break;

#ifdef CPU_KIMKLONE
			case 0xa3:    // STA_K1
				break;
#endif

			case 0xa4:
				_y=ram_seg[Pipe2.bytes.byte1];
				_pc++;
aggFlagY:
				_p.Zero=_y ? 0 : 1;
				_p.Minus=_y & 0x80 ? 1 : 0;
				break;

			case 0xa5:
				_a=ram_seg[Pipe2.bytes.byte1];
				_pc++;
				goto aggFlagA;
				break;

			case 0xa6:
				_x=ram_seg[Pipe2.bytes.byte1];
				_pc++;
aggFlagX:
				_p.Zero=_x ? 0 : 1;
				_p.Minus=_x & 0x80 ? 1 : 0;
				break;

#ifdef CPU_65C02		// rockwell
			case 0xa7:
				ram_seg[Pipe2.bytes.byte1] |= 0x4;
				_pc++;
				break;
#endif

			case 0xa8:
				_y=_a;
				goto aggFlagY;
				break;

			case 0xa9:
				_a=Pipe2.bytes.byte1;
				_pc++;
        goto aggFlagA;
				break;

			case 0xaa:
				_x=_a;
				goto aggFlagX;
				break;

#ifdef CPU_KIMKLONE
			case 0xab:    // TSB_K1
				break;
#endif

			case 0xac:
				_y=GetValue(Pipe2.word);
				_pc+=2;
				goto aggFlagY;
				break;

			case 0xad:
				_a=GetValue(Pipe2.word);
				_pc+=2;
				goto aggFlagA;
				break;

			case 0xae:
				_x=GetValue(Pipe2.word);
				_pc+=2;
				goto aggFlagX;
				break;

#ifdef CPU_65C02		// rockwell
			case 0xaf:
				if(ram_seg[Pipe2.bytes.byte1] & 0x4)
					_pc+=(signed char)Pipe2.bytes.byte2;
				_pc=2;
				break;
#endif

			case 0xb0:
				if(_p.Carry)
					_pc+=(signed char)Pipe2.bytes.byte1;
				_pc++;
				break;

			case 0xb1:
				_a = GetValue(MAKEWORD(ram_seg[Pipe2.bytes.byte1],ram_seg[Pipe2.bytes.byte1+1])+_y);
				_pc++;
				goto aggFlagA;
				break;

#ifdef CPU_65C02
			case 0xb2:
				_a = GetValue(MAKEWORD(ram_seg[Pipe2.bytes.byte1],ram_seg[Pipe2.bytes.byte1+1]));
				_pc++;
				break;
#endif

#ifdef CPU_KIMKLONE
			case 0xb3:    // STA_K1
				break;
#endif

			case 0xb4:
				_y=ram_seg[Pipe2.bytes.byte1+_x];
				_pc++;
				goto aggFlagY;
				break;

			case 0xb5:
				_a=ram_seg[Pipe2.bytes.byte1+_x];
				_pc++;
				goto aggFlagA;
				break;

			case 0xb6:
				_x=ram_seg[Pipe2.bytes.byte1+_y];
				_pc++;
				goto aggFlagX;
				break;

#ifdef CPU_65C02		// rockwell
			case 0xb7:
				ram_seg[Pipe2.bytes.byte1] |= 0x8;
				_pc++;
				break;
#endif

			case 0xb8:
				_p.Overflow=0;
				break;

			case 0xb9:
				_a = GetValue(Pipe2.word+_y);
				_pc+=2;
				goto aggFlagA;
				break;

			case 0xba:
				_x=_s;
				goto aggFlagX;
				break;

#ifdef CPU_KIMKLONE
			case 0xbb:    // TRB_K1
				break;
#endif

			case 0xbc:
				_y = GetValue(Pipe2.word+_x);
				_pc+=2;
				goto aggFlagY;
				break;

			case 0xbd:
				_a = GetValue(Pipe2.word+_x);
				_pc+=2;
				goto aggFlagA;
				break;

			case 0xbe:
				_x = GetValue(Pipe2.word+_y);
				_pc+=2;
				goto aggFlagX;
				break;

#ifdef CPU_65C02		// rockwell
			case 0xbf:
				if(ram_seg[Pipe2.bytes.byte1] & 0x8)
					_pc+=(signed char)Pipe2.bytes.byte2;
				_pc=2;
				break;
#endif

			case 0xc0:
				res3.w=(SWORD)_y-Pipe2.bytes.byte1;
				_pc++;
				goto aggFlagC;
				break;

			case 0xc1:
				res3.w=(SWORD)_a-GetValue(MAKEWORD(ram_seg[Pipe2.bytes.byte1+_x],ram_seg[Pipe2.bytes.byte1+_x+1]));
				_pc++;
				goto aggFlagC;
				break;

#ifdef CPU_KIMKLONE
			case 0xc2:    // LDK2
				break;

			case 0xc3:    // K2_
				break;
#endif

			case 0xc4:
				res3.w=(SWORD)_y-ram_seg[Pipe2.bytes.byte1];
//				wsprintf(myBuf,"%04X: confronto y %02x con %02x (all'ind. %02X), ottengo: %02X\r\n",_pc,_y,ram_seg[Pipe2.bytes.byte1],Pipe2.bytes.byte1,i);
//		_lwrite(spoolFile,myBuf,strlen(myBuf));
				_pc++;
				goto aggFlagC;
				break;

			case 0xc5:
				res3.w=(SWORD)_a-ram_seg[Pipe2.bytes.byte1];
				_pc++;
				goto aggFlagC;
				break;

			case 0xc6:
				res3.b.l=--ram_seg[Pipe2.bytes.byte1];
				_pc++;
				goto aggFlagI;
				break;

#ifdef CPU_65C02		// rockwell
			case 0xc7:
				ram_seg[Pipe2.bytes.byte1] |= 0x10;
				_pc++;
				break;
#endif

			case 0xc8:
				_y++;
//      printf("Y=%02x, P=%02x\n",_y,_p);
				goto aggFlagY;
				break;

			case 0xc9:
				res3.w=(SWORD)_a-Pipe2.bytes.byte1;
				_pc++;
        
aggFlagC:
				_p.Carry=!res3.b.h;

aggFlagI:
				_p.Zero=!res3.b.l;
				_p.Minus=(res3.b.l & 0x80) ? 1 : 0;
				break;

			case 0xca:
				_x--;
				goto aggFlagX;
				break;

#ifdef CPU_65C02		// rockwell
			case 0xcb:			//WAI
				// while(!doReset && !doNMI && !doIRQ) ClrWdt();
				break;
#endif
#ifdef CPU_KIMKLONE
			case 0xcb:    // LDAW
				break;
#endif

			case 0xcc:
				res3.w=(SWORD)_y-GetValue(Pipe2.word);
				_pc+=2;
				goto aggFlagC;
				break;

			case 0xcd:
				res3.w=(SWORD)_a-GetValue(Pipe2.word);
				_pc+=2;
				goto aggFlagC;
				break;

			case 0xce:
				res3.b.l=GetValue(Pipe2.word);
				PutValue(Pipe2.word,--res3.b.l);
				_pc+=2;
				goto aggFlagI;
				break;

#ifdef CPU_65C02		// rockwell
			case 0xcf:
				if(ram_seg[Pipe2.bytes.byte1] & 0x10)
					_pc+=(signed char)Pipe2.bytes.byte2;
				_pc=2;
				break;
#endif

			case 0xd0:
				if(!_p.Zero)
					_pc+=(signed char)Pipe2.bytes.byte1;
				_pc++;
				break;

			case 0xd1:
				res3.w=(SWORD)_a-GetValue(MAKEWORD(ram_seg[Pipe2.bytes.byte1],ram_seg[Pipe2.bytes.byte1+1])+_y);
				_pc++;
				goto aggFlagC;
				break;

#ifdef CPU_65C02
			case 0xd2:
				res3.w=(SWORD)_a-GetValue(MAKEWORD(ram_seg[Pipe2.bytes.byte1],ram_seg[Pipe2.bytes.byte1+1]));
				_pc++;
				goto aggFlagC;
				break;
#endif

#ifdef CPU_KIMKLONE
			case 0xd3:    // LDA_K2
				break;
			case 0xd4:    // LDK2
				break;
#endif

			case 0xd5:
				res3.w=(SWORD)_a-ram_seg[Pipe2.bytes.byte1+_x];
				_pc++;
				goto aggFlagC;
				break;

			case 0xd6:
				res3.b.l=--ram_seg[Pipe2.bytes.byte1+_x];
				_pc++;
				goto aggFlagI;
				break;

#ifdef CPU_65C02		// rockwell
			case 0xd7:
				ram_seg[Pipe2.bytes.byte1] |= 0x20;
				_pc++;
				break;
#endif

			case 0xd8:
				_p.Decimal=0;
				break;

			case 0xd9:
				res3.w=(SWORD)_a - GetValue(Pipe2.word+_y);
				_pc+=2;
				goto aggFlagC;
				break;

#ifdef CPU_65C02
			case 0xda:
				stack_seg[_s--]=_x;
				break;

			case 0xdb: //STP
				// while(!doReset) ClrWdt();
				break;
#endif

#ifdef CPU_KIMKLONE
			case 0xdb:    //STAW
				break;
			case 0xdc:    // LDK1
				break;
#endif

			case 0xdd:
				res3.w=(SWORD)_a - GetValue(Pipe2.word+_x);
				_pc+=2;
				goto aggFlagC;
				break;

			case 0xde:
				res3.b.l=GetValue(Pipe2.word+_x);
				PutValue(Pipe2.word+_x,--res3.b.l);
				_pc+=2;
				goto aggFlagI;
				break;

#ifdef CPU_65C02		// rockwell
			case 0xdf:
				if(ram_seg[Pipe2.bytes.byte1] & 0x20)
					_pc+=(signed char)Pipe2.bytes.byte2;
				_pc=2;
				break;
#endif

			case 0xe0:
				res3.w=(SWORD)_x-Pipe2.bytes.byte1;
				_pc++;
				goto aggFlagC;
				break;

			case 0xe1:
				res2.b.l=GetValue(MAKEWORD(ram_seg[Pipe2.bytes.byte1+_x],ram_seg[Pipe2.bytes.byte1+_x+1]));
				_pc++;
				goto aggSottr;
				break;

#ifdef CPU_KIMKLONE
			case 0xe2:    // LDK3
				break;
			case 0xe3:    // LDA_K2
				break;
#endif

			case 0xe4:
				res3.w=(SWORD)_x - GetValue(((SWORD)Pipe2.bytes.byte1));
				_pc++;
				goto aggFlagC;
				break;

			case 0xe5:
				res2.b.l=ram_seg[Pipe2.bytes.byte1];
				_pc++;
				goto aggSottr;
				break;

			case 0xe6:
				res3.b.l=++ram_seg[Pipe2.bytes.byte1];
				_pc++;
//      printf("I=%02x\n",i);
				goto aggFlagI;
				break;

#ifdef CPU_65C02		// rockwell
			case 0xe7:
				ram_seg[Pipe2.bytes.byte1] |= 0x40;
				_pc++;
				break;
#endif

			case 0xe8:
				_x++;
				goto aggFlagX;
				break;

			case 0xe9:
				res2.b.l=Pipe2.bytes.byte1;
				_pc++;
        
aggSottr:
				res1.b.l=_a;
        res1.b.h=res2.b.h=0;
				res3.w = res1.w-res2.w;
				if(!_p.Carry)
					res3.w--;
				_a=res3.b.l;
				_p.Carry=!res3.b.h;
				/*Overflow can be computed simply in C++ from the inputs and the result. 
				Overflow occurs if (M^result)&(N^result)&0x80 is nonzero. 
				That is, if the sign of both inputs is different from the sign of the result. 
				(Anding with 0x80 extracts just the sign bit from the result.) 
				Another C++ formula is !((M^N) & 0x80) && ((M^result) & 0x80). 
				This means there is overflow if the inputs do not have different signs and the input sign is different from the output sign */ 
//				_p.Overflow=((res1.b.l ^ res2.b.l) & 0x80) ? 1 : 0;		
//        _p.Overflow = (res1.b & 0x40 + res2.b & 0x40) != (res1.b & 0x80 + res2.b & 0x80);
//        _p.Overflow = !!(res1.b.l & 0x40 + res2.b.l & 0x40) != !!(res1.b.l & 0x80 + res2.b.l & 0x80);
//        _p.Overflow = ((res1.b.l ^ res3.b.l) & (res2.b.l ^ res3.b.l) & 0x80) ? 1 : 0;
//        _p.Overflow = !!(((res1.b.l & 0x40) + (res2.b.l & 0x40)) & 0x80) != !!(((res1.w & 0x80) + (res2.w & 0x80)) & 0x100);
        /*if((res1.b.l & 0x80) != (res2.b.l & 0x80)) {
          if(((res1.b.l & 0x80) && !(res3.b.l & 0x80)) || (!(res1.b.l & 0x80) && (res3.b.l & 0x80)))
            _p.Overflow=1;
          else
            _p.Overflow=0;
          }
        else
          _p.Overflow=0;*/
        _p.Overflow = !!res3.b.h != !!((res3.b.l & 0x80) ^ (res1.b.l & 0x80) ^ (res2.b.l & 0x80));

				goto aggFlagA;
				break;

			case 0xea:
				break;

#ifdef CPU_KIMKLONE
			case 0xeb:    // RTS_K3
				break;
#endif

			case 0xec:
				res3.w=(SWORD)_x - GetValue(Pipe2.word);
				_pc+=2;
				goto aggFlagC;
				break;

			case 0xed:
				res2.b.l=GetValue(Pipe2.word);
				_pc+=2;
				goto aggSottr;
				break;

			case 0xee:
				res3.b.l=GetValue(Pipe2.word);
				PutValue(Pipe2.word,++res3.b.l);
				_pc+=2;
				goto aggFlagI;
				break;

#ifdef CPU_65C02		// rockwell
			case 0xef:
				if(ram_seg[Pipe2.bytes.byte1] & 0x40)
					_pc+=(signed char)Pipe2.bytes.byte2;
				_pc=2;
				break;
#endif

			case 0xf0:
				if(_p.Zero)
					_pc+=(signed char)Pipe2.bytes.byte1;
				_pc++;
				break;

			case 0xf1:
				res2.b.l=GetValue(MAKEWORD(ram_seg[Pipe2.bytes.byte1],ram_seg[Pipe2.bytes.byte1+1])+_y);
				_pc++;
				goto aggSottr;
				break;

#ifdef CPU_65C02
			case 0xf2:
				res2.b.l=GetValue(MAKEWORD(ram_seg[Pipe2.bytes.byte1],ram_seg[Pipe2.bytes.byte1+1]));
				_pc++;
				goto aggSottr;
				break;
#endif

#ifdef CPU_KIMKLONE
			case 0xf3:    // LDA_K2
				break;
			case 0xf4:    // LDK3
				break;
#endif

			case 0xf5:
				res2.b.l=ram_seg[Pipe2.bytes.byte1+_x];
				_pc++;
				goto aggSottr;
				break;

			case 0xf6:
				res3.b.l=++ram_seg[Pipe2.bytes.byte1+_x];
				_pc++;
				goto aggFlagI;
				break;

#ifdef CPU_65C02		// rockwell
			case 0xf7:
				ram_seg[Pipe2.bytes.byte1] |= 0x80;
				// e i REGISTRI CPU??
				_pc++;
				break;
#endif

			case 0xf8:
				_p.Decimal=1;
				break;

			case 0xf9:
				res2.b.l=GetValue(Pipe2.word+_y);
				_pc+=2;
				goto aggSottr;
				break;

#ifdef CPU_65C02
			case 0xfa:
				_x=stack_seg[++_s];
				goto aggFlagX;
				break;
#endif

#ifdef CPU_KIMKLONE
			case 0xfb:    // RTI_K3
				break;
			case 0xfc:    // LDK3
				break;
#endif

			case 0xfd:
				res2.b.l=GetValue(Pipe2.word+_x);
				_pc+=2;
				goto aggSottr;
				break;

			case 0xfe:
				res3.b.l=GetValue(Pipe2.word+_x);
				PutValue(Pipe2.word+_x,++res3.b.l);
				_pc+=2;
				goto aggFlagI;
				break;

#ifdef CPU_65C02		// rockwell
			case 0xff:
				if(ram_seg[Pipe2.bytes.byte1] & 0x80)
					_pc+=(signed char)Pipe2.bytes.byte2;
				_pc=2;
				break;
#endif
        
			default:
        
//				wsprintf(myBuf,"Istruzione sconosciuta a %04x: %02x",_pc-1,GetValue(_pc-1));
//				SetWindowText(hStatusWnd,myBuf);
        
				break;
			}
		}
	}

