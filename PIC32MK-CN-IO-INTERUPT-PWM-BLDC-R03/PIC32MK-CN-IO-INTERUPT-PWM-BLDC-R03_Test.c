//!----------------------------------------------------------------------------
//! electronics-lis
//! CH-2000 Neuchatel
//! info@electronics-lis.com
//! https://electronics-lis.com
//! L. Lisowski 20 Decembre 2021
//!----------------------------------------------------------------------------
//!----------------------------------------------------------------------------
// PIC32MK ADC tests
//!----------------------------------------------------------------------------
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
//! check where is <p32mk1024mcf064.h> file on your PC
#include <C:\Program Files\Microchip\xc32\v2.50\pic32mx\include\proc\PIC32MK-MC\p32mk1024mcf064.h>
#include <xc.h>            
#include <sys/attribs.h> 
//!----------------------------------------------------------------------------
//!----------------------------------------------------------------------------
//! These Configuration Bits need to be adapted for your HW and SW application.
//! This is FRC version without Quartz/Oscillator
// DEVCFG3
#pragma config PWMLOCK = OFF, FUSBIDIO2 = OFF, FVBUSIO2 = OFF, PGL1WAY = OFF    
#pragma config PMDL1WAY = OFF, IOL1WAY = OFF, FUSBIDIO1 = OFF, FVBUSIO1 = OFF                    
// DEVCFG2
#pragma config FPLLIDIV = DIV_1, FPLLRNG = RANGE_5_10_MHZ, FPLLICLK = PLL_FRC         
#pragma config FPLLMULT = MUL_60, FPLLODIV = DIV_4, VBATBOREN = ON, DSBOREN = ON      
#pragma config DSWDTPS = DSPS32, DSWDTOSC = LPRC, DSWDTEN = OFF, FDSEN = OFF          
#pragma config BORSEL = HIGH, UPLLEN = OFF               
// DEVCFG1
#pragma config FNOSC = FRC, DMTINTV = WIN_127_128, FSOSCEN = OFF, IESO = ON
#pragma config POSCMOD = OFF, OSCIOFNC = ON, FCKSM = CSECME, WDTPS = PS16  
#pragma config WDTSPGM =STOP, WINDIS = NORMAL, FWDTEN = OFF, FWDTWINSZ = WINSZ_25
#pragma config DMTCNT = DMT8, FDMTEN = OFF
// DEVCFG0
#pragma config DEBUG = OFF, JTAGEN = OFF, ICESEL = ICS_PGx2, TRCEN = ON
#pragma config BOOTISA = MIPS32,FSLEEP = OFF, DBGPER = PG_ALL, SMCLR = MCLR_NORM     
#pragma config SOSCGAIN = GAIN_2X, SOSCBOOST = ON, POSCGAIN = GAIN_LEVEL_3
#pragma config POSCBOOST = ON, EJTAGBEN = NORMAL 
// DEVCP
#pragma config CP = OFF                 // Code Protect (Protection Disabled)
char InitMessage[] = "Processor ==> PIC32MK1024MCF064";
//!----------------------------------------------------------------------------
//!----------------------------------------------------------------------------
/*********************************/
//! Type declarations
/*********************************/
typedef signed char  int8;
typedef unsigned char uint8;
typedef short int16;
typedef unsigned short uint16;
typedef signed int int32;
typedef unsigned int uint32;
/************************************//**
** Macro Definitions
****************************************/
//! Don't forget !!! the result are uint32_l !!!!!!!!
#define TSTBIT(D,i) (D>>i & 1)
#define SETBIT(D,i) (D |= ((uint32)1<<i))
#define CLRBIT(D,i) (D &= ~((uint32)1 << i))
#define SETPARA(D,p,i) (D |= (p<<i))
#define SYSCLK                  120000000L
#define PBCLK                   (SYSCLK/2)
#define UARTspeed               115200
#define TRUE                    1
#define FALSE                   0
/*********************************/
//! Local Variables
/*********************************/
char i, k;
uint32 PHASE;              //! phase1 PHASEx registers provide time base 
uint32 PDC;                //! Duty cycles
uint8  DTR;                  //! Dead Time Values
uint8  ALTDTR;               //!
uint16 count;
/*********************************/
//! Function prototype
/*********************************/
void initU1ART(void);
void initGen(void);
uint8 initPWMcentred(void);
void SetPara(volatile uint32 * reg, uint32 para, uint32 ParaPos);
uint8 BLDCcontrolPWM(uint8 state);
/*********************************/
//! main 
void main()
{  
    //! Clock initialization
    /* switch to PLL input (SPLL) */
    SYSKEY = 0x0;                        /* Ensure OSCCON is locked */
    SYSKEY = 0xAA996655;                 /* write Key1 unlock sequence */
    SYSKEY = 0x556699AA;                 /* write Key2 unlock sequence */
    OSCCONCLR = 0x07000600;
    OSCCONSET = 0x0100;
    OSCCONSET = 1;                       /* Start clock switching */
    SYSKEY = 0x0;                        /* OSCCON is relocked */       
    __builtin_enable_interrupts();      // global interrupt enable
    __XC_UART = 1;                      // printf on the U1ART    
    INTCONbits.MVEC = 1;                // Set the interrupt controller for multi-vector mode  
    PMCONbits.ON = 0;                   //! no PMP
    // Function initialization
    PHASE = 2000;              //! phase1 PHASEx registers provide time base 
    PDC = 1000;                //! Duty cycles
    DTR = 20;                  //! Dead Time Values
    ALTDTR = 20;               //!       
    PMD5bits.U1MD = 0; //! PERIPHERAL MODULE DISABLE
    PMD4bits.PWM11MD = 0; //! PERIPHERAL MODULE DISABLE
    initU1ART();
    printf ("\n\r This is my first IO and Interruption test \n\r"); 
    initGen();       //! PWM part initialization
    if(initPWMcentred() == TRUE){
        printf ("\n\r initPWMcentred accomplished \n\r");
    } 
    else{
        printf ("\n\r initPWMcentred not accomplished \n\r"); 
    }               
    while (1) 
	{       
        while( !U1STAbits.URXDA);   //! wait until data available in RX buffer
        i = U1RXREG;
        while( U1STAbits.UTXBF);    //! wait while TX buffer full
        U1TXREG = i;                //! Echo
        while( !U1STAbits.TRMT);    //! wait for last transmission to finish
        if (i >= '1' && i<='9')     //! U1ART test
        {                    
            PDC1 = PDC2 = PDC3 = (i-'0')*PHASE1/10; //! i * 10 /100 % calculation
            printf("\n\r Duty=%u \tPhase1=%u  \tNewDuty (1-9)*10 % = ", PDC1, PHASE1);
        }
        else {
            printf("\n\r Argument has to be number  ");
        }   
	}
}
/*********************************/
/*********************************/
void initU1ART(void)
{
    // UART init
    // Pins initialization. !!!!! have to be adapted to hardware        
    TRISGbits.TRISG8= 1;       //RG8 digital input
    U1RXRbits.U1RXR = 10;      //SET RX to RG8
    RPG6Rbits.RPG6R = 1;      //SET RG6 to TX 
    // disable UART1 and autobaud, TX and RX enabled only,8N1,idle=HIGH
    U1MODE = 0x0000;         
    U1STAbits.URXEN = 1; U1STAbits.UTXEN = 1;    //!Enable RX and TX
    U1MODE = 0x0000; 
    //SETBIT(U1STA,12);           //! Enable RX 
    //SETBIT(U1STA,10);           //! Enable TX             
    U1BRG = (PBCLK/(16*UARTspeed)) - 1;
    //U1BRG = 0x40;               //! Baud Speed => 115200
    // Interrupt         
    SetPara(&IPC9,3,26);   
    SetPara(&IPC9,3,24);          
    SetPara(&U1STA,0,6);
    //! For future applications    
    SetPara(&IPC10,3,2);   
    SetPara(&IPC10,2,0);  
    SetPara(&U1STA,0,14);   
    CLRBIT(IFS1,8);    
    CLRBIT(IEC1,8);   
    CLRBIT(IFS1,7);  
    CLRBIT(IEC1,7);
    U1MODEbits.ON = 1;          //!< U1ART ON
}
/*********************************/
/*********************************/
void initGen(void)
{
    //! all digital
    ANSELA = 0; ANSELB = 0; ANSELC = 0;ANSELE = 0; ANSELG = 0; 
    //! No pull-up resistor => necessairy to define ditital Output 
    CNPUA = 0; CNPUB= 0; CNPUC= 0; CNPUD= 0; CNPUE= 0; CNPUF= 0; CNPUG= 0; 
    CNPUF= 0;
    //! No pull-down resistor => necessairy to define ditital Output
    CNPDA = 0; CNPDB= 0; CNPDC= 0; CNPDD= 0; CNPDE= 0; CNPDF= 0; 
    CNPDG= 0; CNPDF= 0;
     //! No Change Notification (interruption generated by I/O )
    CNCONA = 0;  CNCONB = 0; CNCONC = 0; CNCOND = 0; CNCONE = 0; CNCONF = 0;
    CNCONG = 0;   
    /* Initialize pins Hall sensors input pin E12, E13, E14*/ 
    CNCONEbits.ON = 1; // Enable CN
    TRISEbits.TRISE14 = 1;  //! PIn E14 as input
    TRISEbits.TRISE13 = 1;  //! PIn E13 as input
    TRISEbits.TRISE12 = 1;  //! PIn E12 as input
    CNPUEbits.CNPUE14 = 1;  //! PIn E14 pull-up resistor
    CNPUEbits.CNPUE13 = 1;   //! PIn E13 pull-up resistor
    CNPUEbits.CNPUE12 = 1;   //! PIn E12 pull-up resistor
    i = PORTEbits.RE12; i = PORTEbits.RE13; i= PORTEbits.RE14; // clear the CN interrupt.
    IPC12bits.CNEIS = 0;     //! Sub-priority bits
    IPC12bits.CNEIP = 5;    //! Interrupt Priority level
    CNENEbits.CNIEE12 = 1;  //! Interruption Enable
    CNENEbits.CNIEE13 = 1;  //! Interruption Enable
    CNENEbits.CNIEE14 = 1;  //! Interruption Enable 
    IFS1bits.CNEIF = 0;     //! clear flag
    IEC1bits.CNEIE = 1;     //Enable interruption
 
    /* Configure ADCCMPCONx No analog comparators are used.*/
        CM1CON = CM2CON = CM3CON = CM4CON = CM5CON = 0;
    /* DAC disabled */
        DAC1CON = DAC2CON = DAC3CON = 0;
    //! Disable and enable peripheral modules
    PMD2 = 1; //! Ampli op and comparator without clock
    return;
}
/*********************************/
/*********************************/
uint8 initPWMcentred(void)
{       
    SETBIT(PB6DIV,15);          //! Output clock is enabled
    SetPara(&PB6DIV,1,0);       //! Peripheral Bus ?6? Clock Divisor = /2
    //! PTCON initialization (Primary timer)
    PTCON = 0;                  //! Initial disable
    SetPara(&PTCON,0,2);        //! PCLKDIV = 1 = Sysclock /2
    //! CHOP initialization (Chop timer)
    CHOP =0;
    //=======================================//
    //! PWMCON1 initialization
    PWMCON1 = 0;                //! Initial settings
    CLRBIT(PWMCON1,3);          //! MTBS = 0 Primary master is the clock source for the MCPWM module
    SetPara(&PWMCON1,1,10);     //! ECAM=1= Symmetric Center-Aligned mode
    SetPara(&PWMCON1,3,6);      //! Dead Time Compensation mode enabled
    SETBIT(PWMCON1,9);          //! ITB=1 PHASE1 registers provide time base 
    //! IOCON1 initialization
    IOCON1 = 0;                 //! Initial settings
    SETBIT(IOCON1,15);          //! PENH=1 PWM module controls PWM1H pin
    SETBIT(IOCON1,14);          //! PENL=1 PWM module controls PWM1L pin
    CLRBIT(IOCON1,13);          //! PWMxH pin is active-high
    CLRBIT(IOCON1,12);          //! PWMxL pin is active-high
    SetPara(&IOCON1,0,10);      //! PMOD=0>PWM I/O pin pair is in Complementary mode
    PHASE1 = PHASE;             //! phase1 PHASEx registers provide time base 
    PDC1 = PDC;                 //! Duty cycles
    DTR1 = DTR;                 //! Dead Time Values
    ALTDTR1 = ALTDTR; 
    //=======================================//
    //! PWMCON2 initialization
    PWMCON2 = 0;                //! Initial settings
    CLRBIT(PWMCON2,3);          //! MTBS = 0 Primary master is the clock source for the MCPWM module
    SetPara(&PWMCON2,1,10);     //! ECAM=1= Symmetric Center-Aligned mode
    SetPara(&PWMCON2,3,6);      //! Dead Time Compensation mode enabled
    SETBIT(PWMCON2,9);          //! ITB=1 PHASE1 registers provide time base 
    //! IOCON2 initialization
    IOCON2= 0;                 //! Initial settings
    SETBIT(IOCON2,15);          //! PENH=1 PWM module controls PWM1H pin
    SETBIT(IOCON2,14);          //! PENL=1 PWM module controls PWM1L pin
    CLRBIT(IOCON2,13);          //! PWMxH pin is active-high
    CLRBIT(IOCON2,12);          //! PWMxL pin is active-high
    SetPara(&IOCON2,0,10);      //! PMOD=0>PWM I/O pin pair is in Complementary mode
    PHASE2 = PHASE;             //! phase1 PHASEx registers provide time base 
    PDC2 = PDC;                 //! Duty cycles
    DTR2 = DTR;                 //! Dead Time Values
    ALTDTR2 = ALTDTR;           //!
    //=======================================//
    //! PWMCON3 initialization
    PWMCON3 = 0;                //! Initial settings
    CLRBIT(PWMCON3,3);          //! MTBS = 0 Primary master is the clock source for the MCPWM module
    SetPara(&PWMCON3,1,10);     //! ECAM=1= Symmetric Center-Aligned mode
    SetPara(&PWMCON3,3,6);      //! Dead Time Compensation mode enabled
    SETBIT(PWMCON3,9);          //! ITB=1 PHASE1 registers provide time base 
    //! IOCON3 initialization
    IOCON3 = 0;                 //! Initial settings
    SETBIT(IOCON3,15);          //! PENH=1 PWM module controls PWM1H pin
    SETBIT(IOCON3,14);          //! PENL=1 PWM module controls PWM1L pin
    CLRBIT(IOCON3,13);          //! PWMxH pin is active-high
    CLRBIT(IOCON3,12);          //! PWMxL pin is active-high
    SetPara(&IOCON3,0,10);      //! PMOD=0>PWM I/O pin pair is in Complementary mode
    PHASE3 = PHASE;             //! phase1 PHASEx registers provide time base 
    PDC3 = PDC;                 //! Duty cycles
    DTR3 = DTR;                 //! Dead Time Values
    ALTDTR3 = ALTDTR;           //!
    //=======================================//
    //! Trigger implementation only example for PWM1 
    SetPara(&TRIG1,0,0);        //! 0 delay
    TRGCON1 = 0;                //! Initial value DTM=0;
    SetPara(&TRGCON1,0,12);     //! TRGDIV1=0,; Trigger1 Divider = 4
    SetPara(&TRGCON1,2,10);     //! TRGSEL=2; Trigger Cycle Selection for Center-Aligned
    //! PWM1 trigger event Interrupt implementation
    //! _PWM1_VECTOR
    SetPara(&IPC43,1,10);       //! Priority
    SetPara(&IPC43,5,8);        //! sub-Priority
    SETBIT(IEC5,13);            //! Interrupt Enable
    CLRBIT(IFS5,13);            //! Clear flag          
    CLRBIT(TRGCON1,7);          //! DTM=0 Secondary trigger event is not combined 
    CLRBIT(TRGCON1,6);          //! STRGIS=0 Interrupt selected 
    SETBIT(PWMCON1,21);         //! TRIGIEN=1 Primary Trigger Interrupt Enable
    count = 0;
    SETBIT(PTCON,15);           //! PWM module enable
    return (TRUE);
 }
/*********************************/
/*********************************/
uint8 BLDCcontrolPWM(uint8 state)
{
    //printf("\n\r Halls=%d", state);
    switch (state)
    {      
    case 5:
        IOCON3SET = 0x340;  
        IOCON3CLR = 0x80;   // PWM3 data => H=0       
        IOCON2SET = 0x300;  // PWM2 over enable
        IOCON2CLR = 0xc0;   // PWM2 data => H=0; L=0       
        IOCON1SET = 0xC000; // PWM1 PWM
        IOCON1CLR = 0x300;  // PWM1 ==> PWM1
        break;
    case 1:
        IOCON3SET = 0x340;  // PWM3 override enable data => L=1
        IOCON3CLR = 0x80;   // PWM3 data => H=0       
        IOCON1SET = 0x300;  // PWM1 over enable
        IOCON1CLR = 0xc0;   // PWM1 data => H=0; L=0       
        IOCON2SET = 0xC000; // PWM2 PWM
        IOCON2CLR = 0x300;  // PWM2 ==> PWM2
        break;
    case 3:
        IOCON1SET = 0x340;  // PWM1 override enable data => L=1
        IOCON1CLR = 0x80;   // PWM1 data => H=0       
        IOCON3SET = 0x300;  // PWM3 over enable
        IOCON3CLR = 0xc0;   // PWM3 data => H=0; L=0        
        IOCON2SET = 0xC000; // PWM2 PWM
        IOCON2CLR = 0x300;  // PWM2 ==> PWM2      
        break;
    case 2:
        IOCON1SET = 0x340;  // PWM1 override enable data => L=1
        IOCON1CLR = 0x80;   // PWM1 data => H=0      
        IOCON2SET = 0x300;  // PWM2 over enable
        IOCON2CLR = 0xc0;   // PWM2 data => H=0; L=0
        IOCON3SET = 0xC000; // PWM3 PWM
        IOCON3CLR = 0x300;  // PWM3 ==> PWM3
        break;
    case 6:
        IOCON2SET = 0x340;  // PWM2 override enable data => L=1
        IOCON2CLR = 0x80;   // PWM2 data => H=0      
        IOCON1SET = 0x300;  // PWM1 over enable
        IOCON1CLR = 0xc0;   // PWM1 data => H=0; L=0      
        IOCON3SET = 0xC000; // PWM3 PWM
        IOCON3CLR = 0x300;  // PWM3 ==> PWM3
        break;
    case 4:
        IOCON2SET = 0x340;  // PWM2 override enable data => L=1
        IOCON2CLR = 0x80;   // PWM2 data => H=0      
        IOCON3SET = 0x300;  // PWM3 over enable
        IOCON3CLR = 0xc0;   // PWM3 data => H=0; L=0       
        IOCON1SET = 0xC000; // PWM1 PWM
        IOCON1CLR = 0x300;  // PWM1 ==> PWM1
        break;
    default:
        
        break;
    }
    return (1);
}
/*********************************/
/*********************************/
void SetPara(volatile uint32 * reg, uint32 para, uint32 ParaPos)
{     
    *reg |= (para<<ParaPos);
    return;
}
/*********************************/
/*********************************/
void __ISR(_PWM1_VECTOR) Pwm1Handler(void) //_PWM1_VECTOR = 173
{    
    /*count++;
    if (count == 2000)          //! Divider to test ISR
    {
        LATFINV = 0x3; 
        //putchar(':');        
    }*/
   LATFINV = 0x3; 
   CLRBIT(PWMCON1,29);         //! Trigger Interrupt Status
   CLRBIT(IFS5,13);            //! clear flag
}
/*********************************/
/*********************************/
void __ISR(_CHANGE_NOTICE_E_VECTOR) IOHandler(void) //_CHANGE_NOTICE_E_VECTOR = 48
{    
   //putchar(':');
   //printf("\n\r HallA=%d  HallB=%d  HallC=%d", PORTEbits.RE12, PORTEbits.RE13, PORTEbits.RE14);
   BLDCcontrolPWM((PORTE >> 12)&7);
   //printf("\n\r Halls=%d", (PORTE >> 12)&7);
   IFS1bits.CNEIF = 0;           //! clear flag
}
/*********************************/
/*********************************/
uint32 excep_code;
uint32 excep_addr;
void _general_exception_handler (unsigned cause, unsigned status)
{
    excep_code = (cause & 0x0000007C) >> 2;
    excep_addr = __builtin_mfc0(_CP0_EPC, _CP0_EPC_SELECT);
    if ((cause & 0x80000000) != 0)
        excep_addr += 4;
    putchar('!');
    while (1);
}