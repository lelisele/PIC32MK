//!----------------------------------------------------------------------------
//! electronics-lis
//! CH-2000 Neuchatel
//! info@electronics-lis.com
//! https://electronics-lis.com
//! L. Lisowski December 2021
//!----------------------------------------------------------------------------
//!----------------------------------------------------------------------------
//! PIC32MK I2C tests REVISION 02
//!----------------------------------------------------------------------------
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
//! check where is <p32mk1024mcf064.h> file on your PC
#include <C:\Program Files\Microchip\xc32\v2.50\pic32mx\include\proc\PIC32MK-MC\p32mk1024mcf064.h>
#include <xc.h>            
#include <sys/attribs.h> 
//!----------------------------------------------------------------------------
//!----------------------------------------------------------------------------
//! This part need to be adapted for your application. 
// DEVCFG3
#pragma config PWMLOCK = OFF, FUSBIDIO2 = OFF, FVBUSIO2 = OFF, PGL1WAY = OFF    
#pragma config PMDL1WAY = OFF, IOL1WAY = OFF, FUSBIDIO1 = OFF, FVBUSIO1 = OFF                    
// DEVCFG2
#pragma config FPLLIDIV = DIV_1, FPLLRNG = RANGE_5_10_MHZ, FPLLICLK = PLL_POSC         
#pragma config FPLLMULT = MUL_48, FPLLODIV = DIV_4, VBATBOREN = ON, DSBOREN = ON      
#pragma config DSWDTPS = DSPS32, DSWDTOSC = LPRC, DSWDTEN = OFF, FDSEN = OFF          
#pragma config BORSEL = HIGH, UPLLEN = OFF               
// DEVCFG1
#pragma config FNOSC = SPLL, DMTINTV = WIN_127_128, FSOSCEN = OFF, IESO = ON
#pragma config POSCMOD = HS, OSCIOFNC = ON, FCKSM = CSECME, WDTPS = PS1048576   
#pragma config WDTSPGM =STOP,WINDIS = NORMAL, FWDTEN = OFF, FWDTWINSZ = WINSZ_25
#pragma config DMTCNT = DMT31, FDMTEN = ON
// DEVCFG0
#pragma config DEBUG = OFF, JTAGEN = OFF, ICESEL = ICS_PGx1, TRCEN = ON
#pragma config BOOTISA = MIPS32,FSLEEP = OFF, DBGPER = PG_ALL, SMCLR = MCLR_NORM     
#pragma config SOSCGAIN = GAIN_2X, SOSCBOOST = ON, POSCGAIN = GAIN_LEVEL_3
#pragma config POSCBOOST = ON, EJTAGBEN = NORMAL 
// DEVCP
#pragma config CP = OFF                 // Code Protect (Protection Disabled)
#define Quartz 10000000
//! That are my Configuration Bits version, you need adapt them for your application 
//!----------------------------------------------------------------------------
//!----------------------------------------------------------------------------
#define _SYSCLK 120000000L            //!   Direct definition
#define _PBCLK (_SYSCLK/2)
/************************************//**
** Type Definitions
****************************************/
typedef signed char  int8_l;
typedef unsigned char uint8_l;
typedef short int16_l;
typedef unsigned short uint16_l;
typedef long int32_l;
typedef unsigned long uint32_l;
/************************************//**
** Macro Definitions
****************************************/
//! Don't forget !!! the result are uint32_l !!!!!!!!
#define TSTBIT(D,i) (uint32_l)D & ((uint32_l)i)
#define SETBIT(D,i) (D |= i)
#define CLRBIT(D,i) (D &= (~i))
/*********************************/
//! Local definition and variables
/*********************************/
char i;
#define TRUE 1
#define FALSE 0
#define ZERO 0
#define ONE 1
#define UARTspeed   115200
#define I2Cspeed    400 //! kHz
/*********************************/
//! I2C definition and variables
/*********************************/
//! I'm using the "mask" methode and not interruption. The I2C is appling
//! excusevelly to configure some components as DAC, ADC etc. during start-up.
//! Thatfor this methode is sufficient. 
//! I've tested only RE12 and RE13 and they are controlled by I2C4.
//! The others RB5 and RB6, RA8 and RB4 and RG7 and RG8 have to be verified
/*********************************/
//! If change the I2C module change only these definitions
/*********************************/
#define I2C_SDA     LATEbits.LATE12 //! pin E12 as SDA and Output
#define I2C_SCL     LATEbits.LATE13 //! pin E13 as SCL and Output
#define I2CxCON     I2C4CON         //! I2C control register
#define I2CxSTAT    I2C4STAT        //! I2C state register
#define I2CxTRN     I2C4TRN         //! I2C Transmitter register
#define I2CxRCV     I2C4RCV         //! I2C state register
#define I2CxADD     I2C4ADD         //! Address register
#define I2CxBRG     I2C4BRG         //! Baud Range register
//! The masks are the same for all I2C modules
//! STAT register
#define I2C_STAT_TBF     0x1    //! Transmit Buffer Full Status bit
#define I2C_STAT_RBF     0x2    //! Receive Buffer Full Status bit
#define I2C_STAT_S       0X8    //! Start bit detected(I2C doc ????)
#define I2C_STAT_P       0x10   //! Stop bit detected (I2C doc ????)
#define I2C_STAT_BCL     0x400  //! Master Bus Collision Detect bit
#define I2C_STAT_TRSTAT  0x4000 //! Transmit Status bit
#define I2C_STAT_ACKSTAT 0x8000 //! ACKSTAT: Acknowledge Status bit
//! CONtrol register
#define I2C_CON_SEN    0x1      //! Start Condition Enable bit
#define I2C_CON_RSEN   0x2      //! Restart Condition Enable 
#define I2C_CON_PEN    0x4      //! Stop Condition Enable
#define I2C_CON_ACKEN  0x10     //! Acknowledge Sequence Enable bit
#define I2C_CON_ACKDT  0x20     //! Acknowledge Data bit
#define I2C_CON_DISSLW 0x200    //! Slew Rate Control Disable bit
#define I2C_CON_SCLREL 0x1000   //! SCLx Release Control bit
#define I2C_CON_ON     0x8000   //! I2C module On Off
/************************************//**
** MCP47328 DAC parameters
****************************************/
//! Command Type Bits C2=0,C1=1,C0=0,W1=1,W0=1,UDC,PD0,PD1=0,Vref,Gx=1 (doc page 41)
#define I2C_MAX_WAITE_LOOPS   100                   //! Max loops into while
#define I2CdacNumberOfBytes         4
#define I2CdacNumberOfCahnnels      4
#define I2CmaxVerif               20
#define I2C12bitsWordSplit2(a)  (uint8_l)(a&(0xff)) 
#define I2C12bitsWordSplit1(a) (uint16_l)((a>>8)&(0xf))
//! Global data
uint16_l _MaxLoops; 
static uint16_l _dacData;         
static uint8_l _NbrChannel;
uint8_l _NbrOfverif; 
uint32_l NbrLoopsTable[I2CmaxVerif],_LastErrorMask;   
//! DAC bytes definition, the two last have to be up-dated by 12 bits data word
uint8_l I2CslaveWriteDac[I2CdacNumberOfCahnnels][I2CdacNumberOfBytes]=
{
    {0xC0, 0x58, 0x90, 0x00}, 
    {0xC0, 0x5A, 0x90, 0x00},
    {0xC0, 0x5C, 0x90, 0x00},
    {0xC0, 0x5E, 0x90, 0x00} 
};
/*********************************/
//! Function prototype
/*********************************/
void initU1ART(void);
void initI2C(void);
uint8_l APPLI_DACwrite(uint16_l ToBeWRite, uint8_l _NbrChannel);
uint8_l VerifStatZERO(uint32_l mask);
uint8_l VerifStatONE(uint32_l mask);
/*********************************/
//! main 
void main()
{ 
    char i;
    uint8_l j, _NbrChannel;
    
    __builtin_enable_interrupts();  // global interrupt enable
    __XC_UART = 1;                  // printf on the U1ART    
    INTCONbits.MVEC = 1; // Set the interrupt controller for multi-vector mode
    //! all digital
    ANSELA = 0; ANSELB = 0; ANSELC = 0;ANSELE = 0; ANSELG = 0; ANSELG = 0;
    //!  PullUp and PullDown canceled
    CNPUA = 0; CNPUB= 0; CNPUC= 0; CNPUD= 0; CNPUE= 0; CNPUF= 0; CNPUG= 0; CNPUF= 0;
    CNPDA = 0; CNPDB= 0; CNPDC= 0; CNPDD= 0; CNPDE= 0; CNPDF= 0; CNPDG= 0; CNPDF= 0;
    PMCONbits.ON = 0;               //! no PMP
    // Function initialization
    initU1ART();
    printf ("\n\r This is my first I2C test  \n\r");
    initI2C();   
    CFGCONbits.IOLOCK = 0;      //! Necessary to define IO pins
    while (1)                   //! U1ART test => Is my PIC32 always in live?
	{  
        while( !U1STAbits.URXDA);   //! wait until data available in RX buffer
        i = U1RXREG;                //1 Hit any key
        while( U1STAbits.UTXBF);    //! wait while TX buffer full
        U1TXREG = i;                //! Echo
        while( !U1STAbits.TRMT);    //! wait for last transmission to finish
        //! You can imagine something nicer (scanf) but I don't have time        
        if(i=='\r') putchar('\n');   //! To be nice
        if(i=='0')_NbrChannel=0;if(i=='1')_NbrChannel=1;
                        if(i=='2')_NbrChannel=2;if(i=='3')_NbrChannel=3;
        if(i=='a')_dacData=500;if(i=='b')_dacData=1000;if(i=='c')_dacData=2000;
        if(i=='d')_dacData=3000;if(i=='e')_dacData=4000;if(i=='z')_dacData=0;
        if (i == 's')               //! To start sequence or pin's output test
        {
            // That can be used to test pins output
            //if (I2C_SDA == 1) I2C_SDA = 0; else I2C_SDA =1;
            //if (I2C_SCL == 1) I2C_SCL = 0; else I2C_SCL =1;
            printf(" DacData=%d Channel=%d\n\r",_dacData, _NbrChannel);
            //! DAC update call
            if ( APPLI_DACwrite(_dacData,_NbrChannel) ){printf(" Call OK");} else 
                            printf("\n\r Error mask =x%x call stopped", _LastErrorMask);
            //! NbrLoops and NbrVerif print, You can removed it. That is useful for test          
            printf (" NbrOfVerif=%d NbrLoopsTable ==>>",_NbrOfverif);
            for (j=1; j <= _NbrOfverif; j++) printf(" %d",  
                    (uint16_l)(I2C_MAX_WAITE_LOOPS - NbrLoopsTable[j])); 
            printf("\n\r");       //! display more logical ;-)
        }
	}
}
/*********************************/
void initU1ART(void)
{
    // UART init
    // Pins initialization. !!!!! have to be adapted to your hardware 
    TRISCbits.TRISC6 = 1;       //!  C6 digital input
    U1RXRbits.U1RXR = 5;        //!  SET RX to RC6 
    RPC7Rbits.RPC7R = 1;        //!  SET RC7 to TX    
    // disable UART1 and autobaud, TX and RX enabled only,8N1,idle=HIGH
    U1MODE = 0x0000;         
    U1STAbits.URXEN = 1;        //! Enable RX 
    U1STAbits.UTXEN = 1;        //! Enable TX   
    U1BRG = (_PBCLK/(16*UARTspeed)) - 1;
    //U1BRG = 32;                 //! Baud Speed => 115200
    // Interrupt         
    IPC9bits.U1RXIP = 3;        //! set IPL 3
    IPC9bits.U1RXIS = 2;        //! sub-priority 2
    U1STAbits.URXISEL = 0;      //! when receive one character
    //! For future applications 
    IPC10bits.U1TXIP = 3;       //! set IPL 3
    IPC10bits.U1TXIS = 2;       //! sub-priority 2
    U1STAbits.UTXISEL = 2;      //! where transmit is empty     
    IFS1bits.U1TXIF = 0;        //!< Clear the Transmit Interrupt Flag
    IEC1bits.U1TXIE = 0;        //!< Disable Transmit Interrupts
    IFS1bits.U1RXIF = 0;        //!< Clear the Recieve Interrupt Flag
    IEC1bits.U1RXIE = 0;        //!< Disable Recieve Interrupt   
    U1MODEbits.ON = 1;          //!< U1ART ON
}
/*********************************/
void initI2C(void)
{
    // I2C init
    // Pins initialization. !!!!! have to be adapted to hardware 
    TRISEbits.TRISE13 = 0;              //!  E13 digital output
    TRISEbits.TRISE12 = 0;              //!  E12 digital output
    //ODCEbits.ODCE12 = 1;              //! Open Collector 
    //ODCEbits.ODCE13 = 1;              //! Open Collector
    CLRBIT(I2CxCON,I2C_CON_ON);         //!< I2C OFF
    I2CxCON = 0;
    //! Perhaps could be simpler way to do it  but the 104 is [ns]
    //! and XC32 doaes it
    I2CxBRG = (uint16_l)(((1000000/(2*I2Cspeed)-104)*(_PBCLK/1000000))/1000)-2;
    //printf("\n\r BRG=x%x",I2CxBRG);   //! if necessary tom test
    //2CxBRG = 66;                      //! Define I2C clock
    //I2C4CONbits.DISSLW = 1;           //! Standard speed 100 KHz
    SETBIT(I2CxCON,I2C_CON_DISSLW);     //! Slew Rate Control Disable bit
    //I2C4CONbits.SCLREL = 1;           //! Release clock
    SETBIT(I2CxCON,I2C_CON_SCLREL);
    I2C4RCV = 0;                        //! Receive Register initialization
    I2C4TRN = 0;                        //! Transmit Register initialization
    I2C4ADD = 8;
    //I2C4CONbits.ON = 1;               //!< I2C ON 
    SETBIT(I2CxCON,I2C_CON_ON); 
    _dacData = 2500; _NbrChannel =0;         //! default values
}
/*********************************/
uint8_l APPLI_DACwrite(uint16_l ToBeWRite, uint8_l _NbrChannel)
{  
    //! Init of verification, you can remove this error and verif message 
    memset(NbrLoopsTable,0,sizeof(NbrLoopsTable));      //! Clear loops table
    _LastErrorMask = _NbrOfverif =0;                    //! Init values
    //! Data preparing 12 bits word spliting 
    I2CslaveWriteDac[_NbrChannel][2] |= I2C12bitsWordSplit1(ToBeWRite);
    I2CslaveWriteDac[_NbrChannel][3] = I2C12bitsWordSplit2(ToBeWRite);
    //! Sequence start
    if(VerifStatZERO(I2C_STAT_TRSTAT)){} else return(FALSE); //! Is master free?
    SETBIT(I2CxCON,I2C_CON_SEN);                            //! Start condition
    if(VerifStatZERO(I2C_STAT_TRSTAT)){} else return(FALSE); //! Is master free  
    //! Write including acknowledge sequence
    for (i=0; i<=I2CdacNumberOfBytes-1; i++)
    {
        //printf("\n\r Chan=x%x Data=x%x",_NbrChannel, I2CslaveWriteDac[_NbrChannel][i]);
        I2CxTRN = I2CslaveWriteDac[_NbrChannel][i];           //! Write bytes
        if(VerifStatZERO(I2C_STAT_TBF)){} else return(FALSE); //! Waite to finish transmit
        //if(VerifStatZERO(I2C_STAT_ACKSTAT)){} else return(FALSE); //! Acknowledge Status bit
        //printf("\n\r Chan=x%x Data=x%x",_NbrChannel, I2CslaveWriteDac[_NbrChannel][i]);   
    }
    SETBIT(I2CxCON,I2C_CON_PEN);                            //! STOP condition
    if(VerifStatZERO(I2C_STAT_TRSTAT)){} else return(FALSE);
    return (TRUE);
}
/*********************************/
/*********************************/
uint8_l VerifStatZERO(uint32_l mask)
{     
    _MaxLoops = I2C_MAX_WAITE_LOOPS;
    while (_MaxLoops--){
        if((TSTBIT(I2CxSTAT,mask)) == 0x0){ 
           _NbrOfverif++; NbrLoopsTable[_NbrOfverif]=_MaxLoops; return (TRUE);
        }}
    _NbrOfverif++; NbrLoopsTable[_NbrOfverif]=_MaxLoops;_LastErrorMask = mask;
    return (FALSE);
}
/*********************************/
/*********************************/
uint8_l VerifStatONE(uint32_l mask) //! Is it really necessary???? perhaps for 
                            //! procedures more complex than simple read/write
{     
    _MaxLoops = I2C_MAX_WAITE_LOOPS;
    while (_MaxLoops--){
        if((TSTBIT(I2CxSTAT,mask)) >0x0){ 
           _NbrOfverif++; NbrLoopsTable[_NbrOfverif]=_MaxLoops; return (TRUE);
        }}
    _NbrOfverif++; NbrLoopsTable[_NbrOfverif]=_MaxLoops;_LastErrorMask = mask;
    return (FALSE);   
}