//!----------------------------------------------------------------------------
//! electronics-lis
//! CH-2000 Neuchatel
//! info@electronics-lis.com
//! https://electronics-lis.com
//! L. Lisowski 20 January 2018
//!----------------------------------------------------------------------------
//!----------------------------------------------------------------------------
// PIC32MK DMA tests
//!----------------------------------------------------------------------------
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <C:\Program Files (x86)\Microchip\xc32\v1.44\pic32mx\include\proc\p32mk1024mcf064.h>
#include <xc.h>            
#include <sys/attribs.h> 
//!----------------------------------------------------------------------------
//!----------------------------------------------------------------------------
//! This part need to be adapted for your application. 
#pragma config PWMLOCK = OFF, FUSBIDIO2 = OFF, FVBUSIO2 = OFF, PGL1WAY = ON    
#pragma config PMDL1WAY = ON, IOL1WAY = ON, FUSBIDIO1 = OFF, FVBUSIO1 = OFF                    
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
//!----------------------------------------------------------------------------
//!----------------------------------------------------------------------------
#define _SYSCLK 120000000L            //!   Direct definition
#define _PBCLK (_SYSCLK/2)
#define _UARTspeed   115200
/*********************************/
//! Extremely important. Only this form  is working
#define VirtToPhys(p) (int)p<0?((int)p&0x1fffffffL):(unsigned int)((unsigned char*)p+0x40000000L)
//! Buffers initialization
#define RX_BUF_SIZE  30
/*********************************/
//! Global data
/*********************************/
char rxBuf[RX_BUF_SIZE];    
int rx_number;              // Global data
char i;
/*********************************/
//! Function prototype
/*********************************/
void initDMA_global(void);
void initDMA0RX(void);
void initU1ART(void);
/*********************************/
void main()
{ 
    unsigned long  k, i;
    __builtin_enable_interrupts();  // global interrupt enable
    __XC_UART = 1;                  // printf on the U1ART    
    INTCONbits.MVEC = 1; // Set the interrupt controller for multi-vector mode  
    // Function initialization
    initU1ART();
    initDMA_global();
    initDMA0RX();
    printf ("\n\r This is my first DMA receiver test \n\r");
    rx_number = 0;
    /*while (1)                     //!U1ART test
	{
        char i;
        while( !U1STAbits.URXDA);   //! wait until data available in RX buffer
        i = U1RXREG;
        while( U1STAbits.UTXBF);    //! wait while TX buffer full
        U1TXREG = i;                //! Echo
        while( !U1STAbits.TRMT);    //! wait for last transmission to finish
	}*/
    while(1)
    { 
        //! Hit ant key and finish with CR to display message
        //! I'm using receiving "one byte by one byte" to be accumulate and
        //! and applied to Command Interpreter
    }
}
/*********************************/
void initU1ART(void)
{
    // UART init
    // Pins initialization. !!!!! have to be adapted to your hardware 
    TRISCbits.TRISC6 = 1;       // C6 digital input
    U1RXRbits.U1RXR = 5;        //SET RX to RC6 
    RPC7Rbits.RPC7R = 1;        //SET RC7 to TX    
    // disable UART1 and autobaud, TX and RX enabled only,8N1,idle=HIGH
    U1MODE = 0x0000;         
    U1STAbits.URXEN = 1;        // Enable RX 
    U1STAbits.UTXEN = 1;        // Enable TX
    U1BRG = (_PBCLK/(16*_UARTspeed)) - 1;
    //U1BRG = 32;               //! Baud Speed => 115200
    // Interrupt         
    IPC9bits.U1RXIP = 3;        // set IPL 3
    IPC9bits.U1RXIS = 2;        // sub-priority 2
    U1STAbits.URXISEL = 0;      //!where receive one character
    
    IPC10bits.U1TXIP = 3;       // set IPL 3
    IPC10bits.U1TXIS = 2;       // sub-priority 2
    U1STAbits.UTXISEL = 2;      // where transmit is empty
            
    IFS1bits.U1TXIF = 0;        //!< Clear the Transmit Interrupt Flag
    IEC1bits.U1TXIE = 0;        //!< Disable Transmit Interrupts
    IFS1bits.U1RXIF = 0;        //!< Clear the Recieve Interrupt Flag
    IEC1bits.U1RXIE = 0;        //!< Disable Recieve Interrupts
    U1MODEbits.ON = 1;          //!< U1ART ON
}
/*********************************/
void initDMA_global(void)       //! Turns on the DMA controller
{
    DMACONbits.ON = 1;
    // Is it necessary ??????
    DCH2CONbits.CHEN = 0;     // Enable channel 
    DCH3CONbits.CHEN = 0;     // Enable channel
    DCH4CONbits.CHEN = 0;     // Enable channel
    DCH5CONbits.CHEN = 0;     // Enable channel
    DCH6CONbits.CHEN = 0;     // Enable channel
    DCH7CONbits.CHEN = 0;     // Enable channel
}
/*********************************/
void initDMA0RX(void)           //! This is the DMA setup for the receiver 
{
    IEC2bits.DMA0IE = 0;            // Interrupt disable
    IFS2bits.DMA0IF = 0;            // Interrupt flag CLR
    DCH1CONbits.CHPRI = 3;          // Channel priority
    DCH0ECON = 0;
    DCH0ECONbits.CHSIRQ = 39;       // transfer init by U1ART see tab. 8-3
    DCH0ECONbits.SIRQEN = 1;        // event enable
    /*DCH0ECONbits.PATEN = 1;
    DCH0CONbits.CHPATLEN = 0;
    DCH0DATbits.CHPDAT = '\r';  */  // 0x0D = CR (carriage return)*/
    DCH0SSA = VirtToPhys((void*)&U1RXREG);              // Source address
    DCH0DSA = VirtToPhys((void*)&rxBuf[rx_number]);     // Destination 
    DCH0SSIZ = 1;                        
    DCH0DSIZ = RX_BUF_SIZE;
    DCH0CSIZ = 1;                                                                     
    DCH0INTbits.CHCCIE = 1;     // Interrupt on block transfer complete
    DCH0INTbits.CHERIE = 1;     // Interrupt on errors
// Interrupt setup
    IPC18bits.DMA0IP = 3;       // Priority
    IPC18bits.DMA0IS = 2;       // Sub priority
    IFS2bits.DMA0IF = 0;        // Interrupt flag CLR
    IEC2bits.DMA0IE = 1;        // Interrupt enable
    DCH0CONbits.CHEN = 1;       // Enable channel
}
/*************************************/
void __ISR(_DMA0_VECTOR, IPL3AUTO) DmaHandler0(void) //_DMA0_VECTOR = 72
{    
    DCH0INTCLR=0x000000ff;              // clear DMA's channel interrupt flags
    IFS2bits.DMA0IF = 0;                // clear interruption flag   
    
    putchar(rxBuf[rx_number]);          // __XC_UART = 1 !!!!!!! 
    if (rx_number == RX_BUF_SIZE-1)// verify if rx_number is bigger than buffer
                                   // it can be done also by DMA ==> address
    {
       puts("f\r\n");
       memset(rxBuf, 0, sizeof rxBuf);  // clean buffer
       /* error procedure call */
       rx_number = 0;                   // init buffer count
       initDMA0RX();                    // reset DMA perhaps could be done by ?? 
       return;                          // ????
    }
    if (rxBuf[rx_number] == '\r')       // can be done by PATERN but I need for call
    {
        putchar('\n'); 
        puts (rxBuf );                  // print buffer
        /* command interpreter call*/
        memset(rxBuf, 0, sizeof rxBuf); // clean buffer
        rx_number = 0;                  // init buffer count
        initDMA0RX();                   // reset DMA perhaps could be done by ??
        return;                         // ????
    }
    rx_number++;   
}
 
