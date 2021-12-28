//!----------------------------------------------------------------------------
//! electronics-lis
//! CH-2000 Neuchatel
//! info@electronics-lis.com
//! https://electronics-lis.com
//! L. Lisowski December 2021
//!----------------------------------------------------------------------------
//!----------------------------------------------------------------------------
//! PIC32MK SPI tests  REVISION 03
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
//! This part need to be adapted for your application.
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
//! That are my Configuration Bits version, you need adapt them for your application 
//!----------------------------------------------------------------------------
//!----------------------------------------------------------------------------
#define SYSCLK 120000000L
#define PBCLK (SYSCLK/2)
#define UARTspeed   115200
char i;
/*********************************/
//! Function prototype
/*********************************/
unsigned int  SPI2WordWriteRead( unsigned int value);
void initU1ART(void);
/*********************************/
//! main 
void main()
{ 
    unsigned long  k, i, j;
     //! Clock initialization
    /* switch to PLL input (SPLL) */
    SYSKEY = 0x0;                        /* Ensure OSCCON is locked */
    SYSKEY = 0xAA996655;                 /* write Key1 unlock sequence */
    SYSKEY = 0x556699AA;                 /* write Key2 unlock sequence */
    OSCCONCLR = 0x07000600;
    OSCCONSET = 0x0100;
    OSCCONSET = 1;                       /* Start clock switching */
    SYSKEY = 0x0;                        /* OSCCON is relocked */
    // Function initialization
    __builtin_enable_interrupts();  // global interrupt enable
    __XC_UART = 1;                  // printf on the U1ART    
    INTCONbits.MVEC = 1; // Set the interrupt controller for multi-vector mode 
    //! all digital
    ANSELA = 0; ANSELB = 0; ANSELC = 0; ANSELE = 0;ANSELG = 0;
    CNPUA = 0; CNPUB= 0; CNPUC= 0; CNPUD= 0; CNPUE= 0; CNPUF= 0; CNPUG= 0; CNPUF= 0;
    CNPDA = 0; CNPDB= 0; CNPDC= 0; CNPDD= 0; CNPDE= 0; CNPDF= 0; CNPDG= 0; CNPDF= 0;
    PMCONbits.ON = 0;               //! no PMP
    // Function initialization
    initU1ART();
    printf ("\n\r This is my first SPI2 test \n\r");  
    //! PBCLK2 clock initialization
    PB2DIVbits.ON = 1;              //! PBCLK2 clock  => ON        
    //! Global initialization 
    IEC1bits.SPI2EIE = IEC1bits.SPI2RXIE = IEC1bits.I2C2SIE = 0; // SPI2 clear Interrupt
    IPC13bits.SPI2EIP = IPC13bits.SPI2EIS = 0; //! Clear Fault priority
    IPC13bits.SPI2RXIP = IPC13bits.SPI2RXIS = 0; //! Clear Receive priority
    IPC13bits.SPI2TXIP = IPC13bits.SPI2TXIS = 0; //! Clear Transmit priority
    IFS1bits.SPI2EIF = IFS1bits.SPI2RXIF = IFS1bits.SPI2TXIF = 0; //! Clear Flag 
    // Set the PINs for SCK, SDI and SDO  !!!!! have to be adapted to hardware 
    SPI2CON = 0;                //! Stop and reset SPI2
    CFGCONbits.IOLOCK = 0;      //! Necessary to define IO pins
    PMD5bits.SPI2MD = 0;        //! For SPI2 clock enabled
    
    //! Check on Table 13-1 and 13-2 in PIC32MK GP/MC in DS60001402D doc
    TRISCbits.TRISC8 = 1;       //! D5 digital input
    SDI2Rbits.SDI2R = 6;        //!  SET SDI2 to RC8
    TRISDbits.TRISD5 = 0;       //! Output
    RPD5Rbits.RPD5R = 4;        //!  SET SDO2 to RD5
    TRISBbits.TRISB6 = 0;       //! Output
    RPB6Rbits.RPB6R = 0;        //! SET SCK2 to RB6 PBCLK2 ANYWAY in Documentation
    CFGCONbits.IOLOCK = 1;

    SPI2CON = 0;                //! Stop and reset SPI2
    j = SPI2BUF;                //! clears the receive buffer
    SPI2BRG = 0;                //! Baud speed definition
    SPI2STATbits.SPIROV = 1;    //! No overflow
    SPI2CONbits.MODE16 = 1;     //! 16 bits
    SPI2CONbits.MSTEN = 1;      //! Master mode
    SPI2CONbits.MCLKSEL = 0;    //1 maser clock select => PBCLK2     
    SPI2CONbits.SMP = 0;        //! Input data sampled in the middle 
    SPI2CONbits.CKE = 1;        //! output data change on idle to  active state transition (raising)
    SPI2CONbits.CKP = 0;        //! Clock active in high position
    SPI2CONbits.ON =1;          //! SPI@ => ON

    while (1) // U1ART test
	{
        char i;
        while( !U1STAbits.URXDA);   //! wait until data available in RX buffer
        i = U1RXREG;                //1 Hit any key
        while( U1STAbits.UTXBF);    //! wait while TX buffer full
        if (i != 0)
        {
            //! Test in loop-back configuration => SDO2 connected to SDI2
            printf ("\n\r Send=%d Receive =%d", i, SPI2WordWriteRead ((unsigned int) i ));
            //SPI2WordWriteRead ((int) i);    //! Send int by SPI2
        }
	}
}
/*********************************/
void initU1ART(void)
{
    // UART init
    // Pins initialization. !!!!! have to be adapted to hardware        
    TRISGbits.TRISG8= 1;       //RG8 digital input
    U1RXRbits.U1RXR = 10;        //SET RX to RG8
    RPG6Rbits.RPG6R = 1;        //SET RG6 to TX   
    // disable UART1 and autobaud, TX and RX enabled only,8N1,idle=HIGH
    U1MODE = 0x0000;         
    U1STAbits.URXEN = 1;        //! Enable RX 
    U1STAbits.UTXEN = 1;        //! Enable TX
    U1BRG = (PBCLK/(16*UARTspeed)) - 1;
    //U1BRG = 32;               //! Baud Speed => 115200
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
unsigned int  SPI2WordWriteRead( unsigned int value)
{
    unsigned int temp;
    temp = SPI2BUF; // dummy read of the SPI2BUF register to clear the SPIRBF flag
    SPI2BUF = value;	// write the data out to the SPI peripheral
    while(!SPI2STATbits.SPIRBF); //! wite to finish transfer
    return((unsigned int) SPI2BUF);  
}

 