//rappel : pour lire ou ecrire plusieurs octet, il faut mettre à 1 le bit d'adresse du registre. 
//sinon, ce seral tjs le meme octet qui sera envoyé
// PIC32MZ2048EFM144 Configuration Bit Settings

// 'C' source line config statements

// DEVCFG3
#pragma config USERID = 0xFFFF          // Enter Hexadecimal value (Enter Hexadecimal value)
#pragma config FMIIEN = ON              // Ethernet RMII/MII Enable (MII Enabled)
#pragma config FETHIO = ON              // Ethernet I/O Pin Select (Default Ethernet I/O)
#pragma config PGL1WAY = ON             // Permission Group Lock One Way Configuration (Allow only one reconfiguration)
#pragma config PMDL1WAY = ON            // Peripheral Module Disable Configuration (Allow only one reconfiguration)
#pragma config IOL1WAY = ON             // Peripheral Pin Select Configuration (Allow only one reconfiguration)
#pragma config FUSBIDIO = ON            // USB USBID Selection (Controlled by the USB Module)

// DEVCFG2
#pragma config FPLLIDIV = DIV_8         // System PLL Input Divider (8x Divider)
#pragma config FPLLRNG = RANGE_34_68_MHZ// System PLL Input Range (34-68 MHz Input)
#pragma config FPLLICLK = PLL_FRC       // System PLL Input Clock Selection (FRC is input to the System PLL)
#pragma config FPLLMULT = MUL_128       // System PLL Multiplier (PLL Multiply by 128)
#pragma config FPLLODIV = DIV_32        // System PLL Output Clock Divider (32x Divider)
#pragma config UPLLFSEL = FREQ_24MHZ    // USB PLL Input Frequency Selection (USB PLL input is 24 MHz)

// DEVCFG1
#pragma config FNOSC = FRCDIV           // Oscillator Selection Bits (Fast RC Osc w/Div-by-N (FRCDIV))

#pragma config DMTINTV = WIN_127_128    // DMT Count Window Interval (Window/Interval value is 127/128 counter value)
#pragma config FSOSCEN = ON             // Secondary Oscillator Enable (Enable SOSC)
#pragma config IESO = ON                // Internal/External Switch Over (Enabled)
#pragma config POSCMOD = OFF            // Primary Oscillator Configuration (Primary osc disabled)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FCKSM = CSECME           // Clock Switching and Monitor Selection (Clock Switch Enabled, FSCM Enabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config WDTSPGM = STOP           // Watchdog Timer Stop During Flash Programming (WDT stops during Flash programming)
#pragma config WINDIS = NORMAL          // Watchdog Timer Window Mode (Watchdog Timer is in non-Window mode)
#pragma config FWDTEN = ON              // Watchdog Timer Enable (WDT Enabled)
#pragma config FWDTWINSZ = WINSZ_25     // Watchdog Timer Window Size (Window size is 25%)
#pragma config DMTCNT = DMT31           // Deadman Timer Count Selection (2^31 (2147483648))
#pragma config FDMTEN = ON              // Deadman Timer Enable (Deadman Timer is enabled)

// DEVCFG0
#pragma config DEBUG = OFF              // Background Debugger Enable (Debugger is disabled)
#pragma config JTAGEN = ON              // JTAG Enable (JTAG Port Enabled)
#pragma config ICESEL = ICS_PGx1        // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
#pragma config TRCEN = ON               // Trace Enable (Trace features in the CPU are enabled)
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
#pragma config TSEQ = 0xFFFF            // Boot Flash True Sequence Number (Enter Hexadecimal value)
#pragma config CSEQ = 0xFFFF            // Boot Flash Complement Sequence Number (Enter Hexadecimal value)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#define SLAVE_ADDR 0x76
#include <xc.h>

void i2c_master_setup(void) ;

void i2c_master_restart(void);
void i2c_master_send(unsigned char byte);
unsigned char i2c_master_recv(void);
void i2c_master_stop(void);
void i2c_master_ack(int val);
void bme_280_setup(void);
void lecture_chip_id(void);

void lec_acc(unsigned char data[]);
int a;
void i2c_master_setup(void)
{
I2C1BRG = 390; // I2CBRG = [1/(2*Fsck) - PGD]*Pblck - 2
// Fsck is the freq (1,275 kHz here), PGD = 104 ns and Pblck +1micros // 

I2C1CONbits.ON = 1; // turn on the I2C1 module
}

// Start a transmission on the I2C bus // fct start
void i2c_master_start(void)
{
I2C1CONbits.SEN = 1; // send the start bit
while(I2C1CONbits.SEN) { ; } // wait for the start bit to be sent
}

void i2c_master_restart(void) // fct restart
{
I2C1CONbits.RSEN = 1; // send a restart
while(I2C1CONbits.RSEN) { ; } // wait for the restart to clear
}

void i2c_master_send(unsigned char byte)
{ // send a byte to slave
a=0;
I2C1TRN = byte; // if an address, bit0=0forwrite, 1 for read
while(I2C1STATbits.TRSTAT)
{ ;
} // wait for the transmission to finish
if(I2C1STATbits.ACKSTAT)
{ // if this is high, slave has not acknowledged
a=1;
}
}

unsigned char i2c_master_recv(void)  
{ // receive a byte from the slave
I2C1CONbits.RCEN = 1; // start receiving data
while(!I2C1STATbits.RBF) { ; } // wait to receive the data
return I2C1RCV; // read and return the data
}


void i2c_master_ack(int val)
{ // sends ACK = 0 (slave should send another byte)
// or NACK = 1 (no more bytes requested from slave)
I2C1CONbits.ACKDT = val; // store ACK/NACK in ACKDT
I2C1CONbits.ACKEN = 1; // send ACKDT
while(I2C1CONbits.ACKEN) { ; } // wait for ACK/NACK to be sent
}

void i2c_master_stop(void) // fct stop ok
{ // send a STOP:
I2C1CONbits.PEN = 1; // comm is complete and master relinquishes bus
while(I2C1CONbits.PEN) { ; } // wait for STOP to complete
}

void lec_acc(unsigned char data[]) 
{
   i2c_master_start();
        i2c_master_send(SLAVE_ADDR << 1); // send the slave address, left shifted by 1,
        
        i2c_master_send(0xF7); // bit 7 of adress register must be @1 for transmitting several bytes
       // i2c_master_send(master_write1); // send another byte to the slave
        
        i2c_master_restart();
        i2c_master_send((SLAVE_ADDR << 1) | 1); // send slave address, left shifted by 1

        data[0]= i2c_master_recv(); // receive a byte from the bus
        i2c_master_ack(0); // send ACK (0): master wants another byte!
        
        data[1] = i2c_master_recv(); // receive a byte from the bus
        i2c_master_ack(0); // send ACK (0): master wants another byte!
        
        data[2] = i2c_master_recv(); // receive a byte from the bus
        i2c_master_ack(0); // send ACK (0): master wants another byte!
        
        data[3] = i2c_master_recv(); // receive a byte from the bus
        i2c_master_ack(0); // send ACK (0): master wants another byte!
        
        data[4]= i2c_master_recv(); // receive a byte from the bus
        i2c_master_ack(0); // send ACK (0): master wants another byte!
        
        data[5]= i2c_master_recv(); // receive a byte from the bus
     
        i2c_master_ack(1); // send NACK (1): master needs no more bytes
        i2c_master_stop(); // send STOP: end transmission, give up bus  
}

void bme_280_setup(void){
    
    i2c_master_start();
    i2c_master_send(SLAVE_ADDR<<1);
    i2c_master_send(0xF4);
    i2c_master_send(0b10101011); // osrs_t -> 101 (16) | osrs_p -> 010 (2) | Mode -> 11 (Normal)
    i2c_master_send(0xF5);
    i2c_master_send(0b00010000); // t_sb -> 000 (5ms) | filter -> 100 (16) | spi -> (?)0 
    i2c_master_stop();
}

void lecture_chip_id(void){
    unsigned char id = 0;
    
    i2c_master_start();
    i2c_master_send(SLAVE_ADDR<<1);
    i2c_master_send(0xD0);
    
    i2c_master_restart(); // Ptet Start ?
    i2c_master_send((SLAVE_ADDR<<1)|1);
    id = i2c_master_recv();
    i2c_master_ack(1);
    i2c_master_stop();
    
}
void main ()
{

TRISDbits.TRISD14 = 0 ; 
LATDbits.LATD14 = 0 ;
unsigned char master_write0 = 0x0F; // first byte that master writes
unsigned char master_read0 =0x00;// byte received
unsigned char data[8] = {0} ;
unsigned char pressure = 0;

i2c_master_setup();

bme_280_setup();
lecture_chip_id();

while(1)
    {  
    lec_acc(data);
    pressure = (data[0]<<12 | data[1]<<4 | data[2]>>4); 
    }

} 
 