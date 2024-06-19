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
#include <stdio.h> // Pour sprintf


typedef int16_t BME280_S16_t;
typedef uint16_t BME280_U16_t;
typedef int32_t BME280_S32_t;
typedef uint32_t BME280_U32_t;
typedef uint8_t BME280_U8_t;
typedef int8_t BME280_S8_t;
typedef int64_t BME280_S64_t;

void i2c_master_setup(void);
void i2c_master_start(void);
void i2c_master_restart(void);
void i2c_master_send(unsigned char byte);
unsigned char i2c_master_recv(void);
void i2c_master_stop(void);
void i2c_master_ack(int val);
void bme_280_setup(void);
void lecture_chip_id(void);
unsigned char lecture(unsigned char adresse);
void read_compensation_parameters();
double BME280_compensate_T_double(BME280_S32_t adc_T);
double BME280_compensate_P_double(BME280_S32_t adc_P);
double BME280_compensate_H_double(BME280_S32_t adc_H);
void lec_acc(unsigned char data[]);
void UART_init();
void UART_printChar(char c);
char UART_getChar();
void UART_getString(char *string, int length);
void doubleToString(double value, char* buffer, int precision);

int a;
unsigned char data1 = 0;
unsigned char data2 = 0;
unsigned char data3 = 0;
unsigned char data4 = 0;
unsigned char data5 = 0;
unsigned char data6 = 0;
unsigned char data7 = 0;
unsigned char data8 = 0;
// Definition des types

void UART_init() {
    U2MODEbits.BRGH = 0;                // Baud Rate = 9600
    U2BRG = 25;
    
    U2MODEbits.SIDL = 0;                // Continue operation in SLEEP mode
    U2MODEbits.ABAUD = 0;
    U2MODEbits.IREN = 0;                // IrDA is disabled
    U2MODEbits.RTSMD = 1;               // U1RTS pin is in simplex mode
    U2MODEbits.UEN = 0b00;              // U1TX, U1RX are enabled
    U2MODEbits.WAKE = 1;                // Wake-up enabled
    U2MODEbits.LPBACK = 0;              // Loopback mode is disabled
    U2MODEbits.RXINV = 0;               // U1RX IDLE state is '1'
    U2MODEbits.PDSEL = 0b00;            // 8-bit data, no parity
    U2MODEbits.STSEL = 0;               // 1 stop bit
    U2STAbits.UTXINV = 0;               // U6TX IDLE state is '1'
    U2MODEbits.ON = 1;                  // UART6U is enabled
    U2STAbits.URXEN = 1;                // UART6 receiver is enabled
    U2STAbits.UTXEN = 1;                // UART6 transmitter is enabled

    U2RXR = 12;     // RE9 = U2RX
    RPG9R = 2;      // RG9 = U2TX
    
    TRISEbits.TRISE9 = 1; // input
    TRISGbits.TRISG9 = 0; // output
}
   
void UART_printChar(char c) {
    U2STAbits.UTXEN = 1;                // Make sure transmitter is enabled
    // while(CTS)                       // Optional CTS use
    while(U2STAbits.UTXBF);             // Wait while buffer is full
    U2TXREG = c;                        // Transmit character
}
 
void UART_printString(char *string) {
    U2STAbits.UTXEN = 1;                // Make sure transmitter is enabled
    
    while(*string) {
        while(U2STAbits.UTXBF);         // Wait while buffer is full
        U2TXREG = *string;              // Transmit one character
        string++;                       // Go to next character in string
    }
}
  
char UART_getChar() {
    //PORTDbits.RD15 = 0;                // Optional RTS use
    while(!U2STAbits.URXDA);             // Wait for information to be received
    //PORTDbits.RD15 = 1;
    return U2RXREG;                      // Return received character
}
 
void UART_getString(char *string, int length) {  
    int count = length;
    
    do {
        *string = UART_getChar();               // Read in character
        //SendChar(*string);                  // Echo character
        
        if(*string == 0x7F && count>length) { // Backspace conditional
            length++;
            string--;
            continue;
        }
        
        if(*string == '\r')                 // End reading if enter is pressed
            break;
        
        string++;
        length--;
        
    } while(length>1);
    
    *string = '\0';                         // Add null terminator
}


BME280_U16_t dig_T1;
BME280_S16_t dig_T2;
BME280_S16_t dig_T3;
BME280_U16_t dig_P1;
BME280_S16_t dig_P2;
BME280_S16_t dig_P3;
BME280_S16_t dig_P4;
BME280_S16_t dig_P5;
BME280_S16_t dig_P6;
BME280_S16_t dig_P7;
BME280_S16_t dig_P8;
BME280_S16_t dig_P9;
BME280_U8_t dig_H1;
BME280_S16_t dig_H2;
BME280_U8_t dig_H3;
BME280_S16_t dig_H4;
BME280_S16_t dig_H5;
BME280_S8_t dig_H6;

BME280_S32_t t_fine;

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
    while (I2C1CONbits.SEN) { ; } // wait for the start bit to be sent
}

void i2c_master_restart(void) // fct restart
{
    I2C1CONbits.RSEN = 1; // send a restart
    while (I2C1CONbits.RSEN) { ; } // wait for the restart to clear
}

void i2c_master_send(unsigned char byte)
{ // send a byte to slave
    a = 0;
    I2C1TRN = byte; // if an address, bit0=0 for write, 1 for read
    while (I2C1STATbits.TRSTAT) { ; } // wait for the transmission to finish
    if (I2C1STATbits.ACKSTAT) { // if this is high, slave has not acknowledged
        a = 1;
    }
}

unsigned char i2c_master_recv(void)
{ // receive a byte from the slave
    I2C1CONbits.RCEN = 1; // start receiving data
    while (!I2C1STATbits.RBF) { ; } // wait to receive the data
    return I2C1RCV; // read and return the data
}

void i2c_master_ack(int val)
{ // sends ACK = 0 (slave should send another byte)
    // or NACK = 1 (no more bytes requested from slave)
    I2C1CONbits.ACKDT = val; // store ACK/NACK in ACKDT
    I2C1CONbits.ACKEN = 1; // send ACKDT
    while (I2C1CONbits.ACKEN) { ; } // wait for ACK/NACK to be sent
}

void i2c_master_stop(void) // fct stop ok
{ // send a STOP:
    I2C1CONbits.PEN = 1; // comm is complete and master relinquishes bus
    while (I2C1CONbits.PEN) { ; } // wait for STOP to complete
}

void lec_acc(unsigned char data[])
{
    i2c_master_start();
    i2c_master_send(SLAVE_ADDR << 1); // send the slave address, left shifted by 1

    i2c_master_send(0xF7); // bit 7 of adress register must be @1 for transmitting several bytes

    i2c_master_restart();
    i2c_master_send((SLAVE_ADDR << 1) | 1); // send slave address, left shifted by 1

    data1 = i2c_master_recv(); // receive a byte from the bus
    i2c_master_ack(0); // send ACK (0): master wants another byte!

    data2 = i2c_master_recv(); // receive a byte from the bus
    i2c_master_ack(0); // send ACK (0): master wants another byte!

    data3 = i2c_master_recv(); // receive a byte from the bus
    i2c_master_ack(0); // send ACK (0): master wants another byte!

    data4 = i2c_master_recv(); // receive a byte from the bus
    i2c_master_ack(0); // send ACK (0): master wants another byte!

    data5 = i2c_master_recv(); // receive a byte from the bus
    i2c_master_ack(0); // send ACK (0): master wants another byte!

    data6 = i2c_master_recv(); // receive a byte from the bus
    i2c_master_ack(0); // send ACK (0): master wants another byte!
    
    data7 = i2c_master_recv(); // receive a byte from the bus
    i2c_master_ack(0); // send ACK (0): master wants another byte!
    
    data8 = i2c_master_recv(); // receive a byte from the bus

    i2c_master_ack(1); // send NACK (1): master needs no more bytes
    i2c_master_stop(); // send STOP: end transmission, give up bus
}


void bme_280_setup(void)
{
    i2c_master_start();
    i2c_master_send(SLAVE_ADDR << 1);
    i2c_master_send(0xF4);
    i2c_master_send(0b10101011); // osrs_t -> 101 (16) | osrs_p -> 010 (2) | Mode -> 11 (Normal)
    i2c_master_send(0xF5);
    i2c_master_send(0b00010000); // t_sb -> 000 (5ms) | filter -> 100 (16) | spi -> (?)0 
    i2c_master_stop();
}

void lecture_chip_id(void)
{
    unsigned char id = 0;

    i2c_master_start();
    i2c_master_send(SLAVE_ADDR << 1);
    i2c_master_send(0xD0);

    i2c_master_restart(); // Ptet Start ?
    i2c_master_send((SLAVE_ADDR << 1) | 1);
    id = i2c_master_recv();
    i2c_master_ack(1);
    i2c_master_stop();
}

unsigned char lecture(unsigned char adresse)
{
    unsigned char data = 0;
    i2c_master_start();
    i2c_master_send(SLAVE_ADDR << 1); // send the slave address, left shifted by 1
    i2c_master_send(adresse); // bit 7 of adress register must be @1 for transmitting several bytes
    i2c_master_restart();
    i2c_master_send((SLAVE_ADDR << 1) | 1); // send slave address, left shifted by 1 
    data = i2c_master_recv();
    i2c_master_ack(1); // send NACK (1): master needs no more bytes
    i2c_master_stop(); // send STOP: end transmission, give up bus  
    return data;
}

/*--------------------------------------------------------------------*/




void read_compensation_parameters() {
    dig_T1 = (lecture(0x89) << 8) | lecture(0x88);
    dig_T2 = (lecture(0x8B) << 8) | lecture(0x8A);
    dig_T3 = (lecture(0x8D) << 8) | lecture(0x8C);
    dig_P1 = (lecture(0x8F) << 8) | lecture(0x8E);
    dig_P2 = (lecture(0x91) << 8) | lecture(0x90);
    dig_P3 = (lecture(0x93) << 8) | lecture(0x92);
    dig_P4 = (lecture(0x95) << 8) | lecture(0x94);
    dig_P5 = (lecture(0x97) << 8) | lecture(0x96);
    dig_P6 = (lecture(0x99) << 8) | lecture(0x98);
    dig_P7 = (lecture(0x9B) << 8) | lecture(0x9A);
    dig_P8 = (lecture(0x9D) << 8) | lecture(0x9C);
    dig_P9 = (lecture(0x9F) << 8) | lecture(0x9E);
    dig_H1 = lecture(0xA1);
    dig_H2 = (lecture(0xE2) << 8) | lecture(0xE1);
    dig_H3 = lecture(0xE3);
    dig_H4 = (lecture(0xE4) << 4) | (lecture(0xE5) & 0x0F);
    dig_H5 = (lecture(0xE6) << 4) | (lecture(0xE5) >> 4);
    dig_H6 = (int8_t)lecture(0xE7);
}

double BME280_compensate_T_double(BME280_S32_t adc_T) {
    double var1, var2, T;
    var1 = (((double)adc_T)/16384.0 - ((double)dig_T1)/1024.0) * ((double)dig_T2);
    var2 = ((((double)adc_T)/131072.0 - ((double)dig_T1)/8192.0) * (((double)adc_T)/131072.0 - ((double) dig_T1)/8192.0)) * ((double)dig_T3);
    t_fine = (BME280_S32_t)(var1 + var2);
        // Coefficients pour l'équation linéaire
    double a = 0.0007;
    double b = -82;
    // Calcul de la température compensée
    T = a * (var1 + var2) + b;
    return T;
}

double BME280_compensate_P_double(BME280_S32_t adc_P) {
    double var1, var2, p;
    var1 = ((double)t_fine/2.0) - 64000.0;
    var2 = var1 * var1 * ((double)dig_P6) / 32768.0;
    var2 = var2 + var1 * ((double)dig_P5) * 2.0;
    var2 = (var2/4.0)+(((double)dig_P4) * 65536.0);
    var1 = (((double)dig_P3) * var1 * var1 / 524288.0 + ((double)dig_P2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0)*((double)dig_P1);
    if (var1 == 0.0) {
        return 0; // éviter une division par zéro
    }
    p = 1048576.0 - (double)adc_P;
    p = (p - (var2 / 4096.0)) * 6250.0 / var1;
    var1 = ((double)dig_P9) * p * p / 2147483648.0;
    var2 = p * ((double)dig_P8) / 32768.0;
    p = p + ((var1 + var2 + ((double)dig_P7)) / 16.0) ;
    return p;
}

double BME280_compensate_H_double(BME280_S32_t adc_H) {
    double var_H = 0;
    var_H = (((double)t_fine) - 76800.0);
    var_H = (adc_H - (((double)dig_H4) * 64.0 + ((double)dig_H5) / 16384.0 * var_H)) * (((double)dig_H2) / 65536.0 * (1.0 + ((double)dig_H6) / 67108864.0 * var_H * (1.0 + ((double)dig_H3) / 67108864.0 * var_H)));
    var_H = var_H * (1.0 - ((double)dig_H1) * var_H / 524288.0);
    
    double a = 5.3435;
    double b = -318.6;
    // Calcul de la température compensée
    var_H = a * (var_H) + b;
    var_H = var_H*100;
    if (var_H > 100.0) var_H = 100.0;
    else if (var_H < 0.0) var_H = 0.0;
    return var_H;
}

/*--------------------------------------------------------------------*/
void doubleToString(double value, char* buffer, int precision) {
    int intPart = (int)value; // Partie entière
    double fracPart = value - intPart; // Partie fractionnaire
    int fracAsInt;
    int i;

    // Convertir la partie entière en chaîne
    i = 0;
    if (intPart == 0) {
        buffer[i++] = '0';
    } else {
        if (intPart < 0) {
            buffer[i++] = '-';
            intPart = -intPart;
        }
        int digits[10]; // Temporaire pour stocker les chiffres
        int j = 0;
        while (intPart > 0) {
            digits[j++] = intPart % 10;
            intPart /= 10;
        }
        while (j > 0) {
            buffer[i++] = '0' + digits[--j];
        }
    }

    buffer[i++] = '.'; // Ajouter le point décimal

    // Convertir la partie fractionnaire en chaîne
    fracPart *= 10;
    while (precision > 0) {
        fracAsInt = (int)fracPart;
        buffer[i++] = '0' + fracAsInt;
        fracPart -= fracAsInt;
        fracPart *= 10;
        precision--;
    }

    buffer[i] = '\0'; // Terminer la chaîne
}
/*--------------------------------------------------------------------*/
void configurer_Timer1() 
{
    T1CONbits.TON = 0;      // D sactiver le Timer1 pendant la configuration
    T1CONbits.TCS = 0;      // S lectionner l'horloge interne comme source d'horloge
    T1CONbits.TCKPS = 0b11; // Diviseur de fr quence 256
    PR1 = 249999;            // P riode de 256*5860 cycle d'horloge 
    TMR1 = 0;               // R initialiser le compteur Timer1
    T1CONbits.TON = 1;      // Activer le Timer1
}
/*--------------------------------------------------------------------*/


void main(void)
{
    configurer_Timer1();
    TRISDbits.TRISD14 = 0;
    LATDbits.LATD14 = 0;
    char buffer[50];
    unsigned char master_write0 = 0x0F; // first byte that master writes
    unsigned char master_read0 = 0x00; // byte received
    double temperature = 0;
    double pressure = 0;
    double humidite = 0;

    i2c_master_setup();
    bme_280_setup();
    //lecture_chip_id();
    UART_init();

    unsigned char data[6];

    while (1)
    {
        lec_acc(data);
        BME280_S32_t raw_temp = ((unsigned long)data4 << 12) | ((unsigned long)data5 << 4) | ((data6 >> 4) & 0x0F);
        BME280_S32_t raw_press = ((unsigned long)data1<< 12) | ((unsigned long)data2 << 4) | ((data3 >> 4) & 0x0F);
        BME280_S32_t raw_hum = ((unsigned long)data7 << 8) | ((unsigned long)data8);
        read_compensation_parameters();
        temperature = BME280_compensate_T_double(raw_temp);
        pressure = (BME280_compensate_P_double(raw_press)/100) +14;
        humidite = BME280_compensate_H_double(raw_hum);
        
            // Convertir la valeur de la variable double en chaîne de caractères
        sprintf(buffer, "Temperature: %.3f \n", temperature);
        UART_printString(buffer);
        sprintf(buffer, "Pressure: %.3f \n", pressure);
        UART_printString(buffer);
        sprintf(buffer, "Humidite: %.3f \n", humidite);
        UART_printString(buffer);
        UART_printString("\r\n\n");
        
        IFS0bits.T1IF = 0; // r -initialisation du drapeau li  au timer1
        while ( IFS0bits.T1IF == 0 ); // attente de la lev e du drapeau
    }
}
