#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#define SUCCESS 5
#define FAIL 10
#define IDK 3
#define PCA9555_0_ADDRESS 0x40 //A0=A1=A2=0 by hardware
#define TWI_READ 1 // reading from twi device
#define TWI_WRITE 0 // writing to twi device
#define SCL_CLOCK 100000L // twi clock in Hz
//Fscl=Fcpu/(16+2*TWBR0_VALUE*PRESCALER_VALUE)
#define TWBR0_VALUE ((F_CPU/SCL_CLOCK)-16)/2
// PCA9555 REGISTERS
typedef enum {
    REG_INPUT_0 = 0,
    REG_INPUT_1 = 1,
    REG_OUTPUT_0 = 2,
    REG_OUTPUT_1 = 3,
    REG_POLARITY_INV_0 = 4,
    REG_POLARITY_INV_1 = 5,
    REG_CONFIGURATION_0 = 6,
    REG_CONFIGURATION_1 = 7
} PCA9555_REGISTERS;

uint16_t lastscan = 0x0000;
//----------- Master Transmitter/Receiver -------------------
#define TW_START 0x08
#define TW_REP_START 0x10
//---------------- Master Transmitter ----------------------
#define TW_MT_SLA_ACK 0x18
#define TW_MT_SLA_NACK 0x20
#define TW_MT_DATA_ACK 0x28
//---------------- Master Receiver ----------------
#define TW_MR_SLA_ACK 0x40
#define TW_MR_SLA_NACK 0x48
#define TW_MR_DATA_NACK 0x58
#define TW_STATUS_MASK 0b11111000
#define TW_STATUS (TWSR0 & TW_STATUS_MASK)

//initialize TWI clock
void twi_init(void){
	TWSR0 = 0; // PRESCALER_VALUE=1
	TWBR0 = TWBR0_VALUE; // SCL_CLOCK 100KHz
}
// Read one byte from the twi device (request more data from device)
unsigned char twi_readAck(void){
	TWCR0 = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	while(!(TWCR0 & (1<<TWINT)));
	return TWDR0;
}
//Read one byte from the twi device, read is followed by a stop condition
unsigned char twi_readNak(void){
	TWCR0 = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR0 & (1<<TWINT)));
	return TWDR0;
}
// Issues a start condition and sends address and transfer direction.
// return 0 = device accessible, 1= failed to access device
unsigned char twi_start(unsigned char address){
	uint8_t twi_status;
	// send START condition
	TWCR0 = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
	// wait until transmission completed
	while(!(TWCR0 & (1<<TWINT)));
	// check value of TWI Status Register.
	twi_status = TW_STATUS & 0xF8;
	if ( (twi_status != TW_START) && (twi_status != TW_REP_START)) return 1;
	// send device address
	TWDR0 = address;
	TWCR0 = (1<<TWINT) | (1<<TWEN);
	// wail until transmission completed and ACK/NACK has been received
	while(!(TWCR0 & (1<<TWINT)));
	// check value of TWI Status Register.
	twi_status = TW_STATUS & 0xF8;
	if ( (twi_status != TW_MT_SLA_ACK) && (twi_status != TW_MR_SLA_ACK) )
	{
	return 1;
	}
	return 0;
}
// Send start condition, address, transfer direction.
// Use ack polling to wait until device is ready
void twi_start_wait(unsigned char address){
	uint8_t twi_status;
	while ( 1 )
    	{
    	// send START condition
    	TWCR0 = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    	// wait until transmission completed
    	while(!(TWCR0 & (1<<TWINT)));
    	// check value of TWI Status Register.
    	twi_status = TW_STATUS & 0xF8;
    	if ( (twi_status != TW_START) && (twi_status != TW_REP_START)) continue;
    	// send device address
    	TWDR0 = address;
    	TWCR0 = (1<<TWINT) | (1<<TWEN);
    	// wail until transmission completed
    	while(!(TWCR0 & (1<<TWINT)));
    	// check value of TWI Status Register.
    	twi_status = TW_STATUS & 0xF8;
    	if ( (twi_status == TW_MT_SLA_NACK )||(twi_status ==TW_MR_DATA_NACK) )
    	{
    	/* device busy, send stop condition to terminate write operation */
    	TWCR0 = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
    	// wait until stop condition is executed and bus released
    	while(TWCR0 & (1<<TWSTO));
    	continue;
    	}
    	break;
	}
}
// Send one byte to twi device, Return 0 if write successful or 1 if write failed
unsigned char twi_write( unsigned char data ){
	// send data to the previously addressed device
	TWDR0 = data;
	TWCR0 = (1<<TWINT) | (1<<TWEN);
	// wait until transmission completed
	while(!(TWCR0 & (1<<TWINT)));
	if( (TW_STATUS & 0xF8) != TW_MT_DATA_ACK) return 1;
	return 0;
}
// Send repeated start condition, address, transfer direction
//Return: 0 device accessible
// 1 failed to access device
unsigned char twi_rep_start(unsigned char address){
	return twi_start( address );
}
// Terminates the data transfer and releases the twi bus
void twi_stop(void){
	// send stop condition
	TWCR0 = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	// wait until stop condition is executed and bus released
	while(TWCR0 & (1<<TWSTO));
}
void PCA9555_0_write(PCA9555_REGISTERS reg, uint8_t value){
	twi_start_wait(PCA9555_0_ADDRESS + TWI_WRITE);
	twi_write(reg);
	twi_write(value);
	twi_stop();
	}
uint8_t PCA9555_0_read(PCA9555_REGISTERS reg){
	uint8_t ret_val;
	twi_start_wait(PCA9555_0_ADDRESS + TWI_WRITE);
	twi_write(reg);
	twi_rep_start(PCA9555_0_ADDRESS + TWI_READ);
	ret_val = twi_readNak();
	twi_stop();
	return ret_val;
}

uint8_t set;

void write_2_nibbles(uint8_t);

void lcd_command(uint8_t command){
    int a = PCA9555_0_read(REG_OUTPUT_0);
    a&=0b11111011;
    PCA9555_0_write(REG_OUTPUT_0,a);
    //PORTD &= ~(1 << PD2); //Clear LCD_RS
    write_2_nibbles(command);
    _delay_us(250);
}

void lcd_data(uint8_t data){
    int a=PCA9555_0_read(REG_OUTPUT_0);
    a|=0b00000100;    
    PCA9555_0_write(REG_OUTPUT_0,a);
    //PORTD |= (1 << PD2); //Set LCD_RS
    write_2_nibbles(data);
    _delay_us(250);
}

void lcd_string(const char* str){
    for (size_t i = 0; i < strlen(str); i++) {
        if (str[i]!='\n')
            lcd_data(str[i]);  // Display each character
    }
}

void lcd_clear_display(void){
    lcd_command(0x01); //Clear LCD display
    _delay_ms(5);
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
    // Calculate the DDRAM address for the specified row and column
    uint8_t address = 0x40 * row + col;
    
    // Set the cursor position
    lcd_command(0x80 | address);
}

void write_2_nibbles(uint8_t data){
    uint8_t data_l, temp = data;
    data_l=PCA9555_0_read(REG_INPUT_0);
    //data_l = PIND;
    data_l &= 0x0F;
    
    data &= 0xF0;
    //data |= data_l;
    data += data_l;
    PCA9555_0_write(REG_OUTPUT_0,data);
    //PORTD = data;
   // int a = PCA9555_0_read(REG_OUTPUT_0); unecessary
    data |=0b00001000;
    PCA9555_0_write(REG_OUTPUT_0,data);
    //PORTD |= (1 << PD3);
    asm("nop");
    asm("nop");
//    int b = PCA9555_0_read(REG_OUTPUT_0);
    data &= 0b11110111;
    PCA9555_0_write(REG_OUTPUT_0,data);   
    //PORTD &= ~(1 << PD3);
    
    
    data = temp;
    data = (data << 4) | (data >> 4);
    data &= 0xF0;
    data += data_l;
    PCA9555_0_write(REG_OUTPUT_0,data);   
    //PORTD = data;
    
//    int c = PCA9555_0_read(REG_OUTPUT_0);
    data|=0b00001000;
    PCA9555_0_write(REG_OUTPUT_0,data);
    asm("nop");
    asm("nop");
    //PORTD |= (1 << PD3);
//    int d = PCA9555_0_read(REG_OUTPUT_0);
    data&=0b11110111;
    PCA9555_0_write(REG_OUTPUT_0,data);   
    //PORTD &= ~(1 << PD3);    
}

void lcd_init(void){
    _delay_ms(200);
    uint8_t set=0x30;
    PCA9555_0_write(REG_OUTPUT_0,set);   
    //PORTD = set;
    int c = PCA9555_0_read(REG_OUTPUT_0);
    c|=0b00001000;
    PCA9555_0_write(REG_OUTPUT_0,c);
    //PORTD |= (1 << PD3);
    int d = PCA9555_0_read(REG_OUTPUT_0);
    d&=0b11110111;
    PCA9555_0_write(REG_OUTPUT_0,d);   
    //PORTD &= ~(1 << PD3); 
    _delay_ms(250);
    
    _delay_ms(250);
    set=0x30;
    PCA9555_0_write(REG_OUTPUT_0,set);   
    //PORTD = set;    
    c = PCA9555_0_read(REG_OUTPUT_0);
    c|=0b00001000;
    PCA9555_0_write(REG_OUTPUT_0,c);
    //PORTD |= (1 << PD3);
    d = PCA9555_0_read(REG_OUTPUT_0);
    d&=0b11110111;
    PCA9555_0_write(REG_OUTPUT_0,d);   
    //PORTD &= ~(1 << PD3); 
    _delay_ms(250);
    _delay_ms(250);

    set=0x30;
    PCA9555_0_write(REG_OUTPUT_0,set);   
    //PORTD = set;    
    c = PCA9555_0_read(REG_OUTPUT_0);
    c|=0b00001000;
    PCA9555_0_write(REG_OUTPUT_0,c);
    //PORTD |= (1 << PD3);
    d = PCA9555_0_read(REG_OUTPUT_0);
    d&=0b11110111;
    PCA9555_0_write(REG_OUTPUT_0,d);   
    //PORTD &= ~(1 << PD3); 
    _delay_ms(250);
    
    
    PCA9555_0_write(REG_OUTPUT_0, 0x20);
    d = PCA9555_0_read(REG_OUTPUT_0);
    d|=0x08;
    PCA9555_0_write(REG_OUTPUT_0,d);   
    asm("nop");   
    asm("nop");
    d = PCA9555_0_read(REG_OUTPUT_0);
    d&=0b11110111;
    PCA9555_0_write(REG_OUTPUT_0,d);   
    _delay_ms(250);

    set = 0x28;
    lcd_command(set);
    
    set = 0x0c;
    lcd_command(set);
    
    lcd_clear_display();
    lcd_set_cursor(0,0);
    lcd_command(0x06);
}

int scan_row(int row){
    int value;
    int r = row;
    r = ~(1 << row);
    PCA9555_0_write(REG_CONFIGURATION_0, 0x00); //Set EXT_PORT0 as output
    PCA9555_0_write(REG_CONFIGURATION_1, r); //Set EXT_PORT1
    PCA9555_0_write(REG_OUTPUT_1,0x00);
    PCA9555_0_write(REG_OUTPUT_0,0x00);
    value = PCA9555_0_read(REG_INPUT_1);
    value = ~(value >> 4);
    PCA9555_0_write(REG_OUTPUT_0, value);       //this is not needed
    return value;
}

uint16_t scan_keypad(void){
    uint16_t value = 0;
    for(int i=0; i<4; i++){
       int row = scan_row(i);
       value |= (row & 0x0F) << (4*i);
    }
    return value;
}

uint16_t scan_keypad_rising_edge(void){
    uint16_t now, change;
    _delay_ms(10);
    now = scan_keypad();
    change = now ^ lastscan;        // '^' means XOR
    lastscan = now;
    return change;
}

int keypad_to_ascii(void){
    uint16_t keypadValue = scan_keypad();
    switch(keypadValue) {
        case 0b0000000000000001: return 42;     //*
        case 0b0000000000000010: return 48;     //0
        case 0b0000000000000100: return 35;     //#
        case 0b0000000000001000: return 68;     //D
        case 0b0000000000010000: return 55;     //7
        case 0b0000000000100000: return 56;     //8
        case 0b0000000001000000: return 57;     //9
        case 0b0000000010000000: return 67;     //C
        case 0b0000000100000000: return 52;     //4
        case 0b0000001000000000: return 53;     //5
        case 0b0000010000000000: return 54;     //6
        case 0b0000100000000000: return 66;     //B
        case 0b0001000000000000: return 49;     //1
        case 0b0010000000000000: return 50;     //2
        case 0b0100000000000000: return 51;     //3
        case 0b1000000000000000: return 65;     //A
        default: return 0;                      //None
    }
}


/* Routine: usart_init
Description:
This routine initializes the
usart as shown below.
------- INITIALIZATIONS -------
Baud rate: 9600 (Fck= 8MH)
Asynchronous mode
Transmitter on
Reciever on
Communication parameters: 8 Data ,1 Stop, no Parity
--------------------------------
parameters: ubrr to control the BAUD.
return value: None.*/

void usart_init(unsigned int ubrr){
    UCSR0A=0;
    UCSR0B=(1<<RXEN0)|(1<<TXEN0);
    UBRR0H=(unsigned char)(ubrr>>8);
    UBRR0L=(unsigned char)ubrr;
    UCSR0C=(3 << UCSZ00);
    return;
}
/* Routine: usart_transmit
Description:
This routine sends a byte of data
5
using usart.
parameters:
data: the byte to be transmitted
return value: None. */
void usart_transmit(uint8_t data){
    while(!(UCSR0A&(1<<UDRE0))); //wait until be ready to receive new data
    UDR0=data;
}
 
/* Routine: usart_receive
Description:
This routine receives a byte of data
from usart.
parameters: None.
return value: the received byte */
uint8_t usart_receive(){
    while(!(UCSR0A&(1<<RXC0))); //wait until there is data for reading
    return UDR0;
}

uint8_t Helper1, Helper2;

void one_wire_reset() {
    
    DDRD |= (1<<PD4);   // set PD4 as output
    
    PORTD &= ~(1 << PD4);    
    _delay_us(480);
    
    DDRD &= ~(1 << PD4);  // Set PD4 as input
    PORTD &= ~(1 << PD4); // Disable pull-up
    
    _delay_us(100);
     
    Helper1 = PIND;
    Helper1 = (Helper1>>4);     // put pd4 in msb.   
     
    _delay_us(380);
    
    if(Helper1 == 0)    Helper1 = 0x01;
    else            Helper1 = 0x00;

   
    
}

void one_wire_receive_bit() {
     
    DDRD |= (1<<PD4);   // set PD4 as output
    PORTD &= ~(1<<PD4);
    
    _delay_us(2);
    
     DDRD &= ~(1<<PD4);   // set PD4 as input
     PORTD &= ~(1<<PD4);
     
     _delay_us(10);
     
     Helper1 = 0;
     
     Helper1 = PIND;
     Helper1 = (Helper1>>4);
     
     if(Helper1 == 0) Helper1 = 0x00;
     else         Helper1 = 0x01;
     
     _delay_us(49);
     
}

void one_wire_transmit_bit() {
    
    DDRD |= (1<<PD4);   // set PD4 as output
    PORTD &= ~(1<<PD4);
    
    _delay_us(2);
   // Helper1 = (Helper1>>4);
    if(Helper1 == 0x00) PORTD &= ~(1<<PD4);
    else            PORTD |= (1<<PD4);
    
    _delay_us(58);
   
    DDRD  &= ~(1<<PD4);   // set PD4 as input
    PORTD &= ~(1<<PD4);
    
    _delay_us(1);
    
}

void one_wire_receive_byte() {
    int i;
    uint8_t Dummy = 0;
    
    for(i=0; i<8; i++) {
        one_wire_receive_bit();
        
        Dummy = (Dummy>>1);
        uint8_t Cache_trans = Helper1;
        Helper1 = (Helper1<<4);
                
        if(Cache_trans == 0)    Dummy |= Helper1;
        else {
            Helper1 = 0x80;
            Dummy |= Helper1;
        }        
    }
    
    Helper1 = Dummy;
}

void one_wire_transmit_byte() {
    
    int i;
    uint8_t Dummy,Cache_trans;
    
    Dummy = Helper1;
    
    for(i=0; i<8; i++) {
        Helper1 = 0;
        Cache_trans = Dummy & 0b00000001;
        if(Cache_trans != 0)   {Helper1 = 0x01;}
        
        one_wire_transmit_bit();
        
        Dummy = (Dummy>>1);
    }
}

void rec_n_prt(uint8_t step) {
    uint8_t b[11];
    uint8_t temp;
//    while((temp=usart_receive())=='\n');
    int i=0;
    while((temp=usart_receive())!='\n'){
        b[i]=temp;
        i++;
    }
    b[i+1]='\0';
    lcd_clear_display();
    lcd_data(step);
    lcd_data('.');
    i=0;
    while (b[i]!='\0') {
        if (b[i]== '"') {
            i++;
            continue;
        }
        lcd_data(b[i]);
        i++;
    }
}

int rec_n_check(void){
    char b[11];
    uint8_t temp;
//    while((temp=usart_receive())=='\n');
    int i=0;
    while((temp=usart_receive())!='\n'){
        b[i]=temp;
        i++;
    }
    b[i+1]='\0';
    // Check the received data
    lcd_clear_display();
    if (strncmp(b, "\"S\"", 2) == 0) {  // Compare only the first 7 characters
        return SUCCESS;
    } else if (strncmp(b, "\"Fail\"", 6) == 0) {  // Compare only the first 4 characters
        return FAIL;
    }
    else return IDK;
    
//    return 0
}

void read_temp(void){
    one_wire_reset();
    Helper1=0xCC;
    one_wire_transmit_byte();
    Helper1=0x44;
    one_wire_transmit_byte();
    one_wire_receive_bit();
    while(Helper1==0)   one_wire_receive_bit();
    one_wire_reset();
    Helper1=0xCC;
    one_wire_transmit_byte();
    Helper1=0xBE;
    one_wire_transmit_byte();
    one_wire_receive_byte();
    Helper2=Helper1;
    one_wire_receive_byte();
}

void status_chk (uint16_t akeraio, uint8_t pressure, char *status) {
        if(akeraio>=37 || akeraio<34) strcpy(status,"CHECKTEMP");

        //Check the pressure
        if((pressure>12 || pressure<4) && pressure!=0) strcpy(status,"CHECKPRESSURE");
        else if (pressure==0)
            strcpy(status,"RIP");
}

int nurseReq=0;

int main(void) {
    twi_init();
    PCA9555_0_write(REG_CONFIGURATION_0, 0x00);
    lcd_init();
    lcd_clear_display();
    read_temp(); //initialize the thermometer for 1st time
    
    
    uint8_t fir=0;
    
    UBRR0H=0b00000000; // Pace of data transfer-16bit in total, this is high 8 bits of number 103
    UBRR0L=0b01100111; // low bits of 103
    uint16_t ubrr=0b01100111;
    usart_init(ubrr);
    lcd_clear_display();
    
    lcd_string("ready");
    _delay_ms(100);
    while(1){
        lcd_clear_display();
        //lcd_string("It connects");
        usart_init(ubrr);
        uint8_t a[]="ESP:connect\n";
        while (1) {
            for (int i=0; i<12; i++) {
                usart_transmit(a[i]);
            }
            int ch = 0;
            while(ch == 0){
                _delay_ms(500);
                ch = rec_n_check();
            }
            _delay_ms(500);
            lcd_clear_display();

            if(ch==SUCCESS) {
                lcd_string("1.Success");
                break;
            }
            else if(ch==FAIL)
                lcd_string("1.Fail");
            else
                lcd_string("IDK");

            _delay_ms(20100);
        }
    //    rec_n_prt('1');
        _delay_ms(1000);

    //==============================================================================
        char url[]="ESP:url:\"http://192.168.1.250:5000/data\"\n";
        while(1) {
            for (int i=0; i<strlen(url); i++) {
                usart_transmit(url[i]);
            }
            int ch = 0;
            while(ch == 0){
                _delay_ms(500);
                ch = rec_n_check();
            }
            lcd_clear_display();
            if(ch==SUCCESS) {
                lcd_string("2.Success");
                break;
            }
            else if(ch==FAIL)
                lcd_string("2.Fail");
            else
                lcd_string("IDK");

            _delay_ms(20100);
        }
    //    rec_n_prt('2');
        _delay_ms(1000);

    //==============================================================================
        Helper1=0, Helper2=0;
        read_temp();
        uint8_t temph=Helper1, templ=Helper2;
        uint16_t temp = (temph & 0b00000111);
        temp = (temp<<8) + templ;
        uint16_t akeraio= temp * 0.0625 + 16;
        char integer[5];
        sprintf(integer, "%d", (int)akeraio); //isolate the integer part as a string
        if (fir==0) {
            fir++;
            read_temp();
            temph=Helper1, templ=Helper2;
            temp = (temph & 0b00000111);
            temp = (temp<<8) + templ;
            akeraio= temp * 0.0625 + 14;
            sprintf(integer, "%d", (int)akeraio); //isolate the integer part as a string
        }


        uint16_t fract = temp & 0x000F;
        fract = fract * 0.0625 * 1000; 
        char fractional[5]; 
        sprintf(fractional, "%d", (int)fract); //isolate the fractional part as a string

        uint8_t len1 = strlen(integer);
        uint8_t len2= strlen(fractional);

        uint8_t totalsize = len1+len2+1;

        char temperature[totalsize+1];

        strcpy(temperature, integer);
        strcat(temperature, ".");
        strcat(temperature, fractional);

    //    for (int i = 0; i < strlen(temperature); i++) {
    //        usart_transmit(temperature[i]);
    //    }
    //    usart_transmit('\n');

    //==============================================================================
        DDRC = 0x00; 
        ADMUX= 0b01100000; //select 5V, left adjust and POT0 (PC0)
        ADCSRA=0b10000111; //ADC enable, no conversion, no interrupt, fADC=125KHz

        ADCSRA |= (1 << ADSC);
        while (ADSC == 1);
        uint8_t adcResult_h;
        for (int i=0; i<2; i++) {
            adcResult_h=ADCH;
            _delay_ms(50);
        }
        int pressure = (adcResult_h*20)/255; // 20cm reference, 8-bit ADC
        char str_pressure[4];
        sprintf(str_pressure, "%d", pressure);

    //==============================================================================
        char status[15];

        DDRD = 0xFF;
        uint16_t A;
        //Checks
        if(nurseReq==0) strcpy(status,"OK");
        //Check if "6" is press or if "#' is press
        A = keypad_to_ascii();
        if(A==35) nurseReq=0;
        if(nurseReq==0){
            if (A == 54){ 
                strcpy(status,"NURSECALL"); //A="6"
                nurseReq=1;
            } else {
                strcpy(status,"OK");
                status_chk(akeraio, pressure, status);
            }
        }



        lcd_clear_display();
        char firstLine[17],secondLine[17];
        strcpy(firstLine,"T=");
        strcat(firstLine,temperature);
        strcat(firstLine,"  P=");
        strcat(firstLine,str_pressure);
        lcd_string(firstLine);

        lcd_set_cursor(1,0);    
        strcpy(secondLine," ");
        strcat(secondLine,status);
        lcd_string(secondLine);
        _delay_ms(1000);
        int team=26;
        char str_payload[200];      
        strcpy(str_payload,"ESP:payload:[{\"name\":\"temperature\",\"value\":\"");
        strcat(str_payload,temperature);
        strcat(str_payload,"\"},{\"name\":\"pressure\",\"value\":\"");
        strcat(str_payload,str_pressure);
        strcat(str_payload,".0\"},{\"name\":\"team\",\"value\":\"26\"},{\"name\":\"status\",\"value\":\"");
        strcat(str_payload,status);
        strcat(str_payload,"\"}]\n");

        while (1) {
            for (int i=0; i<strlen(str_payload); i++) {
                usart_transmit(str_payload[i]);
            }
            int ch = 0;
            while(ch == 0){
                _delay_ms(500);
                ch = rec_n_check();
            }
            lcd_clear_display();
            if(ch==SUCCESS) {
                lcd_string("3.Success");
                break;
            }
            else if(ch==FAIL) 
                lcd_string("3.Fail");

            else
                lcd_string("IDK");

            _delay_ms(20100);
        }

    //    rec_n_prt('3');
        _delay_ms(1000);

    //==============================================================================
        char tra[13];
        strcpy(tra,"ESP:transmit");
        for (int i=0; i<12; i++) {
            usart_transmit(tra[i]);
        }
        usart_transmit('\n');
        rec_n_prt('4');
        _delay_ms(1000);
      }
}