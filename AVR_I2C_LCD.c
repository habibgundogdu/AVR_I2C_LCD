/*
 * File:   I2C_LCD.c
 * Author: habib Gundogdu
 *
 * Created on October 6, 2023, 2:59 PM
 */

#include <avr/io.h>
#include "AVR_I2C_LCD.h"
#include <util/delay.h>

uint8_t i2c_lcd_adres = 0x27;  //PCF8574 adresi
uint8_t i2c_lcd_reg = 0x4e; //PCF8574 write registeri adresi


//void I2c_Lcd_Init(uint8_t LCD_ADDRESI)

void I2c_Lcd_Init(void) {
    _delay_ms(40); // wait for >40ms
    I2c_Lcd_Send_Cmd(0x33);
	_delay_ms(1);
    I2c_Lcd_Send_Cmd(0x32);		    		/* send for 4 bit initialization of LCD  */
    _delay_ms(1);
	I2c_Lcd_Send_Cmd(0x28);              	/* Use 2 line and initialize 5*7 matrix in (4-bit mode)*/
    _delay_ms(1);
	I2c_Lcd_Send_Cmd(0x0c);              	/* Display on cursor off*/
	_delay_ms(1);
    I2c_Lcd_Send_Cmd(0x06);              	/* Increment cursor (shift cursor to right)*/
	_delay_ms(1);
    I2c_Lcd_Send_Cmd(0x01);              	/* Clear display screen*/
	_delay_ms(2);
	I2c_Lcd_Send_Cmd (0x80);					/* Cursor 1st row 0th position */
    _delay_ms(1);
    
//    I2c_Lcd_Send_Cmd(0x20); // 4bit mode
//    // dislay initialisation
//    I2c_Lcd_Send_Cmd(0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
//    I2c_Lcd_Send_Cmd(0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
//    I2c_Lcd_Send_Cmd(0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
//    I2c_Lcd_Send_Cmd(0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)

}
void I2c_Lcd_Send_Cmd(uint8_t data) {
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (data & 0xf0);
    data_l = ((data << 4)&0xf0);
    data_t[0] = data_u | 0x0C; //LED=1,en=1, RW=0, rs=0
    I2C_write1ByteRegister(i2c_lcd_adres, i2c_lcd_reg, data_t[0]);
    data_t[1] = data_u | 0x08; //LED=1,en=0, RW=0, rs=0
    I2C_write1ByteRegister(i2c_lcd_adres, i2c_lcd_reg, data_t[1]);
    data_t[2] = data_l | 0x0C; //LED=1,en=1, RW=0, rs=0
    I2C_write1ByteRegister(i2c_lcd_adres, i2c_lcd_reg, data_t[2]);
    data_t[3] = data_l | 0x08; //LED=1,en=0, RW=0, rs=0
    I2C_write1ByteRegister(i2c_lcd_adres, i2c_lcd_reg, data_t[3]);
    
}


// D7-D6-D5-D4-LED-EN-RW-RS --<lcd

// https://controllerstech.com/i2c-in-esp32-esp-idf-lcd-1602/   adresinde ayr?nt?l? anlat?lm??.
//BYTE dizilisimiz yukaridaki sekilde PCF8574 ba?lant?s?na göre
// P7-P6-P5-P4-P3 -P2-P1-P0 --<pcf8574
//void I2c_Lcd_Send_Cmd(uint8_t LCD_ADDRESI,uint8_t data)

void I2c_Lcd_Send_Byte(uint8_t data) {
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (data & 0xf0);
    data_l = ((data << 4)&0xf0);
    data_t[0] = data_u | 0x0D; //LED=1,en=1, RW=0, rs=1
    I2C_write1ByteRegister(i2c_lcd_adres, i2c_lcd_reg, data_t[0]);
    data_t[1] = data_u | 0x09; //LED=1,en=0, RW=0, rs=1
    I2C_write1ByteRegister(i2c_lcd_adres, i2c_lcd_reg, data_t[1]);
    data_t[2] = data_l | 0x0D; //LED=1,en=1, RW=0, rs=1
    I2C_write1ByteRegister(i2c_lcd_adres, i2c_lcd_reg, data_t[2]);
    data_t[3] = data_l | 0x09; //LED=1,en=0, RW=0, rs=1
    I2C_write1ByteRegister(i2c_lcd_adres, i2c_lcd_reg, data_t[3]);
}

//void I2c_Lcd_Send_String (uint8_t LCD_ADDRESI,const char *str){

void I2c_Lcd_Send_String(const char *str) {
    int i;
    for (i = 0; (str[i] != 0)&&(i < 64); i++) /* Send each char of string till the NULL */ {
        I2c_Lcd_Send_Byte(str[i]);
    }
    // en basiti ileasagidaki kod
    //while (*str) lcd_send_data (*str++);
}

void I2c_Lcd_Clear (void)
{
	I2c_Lcd_Send_Cmd (0x01);
	_delay_ms(1);
}
void I2c_Lcd_Set_Cursor (unsigned char satir,unsigned char sutun){
    unsigned char firstCharAdr[] = {0x80, 0xC0, 0x94, 0xD4};
    I2c_Lcd_Send_Cmd(firstCharAdr[satir] + sutun );
    _delay_us(100);
}

void I2c_Lcd_Cursor_Blink(void)	//displays LCD blinking cursor
{
	I2c_Lcd_Send_Cmd(0x0F);
}

void I2c_Lcd_Shift_Left(void)
{
    I2c_Lcd_Send_Cmd(0x18);
}
void I2c_Lcd_Shift_Right(void)
{
    I2c_Lcd_Send_Cmd(0x1C);
}

void I2C_write1ByteRegister(twi_address_t address, uint8_t reg, uint8_t data) // twi_address_t uint8_t olarak define edilmi?
{
    while (!I2C_Open(address)); // sit here until we get the bus..
    I2C_SetDataCompleteCallback(wr1RegCompleteHandler, &data);
    I2C_SetBuffer(&reg, 1);
    I2C_SetAddressNackCallback(I2C_SetRestartWriteCallback, NULL); //NACK polling?
    I2C_MasterWrite();
    while (TWI_BUSY == I2C_Close()); // sit here until finished.
}

static twi_operations_t wr1RegCompleteHandler(void *ptr) {
    I2C_SetBuffer(ptr, 1);
    I2C_SetDataCompleteCallback(NULL, NULL);
    return TWI_CONTINUE;
}
