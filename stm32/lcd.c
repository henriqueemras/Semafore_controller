#include "lcd.h"
#include "main.h"

void udelay(void) {
	int tempo = 7;
	while (tempo--)
		;
}
void delayus(int tempo) {
	while (tempo--)
		udelay();
}
void SendData(uint8_t com) {
	if ((com & (1 << 0)) == 0)
		D4_0;
	else
		D4_1;
	if ((com & (1 << 1)) == 0)
		D5_0;
	else
		D5_1;
	if ((com & (1 << 2)) == 0)
		D6_0;
	else
		D6_1;
	if ((com & (1 << 3)) == 0)
		D7_0;
	else
		D7_1;
}
void lcd_wrcom4b(uint8_t com) {
	SendData(com);
	RS_0;
	E_1;
	delayus(5);  //5us
	E_0;
	HAL_Delay(5); //5ms
}

void lcd_wrcom(uint8_t com) {
	SendData(com >> 4);
	RS_0;
	E_1;
	delayus(5);  //5us
	E_0;
	delayus(5);  //5us

	SendData(com & 0x0f);
	RS_0;
	E_1;
	delayus(5);  //5us
	E_0;
	HAL_Delay(5); //5ms
}

void lcd_wrchar(char ch) {
    // Envia os 4 bits mais significativos
    SendData(ch >> 4);
    RS_1;               // Define RS como 1 (modo dados)
    E_1;                // Gera pulso Enable
    delayus(1);         // Pulso mínimo
    E_0;
    delayus(1);

    // Envia os 4 bits menos significativos
    SendData(ch & 0x0F);
    E_1;
    delayus(1);
    E_0;

    HAL_Delay(2);       // Reduzido para 2 ms
}

void lcd_init(uint8_t cursor) {
	lcd_wrcom4b(3);
	lcd_wrcom4b(3);
	lcd_wrcom4b(3);
	lcd_wrcom4b(2);
	lcd_wrcom(0x28);
	lcd_wrcom(cursor);
	lcd_wrcom(0x06);
	lcd_wrcom(0x01);
	lcd_backlight(1);
}
void lcd_backlight(uint8_t backlight){
	if (backlight)
	    backlight_on;
	else
	    backlight_off;
}
void lcd_clear(void) {
	lcd_wrcom(0x01);
}
void lcd_goto(uint8_t x, uint8_t y) {
    uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
    lcd_wrcom(0x80 + row_offsets[y] + x);
}
void lcd_wrstr(char *str) {
	while (*str)
		lcd_wrchar(*(str++));
}
void lcd_progchar(uint8_t pos, uint8_t *data) {
    // Limitar pos a 0-7
    pos &= 0x7;

    // Calcular o endereço inicial na CGRAM
    uint8_t cgram_addr = 0x40 | (pos << 3);

    // Enviar comando para setar CGRAM
    lcd_wrcom(cgram_addr);

    // Escrever os 8 bytes do caractere
    for (int i = 0; i < 8; i++) {
        uint8_t byte = data[i];
        lcd_wrchar(byte); // Divide em nibbles na função interna
    }

    // Retornar para DDRAM
    lcd_wrcom(0x80);  // Cursor volta ao início da DDRAM
}
