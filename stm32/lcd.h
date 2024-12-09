#ifndef LCD_H
#define LCD_H

#include "string.h"
#include "main.h"

/*-------controle do cursor---------*/
#define cursor_off 	 0x0c //Apagado
#define cursor_on 	 0x0e //Ligado
#define cursor_blink 0x0f //Piscante
/*-------pinos de controle---------*/
#define E_0  GPIOC -> BSRR  = 1<<(7+16)
#define E_1  GPIOC -> BSRR = 1<<7
#define RS_0 GPIOA -> BSRR  = 1<<(9+16)
#define RS_1 GPIOA -> BSRR = 1<<9
/*-------barramento de controle---------*/
#define D4_0 GPIOB -> BSRR  = 1<<(5+16)
#define D4_1 GPIOB -> BSRR = 1<<5
#define D5_0 GPIOB -> BSRR  = 1<<(4+16)
#define D5_1 GPIOB -> BSRR = 1<<4
#define D6_0 GPIOB -> BSRR  = 1<<(14+16)
#define D6_1 GPIOB -> BSRR = 1<<14
#define D7_0 GPIOA -> BSRR  = 1<<(8+16)
#define D7_1 GPIOA -> BSRR = 1<<8

#define backlight_off (GPIOB->BSRR = (1<<16))
#define backlight_on (GPIOB -> BSRR = (1<<0))

/***Funções do LCD***/
void lcd_wrcom4b(uint8_t com);
void lcd_wrcom(uint8_t com);
void lcd_wrchar(char ch);
void lcd_init(uint8_t cursor);
void lcd_clear(void);
void lcd_goto(uint8_t x, uint8_t y);
void lcd_wrstr(char *str);
void lcd_backlight(uint8_t backlight);
void lcd_progchar(uint8_t nchar, uint8_t *data);
#endif // LCD_H
