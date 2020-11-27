/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2004-2012 by Embedded and Real-Time Systems Laboratory
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2015-2019 by TOPPERS PROJECT Educational Working Group.
 * 
 *  �嵭����Ԥϡ��ʲ���(1)���(4)�ξ������������˸¤ꡤ�ܥ��եȥ���
 *  �����ܥ��եȥ���������Ѥ�����Τ�ޤࡥ�ʲ�Ʊ���ˤ���ѡ�ʣ������
 *  �ѡ������ۡʰʲ������ѤȸƤ֡ˤ��뤳�Ȥ�̵���ǵ������롥
 *  (1) �ܥ��եȥ������򥽡��������ɤη������Ѥ�����ˤϡ��嵭������
 *      ��ɽ�����������Ѿ�浪��Ӳ�����̵�ݾڵ��꤬�����Τޤޤη��ǥ���
 *      ����������˴ޤޤ�Ƥ��뤳�ȡ�
 *  (2) �ܥ��եȥ������򡤥饤�֥������ʤɡ�¾�Υ��եȥ�������ȯ�˻�
 *      �ѤǤ�����Ǻ����ۤ�����ˤϡ������ۤ�ȼ���ɥ�����ȡ�����
 *      �ԥޥ˥奢��ʤɡˤˡ��嵭�����ɽ�����������Ѿ�浪��Ӳ���
 *      ��̵�ݾڵ����Ǻܤ��뤳�ȡ�
 *  (3) �ܥ��եȥ������򡤵�����Ȥ߹���ʤɡ�¾�Υ��եȥ�������ȯ�˻�
 *      �ѤǤ��ʤ����Ǻ����ۤ�����ˤϡ����Τ����줫�ξ�����������
 *      �ȡ�
 *    (a) �����ۤ�ȼ���ɥ�����ȡ����Ѽԥޥ˥奢��ʤɡˤˡ��嵭����
 *        �ɽ�����������Ѿ�浪��Ӳ�����̵�ݾڵ����Ǻܤ��뤳�ȡ�
 *    (b) �����ۤη��֤��̤�������ˡ�ˤ�äơ�TOPPERS�ץ������Ȥ�
 *        ��𤹤뤳�ȡ�
 *  (4) �ܥ��եȥ����������Ѥˤ��ľ��Ū�ޤ��ϴ���Ū�������뤤���ʤ�»
 *      ������⡤�嵭����Ԥ����TOPPERS�ץ������Ȥ����դ��뤳�ȡ�
 *      �ޤ����ܥ��եȥ������Υ桼���ޤ��ϥ���ɥ桼������Τ����ʤ���
 *      ͳ�˴�Ť����ᤫ��⡤�嵭����Ԥ����TOPPERS�ץ������Ȥ�
 *      ���դ��뤳�ȡ�
 * 
 *  �ܥ��եȥ������ϡ�̵�ݾڤ��󶡤���Ƥ����ΤǤ��롥�嵭����Ԥ�
 *  ���TOPPERS�ץ������Ȥϡ��ܥ��եȥ������˴ؤ��ơ�����λ�����Ū
 *  ���Ф���Ŭ������ޤ�ơ������ʤ��ݾڤ�Ԥ�ʤ����ޤ����ܥ��եȥ���
 *  �������Ѥˤ��ľ��Ū�ޤ��ϴ���Ū�������������ʤ�»���˴ؤ��Ƥ⡤��
 *  ����Ǥ�����ʤ���
 * 
 *  $Id: sipeed_st7789.c 2416 2020-01-08 21:55:40Z roi $
 */
/* 
 *  SIPEED ST7789 2.4"LCD����ץ���������
 */

#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>
#include <stdio.h>
#include <string.h>
#include <target_syssvc.h>
#include "syssvc/serial.h"
#include "syssvc/syslog.h"
#include "sipeed_st7789.h"

/*
 *  �����ӥ�������Υ��顼�Υ�����
 */
Inline void
svc_perror(const char *file, int_t line, const char *expr, ER ercd)
{
	if (ercd < 0) {
		t_perror(LOG_ERROR, file, line, expr, ercd);
	}
}

#define	SVC_PERROR(expr)	svc_perror(__FILE__, __LINE__, #expr, (expr))
#define ABS(X)  ((X) > 0 ? (X) : -(X))

#ifndef SPI_CORE_WAIT_TIME
#define SPI_CORE_WAIT_TIME  500
#endif

#define PORT_HIGH           1
#define PORT_LOW            0

#define MAX_BUFFER          8

static uint32_t aTxBuffer[MAX_BUFFER];


static void set_dcx_control(LCD_Handler_t *hlcd)
{
    gpio_set_pin(TADR_GPIO_BASE, hlcd->dcx_no, PORT_LOW);
}

static void set_dcx_data(LCD_Handler_t *hlcd)
{
    gpio_set_pin(TADR_GPIO_BASE, hlcd->dcx_no, PORT_HIGH);
}

/*
 *  LCD�ؤΥ��ޥ�������ؿ�
 */
ER
lcd_writecommand(LCD_Handler_t *hlcd, uint8_t c)
{
	SPI_Handle_t *hspi = hlcd->hspi;
	ER ercd = E_OK;

	aTxBuffer[0] = c;
    set_dcx_control(hlcd);
	hspi->Init.DataSize = 8;
	hspi->Init.InstLength = 8;
	hspi->Init.AddrLength = 0;
    ercd = spi_core_transmit(hspi, hlcd->cs_sel, (uint8_t *)(aTxBuffer), 1);
#if SPI_WAIT_TIME == 0
	if(ercd == E_OK)
		ercd = spi_wait(hspi, SPI_CORE_WAIT_TIME);
#endif
	return ercd;
}

/*
 *  LCD�ؤΥǡ��������ؿ�
 */
ER
lcd_writebyte(LCD_Handler_t *hlcd, uint8_t *buf, uint8_t len)
{
	SPI_Handle_t *hspi = hlcd->hspi;
	uint32_t i;
	ER ercd = E_OK;

	if(len >= MAX_BUFFER)
		return E_PAR;
	for(i = 0 ; i < len ; i++)
		aTxBuffer[i] = buf[i];

    set_dcx_data(hlcd);
	hspi->Init.DataSize = 8;
	hspi->Init.InstLength = 8;
	hspi->Init.AddrLength = 0;
	ercd = spi_core_transmit(hspi, hlcd->cs_sel, (uint8_t *)aTxBuffer, len);
#if SPI_WAIT_TIME == 0
	if(ercd == E_OK)
		ercd = spi_wait(hspi, SPI_CORE_WAIT_TIME);
#endif
	return ercd;
}

ER
lcd_writebyte2(LCD_Handler_t *hlcd, uint8_t *buf, uint8_t len)
{
	SPI_Handle_t *hspi = hlcd->hspi;
	ER ercd = E_OK;

    set_dcx_data(hlcd);
	hspi->Init.DataSize = 8;
	hspi->Init.InstLength = 8;
	hspi->Init.AddrLength = 0;
	ercd = spi_core_transmit(hspi, hlcd->cs_sel, buf, len);
#if SPI_WAIT_TIME == 0
	if(ercd == E_OK)
		ercd = spi_wait(hspi, SPI_CORE_WAIT_TIME);
#endif
	return ercd;
}

/*
 *  LCD�ؤ�16�ӥåȥǡ���ž��
 */
ER
lcd_writehalf(LCD_Handler_t *hlcd, uint32_t *buf, int cnt)
{
	SPI_Handle_t *hspi = hlcd->hspi;
	ER ercd = E_OK;

    set_dcx_data(hlcd);
	hspi->Init.DataSize = 16;
	hspi->Init.InstLength = 16;
	hspi->Init.AddrLength = 0;
	ercd = spi_core_transmit(hspi, hlcd->cs_sel, (uint8_t *)buf, cnt);
#if SPI_WAIT_TIME == 0
	if(ercd == E_OK)
		ercd = spi_wait(hspi, SPI_CORE_WAIT_TIME);
#endif
	return ercd;
}

/*
 *  LCD32�ӥåȥե���
 */
ER
lcd_filldata(LCD_Handler_t *hlcd, uint32_t *data_buf, uint32_t len)
{
	SPI_Handle_t *hspi = hlcd->hspi;
	ER ercd = E_OK;

    set_dcx_data(hlcd);
	hspi->Init.DataSize = 32;
	hspi->Init.InstLength = 0;
	hspi->Init.AddrLength = 32;
	ercd = spi_core_transmit_fill(hspi, hlcd->cs_sel, data_buf, len);
#if SPI_WAIT_TIME == 0
	if(ercd == E_OK)
		spi_wait(hspi, SPI_CORE_WAIT_TIME);
#endif
	return ercd;
}

/*
 *   SIPEED LCD�����
 */
void lcd_init(LCD_Handler_t *hlcd)
{
	GPIO_Init_t init = {0};
    uint8_t data = 0;

	/*
	 *  DCX�ԥ�����
	 */
	fpioa_set_function(hlcd->dcx_pin , FUNC_GPIO0 + hlcd->dcx_no);
	init.mode = GPIO_MODE_OUTPUT;
	init.pull = GPIO_PULLDOWN;
    gpio_setup(TADR_GPIO_BASE, &init, hlcd->dcx_no);
    gpio_set_pin(TADR_GPIO_BASE, hlcd->dcx_no, PORT_HIGH);

	/*
	 *  LCD�ꥻ�å�
	 */
	if(hlcd->rst_pin >= 0){
		fpioa_set_function(hlcd->rst_pin , FUNC_GPIO0 + hlcd->rst_no);
		init.mode = GPIO_MODE_OUTPUT;
		init.pull = GPIO_PULLDOWN;
		gpio_setup(TADR_GPIO_BASE, &init, hlcd->rst_no);
		gpio_set_pin(TADR_GPIO_BASE, hlcd->rst_no, PORT_HIGH);

	    gpio_set_pin(TADR_GPIO_BASE, hlcd->rst_no, 0);
	    gpio_set_pin(TADR_GPIO_BASE, hlcd->rst_no, 1);
	}

	/*
	 *  soft reset
	 */
	lcd_writecommand(hlcd, SOFTWARE_RESET);
	dly_tsk(100);
	/*
	 *  exit sleep
	 */
	lcd_writecommand(hlcd, SLEEP_OFF);
	dly_tsk(100);
	/*
	 *  pixel format
	 */
	lcd_writecommand(hlcd, PIXEL_FORMAT_SET);
	data = 0x55;
    lcd_writebyte(hlcd, &data, 1);
	if(hlcd->dir & DIR_XY_MASK){
		hlcd->_width = ST7789_TFTHEIGHT;
		hlcd->_height = ST7789_TFTWIDTH;
	}
	else{
		hlcd->_width = ST7789_TFTWIDTH;
		hlcd->_height = ST7789_TFTHEIGHT;
	}
	hlcd->colstart = 0;
	hlcd->rowstart = 0;

	lcd_writecommand(hlcd, MEMORY_ACCESS_CTL);
    lcd_writebyte(hlcd, (uint8_t *)&hlcd->dir, 1);

	/*
	 *  inversion on
	 */
	lcd_writecommand(hlcd, INVERSION_DISPALY_ON);

	/*
	 *  display on
	 */
	lcd_writecommand(hlcd, DISPALY_ON);
	hlcd->mode = 1;
}

/*
 *  ɽ��������ɥ�����
 */
void
lcd_setAddrWindow(LCD_Handler_t *hlcd,  uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t data[4] = {0};

    data[0] = (uint8_t)(x0 >> 8);
    data[1] = (uint8_t)(x0);
    data[2] = (uint8_t)(x1 >> 8);
    data[3] = (uint8_t)(x1);
	lcd_writecommand(hlcd, HORIZONTAL_ADDRESS_SET);
    lcd_writebyte(hlcd, data, 4);

    data[0] = (uint8_t)(y0 >> 8);
    data[1] = (uint8_t)(y0);
    data[2] = (uint8_t)(y1 >> 8);
    data[3] = (uint8_t)(y1);
	lcd_writecommand(hlcd, VERTICAL_ADDRESS_SET);
    lcd_writebyte(hlcd, data, 4);

	SVC_PERROR(lcd_writecommand(hlcd, MEMORY_WRITE));
}

/*
 *  RECTANGLE�Υե�������
 *  param1  hlcd: Pointer to LCD Handler
 *  param2  x: start X position
 *  param3  y: start Y position
 *  param4  w: width
 *  param5  h: height
 *  param6  color: color value
 */
void
lcd_fillRect(LCD_Handler_t *hlcd, int16_t x, int16_t y, int16_t w, int16_t h, color_t color)
{
    uint32_t data = ((uint32_t)color << 16) | (uint32_t)color;
	// rudimentary clipping (drawChar w/big text requires this)
	if((x >= hlcd->_width) || (y >= hlcd->_height)) return;
	if((x + w - 1) >= hlcd->_width)  w = hlcd->_width  - x;
	if((y + h - 1) >= hlcd->_height) h = hlcd->_height - y;

	lcd_setAddrWindow(hlcd, x, y, x+w-1, y+h-1);
    SVC_PERROR(lcd_filldata(hlcd, &data, (h*w+1)/2));
}

/*
 *  PIXEL����
 *  param1  hlcd: Pointer to LCD Handler
 *  param2  x: X position
 *  param3  y: Y position
 *  param4  color: color value
 */
void
lcd_drawPixel(LCD_Handler_t *hlcd, int16_t x, int16_t y, color_t color)
{
	uint32_t data = color;
    lcd_setAddrWindow(hlcd, x, y, x, y);
	SVC_PERROR(lcd_writehalf(hlcd, &data, 1));
}

/*
 *  ��ľ����LINE�ι�®����
 *  param1  hlcd: Pointer to LCD Handler
 *  param2  x: start X position
 *  param3  y: start Y position
 *  param4  h: height
 *  param5  color: color value
 */
void
lcd_drawFastVLine(LCD_Handler_t *hlcd, int16_t x, int16_t y, int16_t h, color_t color)
{
    uint32_t data = ((uint32_t)color << 16) | (uint32_t)color;
	// Rudimentary clipping
	if(h == 0) return;
	if((x >= hlcd->_width) || (y >= hlcd->_height)) return;
	if((y+h-1) >= hlcd->_height) h = hlcd->_height-y;
	lcd_setAddrWindow(hlcd, x, y, x, y+h-1);
    SVC_PERROR(lcd_filldata(hlcd, &data, (h+1)/2));
}

/*
 *  ��ʿ����LINE�ι�®����
 *  param1  hlcd: Pointer to LCD Handler
 *  param2  x: start X position
 *  param3  y: start Y position
 *  param4  w: width
 *  param5  color: color value
 */
void lcd_drawFastHLine(LCD_Handler_t *hlcd, int16_t x, int16_t y, int16_t w, color_t color)
{
    uint32_t data = ((uint32_t)color << 16) | (uint32_t)color;
	// Rudimentary clipping
	if(w == 0) return;
	if((x >= hlcd->_width) || (y >= hlcd->_height)) return;
	if((x+w-1) >= hlcd->_width)  w = hlcd->_width-x;
	lcd_setAddrWindow(hlcd, x, y, x+w-1, y);
    SVC_PERROR(lcd_filldata(hlcd, &data, (w+1)/2));
}

/*
 *  DRAW IMAGE LINE����
 *  param1  hlcd: Pointer to LCD Handler
 *  param2  x: X position
 *  param3  y: Y position
 *  param4  w: width
 *  param5  pcolor: color value
 */
void
lcd_drawImageHLine(LCD_Handler_t *hlcd, int16_t x, int16_t y, uint16_t w, uint32_t *pcolor)
{
	if((x >= hlcd->_width) || (y >= hlcd->_height)) return;
	if((x+w-1) >= hlcd->_width)  w = hlcd->_width-x;
	lcd_setAddrWindow(hlcd, x, y, x+w-1, y);
	SVC_PERROR(lcd_writehalf(hlcd, pcolor, w));
}

/*
 *  BITMAP����
 *  param1  hlcd: Pointer to LCD Handler
 *  param2  x0: Bmp X position in the LCD
 *  param3  y0: Bmp Y position in the LCD
 *  param4  pbmp: Pointer to Bmp picture address in the internal Flash
 */
void
lcd_drawBitmap(LCD_Handler_t *hlcd, uint16_t x0, uint16_t y0, uint8_t *pbmp)
{
	uint32_t index = 0, width = 0, height = 0, bit_pixel = 0;
	uint32_t input_color_mode = 0;
	uint32_t i;

	/* Get bitmap data address offset */
	index = *(uint16_t *) (pbmp + 10);
	index |= (*(uint16_t *) (pbmp + 12)) << 16;

	/* Read bitmap width */
	width = *(uint16_t *) (pbmp + 18);
	width |= (*(uint16_t *) (pbmp + 20)) << 16;

	/* Read bitmap height */
	height = *(uint16_t *) (pbmp + 22);
	height |= (*(uint16_t *) (pbmp + 24)) << 16;

	/* Read bit/pixel */
	bit_pixel = *(uint16_t *) (pbmp + 28);

	/* Get the layer pixel format */
	if ((bit_pixel/8) == 4){
		input_color_mode = CM_ARGB8888;
	}
	else if ((bit_pixel/8) == 2){
		input_color_mode = CM_RGB565;
	}
	else{
		input_color_mode = CM_RGB888;
	}

	/* Bypass the bitmap header */
	pbmp += (index + (width * (height - 1) * (bit_pixel/8)));  
	syslog_4(LOG_NOTICE, "## input_color_mode(%d) width(%d) height(%d) bit_pixel(%d) ##", input_color_mode, width, height, bit_pixel);
	for(index=0; index < height; index++){
		uint8_t *p = pbmp;
		color_t color;
		for (i = 0; i < width; i++){
			color  = ((p[0] & 0xf8) | (p[1] >> 5)) << 8;
			color  |= ((p[1] << 3) & 0xE0) | ((p[2] >> 3) & 0x1F);
			lcd_drawPixel(hlcd, x0+i, y0+index, color);
			p += 3;
		}
		pbmp -= width*3;
	}
}

/*
 *  PICTURE����
 *  param1  hlcd: Pointer to LCD Handler
 *  param2  x: Picture X position in the LCD
 *  param3  y: Picture Y position in the LCD
 *  param4  width:  Picture width
 *  param5  height: Picture heigth
 *  param6  pbmp: Pointer to picture address in the internal Flash
 */
void
lcd_drawPicture(LCD_Handler_t *hlcd, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t *pbmp)
{
	SPI_Handle_t *hspi = hlcd->hspi;
	ER ercd = E_OK;

	if((x >= hlcd->_width) || (y >= hlcd->_height)) return;
	if((x+width-1) > hlcd->_width)  return;
	if((y+height-1) > hlcd->_height)  return;

	lcd_setAddrWindow(hlcd, x, y, x+width-1, y+height-1);
    set_dcx_data(hlcd);
	hspi->Init.DataSize = 32;
	hspi->Init.InstLength = 0;
	hspi->Init.AddrLength = 32;
	ercd = spi_core_transmit(hspi, hlcd->cs_sel, (uint8_t *)pbmp, width * height / 2);
#if SPI_WAIT_TIME == 0
	if(ercd == E_OK)
		spi_wait(hspi, SPI_CORE_WAIT_TIME);
#endif
	(void)(ercd);
}

/*
 *  INVERT DISPLAY
 *  param1  hlcd: Pointer to LCD Handler
 *  param2  i: invert value
 */
void
lcd_invertDisplay(LCD_Handler_t *hlcd, bool_t i)
{
	lcd_writecommand(hlcd, (i ? INVERSION_DISPALY_ON : INVERSION_DISPALY_OFF));
}


/*
 *  �����꡼��ե���
 *  param1  pDrawProp: Pointer to Draw Prop
 */
void
lcd_fillScreen(LCD_DrawProp_t *pDrawProp)
{
	lcd_fillRect(pDrawProp->hlcd, 0, 0,  pDrawProp->hlcd->_width, pDrawProp->hlcd->_height, pDrawProp->BackColor);
}

/*
 *  RECTANGLE����
 *  param1  pDrawProp: Pointer to Draw Prop
 *  param2  x: left X position
 *  param3  y: top Y position
 *  param4  w: width
 *  param5  h: height
 */
void
lcd_drawRect(LCD_DrawProp_t *pDrawProp, int16_t x, int16_t y, int16_t w, int16_t h)
{
	LCD_Handler_t *hlcd = pDrawProp->hlcd;
	color_t color;

	// rudimentary clipping (drawChar w/big text requires this)
	if((x >= hlcd->_width) || (y >= hlcd->_height)) return;
	if((x + w - 1) >= hlcd->_width)  w = hlcd->_width  - x;
	if((y + h - 1) >= hlcd->_height) h = hlcd->_height - y;

	color = pDrawProp->TextColor;
	lcd_drawFastVLine(hlcd, x, y, h, color);
	lcd_drawFastHLine(hlcd, x, y+h-1, w, color);
	lcd_drawFastVLine(hlcd, x+w-1, y, h, color);
	lcd_drawFastHLine(hlcd, x, y, w, color);
}

/*
 *  ������
 *  param1  pDrawProp: Pointer to Draw Prop
 *  param2  x0: X position
 *  param3  y0: Y position
 *  param4  Radius: Circle radius
 */
void
lcd_DrawCircle(LCD_DrawProp_t *pDrawProp, uint16_t x0, uint16_t y0, uint16_t Radius)
{
	LCD_Handler_t *hlcd = pDrawProp->hlcd;
 	int32_t   decision;    /* Decision Variable */ 
	uint32_t  current_x;   /* Current X Value */
	uint32_t  current_y;   /* Current Y Value */

	decision = 3 - (Radius << 1);
	current_x = 0;
	current_y = Radius;

	while(current_x <= current_y){
		lcd_drawPixel(hlcd, (x0 + current_x), (y0 - current_y), pDrawProp->TextColor);
		lcd_drawPixel(hlcd, (x0 - current_x), (y0 - current_y), pDrawProp->TextColor);
		lcd_drawPixel(hlcd, (x0 + current_y), (y0 - current_x), pDrawProp->TextColor);
		lcd_drawPixel(hlcd, (x0 - current_y), (y0 - current_x), pDrawProp->TextColor);
		lcd_drawPixel(hlcd, (x0 + current_x), (y0 + current_y), pDrawProp->TextColor);
		lcd_drawPixel(hlcd, (x0 - current_x), (y0 + current_y), pDrawProp->TextColor);
		lcd_drawPixel(hlcd, (x0 + current_y), (y0 + current_x), pDrawProp->TextColor);
		lcd_drawPixel(hlcd, (x0 - current_y), (y0 + current_x), pDrawProp->TextColor);

		if (decision < 0){
			decision += (current_x << 2) + 6;
		}
		else{
			decision += ((current_x - current_y) << 2) + 10;
			current_y--;
		}
		current_x++;
	}
}

/*
 *  ������
 *  param1  pDrawProp: Pointer to Draw Prop
 *  param2  x1: Point 1 X position
 *  param3  y1: Point 1 Y position
 *  param4  x2: Point 2 X position
 *  param5  y2: Point 2 Y position
 */
void
lcd_drawLine(LCD_DrawProp_t *pDrawProp, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	LCD_Handler_t *hlcd = pDrawProp->hlcd;
	int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0, 
	yinc1 = 0, yinc2 = 0, den = 0, num = 0, num_add = 0, num_pixels = 0, 
	curpixel = 0;

	deltax = ABS(x2 - x1);		/* The difference between the x's */
	deltay = ABS(y2 - y1);		/* The difference between the y's */
	x = x1;						/* Start x off at the first pixel */
	y = y1;						/* Start y off at the first pixel */

	if(x2 >= x1){				/* The x-values are increasing */
		xinc1 = 1;
		xinc2 = 1;
	}
	else{						/* The x-values are decreasing */
		xinc1 = -1;
		xinc2 = -1;
	}

	if(y2 >= y1){				/* The y-values are increasing */
		yinc1 = 1;
		yinc2 = 1;
	}
	else{						/* The y-values are decreasing */
		yinc1 = -1;
		yinc2 = -1;
	}

	if(deltax >= deltay){		/* There is at least one x-value for every y-value */
		xinc1 = 0;				/* Don't change the x when numerator >= denominator */
		yinc2 = 0;				/* Don't change the y for every iteration */
		den = deltax;
		num = deltax / 2;
		num_add = deltay;
		num_pixels = deltax;	/* There are more x-values than y-values */
	}
	else{						/* There is at least one y-value for every x-value */
		xinc2 = 0;				/* Don't change the x for every iteration */
		yinc1 = 0;				/* Don't change the y when numerator >= denominator */
		den = deltay;
		num = deltay / 2;
		num_add = deltax;
		num_pixels = deltay;	/* There are more y-values than x-values */
	}

	for (curpixel = 0; curpixel <= num_pixels; curpixel++){
		lcd_drawPixel(hlcd, x, y, pDrawProp->TextColor);	/* Draw the current pixel */
		num += num_add;			/* Increase the numerator by the top of the fraction */
		if(num >= den){			/* Check if numerator >= denominator */
			num -= den;			/* Calculate the new numerator value */
			x += xinc1;			/* Change the x as appropriate */
			y += yinc1;			/* Change the y as appropriate */
		}
		x += xinc2;				/* Change the x as appropriate */
		y += yinc2;				/* Change the y as appropriate */
	}
}

/*
 *  PLOY-LINE����
 *  param1  pDrawProp: Pointer to Draw Prop
 *  param2  Points: Pointer to the points array
 *  param3  PointCount: Number of points
 */
void
lcd_drawPolygon(LCD_DrawProp_t *pDrawProp, pPoint Points, uint16_t PointCount)
{
	int16_t x = 0, y = 0;

	if(PointCount < 2){
		return;
	}

	lcd_drawLine(pDrawProp, Points->X, Points->Y, (Points+PointCount-1)->X, (Points+PointCount-1)->Y);
	while(--PointCount){
		x = Points->X;
		y = Points->Y;
		Points++;
		lcd_drawLine(pDrawProp, x, y, Points->X, Points->Y);
	}
}

