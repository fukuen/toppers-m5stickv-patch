/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 *
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2004-2010 by Embedded and Real-Time Systems Laboratory
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2015-2019 by TOPPERS PROJECT Educational Working Group.
 *
 *  上記著作権者は，以下の(1)~(4)の条件を満たす場合に限り，本ソフトウェ
 *  ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
 *  変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
 *  (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
 *      権表示，この利用条件および下記の無保証規定が，そのままの形でソー
 *      スコード中に含まれていること．
 *  (2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
 *      用できる形で再配布する場合には，再配布に伴うドキュメント（利用
 *      者マニュアルなど）に，上記の著作権表示，この利用条件および下記
 *      の無保証規定を掲載すること．
 *  (3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
 *      用できない形で再配布する場合には，次のいずれかの条件を満たすこ
 *      と．
 *    (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
 *        作権表示，この利用条件および下記の無保証規定を掲載すること．
 *    (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
 *        報告すること．
 *  (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
 *      害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
 *      また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
 *      由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
 *      免責すること．
 * 
 *  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *  よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
 *  に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
 *  アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
 *  の責任を負わない．
 *
 *  $Id: sipeed_st7789.h 2416 2019-11-27 22:07:42Z roi $
 */
/*
 *  SIPEED ST7789 2.4"LCD制御プログラムのヘッダファイル
 */

#ifndef _SIPEED_ST7789_H_
#define _SIPEED_ST7789_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "device.h"
#include "spi.h"

/*
 *  LCDカラーモード定義
 */
#define CM_ARGB8888         0x00000000			/* ARGB8888 color mode */
#define CM_RGB888           0x00000001			/* RGB888 color mode */
#define CM_RGB565           0x00000002			/* RGB565 color mode */

#if defined(MAIXAMIGO)
#define ST7789_TFTWIDTH  320
#define ST7789_TFTHEIGHT 480
#elif defined(MAIXCUBE)
#define ST7789_TFTWIDTH  240
#define ST7789_TFTHEIGHT 240
#else
#define ST7789_TFTWIDTH  240
#define ST7789_TFTHEIGHT 320
#endif

#define NO_OPERATION            0x00
#define SOFTWARE_RESET          0x01
#define READ_ID                 0x04
#define READ_STATUS             0x09
#define READ_POWER_MODE         0x0A
#define READ_MADCTL             0x0B
#define READ_PIXEL_FORMAT       0x0C
#define READ_IMAGE_FORMAT       0x0D
#define READ_SIGNAL_MODE        0x0E
#define READ_SELT_DIAG_RESULT   0x0F
#define SLEEP_ON                0x10
#define SLEEP_OFF               0x11
#define PARTIAL_DISPALY_ON      0x12
#define NORMAL_DISPALY_ON       0x13
#define INVERSION_DISPALY_OFF   0x20
#define INVERSION_DISPALY_ON    0x21
#define GAMMA_SET               0x26
#define DISPALY_OFF             0x28
#define DISPALY_ON              0x29
#define HORIZONTAL_ADDRESS_SET  0x2A
#define VERTICAL_ADDRESS_SET    0x2B
#define MEMORY_WRITE            0x2C
#define COLOR_SET               0x2D
#define MEMORY_READ             0x2E
#define PARTIAL_AREA            0x30
#define VERTICAL_SCROL_DEFINE   0x33
#define TEAR_EFFECT_LINE_OFF    0x34
#define TEAR_EFFECT_LINE_ON     0x35
#define MEMORY_ACCESS_CTL       0x36
#define VERTICAL_SCROL_S_ADD    0x37
#define IDLE_MODE_OFF           0x38
#define IDLE_MODE_ON            0x39
#define PIXEL_FORMAT_SET        0x3A
#define WRITE_MEMORY_CONTINUE   0x3C
#define READ_MEMORY_CONTINUE    0x3E
#define SET_TEAR_SCANLINE       0x44
#define GET_SCANLINE            0x45
#define WRITE_BRIGHTNESS        0x51
#define READ_BRIGHTNESS         0x52
#define WRITE_CTRL_DISPALY      0x53
#define READ_CTRL_DISPALY       0x54
#define WRITE_BRIGHTNESS_CTL    0x55
#define READ_BRIGHTNESS_CTL     0x56
#define WRITE_MIN_BRIGHTNESS    0x5E
#define READ_MIN_BRIGHTNESS     0x5F
#define READ_ID1                0xDA
#define READ_ID2                0xDB
#define READ_ID3                0xDC
#define RGB_IF_SIGNAL_CTL       0xB0
#define NORMAL_FRAME_CTL        0xB1
#define IDLE_FRAME_CTL          0xB2
#define PARTIAL_FRAME_CTL       0xB3
#define INVERSION_CTL           0xB4
#define BLANK_PORCH_CTL         0xB5
#define DISPALY_FUNCTION_CTL    0xB6
#define ENTRY_MODE_SET          0xB7
#define BACKLIGHT_CTL1          0xB8
#define BACKLIGHT_CTL2          0xB9
#define BACKLIGHT_CTL3          0xBA
#define BACKLIGHT_CTL4          0xBB
#define BACKLIGHT_CTL5          0xBC
#define BACKLIGHT_CTL7          0xBE
#define BACKLIGHT_CTL8          0xBF
#define POWER_CTL1              0xC0
#define POWER_CTL2              0xC1
#define VCOM_CTL1               0xC5
#define VCOM_CTL2               0xC7
#define NV_MEMORY_WRITE         0xD0
#define NV_MEMORY_PROTECT_KEY   0xD1
#define NV_MEMORY_STATUS_READ   0xD2
#define READ_ID4                0xD3
#define POSITIVE_GAMMA_CORRECT  0xE0
#define NEGATIVE_GAMMA_CORRECT  0xE1
#define DIGITAL_GAMMA_CTL1      0xE2
#define DIGITAL_GAMMA_CTL2      0xE3
#define INTERFACE_CTL           0xF6

/*
 *  カラー定義
 */
#define ST7789_BLACK            0x0000
#define ST7789_NAVY             0x000F
#define ST7789_DARKGREEN        0x03E0
#define ST7789_DARKCYAN         0x03EF
#define ST7789_MAROON           0x7800
#define ST7789_PURPLE           0x780F
#define ST7789_OLIVE            0x7BE0
#define ST7789_LIGHTGREY        0xC618
#define ST7789_DARKGREY         0x7BEF
#define ST7789_BLUE             0x001F
#define ST7789_GREEN            0x07E0
#define ST7789_CYAN             0x07FF
#define ST7789_RED              0xF800
#define ST7789_MAGENTA          0xF81F
#define ST7789_YELLOW           0xFFE0
#define ST7789_WHITE            0xFFFF
#define ST7789_ORANGE           0xFD20
#define ST7789_GREENYELLOW      0xAFE5
#define ST7789_PINK             0xF81F

/*
 *  LCD DIRECTION
 */
#if defined(MAIXAMIGO)
#define DIR_XY_RLUD             0x88
#define DIR_YX_RLUD             0xA8
#define DIR_XY_LRUD             0xC8
#define DIR_YX_LRUD             0xE8
#define DIR_XY_RLDU             0x08
#define DIR_YX_RLDU             0x28 //default
#define DIR_XY_LRDU             0x48
#define DIR_YX_LRDU             0x68
#define DIR_XY_MASK             0x20
#define DIR_RL_MASK             0x40
#define DIR_UD_MASK             0x80
#elif defined(MAIXCUBE)
#define DIR_XY_RLUD             0xC0 //00
#define DIR_YX_RLUD             0xE0 //20
#define DIR_XY_LRUD             0x80 //40
#define DIR_YX_LRUD             0xA0 //60
#define DIR_XY_RLDU             0x40 //80
#define DIR_YX_RLDU             0x60 //default A0 -> 60
#define DIR_XY_LRDU             0x00 //C0
#define DIR_YX_LRDU             0x20 //E0
#define DIR_XY_MASK             0x20
#define DIR_RL_MASK             0x40
#define DIR_UD_MASK             0x80
#else
#define DIR_XY_RLUD             0x00
#define DIR_YX_RLUD             0x20
#define DIR_XY_LRUD             0x40
#define DIR_YX_LRUD             0x60
#define DIR_XY_RLDU             0x80
#define DIR_YX_RLDU             0xA0
#define DIR_XY_LRDU             0xC0
#define DIR_YX_LRDU             0xE0
#define DIR_XY_MASK             0x20
#define DIR_RL_MASK             0x40
#define DIR_UD_MASK             0x80
#endif

#define TRS_COLOR_T             uint32_t

#ifndef TOPPERS_MACRO_ONLY


/*
 *  カラーの属性を定義
 */
typedef uint16_t   color_t;

typedef struct
{
	SPI_Handle_t            *hspi;
    uint16_t                _width;
    uint16_t                _height;
	uint16_t                colstart;
	uint16_t                rowstart;
	ID                      spi_lock;
    uint8_t                 mode;
    uint8_t                 dir;

	int8_t                  dcx_pin;
	int8_t                  rst_pin;
	int8_t                  cs_sel;
	uint8_t                 dcx_no;
	uint8_t                 rst_no;
} LCD_Handler_t;


/*
 *  描画構造体
 */
typedef struct
{
	LCD_Handler_t           *hlcd;
	color_t                 TextColor;
	color_t                 BackColor;
	void                    *pFont;
}LCD_DrawProp_t;

typedef struct
{
	int16_t X;
	int16_t Y;
}Point, * pPoint;

/*
 *  関数のプロトタイプ宣言
 */
extern ER lcd_writecommand(LCD_Handler_t *hlcd, uint8_t c);
extern ER lcd_writebyte(LCD_Handler_t *hlcd, uint8_t *buf, uint8_t len);
extern ER lcd_writehalf(LCD_Handler_t *hlcd, uint32_t *buf, int cnt);
extern ER lcd_filldata(LCD_Handler_t *hlcd, uint32_t *data_buf, uint32_t len);

extern void lcd_init(LCD_Handler_t *hlcd);
extern void lcd_setAddrWindow(LCD_Handler_t *hlcd, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
extern void lcd_fillRect(LCD_Handler_t *hlcd, int16_t x, int16_t y, int16_t w, int16_t h, color_t color);
extern void lcd_pushColor(LCD_Handler_t *hlcd, color_t color);
extern void lcd_drawPixel(LCD_Handler_t *hlcd, int16_t x, int16_t y, color_t color);
extern void lcd_drawFastVLine(LCD_Handler_t *hlcd, int16_t x, int16_t y, int16_t h, color_t color);
extern void lcd_drawFastHLine(LCD_Handler_t *hlcd, int16_t x, int16_t y, int16_t w, color_t color);
extern void lcd_drawImageHLine(LCD_Handler_t *hlcd, int16_t x, int16_t y, uint16_t w, TRS_COLOR_T *pcolor);
extern void lcd_drawBitmap(LCD_Handler_t *hlcd, uint16_t x0, uint16_t y0, uint8_t *pbmp);
extern void lcd_drawPicture(LCD_Handler_t *hlcd, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t *pbmp);
extern void lcd_invertDisplay(LCD_Handler_t *hlcd, bool_t i);

extern void lcd_fillScreen(LCD_DrawProp_t *pDrawProp);
extern void lcd_drawRect(LCD_DrawProp_t *pDrawProp, int16_t x, int16_t y, int16_t w, int16_t h);
extern void lcd_DrawCircle(LCD_DrawProp_t *pDrawProp, uint16_t x0, uint16_t y0, uint16_t Radius);
extern void lcd_drawLine(LCD_DrawProp_t *pDrawProp, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
extern void lcd_drawPolygon(LCD_DrawProp_t *pDrawProp, pPoint Points, uint16_t PointCount);

#endif /* TOPPERS_MACRO_ONLY */

#ifdef __cplusplus
}
#endif

#endif	/* _SIPEED_ST7789_H_ */

