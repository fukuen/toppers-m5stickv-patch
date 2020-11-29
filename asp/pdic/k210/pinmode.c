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
 *  上記著作権者は，以下の(1)〜(4)の条件を満たす場合に限り，本ソフトウェ
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
 *  $Id: pinmode.c 2416 2019-10-12 12:21:01Z roi $
 */

/*
 *  ARDUNO-PIN(K410)ドライバ
 */

#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>
#include <string.h>
#include <target_syssvc.h>
#include "syssvc/serial.h"
#include "syssvc/syslog.h"
#include "kernel_cfg.h"
#include "device.h"
#include "pinmode.h"

#define MD_PIN_MAP(fpio) _maixduino_pin_map[(fpio)]

static const uint8_t _maixduino_pin_map[17] = {4, 5, 21, 22, 23, 24, 32, 15, 14, 13, 12, 11, 10, 3, 31, 30, 16};


/*
 *  Arduinoピンモード設定
 */
void
pinMode(uint8_t dwPin, uint8_t dwMode){ 
    int gpionumx = gpio_get_gpiohno(MD_PIN_MAP(dwPin), false);
	GPIO_Init_t init = {0};
	int gpionum;

	gpionum = gpionumx;
    if(gpionum >= 0){
        uint8_t function = FUNC_GPIOHS0 + gpionum;
        fpioa_set_function(MD_PIN_MAP(dwPin), function);
		switch(dwMode){
		case INPUT:
			init.mode = GPIO_MODE_INPUT;
			init.pull = GPIO_NOPULL;
			break;
		case INPUT_PULLDOWN:
			init.mode = GPIO_MODE_INPUT;
			init.pull = GPIO_PULLDOWN;
			break;
		case INPUT_PULLUP:
			init.mode = GPIO_MODE_INPUT;
			init.pull = GPIO_PULLUP;
			break;
		case OUTPUT:
		default:
			init.mode = GPIO_MODE_OUTPUT;
			init.pull = GPIO_PULLDOWN;
			break;
		}
        gpio_setup(TADR_GPIOHS_BASE, &init, (uint8_t)gpionum);
    }
    return ;
}

/*
 *  Arduinoデジタルピン出力
 */
void
digitalWrite(uint8_t dwPin, int dwVal){
    int8_t gpio_pin = gpio_get_gpiohno(MD_PIN_MAP(dwPin), false);
    if( gpio_pin >= 0){
        gpio_set_pin(TADR_GPIOHS_BASE, (uint8_t)gpio_pin, dwVal);
    }
    return ;
}

/*
 *  Arduinoデジタルピン入力
 */
int
digitalRead(uint8_t dwPin){
    int8_t gpio_pin = gpio_get_gpiohno(MD_PIN_MAP(dwPin), false);
    if(gpio_pin >= 0){
        return (int)gpio_get_pin(TADR_GPIOHS_BASE, (uint8_t)gpio_pin);
    }
    return -1;
}

/*
 *  Arduionピン、IOピン変換
 */
uint8_t
getGpioPin(uint8_t no)
{
	return _maixduino_pin_map[no];
}


