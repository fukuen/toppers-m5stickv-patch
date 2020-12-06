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
 *  $Id: maixamigo_axp173.c 2416 2020-12-03 18:45:11Z fukuen $
 */

/* 
 *  MAIXAMIGO AXP173 制御プログラムの本体
 */

#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>
#include <stdio.h>
#include <string.h>
#include <target_syssvc.h>
#include "syssvc/serial.h"
#include "syssvc/syslog.h"
#include "kernel_cfg.h"
#include "device.h"
#include "maixamigo_axp173.h"
#include "sysctl.h"

#define TXBUFFERSIZE  2

#if defined(TOPPERS_BASE_PLATFORM_STF7)
#define i2c_send(h, a, b, l)    i2c_memwrite((h), (a), 0, 0, (b), (l), 500)
#else
#define i2c_send(h, a, b, l)    i2c_memwrite((h), (a), 0, 0, (b), (l))
#endif

static uint8_t aTxBuffer[TXBUFFERSIZE];

/*
 *  AXP173の初期化
 */
ER
axp173_init(AXP173_Handler_t *haxp)
{
	I2C_Handle_t *hi2c = haxp->hi2c;
	ER ercd;

#ifdef MAIXAMIGO
	/*
	 *  LDO4 - 0.8V (default 0x48 1.8V)
	 */
    aTxBuffer[0] = 0x27;
    aTxBuffer[1] = 0x20;
	if((ercd = i2c_send(hi2c, haxp->saddr, aTxBuffer, TXBUFFERSIZE)) != E_OK)
		return ercd;

    // Wait 1.08ms
	dly_tsk(2);
    /*
	 *  LDO2/3 - LDO2 1.8V / LDO3 3.0V
	 */
    aTxBuffer[0] = 0x28;
    aTxBuffer[1] = 0x0C;
	if((ercd = i2c_send(hi2c, haxp->saddr, aTxBuffer, TXBUFFERSIZE)) != E_OK)
		return ercd;
#else
	/*
	 *  Clear the interrupts
	 */
    aTxBuffer[0] = 0x46;
    aTxBuffer[1] = 0xFF;
	if((ercd = i2c_send(hi2c, haxp->saddr, aTxBuffer, TXBUFFERSIZE)) != E_OK)
		return ercd;

    // Wait 1.08ms
	dly_tsk(2);

	/*
	 *  set target voltage and current of battery(axp173 datasheet PG.)
	 *  charge current (default)780mA -> 190mA
	 */
    aTxBuffer[0] = 0x33;
    aTxBuffer[1] = 0xC1;
	if((ercd = i2c_send(hi2c, haxp->saddr, aTxBuffer, TXBUFFERSIZE)) != E_OK)
		return ercd;

    // Wait 1.08ms
	dly_tsk(2);

    /*
	 *  REG 10H: EXTEN & DC-DC2 control
	 */
	if((ercd = i2c_memread(hi2c, haxp->saddr, 0x10, 1, aTxBuffer, 1)) != E_OK)
		return ercd;
    aTxBuffer[1] = aTxBuffer[0] & 0xFC;
    aTxBuffer[0] = 0x10;
	if((ercd = i2c_send(hi2c, haxp->saddr, aTxBuffer, TXBUFFERSIZE)) != E_OK)
		return ercd;
#endif

	return ercd;
}

