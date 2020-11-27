/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 *
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2004-2010 by Embedded and Real-Time Systems Laboratory
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2015-2017 by TOPPERS PROJECT Educational Working Group.
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
 *  $Id: m5stickv_axp192.h 2416 2017-11-23 11:03:25Z roi $
 */

/*
 *  M5STICKV AXP192 制御プログラムのヘッダファイル
 */

#ifndef _M5STICKV_AXP192_H_
#define _M5STICKV_AXP192_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "device.h"
#include "i2c.h"

#ifndef TOPPERS_MACRO_ONLY

/*
 *  ハンドラ構造体定義
 */
typedef struct
{
	I2C_Handle_t            *hi2c;
	uint16_t                saddr;
}AXP192_Handler_t;

/*
 *  関数のプロトタイプ宣言
 */
extern ER axp192_init(AXP192_Handler_t *hlcd);
extern ER axp192_set_command(AXP192_Handler_t *hlcd, uint8_t c);
extern ER axp192_set_data(AXP192_Handler_t *hlcd, uint8_t *p, uint8_t len);

#endif /* TOPPERS_MACRO_ONLY */

#ifdef __cplusplus
}
#endif

#endif	/* _M%STICKV_AXP192_H_ */
