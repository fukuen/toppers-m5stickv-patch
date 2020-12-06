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
 *  $Id: maixamigo_axp173.c 2416 2020-12-03 18:45:11Z fukuen $
 */

/* 
 *  MAIXAMIGO AXP173 ����ץ���������
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
 *  AXP173�ν����
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

