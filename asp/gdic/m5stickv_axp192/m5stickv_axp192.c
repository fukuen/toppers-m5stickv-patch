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
 *  $Id: m5stickv_axp192.c 2416 2019-08-01 18:45:11Z roi $
 */

/* 
 *  M5STICKV AXP192 ����ץ���������
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
#include "m5stickv_axp192.h"
#include "sysctl.h"

#define TXBUFFERSIZE  2

#if defined(TOPPERS_BASE_PLATFORM_STF7)
#define i2c_send(h, a, b, l)    i2c_memwrite((h), (a), 0, 0, (b), (l), 500)
#else
#define i2c_send(h, a, b, l)    i2c_memwrite((h), (a), 0, 0, (b), (l))
#endif

static uint8_t aTxBuffer[TXBUFFERSIZE];

/*
 *  AXP192�ν����
 */
ER
axp192_init(AXP192_Handler_t *hlcd)
{
	I2C_Handle_t *hi2c = hlcd->hi2c;
	ER ercd;

//	sysctl_set_power_mode(SYSCTL_POWER_BANK3, SYSCTL_POWER_V33);
	/*
	 *  Clear the interrupts
	 */
	syslog(LOG_NOTICE, "STEP1!!!!.");
    aTxBuffer[0] = 0x46;
    aTxBuffer[1] = 0xFF;
	if((ercd = i2c_send(hi2c, hlcd->saddr, aTxBuffer, TXBUFFERSIZE)) != E_OK)
		return ercd;

    // Wait 1.08ms
	dly_tsk(2);
    /*
	 *  K210_VCore(DCDC2) set to 0.9V
	 */
	syslog(LOG_NOTICE, "STEP2!!!!.");
    aTxBuffer[0] = 0x23;
    aTxBuffer[1] = 0x08;
	if((ercd = i2c_send(hi2c, hlcd->saddr, aTxBuffer, TXBUFFERSIZE)) != E_OK)
		return ercd;

    // Wait 1.08ms
	dly_tsk(2);
	/*
	 *  190mA Charging Current
	 */
    aTxBuffer[0] = 0x33;
    aTxBuffer[1] = 0xC1;
	if((ercd = i2c_send(hi2c, hlcd->saddr, aTxBuffer, TXBUFFERSIZE)) != E_OK)
		return ercd;

    // Wait 1.08ms
	dly_tsk(2);
	/*
	 *  4s shutdown
	 */
    aTxBuffer[0] = 0x36;
    aTxBuffer[1] = 0x6C;
	if((ercd = i2c_send(hi2c, hlcd->saddr, aTxBuffer, TXBUFFERSIZE)) != E_OK)
		return ercd;

    // Wait 1.08ms
	dly_tsk(2);
	/*
	 *  LCD Backlight: GPIO0 3.3V
	 */
    aTxBuffer[0] = 0x91;
    aTxBuffer[1] = 0xF0;
	if((ercd = i2c_send(hi2c, hlcd->saddr, aTxBuffer, TXBUFFERSIZE)) != E_OK)
		return ercd;

    // Wait 1.08ms
	dly_tsk(2);
	/*
	 *  GPIO LDO mode
	 */
    aTxBuffer[0] = 0x90;
    aTxBuffer[1] = 0x02;
	if((ercd = i2c_send(hi2c, hlcd->saddr, aTxBuffer, TXBUFFERSIZE)) != E_OK)
		return ercd;

    // Wait 1.08ms
	dly_tsk(2);
	/*
	 *  VDD2.8V net: LDO2 3.3V,  VDD 1.5V net: LDO3 1.8V
	 */
    aTxBuffer[0] = 0x28;
    aTxBuffer[1] = 0xF0;
	if((ercd = i2c_send(hi2c, hlcd->saddr, aTxBuffer, TXBUFFERSIZE)) != E_OK)
		return ercd;

    // Wait 1.08ms
	dly_tsk(2);
	/*
	 * VDD1.8V net:  DC-DC3 1.8V
	 */
    aTxBuffer[0] = 0x27;
    aTxBuffer[1] = 0x2C;
	if((ercd = i2c_send(hi2c, hlcd->saddr, aTxBuffer, TXBUFFERSIZE)) != E_OK)
		return ercd;

    // Wait 1.08ms
	dly_tsk(2);
	/*
	 *  open all power and EXTEN
	 */
    aTxBuffer[0] = 0x12;
    aTxBuffer[1] = 0xFF;
	if((ercd = i2c_send(hi2c, hlcd->saddr, aTxBuffer, TXBUFFERSIZE)) != E_OK)
		return ercd;

    // Wait 1.08ms
	dly_tsk(2);
	/*
	 *  VDD 0.9v net: DC-DC2 0.9V
	 */
    aTxBuffer[0] = 0x23;
    aTxBuffer[1] = 0x08;
	if((ercd = i2c_send(hi2c, hlcd->saddr, aTxBuffer, TXBUFFERSIZE)) != E_OK)
		return ercd;

    // Wait 1.08ms
	dly_tsk(2);
	/*
	 *  Cutoff voltage 3.2V
	 */
    aTxBuffer[0] = 0x31;
    aTxBuffer[1] = 0x03;
	if((ercd = i2c_send(hi2c, hlcd->saddr, aTxBuffer, TXBUFFERSIZE)) != E_OK)
		return ercd;

    // Wait 1.08ms
	dly_tsk(2);
	/*
	 *  Turnoff Temp Protect (Sensor not exist!)
	 */
    aTxBuffer[0] = 0x39;
    aTxBuffer[1] = 0xFC;
	if((ercd = i2c_send(hi2c, hlcd->saddr, aTxBuffer, TXBUFFERSIZE)) != E_OK)
		return ercd;
	return ercd;
}

