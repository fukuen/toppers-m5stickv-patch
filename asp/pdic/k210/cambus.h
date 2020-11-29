/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2008-2011 by Embedded and Real-Time Systems Laboratory
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
 *  @(#) $Id: cambus.h 698 2020-11-28 22:10:08Z fukuen $
 */
/*
 * 
 *  K210 CAMBUS�ǥХ����ɥ饤�Фγ������
 *
 */

#ifndef _CAMBUS_H_
#define _CAMBUS_H_

#include <kernel.h>
#include <target_syssvc.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAIX_PORTID
#define MAIX_PORTID        3
#endif

#if MAIX_PORTID == 1
#define MAIX_I2C_PORTID    I2C1_PORTID
#define MAIX_INHNO_I2CEV   IRQ_VECTOR_I2C0	/* ����ߥϥ�ɥ��ֹ� */
#define MAIX_INTNO_I2CEV   IRQ_VECTOR_I2C0	/* ������ֹ� */
#define MAIX_INTPRI_I2CEV  -5			/* �����ͥ���� */
#define MAIX_INTATR_I2CEV  0				/* �����°�� */
#endif
#if MAIX_PORTID == 2
#define MAIX_I2C_PORTID    I2C2_PORTID
#define MAIX_INHNO_I2CEV   IRQ_VECTOR_I2C1	/* ����ߥϥ�ɥ��ֹ� */
#define MAIX_INTNO_I2CEV   IRQ_VECTOR_I2C1	/* ������ֹ� */
#define MAIX_INTPRI_I2CEV  -5			/* �����ͥ���� */
#define MAIX_INTATR_I2CEV  0				/* �����°�� */
#endif
#if MAIX_PORTID == 3
#define MAIX_I2C_PORTID    I2C3_PORTID
#define MAIX_INHNO_I2CEV   IRQ_VECTOR_I2C2	/* ����ߥϥ�ɥ��ֹ� */
#define MAIX_INTNO_I2CEV   IRQ_VECTOR_I2C2	/* ������ֹ� */
#define MAIX_INTPRI_I2CEV  -5			/* �����ͥ���� */
#define MAIX_INTATR_I2CEV  0				/* �����°�� */
#endif

#define GC0328_ADDR     (0x42)
//#define GC0328_ADDR     (0x21)

#ifndef TOPPERS_MACRO_ONLY

extern ER cambus_init(int8_t pin_clk, int8_t pin_sda, uint8_t gpio_clk, uint8_t gpio_sda);
extern ER cambus_deinit();
extern int cambus_scan();
extern int cambus_scan_gc0328(void);
extern void cambus_set_writeb_delay(uint32_t delay);
extern ER cambus_writeb(uint8_t slv_addr, uint16_t reg_addr, uint8_t reg_data);
extern ER cambus_writeb(uint8_t slv_addr, uint16_t reg_addr, uint8_t reg_data);
extern uint8_t cambus_reg_width();

#endif /* TOPPERS_MACRO_ONLY */

#ifdef __cplusplus
}
#endif

#endif	/* _CAMBUS_H_ */

