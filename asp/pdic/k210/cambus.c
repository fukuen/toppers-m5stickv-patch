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
 *  @(#) $Id: cambus.c 699 2020-11-29 22:10:19Z fukuen $
 */
/*
 * 
 *  K210 CAMBUS�ɥ饤�дؿ���
 *
 */
#include <stddef.h>
#include <kernel.h>
#include <sil.h>
#include <t_syslog.h>
#include <target_syssvc.h>
#include <kernel_cfg.h>
#include "device.h"
#include "dvp.h"
#include "maix_i2c.h"
#include "cambus.h"

static uint32_t write_bus_delay = 10; //ms
static uint8_t sccb_reg_width = 8;

I2C_Init_t sccb_i2c_initd;
I2C_Handle_t *hi2c;

void cambus_set_writeb_delay(uint32_t delay)
{
    write_bus_delay = delay;
}

ER
sccb_i2c_init(uint8_t pin_clk, uint8_t pin_sda, uint8_t gpio_clk, uint8_t gpio_sda, uint32_t freq)
{
//    fpioa_set_function(pin_clk, FUNC_I2C0_SCLK + (MAIX_I2C_PORTID - 1) * 2);
//    fpioa_set_function(pin_sda, FUNC_I2C0_SDA + (MAIX_I2C_PORTID - 1) * 2);

	sccb_i2c_initd.ClockSpeed      = freq;
	sccb_i2c_initd.OwnAddress1     = 0;
	sccb_i2c_initd.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
	sccb_i2c_initd.SclPin          = pin_clk;
	sccb_i2c_initd.SdaPin          = pin_sda;
	sccb_i2c_initd.semid           = MAIX_I2CTRS_SEM;
	sccb_i2c_initd.semlock         = MAIX_I2CLOC_SEM;

	if((hi2c = maix_i2c_init(MAIX_I2C_PORTID, &sccb_i2c_initd)) == NULL){
		/* Initialization Error */
		syslog_0(LOG_ERROR, "## SCCB_I2C ERROR(1) ##");
		return E_SYS;
	}
	syslog_1(LOG_NOTICE, "SCCB_I2C INITAILIZE(%d) !", MAIX_I2C_PORTID);

	hi2c->writecallback = NULL;
	hi2c->readcallback  = NULL;
	hi2c->errorcallback = NULL;

    return E_OK;
}

ER
sccb_i2c_write_byte(uint8_t addr, uint16_t reg, uint8_t reg_len, uint8_t data, uint16_t timeout_ms)
{
    uint8_t tmp[3];
    if(reg_len == 8)
    {
        tmp[0] = reg & 0xFF;
        tmp[1] = data;
		return maix_i2c_memwrite(hi2c, addr, 0, 0, tmp, 2, 10);
    }
    else
    {
        tmp[0] = (reg>>8) & 0xFF;
        tmp[1] = reg&0xFF;
        tmp[2] = data;
		return maix_i2c_memwrite(hi2c, addr, 0, 0, tmp, 3, 10);
    }
    return 0;
}

ER
sccb_i2c_read_byte(uint8_t addr, uint16_t reg, uint8_t reg_len, uint8_t* data, uint16_t timeout_ms)
{
    *data = 0;
    uint8_t tmp[2];
    if(reg_len == 8)
    {
        tmp[0] = reg & 0xFF;
		ER ret = maix_i2c_memwrite(hi2c, addr, 0, 0, tmp, 1, 10);
        if(ret != E_OK)
            return ret;
		ret = maix_i2c_memread(hi2c, addr, 0, 0, data, 1, 10);
        return ret;
    }
    else
    {
        tmp[0] = (reg>>8) & 0xFF;
        tmp[1] = reg&0xFF;
		ER ret = maix_i2c_memwrite(hi2c, addr, 0, 0, tmp, 2, 10);
        if(ret != E_OK)
            return ret;
		ret = maix_i2c_memread(hi2c, addr, 0, 0, data, 1, 10);
        return ret;
    }
    return 0;
}

ER
sccb_i2c_recieve_byte(uint8_t addr, uint8_t* data, uint16_t timeout_ms)
{
	ER ret = maix_i2c_memread(hi2c, addr, 0, 0, data, 1, 10);
    return ret;
}

/*
 *  CAMBUS�������
 *  parameter1  pin_clk: SCCB_CLK_PIN
 *  parameter2  pin_sda: SCCB_SDA_PIN
 *  parameter3  gpio_clk: SCCB_CLK_NO
 *  parameter4  gpio_sda: SCCB_SDA_NO
 *  return      ER������
 */
ER
cambus_init(int8_t pin_clk, int8_t pin_sda, uint8_t gpio_clk, uint8_t gpio_sda)
{
    if(pin_clk<0 || pin_sda<0)
        return E_PAR;
    sccb_i2c_init(pin_clk, pin_sda, gpio_clk, gpio_sda, 100000);
	return E_OK;
}

/*
 *  CAMBUS��̵����
 *  return      ER������
 */
ER
cambus_deinit(void)
{
    maix_i2c_deinit(hi2c);
	return E_OK;
}

ER
cambus_read_id(uint8_t addr,uint16_t *manuf_id, uint16_t *device_id)
{
    *manuf_id = 0;
    *device_id = 0;
    uint8_t tmp = 0;
    ER ret = E_OK;

    // sccb_i2c_write_byte(addr, 0xFF, sccb_reg_width, 0x01, 10); // for OV2640
    ret |= sccb_i2c_read_byte(addr, 0x1C, sccb_reg_width, &tmp, 100);
	if(ret != E_OK)
		return ret;
    *manuf_id = tmp << 8;
    ret |= sccb_i2c_read_byte(addr, 0x1D, sccb_reg_width, &tmp, 100);
	if(ret != E_OK)
		return ret;
    *manuf_id |= tmp;
    ret |= sccb_i2c_read_byte(addr, 0x0A, sccb_reg_width, &tmp, 100);
	if(ret != E_OK)
		return ret;
    *device_id = tmp << 8;
    ret |= sccb_i2c_read_byte(addr, 0x0B, sccb_reg_width, &tmp, 100);
    *device_id |= tmp;
	return ret;
}

ER
cambus_read16_id(uint8_t addr,uint16_t *manuf_id, uint16_t *device_id)
{
    *manuf_id = 0;
    *device_id = 0;
    uint8_t tmp = 0;
    ER ret = E_OK;
    //TODO: 0x300A 0x300B maybe just for OV3660
    // sccb_i2c_write_byte(addr, 0xFF, sccb_reg_width, 0x01, 10);
    ret |= sccb_i2c_read_byte(addr, 0x300A, sccb_reg_width, &tmp, 100);
    if(ret != E_OK)
        return ret;
    *device_id = tmp << 8;
    ret |= sccb_i2c_read_byte(addr, 0x300B, sccb_reg_width, &tmp, 100);
    *device_id |= tmp;
    // printk("ret:%d %04x %04x\r\n",ret, *manuf_id, *device_id);
	return ret;
}

int cambus_scan()
{
	uint16_t manuf_id = 0;
	uint16_t device_id = 0;
    sccb_reg_width = 8;
    for (uint8_t addr=0x08; addr<=0x77; addr++) {
		if(cambus_read_id(addr ,&manuf_id,&device_id) != E_OK)
            continue;
        if(device_id!=0 && device_id!=0xffff)
        {
            return addr;
        }
    }
    sccb_reg_width = 16;
    for (uint8_t addr=0x08; addr<=0x77; addr++) {
		if( cambus_read16_id(addr ,&manuf_id,&device_id) != E_OK)
            continue;
        if(device_id!=0 && device_id!=0xffff)
        {
            return addr;
        }
    }
    return 0; // not found
}

int cambus_scan_gc0328(void)
{
    uint8_t id;
    sccb_reg_width = 8;
    sccb_i2c_write_byte(GC0328_ADDR, 0xFE, sccb_reg_width, 0x00, 10);
    sccb_i2c_read_byte(GC0328_ADDR, 0xF0, sccb_reg_width, &id, 10);
    if (id != 0x9d)
    {
        // mp_printf(&mp_plat_print, "error gc0328 detect, ret id is 0x%x\r\n", id);
        return 0;
    }
    return id;
}

ER
cambus_readb(uint8_t slv_addr, uint16_t reg_addr, uint8_t *reg_data)
{

    ER ret = E_OK;
    sccb_i2c_read_byte(slv_addr, reg_addr, sccb_reg_width, reg_data, 10);
	if(0xff == *reg_data)
		ret = E_SYS;

    return ret;

}

ER
cambus_writeb(uint8_t slv_addr, uint16_t reg_addr, uint8_t reg_data)
{
    sccb_i2c_write_byte(slv_addr, reg_addr, sccb_reg_width, reg_data, 10);
	dly_tsk(write_bus_delay);
	return E_OK;
}

uint8_t cambus_reg_width()
{
    return sccb_reg_width;
}
