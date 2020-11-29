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
 *  @(#) $Id: maix_i2c.c 699 2020-11-26 17:44:27Z fukuen $
 */
/*
 * 
 *  K210 SCCB I2C�ɥ饤�дؿ���
 *
 */
#include "kernel_impl.h"
#include <t_syslog.h>
#include <t_stdlib.h>
#include <string.h>
#include <sil.h>
#include <target_syssvc.h>
#include "device.h"
#include "i2c.h"
#include "maix_i2c.h"

/*
 *  SIL�ؿ��Υޥ������
 */
#define sil_orw_mem(a, b)		sil_wrw_mem((a), sil_rew_mem(a) | (b))
#define sil_andw_mem(a, b)		sil_wrw_mem((a), sil_rew_mem(a) & ~(b))
#define sil_modw_mem(a, b, c)	sil_wrw_mem((a), (sil_rew_mem(a) & (~b)) | (c))

/*
 *  I2C�ݡ���ID��������֥�å�����Ф�����Υޥ���
 */
#define INDEX_I2C(i2cid)        ((uint_t)((i2cid) - 1))

#ifndef I2C_TIMEOUT
#define I2C_TIMEOUT             500				/* 500ms */
#endif

#define I2C_CON_SPEED_STANDARD  0x00000020		/* <=100Kbit/s */
#define I2C_CON_SPEED_FAST      0x00000040		/* <=400Kbit/s or <=1000Kbit/s */
#define I2C_CON_SPEED_HIGH      0x00000060		/* <=3.4Mbit/s */

#define DEFAULT_THRESHOLD       3
#define DEFAULT_MASTER          (I2C_CON_MASTER_MODE | I2C_CON_SLAVE_DISABLE | I2C_CON_RESTART_EN)
#define DEFAULT_SLAVE           (I2C_CON_STOP_DET_IFADDRESSED)
#define DEFAULT_INTRM           (I2C_INTR_MASK_START_DET | I2C_INTR_MASK_STOP_DET | I2C_INTR_MASK_RD_REQ)

/*
 *  I2C�ݡ�������ơ��֥�
 */
static const I2C_PortControlBlock maix_i2c_pcb[NUM_I2CPORT] = {
  {	TADR_I2C0_BASE, SYSCTL_CLK_EN_PERI_I2C0_CLK_EN, 8,  FUNC_I2C0_SCLK, FUNC_I2C0_SDA },
  {	TADR_I2C1_BASE, SYSCTL_CLK_EN_PERI_I2C1_CLK_EN, 16, FUNC_I2C1_SCLK, FUNC_I2C1_SDA },
  {	TADR_I2C2_BASE, SYSCTL_CLK_EN_PERI_I2C2_CLK_EN, 24, FUNC_I2C2_SCLK, FUNC_I2C2_SDA }
};

static I2C_Handle_t maixI2cHandle[3];


/*
 *  I2Cž���Ԥ�
 *  parameter1 hi2c:   I2C�ϥ�ɥ�ؤΥݥ���
 *  parameter2 Timeout:�����ॢ���Ȼ���(ms)
 */
static ER
maix_i2c_transwait(I2C_Handle_t *hi2c, uint32_t Timeout)
{
	uint32_t tick = 0;

	while(hi2c->status != I2C_STATUS_READY){
		if(tick >= Timeout)
			return E_TMOUT;
		if(hi2c->Init.semid != 0)
			twai_sem(hi2c->Init.semid, 1);
		else
			dly_tsk(1);
		tick++;
	}
	if(hi2c->ErrorCode != 0)
		return E_SYS;
	else
		return E_OK;
}

/*
 *  I2C�ǡ��������⥸�塼��
 */
static void
maix_i2c_send_data(I2C_Handle_t *hi2c)
{
    uint32_t fifo_len, i,  tmp;

	if(hi2c->tXferCount > 0){
		uint8_t  *send_buf;
		int  addr_len = hi2c->aXferSize - hi2c->aXferCount;
        fifo_len = 8 - sil_rew_mem((uint32_t *)(hi2c->base+TOFF_I2C_TXFLR));
		if(addr_len > 0){
	        fifo_len = addr_len < fifo_len ? addr_len : fifo_len;
			send_buf = &hi2c->aBuffer[hi2c->aXferCount];
		}
		else{
			fifo_len = hi2c->tXferCount < fifo_len ? hi2c->tXferCount : fifo_len;
			send_buf = hi2c->pBuffPtr;
		}
        for(i = 0 ; i < fifo_len ; i++)
			sil_wrw_mem((uint32_t *)(hi2c->base+TOFF_I2C_DATA_CMD), *send_buf++);
		if(addr_len > 0)
			hi2c->aXferCount += fifo_len;
		else{
			hi2c->pBuffPtr  += fifo_len;
			hi2c->XferCount += fifo_len;
		}
        hi2c->tXferCount -= fifo_len;
    }
	(void)(tmp);
}

/*
 *  I2C�ǡ��������⥸�塼��
 */
static void
maix_i2c_recv_data(I2C_Handle_t *hi2c)
{
	uint32_t fifo_len, i, tmp;
	int rx_len = hi2c->XferSize - hi2c->XferCount;

	if(hi2c->tXferCount > 0 || rx_len > 0){
		fifo_len = sil_rew_mem((uint32_t *)(hi2c->base+TOFF_I2C_RXFLR));
		fifo_len = rx_len < fifo_len ? rx_len : fifo_len;
		for(i = 0 ; i < fifo_len ; i++){
			tmp = (uint8_t)sil_rew_mem((uint32_t *)(hi2c->base+TOFF_I2C_DATA_CMD));
			*hi2c->pBuffPtr++ = (uint8_t)tmp;
		}
		hi2c->XferCount += fifo_len;
		fifo_len = 8 - sil_rew_mem((uint32_t *)(hi2c->base+TOFF_I2C_TXFLR));
		fifo_len = hi2c->tXferCount < fifo_len ? hi2c->tXferCount : fifo_len;
		for(i = 0 ; i < fifo_len ; i++)
			sil_wrw_mem((uint32_t *)(hi2c->base+TOFF_I2C_DATA_CMD), I2C_DATA_CMD_CMD);
		hi2c->tXferCount -= fifo_len;
	}
}


/*
 *  I2C�ǥХ����ν����
 *  parameter1 hi2c  I2C�ϥ�ɥ�ؤΥݥ���
 *  return           ���ｪλ����E_OK
 */
I2C_Handle_t *
maix_i2c_init(ID portid, I2C_Init_t *ii2c)
{
	I2C_Handle_t *hi2c;
	const I2C_PortControlBlock *pcb;
    uint32_t v_i2c_freq = 0;
	uint32_t threshold = 0;
    int32_t v_period_clk_cnt2 = 0;
	uint16_t v_period_clk_cnt;
	uint8_t  no;
    uint32_t speed_mode = I2C_CON_SPEED_STANDARD;

	if(portid < I2C1_PORTID || portid > NUM_I2CPORT)
		return NULL;

	no = INDEX_I2C(portid);
	hi2c = &maixI2cHandle[no];
	hi2c->i2cid = portid;
	pcb = &maix_i2c_pcb[no];
	hi2c->base  = pcb->base;
	memcpy(&hi2c->Init, ii2c, sizeof(I2C_Init_t));

	/*
	 *  �ե��󥯥����ԥ�����
	 */
	if(ii2c->SdaPin >= 0)
		fpioa_set_function(ii2c->SdaPin, pcb->sdafunc);
	if(ii2c->SclPin >= 0)
		fpioa_set_function(ii2c->SclPin, pcb->sclfunc);

	/*
	 *  ����å�����
	 */
	sil_orw_mem((uint32_t *)(TADR_SYSCTL_BASE+TOFF_SYSCTL_CLK_EN_CENT), SYSCTL_CLK_SEL0_APB0_CLK_SEL);
	sil_orw_mem((uint32_t *)(TADR_SYSCTL_BASE+TOFF_SYSCTL_CLK_EN_PERI), (SYSCTL_CLK_EN_PERI_I2C0_CLK_EN<<no));
	threshold  = sil_rew_mem((uint32_t *)(TADR_SYSCTL_BASE+TOFF_SYSCTL_CLK_TH5));
	threshold &= ~(0xff << pcb->thresholdshift);
	threshold |= (DEFAULT_THRESHOLD << pcb->thresholdshift);
	sil_wrw_mem((uint32_t *)(TADR_SYSCTL_BASE+TOFF_SYSCTL_CLK_TH5), threshold);

	sil_orw_mem((uint32_t *)(TADR_SYSCTL_BASE+TOFF_SYSCTL_CLK_EN_CENT), SYSCTL_CLK_SEL0_APB0_CLK_SEL);
	sil_orw_mem((uint32_t *)(TADR_SYSCTL_BASE+TOFF_SYSCTL_CLK_EN_PERI), (SYSCTL_CLK_EN_PERI_I2C0_CLK_EN<<no));
	threshold  = sil_rew_mem((uint32_t *)(TADR_SYSCTL_BASE+TOFF_SYSCTL_CLK_TH5));
	threshold &= ~(0xff << pcb->thresholdshift);
	threshold |= (DEFAULT_THRESHOLD << pcb->thresholdshift);
	sil_wrw_mem((uint32_t *)(TADR_SYSCTL_BASE+TOFF_SYSCTL_CLK_TH5), threshold);

	threshold = sil_rew_mem((uint32_t *)(TADR_SYSCTL_BASE+TOFF_SYSCTL_CLK_TH5));
	threshold = (threshold >> pcb->thresholdshift) & 0xff;
    v_i2c_freq = get_pll_clock(0) / ((threshold + 1) * 2);
	v_period_clk_cnt2 = (((int64_t)v_i2c_freq * 1000L) / ii2c->ClockSpeed / 2) + 500;
	v_period_clk_cnt  = v_period_clk_cnt2 / 1000;

    if(v_period_clk_cnt <= 6)
        v_period_clk_cnt = 6;
    if(v_period_clk_cnt >= 65525)
        v_period_clk_cnt = 65525;
    if((ii2c->ClockSpeed > 100000) && (ii2c->ClockSpeed <= 1000000))
        speed_mode = I2C_CON_SPEED_FAST;
    else if(ii2c->ClockSpeed > 1000000)
        speed_mode = I2C_CON_SPEED_HIGH;
	else
        speed_mode = I2C_CON_SPEED_STANDARD;

	sil_wrw_mem((uint32_t *)(hi2c->base+TOFF_I2C_ENABLE), 0);
	if(ii2c->OwnAddress1 == 0)	/* master */
		sil_wrw_mem((uint32_t *)(hi2c->base+TOFF_I2C_CON), (DEFAULT_MASTER | ii2c->AddressingMode | speed_mode));
	else						/* slave */
		sil_wrw_mem((uint32_t *)(hi2c->base+TOFF_I2C_CON), (DEFAULT_SLAVE | ii2c->AddressingMode | speed_mode));
	sil_wrw_mem((uint32_t *)(hi2c->base+TOFF_I2C_SS_SCL_HCNT), v_period_clk_cnt);
	sil_wrw_mem((uint32_t *)(hi2c->base+TOFF_I2C_SS_SCL_LCNT), v_period_clk_cnt);

	sil_wrw_mem((uint32_t *)(hi2c->base+TOFF_I2C_TAR), ii2c->OwnAddress1/2);
	sil_wrw_mem((uint32_t *)(hi2c->base+TOFF_I2C_RX_TL), 0);
	sil_wrw_mem((uint32_t *)(hi2c->base+TOFF_I2C_TX_TL), 0);
	sil_wrw_mem((uint32_t *)(hi2c->base+TOFF_I2C_INTR_MASK), DEFAULT_INTRM);
	sil_wrw_mem((uint32_t *)(hi2c->base+TOFF_I2C_DMA_CR), (I2C_DMA_CR_RDMAE | I2C_DMA_CR_TDMAE));
	sil_wrw_mem((uint32_t *)(hi2c->base+TOFF_I2C_DMA_RDLR), 0);
	sil_wrw_mem((uint32_t *)(hi2c->base+TOFF_I2C_DMA_TDLR), 4);
	sil_wrw_mem((uint32_t *)(hi2c->base+TOFF_I2C_ENABLE), I2C_ENABLE_ENABLE);
	hi2c->status = I2C_STATUS_READY;
	syslog_3(LOG_NOTICE, "## abrt[%08x] dmacr[%08x] intrm[%08x] ##", sil_rew_mem((uint32_t *)(hi2c->base+TOFF_I2C_TX_ABRT_SOURCE)), sil_rew_mem((uint32_t *)(hi2c->base+TOFF_I2C_DMA_CR)), sil_rew_mem((uint32_t *)(hi2c->base+TOFF_I2C_INTR_MASK)));
	return hi2c;
}

/*
 *  I2C�ǥХ�����̵����
 *  parameter1 hi2c  I2C�ϥ�ɥ�ؤΥݥ���
 *  return           ���ｪλ����E_OK
 */
ER
maix_i2c_deinit(I2C_Handle_t *hi2c)
{
	uint8_t no;

	if(hi2c == NULL)
		return E_PAR;
	sil_wrw_mem((uint32_t *)(hi2c->base+TOFF_I2C_ENABLE), 0);
	no = INDEX_I2C(hi2c->i2cid);
	hi2c->status = I2C_STATUS_RESET;
	sil_andw_mem((uint32_t *)(TADR_SYSCTL_BASE+TOFF_SYSCTL_CLK_EN_PERI), (SYSCTL_CLK_EN_PERI_I2C0_CLK_EN<<no));
	return E_OK;
}

/*
 *  I2C���졼�ּ���
 *  parameter1 hi2c:   I2C�ϥ�ɥ�ؤΥݥ���
 *  parameter2 pData:  �ɤ߽Ф��Хåե��ؤΥݥ���
 *  parameter3 Size:   �ɤ߽Ф�������
 *  return             ���ｪλ�ʤ�E_OK
 */
ER
maix_i2c_slaveread(I2C_Handle_t *hi2c, uint8_t *pData, uint16_t Size)
{
	ER ercd = E_OK;
	uint32_t tmp;

	if(hi2c == NULL || pData == NULL || Size == 0)
		return E_PAR;
	if(hi2c->status != I2C_STATUS_READY)
		return E_OBJ;

	/*
	 *  �ɤ߽Ф���å�
	 */
	if(hi2c->Init.semlock != 0)
		wai_sem(hi2c->Init.semlock);

	tmp = sil_rew_mem((uint32_t *)(hi2c->base+TOFF_I2C_CLR_TX_ABRT));
	sil_wrw_mem((uint32_t *)(hi2c->base+TOFF_I2C_CLR_TX_ABRT), tmp);

	hi2c->aXferSize  = 0;
	hi2c->aXferCount = 0;
	hi2c->pBuffPtr   = pData;
	hi2c->XferSize   = Size;
	hi2c->XferCount  = 0;
	hi2c->tXferCount = Size;
	hi2c->status     = I2C_STATUS_BUSY_RX;
	sil_orw_mem((uint32_t *)(hi2c->base+TOFF_I2C_INTR_MASK), I2C_INTR_MASK_RX_FULL);

	maix_i2c_recv_data(hi2c);
	ercd = maix_i2c_transwait(hi2c, I2C_TIMEOUT);

	/*
	 *  �ɤ߽Ф���å����
	 */
	if(hi2c->Init.semlock != 0)
		sig_sem(hi2c->Init.semlock);
	return ercd;
}

/*
 *  I2C���졼������
 *  parameter1 hi2c:   I2C�ϥ�ɥ�ؤΥݥ���
 *  parameter2 pData:  �ɤ߽Ф��Хåե��ؤΥݥ���
 *  parameter3 Size:   �ɤ߽Ф�������
 *  return             ���ｪλ�ʤ�E_OK
 */
ER
maix_i2c_slavewrite(I2C_Handle_t *hi2c, uint8_t *pData, uint16_t Size)
{
	ER ercd = E_OK;
	uint32_t tmp;

	if(hi2c == NULL || pData == NULL || Size == 0)
		return E_PAR;
	if(hi2c->status != I2C_STATUS_READY)
		return E_OBJ;

	/*
	 *  �񤭹��ߥ�å�
	 */
	if(hi2c->Init.semlock != 0)
		wai_sem(hi2c->Init.semlock);

	tmp = sil_rew_mem((uint32_t *)(hi2c->base+TOFF_I2C_CLR_TX_ABRT));
	sil_wrw_mem((uint32_t *)(hi2c->base+TOFF_I2C_CLR_TX_ABRT), tmp);

	hi2c->aXferSize  = 0;
	hi2c->aXferCount = 0;
	hi2c->pBuffPtr   = pData;
	hi2c->XferSize   = Size;
	hi2c->XferCount  = 0;
	hi2c->tXferCount = Size;
	hi2c->status     = I2C_STATUS_BUSY_TX;

	maix_i2c_send_data(hi2c);
	sil_orw_mem((uint32_t *)(hi2c->base+TOFF_I2C_INTR_MASK), I2C_INTR_MASK_TX_EMPTY);
	ercd = maix_i2c_transwait(hi2c, I2C_TIMEOUT);

	while((sil_rew_mem((uint32_t *)(hi2c->base+TOFF_I2C_STATUS)) & I2C_STATUS_TFE) == 0){
		syslog_2(LOG_ERROR, "i2c_slavewrite status[%08x] abrt[%08x]", sil_rew_mem((uint32_t *)(hi2c->base+TOFF_I2C_STATUS)), sil_rew_mem((uint32_t *)(hi2c->base+TOFF_I2C_TX_ABRT_SOURCE)));
		dly_tsk(50);
	}

	/*
	 *  �񤭹��ߥ�å����
	 */
	if(hi2c->Init.semlock != 0)
		sig_sem(hi2c->Init.semlock);
	return ercd;
}

/*
 *  I2C�ǡ����꡼��
 *  parameter1 hi2c:       I2C�ϥ�ɥ�ؤΥݥ���
 *  parameter2 DevAddress: ���졼�֥��ɥ쥹
 *  parameter3 MemAddress: ���ꥢ�ɥ쥹
 *  parameter4 MemAddSize: ���ꥢ�ɥ쥹������
 *  parameter5 pData:      �ɤ߽Ф��Хåե��ؤΥݥ���
 *  parameter6 Size:       �ɤ߽Ф�������
 *  return                 ���ｪλ�ʤ�E_OK
 */
ER
maix_i2c_memread(I2C_Handle_t *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
	ER ercd = E_OK;
	int32_t  send_buf_len, timeout = Timeout;
	uint32_t fifo_len, abrt, i, tmp;

	if(hi2c == NULL || pData == NULL || Size == 0)
		return E_PAR;
	if(hi2c->status != I2C_STATUS_READY)
		return E_OBJ;

	/*
	 *  �ɤ߽Ф���å�
	 */
	if(hi2c->Init.semlock != 0)
		wai_sem(hi2c->Init.semlock);

	tmp = sil_rew_mem((uint32_t *)(hi2c->base+TOFF_I2C_CLR_TX_ABRT));
	sil_wrw_mem((uint32_t *)(hi2c->base+TOFF_I2C_CLR_TX_ABRT), tmp);

	if(MemAddSize == 2){
		hi2c->aBuffer[0] = (uint8_t)(MemAddress >> 8);
		hi2c->aBuffer[1] = (uint8_t)MemAddress;
	}
	else if(MemAddSize == 1)
		hi2c->aBuffer[0] = (uint8_t)MemAddress;
	hi2c->aXferSize  = MemAddSize;
	hi2c->aXferCount = 0;
	hi2c->pBuffPtr   = pData;
	hi2c->XferSize   = Size;
	hi2c->XferCount  = 0;
	hi2c->tXferCount = Size;
	hi2c->status     = I2C_STATUS_BUSY_RX;

	sil_wrw_mem((uint32_t *)(hi2c->base+TOFF_I2C_TAR), DevAddress/2);
	while(hi2c->aXferSize > hi2c->aXferCount){
		send_buf_len = hi2c->aXferSize - hi2c->aXferCount;
        fifo_len = 8 - sil_rew_mem((uint32_t *)(hi2c->base+TOFF_I2C_TXFLR));
        fifo_len = send_buf_len < fifo_len ? send_buf_len : fifo_len;
		if(fifo_len == 0){
			dly_tsk(1);
			if(--timeout == 0){
				hi2c->ErrorCode = I2C_ERROR_TIMEOUT;
				if(hi2c->Init.semlock != 0)
					sig_sem(hi2c->Init.semlock);
				return E_TMOUT;
			}
		}
        for(i = 0 ; i < fifo_len; i++)
			sil_wrw_mem((uint32_t *)(hi2c->base+TOFF_I2C_DATA_CMD), hi2c->aBuffer[hi2c->aXferCount++]);
		if((abrt = sil_rew_mem((uint32_t *)(hi2c->base+TOFF_I2C_TX_ABRT_SOURCE))) != 0){
			hi2c->ErrorCode = abrt;
			if(hi2c->Init.semlock != 0)
				sig_sem(hi2c->Init.semlock);
			return E_SYS;
		}
        hi2c->aXferCount += fifo_len;
    }
	sil_orw_mem((uint32_t *)(hi2c->base+TOFF_I2C_INTR_MASK), I2C_INTR_MASK_RX_FULL);

	maix_i2c_recv_data(hi2c);
	ercd = maix_i2c_transwait(hi2c, Timeout);

	/*
	 *  �ɤ߽Ф���å����
	 */
	if(hi2c->Init.semlock != 0)
		sig_sem(hi2c->Init.semlock);
	return ercd;
}

/*
 *  I2C�ǡ����饤��
 *  parameter1 hi2c:       I2C�ϥ�ɥ�ؤΥݥ���
 *  parameter2 DevAddress: ���졼�֥��ɥ쥹
 *  parameter3 MemAddress: ���ꥢ�ɥ쥹
 *  parameter4 MemAddSize: ���ꥢ�ɥ쥹������
 *  parameter5 pData:      ����ߥХåե��ؤΥݥ���
 *  parameter6 Size:       ����ߥ�����
 *  return                 ���ｪλ�ʤ�E_OK
 */
ER
maix_i2c_memwrite(I2C_Handle_t *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
	ER ercd;
	uint32_t tmp;

	if(hi2c == NULL || pData == NULL || Size == 0)
		return E_PAR;
	if(hi2c->status != I2C_STATUS_READY)
		return E_OBJ;

	/*
	 *  �񤭹��ߥ�å�
	 */
	if(hi2c->Init.semlock != 0)
		wai_sem(hi2c->Init.semlock);

	tmp = sil_rew_mem((uint32_t *)(hi2c->base+TOFF_I2C_CLR_TX_ABRT));
	sil_wrw_mem((uint32_t *)(hi2c->base+TOFF_I2C_CLR_TX_ABRT), tmp);

	if(MemAddSize == 2){
		hi2c->aBuffer[0] = (uint8_t)(MemAddress >> 8);
		hi2c->aBuffer[1] = (uint8_t)MemAddress;
	}
	else if(MemAddSize == 1)
		hi2c->aBuffer[0] = (uint8_t)MemAddress;
	hi2c->aXferSize  = MemAddSize;
	hi2c->aXferCount = 0;
	hi2c->pBuffPtr   = pData;
	hi2c->XferSize   = Size;
	hi2c->XferCount  = 0;
	hi2c->tXferCount = MemAddSize + Size;
	hi2c->status     = I2C_STATUS_BUSY_TX;

	sil_wrw_mem((uint32_t *)(hi2c->base+TOFF_I2C_TAR), DevAddress/2);
	maix_i2c_send_data(hi2c);
	sil_orw_mem((uint32_t *)(hi2c->base+TOFF_I2C_INTR_MASK), I2C_INTR_MASK_TX_EMPTY);
	ercd = maix_i2c_transwait(hi2c, Timeout);

	while((sil_rew_mem((uint32_t *)(hi2c->base+TOFF_I2C_STATUS)) & I2C_STATUS_TFE) == 0){
		syslog_2(LOG_NOTICE, "i2c_memwrite status[%08x] abrt[%08x]", sil_rew_mem((uint32_t *)(hi2c->base+TOFF_I2C_STATUS)), sil_rew_mem((uint32_t *)(hi2c->base+TOFF_I2C_TX_ABRT_SOURCE)));
		dly_tsk(100);
	}

	/*
	 *  �񤭹��ߥ�å����
	 */
	if(hi2c->Init.semlock != 0)
		sig_sem(hi2c->Init.semlock);
	return ercd;
}

/*
 *  I2C����ߥϥ�ɥ�
 */
void
maix_i2c_isr(intptr_t exinf)
{
	I2C_Handle_t *hi2c = &maixI2cHandle[INDEX_I2C(exinf)];
	uint32_t istatus = sil_rew_mem((uint32_t *)(hi2c->base+TOFF_I2C_INTR_STAT));
	uint32_t abrt   = sil_rew_mem((uint32_t *)(hi2c->base+TOFF_I2C_TX_ABRT_SOURCE));
	uint32_t status  = sil_rew_mem((uint32_t *)(hi2c->base+TOFF_I2C_STATUS));
	uint32_t tmp;
	syslog_3(LOG_DEBUG, "i2c_isr[%08x] abrt[%08x] status[%08x]", istatus, abrt, status);
	if(istatus & I2C_INTR_STAT_START_DET){
		tmp = sil_rew_mem((uint32_t *)(hi2c->base+TOFF_I2C_CLR_START_DET));
	}
	if(istatus & I2C_INTR_STAT_STOP_DET){
		tmp = sil_rew_mem((uint32_t *)(hi2c->base+TOFF_I2C_CLR_STOP_DET));
		if((status & I2C_STATUS_ACTIVITY) == 0){
			hi2c->status = I2C_STATUS_READY;
			hi2c->ErrorCode = abrt;
			if(abrt != 0 && hi2c->errorcallback != NULL)
				hi2c->errorcallback(hi2c);
			if(hi2c->Init.semid != 0)
				isig_sem(hi2c->Init.semid);
		}
	}
	if(istatus & I2C_INTR_STAT_TX_EMPTY){
		tmp = sil_rew_mem((uint32_t *)(hi2c->base+I2C_INTR_STAT_TX_EMPTY));
		if(abrt != 0){
			hi2c->status = I2C_STATUS_READY;
			hi2c->ErrorCode = abrt;
			if(hi2c->errorcallback != NULL)
				hi2c->errorcallback(hi2c);
			hi2c->tXferCount = 0;
		}
		else if(hi2c->status == I2C_STATUS_BUSY_TX){
			if(hi2c->tXferCount > 0)
				maix_i2c_send_data(hi2c);
		}
		if(hi2c->tXferCount == 0){
			sil_andw_mem((uint32_t *)(hi2c->base+TOFF_I2C_INTR_MASK), I2C_INTR_MASK_TX_EMPTY);
			if(hi2c->writecallback != NULL)
				hi2c->writecallback(hi2c);
		}
	}
	if(istatus & I2C_INTR_STAT_RX_FULL){
		if(abrt != 0){
			hi2c->status = I2C_STATUS_READY;
			hi2c->ErrorCode = abrt;
			if(hi2c->errorcallback != NULL)
				hi2c->errorcallback(hi2c);
			hi2c->tXferCount = 0;
			hi2c->XferCount  = hi2c->XferSize;
		}
		else if(hi2c->status == I2C_STATUS_BUSY_RX){
			if(hi2c->XferSize > hi2c->XferCount)
				maix_i2c_recv_data(hi2c);
		}
		if(hi2c->XferSize <= hi2c->XferCount){
			sil_andw_mem((uint32_t *)(hi2c->base+TOFF_I2C_INTR_MASK), I2C_INTR_MASK_RX_FULL);
			if(hi2c->readcallback != NULL)
				hi2c->readcallback(hi2c);
		}
	}
	if(istatus & I2C_INTR_STAT_RD_REQ){
		tmp = sil_rew_mem((uint32_t *)(hi2c->base+TOFF_I2C_CLR_RD_REQ));
    }
	(void)(tmp);
}

