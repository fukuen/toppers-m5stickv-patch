/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2008-2011 by Embedded and Real-Time Systems Laboratory
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2015-2019 by TOPPERS PROJECT Educational Working Group.
 *  Copyright (C) 2020-2021 by fukuen
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
 *  @(#) $Id: i2s.c 699 2020-12-26 17:44:27Z fukuen $
 */
/*
 * 
 *  K210 I2S�ɥ饤�дؿ���
 *
 */
#include "kernel_impl.h"
#include <t_syslog.h>
#include <t_stdlib.h>
#include <string.h>
#include <sil.h>
#include <target_syssvc.h>
#include "device.h"
#include "i2s.h"
#include "sysctl.h"
#include <math.h>

/*
 *  SIL�ؿ��Υޥ������
 */
#define sil_orw_mem(a, b)		sil_wrw_mem((a), sil_rew_mem(a) | (b))
#define sil_andw_mem(a, b)		sil_wrw_mem((a), sil_rew_mem(a) & ~(b))
#define sil_modw_mem(a, b, c)	sil_wrw_mem((a), (sil_rew_mem(a) & (~b)) | (c))

/*
 *  I2S�ݡ���ID��������֥�å�����Ф�����Υޥ���
 */
#define INDEX_I2S(i2sid)        ((uint_t)((i2sid) - 1))

#define DEFAULT_THRESHOLD       7

/*
 *  I2S�ݡ�������ơ��֥�
 */
static const I2S_PortControlBlock i2s_pcb[NUM_I2SPORT] = {
  {	TADR_I2S0_BASE, SYSCTL_CLK_EN_PERI_I2S0_CLK_EN, SYSCTL_PERI_RESET_I2S0_RESET, TADR_SYSCTL_BASE+TOFF_SYSCTL_CLK_TH3, SYSCTL_CLK_TH3_I2S0_CLK_THHD,  0 },
  {	TADR_I2S1_BASE, SYSCTL_CLK_EN_PERI_I2S1_CLK_EN, SYSCTL_PERI_RESET_I2S1_RESET, TADR_SYSCTL_BASE+TOFF_SYSCTL_CLK_TH3, SYSCTL_CLK_TH3_I2S1_CLK_THHD, 16 },
  {	TADR_I2S2_BASE, SYSCTL_CLK_EN_PERI_I2S2_CLK_EN, SYSCTL_PERI_RESET_I2S2_RESET, TADR_SYSCTL_BASE+TOFF_SYSCTL_CLK_TH4, SYSCTL_CLK_TH4_I2S2_CLK_THHD,  0 }
};

static I2S_Handle_t i2sHandle[NUM_I2SPORT];
static DMA_Handle_t i2s_dma_handle[NUM_I2SPORT][2];


/*
 *  DMA������Хå��ؿ�
 */
static void
i2s_dma_comp(DMA_Handle_t *hdma)
{
	I2S_Handle_t *hi2s = (I2S_Handle_t *)hdma->localdata;
	if(hi2s != NULL && hi2s->Init.semdmaid != 0){
		isig_sem(hi2s->Init.semdmaid);
	}
	if(hi2s != NULL && hdma->status == DMA_STATUS_READY_TRN1){
		hi2s->status = I2S_STATUS_READY;
		if(hi2s->writecallback != NULL && (hi2s->Init.RxTxMode == I2S_TRANSMITTER || hi2s->Init.RxTxMode == I2S_BOTH)){
			hi2s->writecallback(hi2s);
		}
		if(hi2s->readcallback != NULL && (hi2s->Init.RxTxMode == I2S_RECEIVER || hi2s->Init.RxTxMode == I2S_BOTH)){
			hi2s->readcallback(hi2s);
		}
	}
}

/*
 *  DMA���顼������Хå��ؿ�
 */
static void
i2s_dma_error(DMA_Handle_t *hdma)
{
	I2S_Handle_t *hi2s = (I2S_Handle_t *)hdma->localdata;
	if(hi2s != NULL && hi2s->Init.semdmaid != 0){
		isig_sem(hi2s->Init.semdmaid);
	}
	if(hi2s != NULL){
		hi2s->status = I2S_STATUS_TIMEOUT;
		if(hi2s->errorcallback != NULL){
			hi2s->errorcallback(hi2s);
		}
	}
}

/*
 *  DMAž�����ϴؿ�
 */
DMA_Handle_t *
i2s_dmac_set_single_mode(I2S_Handle_t *hi2s, uint8_t rtx,
						  const void *src, void *dest,
						  uint8_t src_inc, uint8_t dest_inc,
						  uint8_t dmac_burst_size, uint8_t dmac_trans_width,
						  size_t block_size)
{
	DMA_Handle_t *hdma;
	int mem_type_src, mem_type_dest;
	uint8_t flow_control;

	if(rtx == 0){
		hdma = hi2s->hdmatx;
		flow_control = DMA_MEMORY_TO_PERIPH;
		mem_type_src = 1;
		mem_type_dest = 0;
		hi2s->status = I2S_STATUS_BUSY_TX;
	}
	else{
		hdma = hi2s->hdmarx;
		flow_control = DMA_PERIPH_TO_MEMORY;
		mem_type_src = 0;
		mem_type_dest = 1;
		hi2s->status = I2S_STATUS_BUSY_RX;
	}
	hi2s->status = I2S_STATUS_BUSY;

	hdma->Init.Direction    = flow_control;	/* DMAž������ */
	hdma->Init.SrcHandShake = (mem_type_src ? DMAC_HS_SOFTWARE : DMAC_HS_HARDWARE);	/* �������ϥ�ɥ������� */
	hdma->Init.DrcHandShake = (mem_type_dest ? DMAC_HS_SOFTWARE : DMAC_HS_HARDWARE);	/* �ǥ��ƥ��͡������ϥ�ɥ������� */
	hdma->Init.SrcInc       = src_inc;	/* ���������󥯥�������� */
	hdma->Init.DstInc       = dest_inc;	/* �ǥ��ƥ��͡�����󥤥󥯥�������� */
	hdma->Init.SrcTransWidth = dmac_trans_width;	/* ������ž���� */
	hdma->Init.DstTransWidth = dmac_trans_width;	/* �ǥ��ƥ��͡������ž���� */
	hdma->Init.SrcBurstSize = dmac_burst_size;	/* �������С����ȥ����� */
	hdma->Init.DstBurstSize = dmac_burst_size;	/* �ǥ��ƥ��͡������С����ȥ����� */
	dma_reset(hdma);
	dma_start(hdma, (uintptr_t)src, (uintptr_t)dest, block_size);
	return hdma;
}

/*
 *  SPI-DMAž����λ�Ԥ�
 */
ER
i2s_dmac_wait_idle(DMA_Handle_t * hdma)
{
	I2S_Handle_t *hi2s = (I2S_Handle_t *)hdma->localdata;
	ER ercd = E_OK;
	int tick = DMA_TRS_TIMEOUT;

	while((hdma->status == DMA_STATUS_BUSY) && tick > 0){
		if(hi2s != NULL && hi2s->Init.semdmaid != 0){
	 		ercd = twai_sem(hi2s->Init.semdmaid, 5);
		}
		else
			dly_tsk(1);
		tick--;
	}
	dma_end(hdma);
	if(hdma->ErrorCode != 0)
		ercd = E_OBJ;
	else if(tick == 0)
		ercd = E_TMOUT;
	return ercd;
}

static ER
i2s_recv_channel_enable(I2S_Handle_t *hi2s, i2s_channel_num_t channel_num, uint32_t enable)
{
	if (channel_num < I2S_CHANNEL_0 || channel_num > I2S_CHANNEL_3)
		return E_PAR;
	/* Bit 0 is receive channel enable/disable, 0 for receive channel disable,
	 *1 for receive channel enable
	 */
	/* Bits [31:1] is reseved */
	sil_modw_mem((uint32_t *)(hi2s->base+TOFF_I2S_CHANNEL+channel_num*I2S_CHANNEL_WINDOW_SIZE+TOFF_I2SC_RER), I2SC_RER_RXCHENX, enable);
	return E_OK;
}

static ER
i2s_transmit_channel_enable(I2S_Handle_t *hi2s, i2s_channel_num_t channel_num, uint32_t enable)
{
	if (channel_num < I2S_CHANNEL_0 || channel_num > I2S_CHANNEL_3)
		return E_PAR;

	/* Bit 0 is transmit channel enable/disable, 0 for transmit channel disable,
	 * 1 for transmit channel enable
	 */
	/* Bits [31:1] is reseved */
	sil_modw_mem((uint32_t *)(hi2s->base+TOFF_I2S_CHANNEL+channel_num*I2S_CHANNEL_WINDOW_SIZE+TOFF_I2SC_TER), I2SC_TER_TXCHENX, enable);
	return E_OK;
}

static void
i2s_receive_enable(I2S_Handle_t *hi2s, i2s_channel_num_t channel_num)
{
	/* Bit 0 is receiver block  enable,
	* 0 for receiver disable
	* 1 for receiver enable
	*/
	/* Bits [31:1] is reserved */
	sil_orw_mem((uint32_t *)(hi2s->base+TOFF_I2S_IRER), 0x00000001);
	/* Receiver block enable */

	i2s_recv_channel_enable(hi2s, channel_num, 1);
	/* Receive channel enable */
}

static void
i2s_transimit_enable(I2S_Handle_t *hi2s, i2s_channel_num_t channel_num)
{
	/* Bit 0 is transmitter block  enable,
	* 0 for transmitter disable
	* 1 for transmitter enable
	*/
	/* Bits [31:1] is reserved */
	sil_orw_mem((uint32_t *)(hi2s->base+TOFF_I2S_ITER), 0x00000001);
	/* Transmitter block enable */

	i2s_transmit_channel_enable(hi2s, channel_num, 1);
	/* Transmit channel enable */
}

static void
i2s_set_enable(I2S_Handle_t *hi2s, uint32_t enable)
{
	/* Bit 0 is ien, 0 for disable i2s and 1 for enable i2s */
	/* Bits [31:1] is reserved */
	sil_modw_mem((uint32_t *)(hi2s->base+TOFF_I2S_IER), 0x00000001, enable);
}

static void
i2s_disable_block(I2S_Handle_t *hi2s, i2s_transmit_t rxtx_mode)
{
	if (rxtx_mode == I2S_RECEIVER)
	{
		/* Bit 0 is receiver block  enable,
		* 0 for receiver disable
		* 1 for receiver enable
		*/
		/* Bits [31:1] is reserved */
		sil_andw_mem((uint32_t *)(hi2s->base+TOFF_I2S_IRER), 0x00000001);
		/* Receiver block disable */
	}
	else
	{
		/* Bit 0 is transmitter block  enable,
		* 0 for transmitter disable
		* 1 for transmitter enable
		*/
		/* Bits [31:1] is reserved */
		sil_andw_mem((uint32_t *)(hi2s->base+TOFF_I2S_ITER), 0x00000001);
		/* Transmitter block disable */
	}
}

static ER
i2s_set_rx_word_length(I2S_Handle_t *hi2s, i2s_word_length_t word_length, i2s_channel_num_t channel_num)
{
	if (word_length > RESOLUTION_32_BIT || word_length < IGNORE_WORD_LENGTH)
		return E_PAR;
	if (channel_num < I2S_CHANNEL_0 || channel_num > I2S_CHANNEL_3)
		return E_PAR;

	/* Bits [2:0] is used to program desired data resolution of
	 * receiver/transmitter,
	 * 0x0 for ignore the word length
	 * 0x1 for 12-bit data resolution of the receiver/transmitter,
	 * 0x2 for 16-bit data resolution of the receiver/transmitter,
	 * 0x3 for 20-bit data resolution of the receiver/transmitter,
	 * 0x4 for 24-bit data resolution of the receiver/transmitter,
	 * 0x5 for 32-bit data resolution of the receiver/transmitter
	 */
	/* Bits [31:3] is reseved */
	sil_modw_mem((uint32_t *)(hi2s->base+TOFF_I2S_CHANNEL+channel_num*I2S_CHANNEL_WINDOW_SIZE+TOFF_I2SC_RCR), I2SC_RCR_WLEN, word_length);
	return E_OK;
}

static ER
i2s_set_tx_word_length(I2S_Handle_t *hi2s, i2s_word_length_t word_length, i2s_channel_num_t channel_num)
{
	if (word_length > RESOLUTION_32_BIT || word_length < IGNORE_WORD_LENGTH)
		return E_PAR;
	if (channel_num < I2S_CHANNEL_0 || channel_num > I2S_CHANNEL_3)
		return E_PAR;

	/* Bits [2:0] is used to program desired data resolution of
	 * receiver/transmitter,
	 * 0x0 for ignore the word length
	 * 0x1 for 12-bit data resolution of the receiver/transmitter,
	 * 0x2 for 16-bit data resolution of the receiver/transmitter,
	 * 0x3 for 20-bit data resolution of the receiver/transmitter,
	 * 0x4 for 24-bit data resolution of the receiver/transmitter,
	 * 0x5 for 32-bit data resolution of the receiver/transmitter
	 */
	/* Bits [31:3] is reseved */
	sil_modw_mem((uint32_t *)(hi2s->base+TOFF_I2S_CHANNEL+channel_num*I2S_CHANNEL_WINDOW_SIZE+TOFF_I2SC_TCR), I2SC_TCR_WLEN, word_length);
	return E_OK;
}

static ER
i2s_master_configure(I2S_Handle_t *hi2s,
					i2s_word_select_cycles_t word_select_size,
					i2s_sclk_gating_cycles_t gating_cycles,
					i2s_work_mode_t word_mode)
{
	if (word_select_size < SCLK_CYCLES_16 || word_select_size > SCLK_CYCLES_32)
		return E_PAR;
	if (gating_cycles < NO_CLOCK_GATING || gating_cycles > CLOCK_CYCLES_24)
		return E_PAR;

	sil_andw_mem((uint32_t *)(hi2s->base+TOFF_I2S_CER), I2S_CER_CLKEN);

	/* Bits [4:3] used program  the number of sclk cycles for which the
	 * word select line stayd in the left aligned or right aligned mode.
	 * 0x0 for 16sclk cycles, 0x1 for 24 sclk cycles 0x2 for 32 sclk
	 * cycles
	 */
	uint32_t ccr = sil_rew_mem((uint32_t *)(hi2s->base+TOFF_I2S_CCR));
	ccr &= ~I2S_CCR_CLK_WORD_SIZE;
	ccr |= (word_select_size<<3);
//	sil_modw_mem((uint32_t *)(hi2s->base+TOFF_I2S_CCR), I2S_CCR_CLK_WORD_SIZE, word_select_size<<3);
	/* Bits [2:0] is used to program  the gating of sclk,
	 * 0x0 for clock gating is diable,
	 * 0x1 for gating after 12 sclk cycles
	 * 0x2 for gating after 16 sclk cycles
	 * 0x3 for gating after 20 sclk cycles
	 * 0x4 for gating after 24 sclk cycles
	 */
	ccr &= ~I2S_CCR_CLK_GATE;
	ccr |= gating_cycles;
//	sil_modw_mem((uint32_t *)(hi2s->base+TOFF_I2S_CCR), I2S_CCR_CLK_GATE, gating_cycles);
	/* Bit[5:7] is alignment mode setting.
	 * 0x1 for standard i2s format
	 * 0x2 for right aligned format
	 * 0x4 for left aligned format
	 */
	ccr &= ~I2S_CCR_ALIGN_MODE;
	ccr |= (word_mode<<5);
//	sil_modw_mem((uint32_t *)(hi2s->base+TOFF_I2S_CCR), I2S_CCR_ALIGN_MODE, word_mode<<5);
	sil_wrw_mem((uint32_t *)(hi2s->base+TOFF_I2S_CCR), ccr);

	/* Bit 0 is clock generation enable/disable,
	 * 0 for clock generation disable,
	 * 1 for clock generation enable
	 */
	/* Bits [31:1] is reserved */
	sil_orw_mem((uint32_t *)(hi2s->base+TOFF_I2S_CER), I2S_CER_CLKEN);
	/* Clock generation enable */
	return E_OK;
}

static ER
i2s_set_rx_threshold(I2S_Handle_t *hi2s,
					i2s_fifo_threshold_t threshold,
					i2s_channel_num_t channel_num)
{
	if (threshold < TRIGGER_LEVEL_1 || threshold > TRIGGER_LEVEL_16)
		return E_PAR;
	if (channel_num < I2S_CHANNEL_0 || channel_num > I2S_CHANNEL_3)
		return E_PAR;

	/* Bits [3:0] is used program the trigger level in the RX FIFO at
	 * which the receiver data available interrupt generate,
	 * 0x0 for interrupt trigger when FIFO level is 1,
	 * 0x2 for interrupt trigger when FIFO level is 2,
	 * 0x3 for interrupt trigger when FIFO level is 4,
	 * 0x4 for interrupt trigger when FIFO level is 5,
	 * 0x5 for interrupt trigger when FIFO level is 6,
	 * 0x6 for interrupt trigger when FIFO level is 7,
	 * 0x7 for interrupt trigger when FIFO level is 8,
	 * 0x8 for interrupt trigger when FIFO level is 9,
	 * 0x9 for interrupt trigger when FIFO level is 10,
	 * 0xa for interrupt trigger when FIFO level is 11,
	 * 0xb for interrupt trigger when FIFO level is 12,
	 * 0xc for interrupt trigger when FIFO level is 13,
	 * 0xd for interrupt trigger when FIFO level is 14,
	 * 0xe for interrupt trigger when FIFO level is 15,
	 * 0xf for interrupt trigger when FIFO level is 16
	 */
	/* Bits [31:4] is reserved */
	sil_modw_mem((uint32_t *)(hi2s->base+TOFF_I2S_CHANNEL+channel_num*I2S_CHANNEL_WINDOW_SIZE+TOFF_I2SC_RFCR), I2SC_RFCR_RXCHDT, threshold);
	return E_OK;
}

static ER
i2s_set_tx_threshold(I2S_Handle_t *hi2s,
					i2s_fifo_threshold_t threshold,
					i2s_channel_num_t channel_num)
{
	if (threshold < TRIGGER_LEVEL_1 || threshold > TRIGGER_LEVEL_16)
		return E_PAR;
	if (channel_num < I2S_CHANNEL_0 || channel_num > I2S_CHANNEL_3)
		return E_PAR;

	/* Bits [3:0] is used program the trigger level in the TX FIFO at
	 * which the receiver data available interrupt generate,
	 * 0x0 for interrupt trigger when FIFO level is 1,
	 * 0x2 for interrupt trigger when FIFO level is 2,
	 * 0x3 for interrupt trigger when FIFO level is 4,
	 * 0x4 for interrupt trigger when FIFO level is 5,
	 * 0x5 for interrupt trigger when FIFO level is 6,
	 * 0x6 for interrupt trigger when FIFO level is 7,
	 * 0x7 for interrupt trigger when FIFO level is 8,
	 * 0x8 for interrupt trigger when FIFO level is 9,
	 * 0x9 for interrupt trigger when FIFO level is 10,
	 * 0xa for interrupt trigger when FIFO level is 11,
	 * 0xb for interrupt trigger when FIFO level is 12,
	 * 0xc for interrupt trigger when FIFO level is 13,
	 * 0xd for interrupt trigger when FIFO level is 14,
	 * 0xe for interrupt trigger when FIFO level is 15,
	 * 0xf for interrupt trigger when FIFO level is 16
	 */
	/* Bits [31:4] is reserved */
	sil_modw_mem((uint32_t *)(hi2s->base+TOFF_I2S_CHANNEL+channel_num*I2S_CHANNEL_WINDOW_SIZE+TOFF_I2SC_TFCR), I2CS_TFCR_TXCHET, threshold);
	return E_OK;
}

static ER
i2s_set_mask_interrupt(I2S_Handle_t *hi2s,
						i2s_channel_num_t channel_num,
						uint32_t rx_available_int, uint32_t rx_overrun_int,
						uint32_t tx_empty_int, uint32_t tx_overrun_int)
{
	if (channel_num < I2S_CHANNEL_0 || channel_num > I2S_CHANNEL_3)
		return E_PAR;

	uint32_t imr = sil_rew_mem((uint32_t *)(hi2s->base+TOFF_I2S_CHANNEL+channel_num*I2S_CHANNEL_WINDOW_SIZE+TOFF_I2SC_IMR));
	if (rx_available_int == 1)
		imr |= I2SC_IMR_RXDAM;
	else
		imr &= ~I2SC_IMR_RXDAM;
	if (rx_overrun_int == 1)
		imr |= I2SC_IMR_RXFOM;
	else
		imr &= ~I2SC_IMR_RXFOM;

	if (tx_empty_int == 1)
		imr |= I2SC_IMR_TXFEM;
	else
		imr &= ~I2SC_IMR_TXFEM;
	if (tx_overrun_int == 1)
		imr |= I2SC_IMR_TXFOM;
	else
		imr &= ~I2SC_IMR_TXFOM;
	sil_wrw_mem((uint32_t *)(hi2s->base+TOFF_I2S_CHANNEL+channel_num*I2S_CHANNEL_WINDOW_SIZE+TOFF_I2SC_IMR), imr);

	return E_OK;
}

static ER
i2s_transmit_dma_enable(I2S_Handle_t *hi2s, uint32_t enable)
{
	if(hi2s == NULL)
		return E_PAR;

	/* Bit[8] is DMA transmit enable control */
	sil_modw_mem((uint32_t *)(hi2s->base+TOFF_I2S_CCR), I2S_CCR_DMA_TX_EN, (enable == 0) ? 0 : I2S_CCR_DMA_TX_EN);
	return E_OK;
}

static ER
i2s_receive_dma_enable(I2S_Handle_t *hi2s, uint32_t enable)
{
	if(hi2s == NULL)
		return E_PAR;

	/* Bit[9] is DMA receive enable control */
	sil_modw_mem((uint32_t *)(hi2s->base+TOFF_I2S_CCR), I2S_CCR_DMA_RX_EN, (enable == 0) ? 0 : I2S_CCR_DMA_RX_EN);
	return E_OK;
}

ER
i2s_set_dma_divide_16(I2S_Handle_t *hi2s, uint32_t enable)
{
	if(hi2s == NULL)
		return E_PAR;

	/* Bit[10] split 32bit data to two 16 bit data and filled in left
	 * and right channel. Used with dma_tx_en or dma_rx_en
	 */
	sil_modw_mem((uint32_t *)(hi2s->base+TOFF_I2S_CCR), I2S_CCR_DMA_DIVIED_16, (enable == 0) ? 0 : I2S_CCR_DMA_DIVIED_16);
	return E_OK;
}

int
i2s_get_dma_divide_16(I2S_Handle_t *hi2s)
{
	if(hi2s == NULL)
		return E_PAR;

	/* Bit[10] split 32bit data to two 16 bit data and filled in left
	 * and right channel. Used with dma_tx_en or dma_rx_en
	 */
	int dma_divide_16 = sil_rew_mem((uint32_t *)(hi2s->base+TOFF_I2S_CCR));
	return ((dma_divide_16 & I2S_CCR_DMA_DIVIED_16) == 0) ? 0 : 1;
}

/*
 *  I2S�����¹Դؿ� non DMA
 *  parameter1  hi2s: I2S�ϥ�ɥ�ؤΥݥ���
 *  parameter2  channel_num: �����ͥ��ֹ�
 *  parameter3  buf: �����Хåե��ؤΥݥ���
 *  parameter4  buf_len: ����������
 *  return ER������
 */
ER
i2s_receive_data_standard(I2S_Handle_t *hi2s, i2s_channel_num_t channel_num, uint64_t *buf, size_t buf_len)
{
	uint32_t i = 0;
	uint32_t rxda;

	sil_rew_mem((uint32_t *)(hi2s->base+TOFF_I2S_CHANNEL+channel_num*I2S_CHANNEL_WINDOW_SIZE+TOFF_I2SC_ROR));
	/*clear over run*/

	for (i = 0; i < buf_len; i++)
	{
		rxda = sil_rew_mem((uint32_t *)(hi2s->base+TOFF_I2S_CHANNEL+channel_num*I2S_CHANNEL_WINDOW_SIZE+TOFF_I2SC_ISR));
		rxda &= I2SC_ISR_RXDA;
		if (rxda == 0)
		{
			dly_tsk(1);
		}
		else
		{
			buf[i] = sil_rew_mem((uint32_t *)(hi2s->base+TOFF_I2S_CHANNEL+channel_num*I2S_CHANNEL_WINDOW_SIZE+TOFF_I2SC_LEFT_RXTX));
			buf[i] <<= 32;
			buf[i] = sil_rew_mem((uint32_t *)(hi2s->base+TOFF_I2S_CHANNEL+channel_num*I2S_CHANNEL_WINDOW_SIZE+TOFF_I2SC_RIGHT_RXTX));
		}
	}
	return E_OK;
}

/*
 *  I2S�����¹Դؿ�
 *  parameter1  hi2s: I2S�ϥ�ɥ�ؤΥݥ���
 *  parameter2  buf: �����Хåե��ؤΥݥ���
 *  parameter3  buf_len: ����������
 *  return ER������
 */
ER
i2s_receive_data(I2S_Handle_t *hi2s, uint32_t *buf, size_t buf_len)
{
	if(hi2s == NULL)
		return E_PAR;

	ER ercd = E_OK;
	DMA_Handle_t *hdmarx = hi2s->hdmarx;

	i2s_dmac_wait_idle(hdmarx);
	hdmarx = i2s_dmac_set_single_mode(hi2s, 1, (const void *)(hi2s->base+TOFF_I2S_RXDMA), (void *)buf,
							DMAC_ADDR_NOCHANGE, DMAC_ADDR_INCREMENT,
							DMAC_MSIZE_1, DMAC_TRANS_WIDTH_32, buf_len);

	return E_OK;
}

/*
 *  I2S�����¹Դؿ� non DMA
 *  parameter1  hi2s: I2S�ϥ�ɥ�ؤΥݥ���
 *  parameter2  channel_num: �����ͥ��ֹ�
 *  parameter3  pcm: �����Хåե��ؤΥݥ���
 *  parameter4  buf_len: ����������
 *  parameter5  single_length: �ӥå�Ĺ
 *  return ER������
 */
ER
i2s_send_data_standard(I2S_Handle_t *hi2s, i2s_channel_num_t channel_num, const uint8_t *pcm, size_t buf_len,
				  size_t single_length)
{
	uint32_t left_buffer = 0;
	uint32_t right_buffer = 0;
	uint32_t i = 0;
	uint32_t j = 0;
	if (channel_num < I2S_CHANNEL_0 || channel_num > I2S_CHANNEL_3)
		return E_PAR;

	buf_len = buf_len / (single_length / 8) / 2; /* sample num */
	sil_rew_mem((uint32_t *)(hi2s->base+TOFF_I2S_CHANNEL+channel_num*I2S_CHANNEL_WINDOW_SIZE+TOFF_I2SC_TOR));
	/* read clear overrun flag */

	for (j = 0; j < buf_len;)
	{
		uint32_t txfe = sil_rew_mem((uint32_t *)(hi2s->base+TOFF_I2S_CHANNEL+channel_num*I2S_CHANNEL_WINDOW_SIZE+TOFF_I2SC_ISR));
		txfe &= I2SC_ISR_TXFE;
		if (txfe == 0)
		{
			dly_tsk(1);
		}
		else
		{
			switch(single_length)
			{
				case 16:
					left_buffer = ((uint16_t *)pcm)[i++];
					right_buffer = ((uint16_t *)pcm)[i++];
					break;
				case 24:
					left_buffer = 0;
					left_buffer |= pcm[i++];
					left_buffer |= pcm[i++] << 8;
					left_buffer |= pcm[i++] << 16;
					right_buffer = 0;
					right_buffer |= pcm[i++];
					right_buffer |= pcm[i++] << 8;
					right_buffer |= pcm[i++] << 16;
					break;
				case 32:
					left_buffer = ((uint32_t *)pcm)[i++];
					right_buffer = ((uint32_t *)pcm)[i++];
					break;
				default:
					left_buffer = pcm[i++];
					right_buffer = pcm[i++];
					break;
			}
			sil_wrw_mem((uint32_t *)(hi2s->base+TOFF_I2S_CHANNEL+channel_num*I2S_CHANNEL_WINDOW_SIZE+TOFF_I2SC_LEFT_RXTX), left_buffer);
			sil_wrw_mem((uint32_t *)(hi2s->base+TOFF_I2S_CHANNEL+channel_num*I2S_CHANNEL_WINDOW_SIZE+TOFF_I2SC_RIGHT_RXTX), right_buffer);
			j++;
		}
	}
	return E_OK;
}

/*
 *  I2S�����¹Դؿ�
 *  parameter1  hi2s: I2S�ϥ�ɥ�ؤΥݥ���
 *  parameter2  buf: �����Хåե��ؤΥݥ���
 *  parameter3  buf_len: ����������
 *  return ER������
 */
ER
i2s_send_data(I2S_Handle_t *hi2s, const void *buf, size_t buf_len)
{
	if(hi2s == NULL)
		return E_PAR;

	ER ercd = E_OK;
	DMA_Handle_t *hdma = hi2s->hdmatx;

	i2s_dmac_wait_idle(hdma);
	hdma = i2s_dmac_set_single_mode(hi2s, 0, (const void *)buf, 
						(void *)(hi2s->base+TOFF_I2S_TXDMA), DMAC_ADDR_INCREMENT, DMAC_ADDR_NOCHANGE,
						DMAC_MSIZE_1, DMAC_TRANS_WIDTH_32, buf_len);

	return ercd;
}

static inline void
i2s_set_sign_expand_en(I2S_Handle_t *hi2s, uint32_t enable)
{
	sil_modw_mem((uint32_t *)(hi2s->base+TOFF_I2S_CCR), I2S_CCR_SIGN_EXPAND_EN, (enable == 0) ? 0 : I2S_CCR_SIGN_EXPAND_EN);
}

void
i2s_rx_channel_config(I2S_Handle_t *hi2s,
	i2s_channel_num_t channel_num,
	i2s_word_length_t word_length,
	i2s_word_select_cycles_t word_select_size,
	i2s_fifo_threshold_t trigger_level,
	i2s_work_mode_t word_mode)
{
	i2s_recv_channel_enable(hi2s, channel_num, 0);
	/* Receive channel disable */

	sil_wrw_mem((uint32_t *)(hi2s->base+TOFF_I2S_CHANNEL+channel_num*I2S_CHANNEL_WINDOW_SIZE+TOFF_I2SC_TER), 0);
	/* disable tx */

	sil_wrw_mem((uint32_t *)(hi2s->base+TOFF_I2S_CHANNEL+channel_num*I2S_CHANNEL_WINDOW_SIZE+TOFF_I2SC_RFF), 1);
	/* flash individual fifo */

	sil_wrw_mem((uint32_t *)(hi2s->base+TOFF_I2S_RXFFR), 1);
	/* flush rx fifo*/

	i2s_set_rx_word_length(hi2s, word_length, channel_num);
	/* Word buf_len is RESOLUTION_32_BIT */

	i2s_master_configure(hi2s, word_select_size, NO_CLOCK_GATING, word_mode);
	/* word select size is 32 bits,no clock gating */

	i2s_set_rx_threshold(hi2s, trigger_level, channel_num);
	/* Interrupt trigger when FIFO level is 8 */

	sil_rew_mem((uint32_t *)(hi2s->base+TOFF_I2S_CHANNEL+channel_num*I2S_CHANNEL_WINDOW_SIZE+TOFF_I2SC_ROR));
	/* clear RX FIFO data overrun interrupt */
	sil_rew_mem((uint32_t *)(hi2s->base+TOFF_I2S_CHANNEL+channel_num*I2S_CHANNEL_WINDOW_SIZE+TOFF_I2SC_TOR));
	/* clear TX FIFO data overrun interrupt */

	i2s_recv_channel_enable(hi2s, channel_num, 1);
}

void
i2s_tx_channel_config(I2S_Handle_t *hi2s,
	i2s_channel_num_t channel_num,
	i2s_word_length_t word_length,
	i2s_word_select_cycles_t word_select_size,
	i2s_fifo_threshold_t trigger_level,
	i2s_work_mode_t word_mode)
{
	sil_wrw_mem((uint32_t *)(hi2s->base+TOFF_I2S_CHANNEL+channel_num*I2S_CHANNEL_WINDOW_SIZE+TOFF_I2SC_RER), 0);
	/* disable rx */

	i2s_transmit_channel_enable(hi2s, channel_num, 0);
	/* Transmit channel disable */

	sil_wrw_mem((uint32_t *)(hi2s->base+TOFF_I2S_TXFFR), 1);
	/* flush tx fifo */
	sil_wrw_mem((uint32_t *)(hi2s->base+TOFF_I2S_CHANNEL+channel_num*I2S_CHANNEL_WINDOW_SIZE+TOFF_I2SC_TFF), 1);
	/* flush individual fifo */

	i2s_set_tx_word_length(hi2s, word_length, channel_num);
	/* Word buf_len is RESOLUTION_16_BIT */

	i2s_master_configure(hi2s, word_select_size, NO_CLOCK_GATING, word_mode);
	/* word select size is 16 bits,gating after 16 bit */

	i2s_set_tx_threshold(hi2s, trigger_level, channel_num);
	/* Interrupt trigger when FIFO level is 8 */

	i2s_transmit_channel_enable(hi2s, channel_num, 1);
}

/*
 *  I2S�ǥХ����ν����
 *  parameter1 portid  �ݡ���ID
 *  parameter2 ii2s    I2S������ϥ�ɥ�ؤΥݥ���
 *  return             ���ｪλ����I2S�ϥ�ɥ�ؤΥݥ���
 */
I2S_Handle_t *
i2s_init(ID portid, I2S_Init_t *ii2s)
{
	I2S_Handle_t *hi2s;
	DMA_Handle_t *hdma;
	const I2S_PortControlBlock *pcb;
	uint32_t threshold = 0;
	uint8_t  no;

	if(portid < I2S1_PORTID || portid > NUM_I2SPORT)
		return NULL;

	uint32_t rx_channel_mask = ii2s->RxChannelMask;
	uint32_t tx_channel_mask = ii2s->TxChannelMask;
	if( (tx_channel_mask & rx_channel_mask) != 0)
		return NULL;

	no = INDEX_I2S(portid);
	hi2s = &i2sHandle[no];
	hi2s->i2sid = portid;
	hi2s->i2s_num = no;
	pcb = &i2s_pcb[no];
	hi2s->base  = pcb->base;
	memcpy(&hi2s->Init, ii2s, sizeof(I2S_Init_t));

	/*
	 *  ����å�����
	 */
	sil_orw_mem((uint32_t *)(TADR_SYSCTL_BASE+TOFF_SYSCTL_CLK_EN_CENT), SYSCTL_CLK_EN_CENT_APB0_CLK_EN);
	sil_orw_mem((uint32_t *)(TADR_SYSCTL_BASE+TOFF_SYSCTL_CLK_EN_PERI), pcb->clkenable);
	sil_orw_mem((uint32_t *)(TADR_SYSCTL_BASE+TOFF_SYSCTL_PERI_RESET), pcb->perireset);
	sil_dly_nse(10000);
	sil_andw_mem((uint32_t *)(TADR_SYSCTL_BASE+TOFF_SYSCTL_PERI_RESET), pcb->perireset);
	sil_modw_mem((uint32_t *)(pcb->thresholdctrl), pcb->thresholdmask, (DEFAULT_THRESHOLD << pcb->thresholdshift));
	/*96k:5,44k:12,24k:23,22k:25 16k:35 sampling*/
	/*sample rate*32bit*2 =75MHz/((N+1)*2) */
	i2s_set_enable(hi2s, 1);
	i2s_disable_block(hi2s, I2S_TRANSMITTER);
	i2s_disable_block(hi2s, I2S_RECEIVER);

	/*
	 *  �ԥ�ե��󥯥��������
	 */
	if(ii2s->MclkPin >= 0)
		fpioa_set_function(ii2s->MclkPin, (uint8_t)(FUNC_I2S0_MCLK + hi2s->i2s_num * 11));
	if(ii2s->SclkPin >= 0)
	{
		fpioa_set_function(ii2s->SclkPin, (uint8_t)(FUNC_I2S0_SCLK + hi2s->i2s_num * 11));
	}
	if(ii2s->WsPin >= 0)
	{
		fpioa_set_function(ii2s->WsPin, (uint8_t)(FUNC_I2S0_WS + hi2s->i2s_num * 11));
	}
	if(ii2s->InD0Pin >= 0)
		fpioa_set_function(ii2s->InD0Pin, (uint8_t)(FUNC_I2S0_IN_D0 + hi2s->i2s_num * 11));
	if(ii2s->InD1Pin >= 0)
		fpioa_set_function(ii2s->InD1Pin, (uint8_t)(FUNC_I2S0_IN_D1 + hi2s->i2s_num * 11));
	if(ii2s->InD2Pin >= 0)
		fpioa_set_function(ii2s->InD2Pin, (uint8_t)(FUNC_I2S0_IN_D2 + hi2s->i2s_num * 11));
	if(ii2s->InD3Pin >= 0)
		fpioa_set_function(ii2s->InD3Pin, (uint8_t)(FUNC_I2S0_IN_D3 + hi2s->i2s_num * 11));
	if(ii2s->OutD0Pin >= 0)
		fpioa_set_function(ii2s->OutD0Pin, (uint8_t)(FUNC_I2S0_OUT_D0 + hi2s->i2s_num * 11));
	if(ii2s->OutD1Pin >= 0)
		fpioa_set_function(ii2s->OutD1Pin, (uint8_t)(FUNC_I2S0_OUT_D1 + hi2s->i2s_num * 11));
	if(ii2s->OutD2Pin >= 0)
		fpioa_set_function(ii2s->OutD2Pin, (uint8_t)(FUNC_I2S0_OUT_D2 + hi2s->i2s_num * 11));
	if(ii2s->OutD3Pin >= 0)
		fpioa_set_function(ii2s->OutD3Pin, (uint8_t)(FUNC_I2S0_OUT_D3 + hi2s->i2s_num * 11));

	if (hi2s->Init.RxTxMode == I2S_TRANSMITTER || hi2s->Init.RxTxMode == I2S_BOTH)
	{
		for (int i=0; i<4; i++)
		{
			if ((tx_channel_mask & 0x3) == 0x3)
			{
				i2s_set_mask_interrupt(hi2s, I2S_CHANNEL_0 + i, 1, 1, 1, 1);
				i2s_transimit_enable(hi2s, I2S_CHANNEL_0 + i);
			}
			else
			{
				i2s_transmit_channel_enable(hi2s, I2S_CHANNEL_0 + i, 0);
			}
			tx_channel_mask >>= 2;
		}
		i2s_transmit_dma_enable(hi2s, 1);
	}

	if (hi2s->Init.RxTxMode == I2S_RECEIVER || hi2s->Init.RxTxMode == I2S_BOTH)
	{
		for (int i=0; i<4; i++)
		{
			if ((rx_channel_mask & 0x3) == 0x3)
			{
				i2s_set_mask_interrupt(hi2s, I2S_CHANNEL_0 + i, 1, 1, 1, 1);
				i2s_receive_enable(hi2s, I2S_CHANNEL_0 + i);
			}
			else
			{
				i2s_recv_channel_enable(hi2s, I2S_CHANNEL_0 + i, 0);
			}
			rx_channel_mask >>= 2;
		}
		/* Set expand_en when receive */
		i2s_set_sign_expand_en(hi2s, 1);
		i2s_receive_dma_enable(hi2s, 1);
	}
	hi2s->status = I2S_STATUS_READY;

	rx_channel_mask = ii2s->RxChannelMask;
	tx_channel_mask = ii2s->TxChannelMask;
	if (hi2s->Init.RxTxMode == I2S_TRANSMITTER || hi2s->Init.RxTxMode == I2S_BOTH)
	{
		for (int i=0; i<4; i++)
		{
			if ((tx_channel_mask & 0x3) == 0x3)
			{
				i2s_tx_channel_config(hi2s, I2S_CHANNEL_0 + i, hi2s->Init.word_length,
					hi2s->Init.word_select_size, hi2s->Init.trigger_level, hi2s->Init.word_mode);
			}
			tx_channel_mask >>= 2;
		}
	}

	if (hi2s->Init.RxTxMode == I2S_RECEIVER || hi2s->Init.RxTxMode == I2S_BOTH)
	{
		for (int i=0; i<4; i++)
		{
			if ((rx_channel_mask & 0x3) == 0x3)
			{
				i2s_rx_channel_config(hi2s, I2S_CHANNEL_0 + i, hi2s->Init.word_length,
					hi2s->Init.word_select_size, hi2s->Init.trigger_level, hi2s->Init.word_mode);
			}
			rx_channel_mask >>= 2;
		}
	}

	/*
	 *  �ϥ�ɥ�����
	 */
	hi2s->readcallback = NULL;
	hi2s->writecallback = NULL;
	hi2s->errorcallback = NULL;
	hi2s->hdmatx = NULL;
	hi2s->hdmarx = NULL;
	if(ii2s->TxDMAChannel >= 0){
		hdma = &i2s_dma_handle[no][0];
		hdma->chnum = ii2s->TxDMAChannel;
		if(ii2s->RxDMAChannel >= 0)
			hdma->xfercallback      = NULL;
		else
			hdma->xfercallback      = i2s_dma_comp;
		hdma->errorcallback     = NULL;
//		hdma->errorcallback     = i2s_dma_error;
		hdma->Init.Request      = DMA_SELECT_I2S0_TX_REQ + hi2s->i2s_num * 2;	/* DMA���� */
		hdma->Init.Direction    = DMA_MEMORY_TO_PERIPH;	/* DMAž������ */
		hdma->Init.SrcMultBlock = DMAC_MULTBLOCK_CONT;	/* �������ޥ���֥�å������� */
		hdma->Init.DrcMultBlock = DMAC_MULTBLOCK_CONT;	/* �ǥ��ƥ��͡������ޥ���֥�å������� */
		hdma->Init.SrcHandShake = DMAC_HS_SOFTWARE;	/* �������ϥ�ɥ������� */
		hdma->Init.DrcHandShake = DMAC_HS_HARDWARE;	/* �ǥ��ƥ��͡������ϥ�ɥ������� */
		hdma->Init.SrcHwhsPol   = DMAC_HWHS_POLARITY_LOW;	/* �������ϡ��ɥ������ϥ�ɥ����������� */
		hdma->Init.DrcHwhsPol   = DMAC_HWHS_POLARITY_LOW;	/* �ǥ��ƥ��͡������ϡ��ɥ������ϥ�ɥ����������� */
		hdma->Init.Priority     = 4;	/* ͥ���� */
		hdma->Init.SrcMaster    = DMAC_MASTER1;	/* �������ޥ��������� */
		hdma->Init.DstMaster    = DMAC_MASTER2;	/* �ǥ��ƥ��͡������ޥ��������� */
		hdma->Init.SrcInc       = DMAC_ADDR_INCREMENT;	/* ���������󥯥�������� */
		hdma->Init.DstInc       = DMAC_ADDR_NOCHANGE;	/* �ǥ��ƥ��͡�����󥤥󥯥�������� */
		hdma->Init.SrcTransWidth = DMAC_TRANS_WIDTH_32;	/* ������ž���� */
		hdma->Init.DstTransWidth = DMAC_TRANS_WIDTH_32;	/* �ǥ��ƥ��͡������ž���� */
		hdma->Init.SrcBurstSize = DMAC_MSIZE_1;	/* �������С����ȥ����� */
		hdma->Init.DstBurstSize = DMAC_MSIZE_1;	/* �ǥ��ƥ��͡������С����ȥ����� */
		hdma->Init.IocBlkTrans  = 0;	/* IOC�֥�å�ž�� */
		hdma->localdata         = (void *)hi2s;
		dma_init(hdma);
		hi2s->hdmatx = hdma;
	}
	if(ii2s->RxDMAChannel >= 0){
		hdma = &i2s_dma_handle[no][1];
		hdma->chnum = ii2s->RxDMAChannel;
		hdma->xfercallback      = i2s_dma_comp;
		hdma->errorcallback     = NULL;
//		hdma->errorcallback     = i2s_dma_error;
		hdma->Init.Request      = DMA_SELECT_I2S0_RX_REQ + hi2s->i2s_num * 2;	/* DMA���� */
		hdma->Init.Direction    = DMA_PERIPH_TO_MEMORY;	/* DMAž������ */
		hdma->Init.SrcMultBlock = DMAC_MULTBLOCK_CONT;	/* �������ޥ���֥�å������� */
		hdma->Init.DrcMultBlock = DMAC_MULTBLOCK_CONT;	/* �ǥ��ƥ��͡������ޥ���֥�å������� */
		hdma->Init.SrcHandShake = DMAC_HS_HARDWARE;	/* �������ϥ�ɥ������� */
		hdma->Init.DrcHandShake = DMAC_HS_SOFTWARE;	/* �ǥ��ƥ��͡������ϥ�ɥ������� */
		hdma->Init.SrcHwhsPol   = DMAC_HWHS_POLARITY_LOW;	/* �������ϡ��ɥ������ϥ�ɥ����������� */
		hdma->Init.DrcHwhsPol   = DMAC_HWHS_POLARITY_LOW;	/* �ǥ��ƥ��͡������ϡ��ɥ������ϥ�ɥ����������� */
		hdma->Init.Priority     = 4;	/* ͥ���� */
		hdma->Init.SrcMaster    = DMAC_MASTER1;	/* �������ޥ��������� */
		hdma->Init.DstMaster    = DMAC_MASTER2;	/* �ǥ��ƥ��͡������ޥ��������� */
		hdma->Init.SrcInc       = DMAC_ADDR_NOCHANGE;	/* ���������󥯥�������� */
		hdma->Init.DstInc       = DMAC_ADDR_INCREMENT;	/* �ǥ��ƥ��͡�����󥤥󥯥�������� */
		hdma->Init.SrcTransWidth = DMAC_TRANS_WIDTH_32;	/* ������ž���� */
		hdma->Init.DstTransWidth = DMAC_TRANS_WIDTH_32;	/* �ǥ��ƥ��͡������ž���� */
		hdma->Init.SrcBurstSize = DMAC_MSIZE_1;	/* �������С����ȥ����� */
		hdma->Init.DstBurstSize = DMAC_MSIZE_1;	/* �ǥ��ƥ��͡������С����ȥ����� */
		hdma->Init.IocBlkTrans  = 0;	/* IOC�֥�å�ž�� */
		hdma->localdata         = (void *)hi2s;
		dma_init(hdma);
		hi2s->hdmarx = hdma;
	}

	uint32_t freq = i2s_set_sample_rate(hi2s, ii2s->sample_rate);
	syslog_1(LOG_NOTICE, "actual sample rate %u", freq);

	return hi2s;
}

/*
 *  I2S�ǥХ�����̵����
 *  parameter1 hi2s  I2S�ϥ�ɥ�ؤΥݥ���
 *  return           ���ｪλ����E_OK
 */
ER
i2s_deinit(I2S_Handle_t *hi2s)
{
	if(hi2s == NULL)
		return E_PAR;
	i2s_set_enable(hi2s, 0);
	if(hi2s->hdmatx != NULL){
		dma_deinit(hi2s->hdmatx);
		hi2s->hdmatx = NULL;
	}
	if(hi2s->hdmarx != NULL){
		dma_deinit(hi2s->hdmarx);
		hi2s->hdmarx = NULL;
	}
	hi2s->status = I2S_STATUS_RESET;
	sil_andw_mem((uint32_t *)(TADR_SYSCTL_BASE+TOFF_SYSCTL_CLK_EN_PERI), (SYSCTL_CLK_EN_PERI_I2S0_CLK_EN<<hi2s->i2s_num));
	return E_OK;
}

/*
 *  ����ץ�󥰥졼������
 *  parameter1 hi2s         I2S�ϥ�ɥ�ؤΥݥ���
 *  parameter2 sample_rate  ����ץ�󥰥졼��
 *  return                  ����ץ�󥰥졼��
 */
uint32_t
i2s_set_sample_rate(I2S_Handle_t *hi2s, uint32_t sample_rate)
{
	uint32_t pll2_clock = 0;
	pll2_clock = sysctl_pll_get_freq(SYSCTL_PLL2);

	uint32_t clk_word_size = (sil_rew_mem((uint32_t *)(hi2s->base+TOFF_I2S_CCR)) & I2S_CCR_CLK_WORD_SIZE) >> 3;
	/* 0x0 for 16sclk cycles, 0x1 for 24 sclk cycles 0x2 for 32 sclk */
	uint32_t v_clk_word_size = (clk_word_size + 2) * 8;
	uint32_t threshold = round(pll2_clock / (sample_rate * 2.0 * v_clk_word_size * 2.0) - 1);
	const I2S_PortControlBlock *pcb;
	pcb = &i2s_pcb[hi2s->i2s_num];
	sil_modw_mem((uint32_t *)(pcb->thresholdctrl), pcb->thresholdmask, (threshold << (pcb->thresholdshift)));
	return sysctl_clock_get_freq(SYSCTL_CLOCK_I2S0 + hi2s->i2s_num);
}
