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
 *  @(#) $Id: spi.c 698 2019-12-10 17:41:19Z roi $
 */
/*
 * 
 *  SPI�ɥ饤�дؿ���
 *
 */
#include "kernel_impl.h"
#include <t_syslog.h>
#include <t_stdlib.h>
#include <string.h>
#include <sil.h>
#include <target_syssvc.h>
#include "device.h"
#include "spi.h"

Inline uint64_t
sil_rel_mem(const uint64_t *mem)
{
	uint64_t	data;

	data = *((const volatile uint64_t *) mem);
	return(data);
}

#define sil_orw_mem(a, b)		sil_wrw_mem((a), sil_rew_mem(a) | (b))
#define sil_andw_mem(a, b)		sil_wrw_mem((a), sil_rew_mem(a) & ~(b))
#define sil_modw_mem(a, b, c)	sil_wrw_mem((a), (sil_rew_mem(a) & (~b)) | (c))

/*
 *  SPIO�ݡ���ID��������֥�å�����Ф�����Υޥ���
 */
#define INDEX_SPI(spiid)        ((uint_t)((spiid) - 1))

#define get_framewidth(l)       (((l)+7)/8)

#define SPI_SSIENR_DISABLE      0x00000000
#define SPI_SSIENR_ENABLE       0x00000001

#define SPI_DMACR_RXENABLE      0x00000001
#define SPI_DMACR_TXENABLE      0x00000002

/*
 *  SPI�ϡ��ɥ��������깽¤��
 */

typedef struct _SPI_PortControlBlock{
	unsigned long         base;
	int16_t               func_data;
	int16_t               func_ss;
	int16_t               func_arb;
	int16_t               func_sclk;
} SPI_PortControlBlock;

static const SPI_PortControlBlock spi_pcb[NUM_SPIPORT] = {
	{TADR_SPI0_BASE, FUNC_SPI0_D0,      FUNC_SPI0_SS0,     FUNC_SPI0_ARB, FUNC_SPI0_SCLK     },
	{TADR_SPI1_BASE, FUNC_SPI1_D0,      FUNC_SPI1_SS0,     FUNC_SPI1_ARB, FUNC_SPI1_SCLK     },
	{TADR_SPIS_BASE, FUNC_SPI_SLAVE_D0, FUNC_SPI_SLAVE_SS, -1,            FUNC_SPI_SLAVE_SCLK},
	{TADR_SPI2_BASE, -1,                -1,                -1,            -1                 }
};

SPI_Handle_t SpiHandle[NUM_SPIPORT];
DMA_Handle_t spi_dma_handle[NUM_SPIPORT][2];


/*
 *  ž���⡼������
 */
static ER
spi_set_tmod(SPI_Handle_t *hspi, uint32_t tmod)
{
	SPI_Init_t *init;
    uint32_t inst_l = 4;
    uint32_t addr_l;

	init = &hspi->Init;
    switch (init->InstLength){
	case 0:
		inst_l = 0;
		break;
	case 4:
		inst_l = 1;
		break;
	case 8:
		inst_l = 2;
		break;
	case 16:
		inst_l = 3;
		break;
	default:
		break;
    }
	if(inst_l == 4)
		return E_PAR;

	addr_l = init->AddrLength / 4;
	sil_wrw_mem((uint32_t *)(hspi->base+TOFF_SPI_CTRLR0), (init->WorkMode << hspi->work_mode_offset) | \
		(init->FrameFormat << hspi->frf_offset) | ((init->DataSize - 1) << hspi->dfs_offset));
//	sil_wrw_mem((uint32_t *)(hspi->base+TOFF_SPI_SPI_CTRLR0),
//		((init->WaitCycles << 11) | (inst_l << 8) | (addr_l << 2) | init->IATransMode));
	sil_wrw_mem((uint32_t *)(hspi->base+TOFF_SPI_SPI_CTRLR0), 0x00000000);
	sil_modw_mem((uint32_t *)(hspi->base+TOFF_SPI_CTRLR0), (3 << hspi->tmod_offset), (tmod << hspi->tmod_offset));
	return E_OK;
}

/*
 *  DMA������Хå��ؿ�
 */
static void
spi_dma_comp(DMA_Handle_t *hdma)
{
	SPI_Handle_t *hspi = (SPI_Handle_t *)hdma->localdata;
	if(hspi != NULL && hspi->Init.semdmaid != 0){
		isig_sem(hspi->Init.semdmaid);
	}
}

/*
 *  DMAž�����ϴؿ�
 */
DMA_Handle_t *
spi_dmac_set_single_mode(SPI_Handle_t *hspi, uint8_t rtx,
						  int8_t  ss_no, const void *src, void *dest,
						  uint8_t src_inc, uint8_t dest_inc,
                          uint8_t dmac_burst_size, uint8_t dmac_trans_width,
                          size_t block_size)
{
	DMA_Handle_t *hdma;
    int mem_type_src, mem_type_dest;
    uint8_t flow_control;

	if(rtx == 0){
		hdma = hspi->hdmatx;
        flow_control = DMA_MEMORY_TO_PERIPH;
		mem_type_src = 1;
		mem_type_dest = 0;
	}
	else{
		hdma = hspi->hdmarx;
        flow_control = DMA_PERIPH_TO_MEMORY;
		mem_type_src = 0;
		mem_type_dest = 1;
	}
	if(ss_no < 0)
		ss_no = 0;

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
	sil_wrw_mem((uint32_t *)(hspi->base+TOFF_SPI_SER), (1 << ss_no));
	dma_start(hdma, (uintptr_t)src, (uintptr_t)dest, block_size);
	return hdma;
}

/*
 *  SPI-DMAž����λ�Ԥ�
 */
ER
spi_dmac_wait_done(DMA_Handle_t * hdma)
{
	SPI_Handle_t *hspi = (SPI_Handle_t *)hdma->localdata;
	ER ercd = E_OK;
	int tick = DMA_TRS_TIMEOUT;

	while((hdma->status == DMA_STATUS_BUSY) && tick > 0){
		if(hspi != NULL && hspi->Init.semdmaid != 0){
	 		ercd = twai_sem(hspi->Init.semdmaid, 5);
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

/*
 *  SPI����ž����λ�Ԥ�
 */
ER
spi_inwait(SPI_Handle_t *hspi, uint32_t timeout)
{
	ER ercd = E_OK;
	int tick = timeout;

	sil_wrw_mem((uint32_t *)(hspi->base+TOFF_SPI_IMR), 0x0011);
    while((sil_rew_mem((uint32_t *)(hspi->base+TOFF_SPI_SR)) & 0x05) != 0x04 && tick > 0){
		if(hspi->Init.semid != 0)
			twai_sem(hspi->Init.semid, 5);
		else
			dly_tsk(1);
		tick--;
	}
	sil_wrw_mem((uint32_t *)(hspi->base+TOFF_SPI_SER), 0x00000000);
	sil_wrw_mem((uint32_t *)(hspi->base+TOFF_SPI_SSIENR), SPI_SSIENR_DISABLE);

	if(hspi->ErrorCode != 0)
		ercd = E_OBJ;
	else if(tick == 0)
		ercd = E_TMOUT;
	hspi->TxXferCount = 0;
	hspi->RxXferCount = 0;
	return ercd;
}


/*
 *  SPI�������
 *  parameter1  port: SPI�ݡ����ֹ�
 *  parameter2  spii: SPI������깽¤�ΤؤΥݥ���
 *  return SPI�ϥ�ɥ�ؤΥݥ��󥿡�NULL�ǥ��顼
 */
SPI_Handle_t *
spi_init(ID port, const SPI_Init_t *init)
{
	SPI_Handle_t *hspi;
	DMA_Handle_t *hdma;
	const SPI_PortControlBlock *spcb;
	unsigned long base;
	uint8_t  spi_num;
	uint32_t spi_baudr, clk_th1, threshold;
    uint8_t  dfs_offset, frf_offset, work_mode_offset, tmod_offset;
	uint32_t dsize_err = 0;
    uint32_t inst_l = 4;
    uint32_t addr_l;

	if(port < SPI1_PORTID || port > NUM_SPIPORT)
		return NULL;
	spi_num = INDEX_SPI(port);
	if(init == NULL)
		return NULL;
	if(init->DataSize < 4 && init->DataSize > 32)
		return NULL;
	if(init->AddrLength % 4 != 0 && init->AddrLength > 60)
		return NULL;

	/*
	 *  ����å�����
	 */
    if(spi_num == 3){
		sil_orw_mem((uint32_t *)(TADR_SYSCTL_BASE+TOFF_CLK_SEL0), SYSCTL_CLK_SEL0_SPI3_CLK_SEL);
	}
	if(spi_num < 2)
		sil_orw_mem((uint32_t *)(TADR_SYSCTL_BASE+TOFF_SYSCTL_CLK_EN_CENT), SYSCTL_CLK_EN_CENT_APB2_CLK_EN);
	else if(spi_num == 2)
		sil_orw_mem((uint32_t *)(TADR_SYSCTL_BASE+TOFF_SYSCTL_CLK_EN_CENT), SYSCTL_CLK_EN_CENT_APB0_CLK_EN);
	sil_orw_mem((uint32_t *)(TADR_SYSCTL_BASE+TOFF_SYSCTL_CLK_EN_PERI), (SYSCTL_CLK_EN_PERI_SPI0_CLK_EN<<spi_num));
	sil_andw_mem((uint32_t *)(TADR_SYSCTL_BASE+TOFF_SYSCTL_CLK_TH1), 0xFF << (spi_num*8));

	/*
	 *  �ԥ�ե��󥯥��������
	 */
	spcb = &spi_pcb[spi_num];
	if(spcb->func_ss >= 0 && init->SsPin >= 0)
		fpioa_set_function(init->SsPin, (uint8_t)(spcb->func_ss + init->SsNo));
	if(spcb->func_sclk >= 0 && init->SclkPin >= 0)
		fpioa_set_function(init->SclkPin, (uint8_t)(spcb->func_sclk));
	if(spcb->func_data >= 0){
		if(init->MosiPin >= 0)
			fpioa_set_function(init->MosiPin, (uint8_t)(spcb->func_data));
		if(init->MisoPin >= 0)
			fpioa_set_function(init->MisoPin, (uint8_t)(spcb->func_data+1));

	}
	hspi = &SpiHandle[spi_num];
	base = spcb->base;

    switch(spi_num){
	case 0:
	case 1:
		dfs_offset = 16;
		frf_offset = 21;
		work_mode_offset = 6;
		tmod_offset = 8;
		break;
	case 3:
	default:
		dfs_offset = 0;
		frf_offset = 22;
		work_mode_offset = 8;
		tmod_offset = 10;
		break;
	}

	switch(init->FrameFormat){
	case SPI_FF_DUAL:
		if(init->DataSize % 2 != 0)
			dsize_err = 1;
		break;
	case SPI_FF_QUAD:
		if(init->DataSize % 4 != 0)
			dsize_err = 1;
		break;
	case SPI_FF_OCTAL:
		if(init->DataSize % 8 != 0)
			dsize_err = 1;
		break;
	default:
		break;
	}

	switch(init->InstLength){
	case 0:
		inst_l = 0;
		break;
	case 4:
		inst_l = 1;
		break;
	case 8:
		inst_l = 2;
		break;
	case 16:
		inst_l = 3;
		break;
	default:
		break;
	}
	if(inst_l == 4 || dsize_err){
		syslog_0(LOG_ERROR, "Invalid instruction length");
		return NULL;
	}
	addr_l = init->AddrLength / 4;

	memcpy(&hspi->Init, init, sizeof(SPI_Init_t));

    if(sil_rew_mem((uint32_t *)(base+TOFF_SPI_BAUDR)) == 0)
		sil_wrw_mem((uint32_t *)(base+TOFF_SPI_BAUDR), 0x14);
	/*
	 *  ������Ե���
	 */
	sil_wrw_mem((uint32_t *)(base+TOFF_SPI_IMR), 0x00000000);

	sil_wrw_mem((uint32_t *)(base+TOFF_SPI_DMACR), 0x00000000);
	sil_wrw_mem((uint32_t *)(base+TOFF_SPI_DMATDLR), 0x00000010);
	sil_wrw_mem((uint32_t *)(base+TOFF_SPI_DMARDLR), 0x00000000);
	sil_wrw_mem((uint32_t *)(base+TOFF_SPI_SER), 0x00000000);
	sil_wrw_mem((uint32_t *)(base+TOFF_SPI_SSIENR), SPI_SSIENR_DISABLE);
	sil_wrw_mem((uint32_t *)(base+TOFF_SPI_CTRLR0), (init->WorkMode << work_mode_offset) | \
		(init->FrameFormat << frf_offset) | ((init->DataSize - 1) << dfs_offset));
//	sil_wrw_mem((uint32_t *)(base+TOFF_SPI_SPI_CTRLR0),
//		((init->WaitCycles << 11) | (inst_l << 8) | (addr_l << 2) | init->IATransMode));
	sil_wrw_mem((uint32_t *)(base+TOFF_SPI_SPI_CTRLR0), 0x00000000);
	sil_wrw_mem((uint32_t *)(base+TOFF_SPI_ENDIAN), init->SignBit);

	/*
	 *  ž������å�����
	 */
	clk_th1 = sil_rew_mem((uint32_t *)(TADR_SYSCTL_BASE+TOFF_SYSCTL_CLK_TH1));
	threshold = (clk_th1 >> (spi_num * 8)) & 0xff;
	spi_baudr = (get_pll_clock(0) / ((threshold + 1) * 2)) / init->Prescaler;


	if(spi_baudr < 2 ){
		spi_baudr = 2;
	}
	else if(spi_baudr > 65534){
		spi_baudr = 65534;
	}
	sil_wrw_mem((uint32_t *)(base+TOFF_SPI_BAUDR), spi_baudr);

	/*
	 *  �ϥ�ɥ�����
	 */
	hspi->base = base;
	hspi->spi_num = spi_num;
	hspi->dfs_offset  = dfs_offset;
	hspi->frf_offset  = frf_offset;
	hspi->work_mode_offset = work_mode_offset;
	hspi->tmod_offset = tmod_offset;
	hspi->hdmatx = NULL;
	hspi->hdmarx = NULL;
	if(init->TxDMAChannel >= 0){
		hdma = &spi_dma_handle[init->TxDMAChannel][0];
		hdma->chnum = init->TxDMAChannel;
		if(init->RxDMAChannel >= 0)
			hdma->xfercallback      = NULL;
		else
			hdma->xfercallback      = spi_dma_comp;
		hdma->errorcallback     = NULL;
		hdma->Init.Request      = DMA_SELECT_SSI0_TX_REQ + spi_num * 2;	/* DMA���� */
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
		hdma->Init.SrcBurstSize = DMAC_MSIZE_4;	/* �������С����ȥ����� */
		hdma->Init.DstBurstSize = DMAC_MSIZE_4;	/* �ǥ��ƥ��͡������С����ȥ����� */
		hdma->Init.IocBlkTrans  = 0;	/* IOC�֥�å�ž�� */
		hdma->localdata         = (void *)hspi;
		dma_init(hdma);
		hspi->hdmatx = hdma;
	}
	if(init->RxDMAChannel >= 0){
		hdma = &spi_dma_handle[init->RxDMAChannel][1];
		hdma->chnum = init->RxDMAChannel;
		hdma->xfercallback      = spi_dma_comp;
		hdma->errorcallback     = NULL;
		hdma->Init.Request      = DMA_SELECT_SSI0_RX_REQ + spi_num * 2;	/* DMA���� */
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
		hdma->Init.SrcBurstSize = DMAC_MSIZE_4;	/* �������С����ȥ����� */
		hdma->Init.DstBurstSize = DMAC_MSIZE_4;	/* �ǥ��ƥ��͡������С����ȥ����� */
		hdma->Init.IocBlkTrans  = 0;	/* IOC�֥�å�ž�� */
		hdma->localdata         = (void *)hspi;
		dma_init(hdma);
		hspi->hdmarx = hdma;
	}
	hspi->status = SPI_STATUS_READY;
	hspi->xmode  = 0;
	return hspi;
}

/*
 *  SPI��λ����
 *  parameter1  hspi: SPI�ϥ�ɥ�ؤΥݥ���
 *  return ER������
 */
ER
spi_deinit(SPI_Handle_t *hspi)
{
	if(hspi == NULL)
		return E_PAR;

	if(hspi->hdmatx != NULL){
		dma_deinit(hspi->hdmatx);
		hspi->hdmatx = NULL;
	}
	if(hspi->hdmarx != NULL){
		dma_deinit(hspi->hdmarx);
		hspi->hdmarx = NULL;
	}
	sil_wrw_mem((uint32_t *)(hspi->base+TOFF_SPI_SSIENR), SPI_SSIENR_DISABLE);

	hspi->ErrorCode = SPI_ERROR_NONE;
	hspi->status = SPI_STATUS_RESET;
	return E_OK;
}

/*
 *  �ݡ���󥰥ǡ�������
 */
static void
spi_send_data_normal2(SPI_Handle_t *hspi, int8_t ss_no, const uint8_t *tx_buff, size_t tx_len)
{
	size_t index, fifo_len;
	uint8_t frame_width = get_framewidth(hspi->Init.DataSize);
	uint8_t v_misalign_flag = 0;
	uint32_t v_send_data;
	uint32_t i = 0;

	if((uintptr_t)tx_buff % frame_width){
		v_misalign_flag = 1;
	}
	if(ss_no < 0)
		ss_no = 0;

	sil_wrw_mem((uint32_t *)(hspi->base+TOFF_SPI_SSIENR), SPI_SSIENR_ENABLE);
	sil_wrw_mem((uint32_t *)(hspi->base+TOFF_SPI_SER), (1 << ss_no));
	while(tx_len > 0){
		fifo_len = 32 - sil_rew_mem((uint32_t *)(hspi->base+TOFF_SPI_TXFLR));
		fifo_len = fifo_len < tx_len ? fifo_len : tx_len;
		switch(frame_width){
		case SPI_TRANS_INT:
			fifo_len = fifo_len / 4 * 4;
			if(v_misalign_flag){
				for(index = 0; index < fifo_len; index +=4){
					memcpy(&v_send_data, tx_buff + i , 4);
					sil_wrw_mem((uint32_t *)(hspi->base+TOFF_SPI_DR), v_send_data);
					i += 4;
				}
			}
			else{
				for(index = 0; index < fifo_len / 4; index++)
					sil_wrw_mem((uint32_t *)(hspi->base+TOFF_SPI_DR), ((uint32_t *)tx_buff)[i++]);
			}
			break;
		case SPI_TRANS_SHORT:
			fifo_len = fifo_len / 2 * 2;
			if(v_misalign_flag){
				for(index = 0; index < fifo_len; index +=2){
					memcpy(&v_send_data, tx_buff + i, 2);
					sil_wrw_mem((uint32_t *)(hspi->base+TOFF_SPI_DR), v_send_data);
					i += 2;
				}
			}
			else{
				for(index = 0; index < fifo_len / 2; index++)
					sil_wrw_mem((uint32_t *)(hspi->base+TOFF_SPI_DR), ((uint16_t *)tx_buff)[i++]);
			}
			break;
		default:
			for(index = 0; index < fifo_len; index++)
				sil_wrw_mem((uint32_t *)(hspi->base+TOFF_SPI_DR), tx_buff[i++]);
			break;
 		}
		tx_len -= fifo_len;
	}
}

/*
 *  SPI�����¹Դؿ�
 *  parameter1  hspi: SPI�ϥ�ɥ�ؤΥݥ���
 *  parameter2  ss_no: SS�ֹ�
 *  parameter3  pdata: �����Хåե��ؤΥݥ���
 *  parameter4  length: ����������
 *  return ER������
 */
ER
spi_core_transmit(SPI_Handle_t *hspi, int8_t ss_no, uint8_t *pdata, uint16_t length)
{
	DMA_Handle_t *hdma;
	ER ercd = E_OK;

	if(hspi == NULL)
		return E_PAR;

	if(hspi->Init.semlock != 0)
		wai_sem(hspi->Init.semlock);
	hspi->xmode = SPI_XMODE_TX;
    spi_set_tmod(hspi, SPI_TMOD_TRANS);
	if(hspi->hdmatx != NULL){
		sil_wrw_mem((uint32_t *)(hspi->base+TOFF_SPI_DMACR), SPI_DMACR_TXENABLE);
		sil_wrw_mem((uint32_t *)(hspi->base+TOFF_SPI_SSIENR), SPI_SSIENR_ENABLE);
		hdma = spi_dmac_set_single_mode(hspi, 0, ss_no, (const void *)pdata, 
							(void *)(hspi->base+TOFF_SPI_DR), DMAC_ADDR_INCREMENT, DMAC_ADDR_NOCHANGE,
							DMAC_MSIZE_4, DMAC_TRANS_WIDTH_32, length);
		spi_dmac_wait_done(hdma);
	}
	else{
		spi_send_data_normal2(hspi, ss_no, (const void *)pdata, length);
	}

#if SPI_WAIT_TIME != 0
	ercd = spi_inwait(hspi, SPI_WAIT_TIME * length);

	if(hspi->Init.semlock != 0)
		sig_sem(hspi->Init.semlock);
#endif
	return ercd;
}

/*
 *  SPI�ե��������¹Դؿ�
 *  parameter1  hspi: SPI�ϥ�ɥ�ؤΥݥ���
 *  parameter2  ss_no: SS�ֹ�
 *  parameter3  pdata: �����Хåե��ؤΥݥ���
 *  parameter4  length: ����������
 *  return ER������
 */
ER
spi_core_transmit_fill(SPI_Handle_t *hspi, int8_t ss_no, const uint32_t *tx_buff, size_t tx_len)
{
	DMA_Handle_t *hdmatx;
	ER ercd = E_OK;

	if(hspi == NULL)
		return E_PAR;

	if(hspi->Init.semlock != 0)
		wai_sem(hspi->Init.semlock);

	hspi->xmode = SPI_XMODE_TX;
	spi_set_tmod(hspi, SPI_TMOD_TRANS);
	sil_wrw_mem((uint32_t *)(hspi->base+TOFF_SPI_DMACR), SPI_DMACR_TXENABLE);
	sil_wrw_mem((uint32_t *)(hspi->base+TOFF_SPI_SSIENR), SPI_SSIENR_ENABLE);

    hdmatx = spi_dmac_set_single_mode(hspi, 0, ss_no, tx_buff, (void *)(hspi->base+TOFF_SPI_DR), DMAC_ADDR_NOCHANGE, DMAC_ADDR_NOCHANGE,
                                DMAC_MSIZE_1, DMAC_TRANS_WIDTH_32, tx_len);
    spi_dmac_wait_done(hdmatx);

#if SPI_WAIT_TIME != 0
	ercd = spi_inwait(hspi, SPI_WAIT_TIME * tx_len);

	if(hspi->Init.semlock != 0)
		sig_sem(hspi->Init.semlock);
#endif
	return ercd;
}

/*
 *  SPI�����¹Դؿ�
 *  parameter1  hspi: SPI�ϥ�ɥ�ؤΥݥ���
 *  parameter2  ss_no: SS�ֹ�
 *  parameter3  pdata: �����Хåե��ؤΥݥ���
 *  parameter4  length: ����������
 *  return ER������
 */
ER
spi_core_receive(SPI_Handle_t *hspi, int8_t ss_no, void *rx_buff, size_t rx_len)
{
	DMA_Handle_t * hdmarx;
	ER ercd = E_OK;

	if(hspi == NULL || hspi->spi_num == 2)
		return E_PAR;

	if(hspi->Init.semlock != 0)
		wai_sem(hspi->Init.semlock);

	hspi->xmode = SPI_XMODE_RX;
	spi_set_tmod(hspi, SPI_TMOD_RECV);

	sil_wrw_mem((uint32_t *)(hspi->base+TOFF_SPI_CTRLR1), (rx_len - 1));
	sil_wrw_mem((uint32_t *)(hspi->base+TOFF_SPI_DMACR), SPI_DMACR_RXENABLE);
	sil_wrw_mem((uint32_t *)(hspi->base+TOFF_SPI_SSIENR), SPI_SSIENR_ENABLE);

    hdmarx = spi_dmac_set_single_mode(hspi, 1, ss_no, (void *)(hspi->base+TOFF_SPI_DR), rx_buff, DMAC_ADDR_NOCHANGE, DMAC_ADDR_INCREMENT,
                           DMAC_MSIZE_1, DMAC_TRANS_WIDTH_32, rx_len);
    if(hspi->Init.FrameFormat == SPI_FF_STANDARD)
		sil_wrw_mem((uint32_t *)(hspi->base+TOFF_SPI_DR), 0xFFFFFFFF);
    spi_dmac_wait_done(hdmarx);

	sil_wrw_mem((uint32_t *)(hspi->base+TOFF_SPI_SER), 0x00000000);
	sil_wrw_mem((uint32_t *)(hspi->base+TOFF_SPI_SSIENR), SPI_SSIENR_DISABLE);

#if SPI_WAIT_TIME != 0
	ercd = spi_inwait(hspi, SPI_WAIT_TIME * rx_len);

	if(hspi->Init.semlock != 0)
		sig_sem(hspi->Init.semlock);
#endif
	return ercd;
}

/*
 *  SPI�������¹Դؿ�
 *  parameter1  hspi: SPI�ϥ�ɥ�ؤΥݥ���
 *  parameter2  ss_no: SS�ֹ�
 *  parameter3  ptxdata: �����Хåե��ؤΥݥ���
 *  parameter4  prxdata: �����Хåե��ؤΥݥ���
 *  parameter5  length: ������������
 *  return ER������
 */
ER
spi_core_transrecv(SPI_Handle_t *hspi, int8_t ss_no, const uint8_t *tx_buf, uint8_t *rx_buf, size_t len)
{
	DMA_Handle_t * hdmarx, *hdmatx;
    uint8_t frame_width = get_framewidth(hspi->Init.DataSize);
    size_t v_len = len / frame_width;
	ER ercd = E_OK;

	if(hspi == NULL)
		return E_PAR;

	if(hspi->Init.semlock != 0)
		wai_sem(hspi->Init.semlock);

	hspi->xmode = SPI_XMODE_TXRX;
    spi_set_tmod(hspi, SPI_TMOD_TRANS_RECV);


	if(hspi->hdmatx != NULL){
		sil_wrw_mem((uint32_t *)(hspi->base+TOFF_SPI_DMACR), (SPI_DMACR_TXENABLE | SPI_DMACR_RXENABLE));
		sil_wrw_mem((uint32_t *)(hspi->base+TOFF_SPI_SSIENR), SPI_SSIENR_ENABLE);

	    hdmarx = spi_dmac_set_single_mode(hspi, 1, ss_no, (void *)(hspi->base+TOFF_SPI_DR), rx_buf, DMAC_ADDR_NOCHANGE, DMAC_ADDR_INCREMENT,
                           DMAC_MSIZE_1, DMAC_TRANS_WIDTH_32, v_len);
	    hdmatx = spi_dmac_set_single_mode(hspi, 0, -1, tx_buf, (void *)(hspi->base+TOFF_SPI_DR), DMAC_ADDR_INCREMENT, DMAC_ADDR_NOCHANGE,
                           DMAC_MSIZE_4, DMAC_TRANS_WIDTH_32, v_len);

	    spi_dmac_wait_done(hdmatx);
	}
	else{
		sil_wrw_mem((uint32_t *)(hspi->base+TOFF_SPI_DMACR), SPI_DMACR_RXENABLE);
		sil_wrw_mem((uint32_t *)(hspi->base+TOFF_SPI_SSIENR), SPI_SSIENR_ENABLE);

	    hdmarx = spi_dmac_set_single_mode(hspi, 1, ss_no, (void *)(hspi->base+TOFF_SPI_DR), rx_buf, DMAC_ADDR_NOCHANGE, DMAC_ADDR_INCREMENT,
                           DMAC_MSIZE_1, DMAC_TRANS_WIDTH_32, v_len);
		spi_send_data_normal2(hspi, -1, (const void *)tx_buf, len);
	}
	spi_dmac_wait_done(hdmarx);

	sil_wrw_mem((uint32_t *)(hspi->base+TOFF_SPI_SER), 0x00000000);
	sil_wrw_mem((uint32_t *)(hspi->base+TOFF_SPI_SSIENR), SPI_SSIENR_DISABLE);

#if SPI_WAIT_TIME != 0
	ercd = spi_inwait(hspi, SPI_WAIT_TIME * len);

	if(hspi->Init.semlock != 0)
		sig_sem(hspi->Init.semlock);
#endif
	return ercd;
}

/*
 *  SPIž����λ�Ԥ�
 */
ER
spi_wait(SPI_Handle_t *hspi, uint32_t timeout)
{
	ER ercd = E_OK;

#if SPI_WAIT_TIME == 0
	if(hspi == NULL)
		return E_PAR;
	ercd = spi_inwait(hspi, timeout);
	if(hspi->Init.semlock != 0)
		sig_sem(hspi->Init.semlock);
#endif
	return ercd;
}


/*
 *  SPI����ߥ����ӥ��롼����
 */
void
spi_handler(SPI_Handle_t *hspi)
{
	volatile uint32_t imr, isr, tmp;

	imr = sil_rew_mem((uint32_t *)(hspi->base+TOFF_SPI_IMR));
	isr = sil_rew_mem((uint32_t *)(hspi->base+TOFF_SPI_ISR));
	sil_wrw_mem((uint32_t *)(hspi->base+TOFF_SPI_IMR), 0);

	syslog_2(LOG_DEBUG, "spi_handler imr[%08x] isr[%08x]", imr, isr);
	tmp = sil_rew_mem((uint32_t *)(hspi->base+TOFF_SPI_ICR));
	if(hspi->Init.semid != 0)
		isig_sem(hspi->Init.semid);
	(void)(tmp);
}

/*
 *  SPI����ߥ����ӥ��롼����
 */
void spi_isr(intptr_t exinf)
{
  spi_handler(&SpiHandle[INDEX_SPI((uint32_t)exinf)]);
}


