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
 *  @(#) $Id: fft.c 699 2020-12-26 17:44:27Z fukuen $
 */
/*
 * 
 *  K210 FFT�ɥ饤�дؿ���
 *
 */
#include "kernel_impl.h"
#include <t_syslog.h>
#include <t_stdlib.h>
#include <stddef.h>
#include <string.h>
#include "kendryte-k210.h"
#include "sysctl.h"
#include "device.h"
#include "fft.h"

/*
 *  SIL�ؿ��Υޥ������
 */
#define sil_orw_mem(a, b)		sil_wrw_mem((a), sil_rew_mem(a) | (b))
#define sil_andw_mem(a, b)		sil_wrw_mem((a), sil_rew_mem(a) & ~(b))
#define sil_modw_mem(a, b, c)	sil_wrw_mem((a), (sil_rew_mem(a) & (~b)) | (c))

FFT_Handle_t fft_handle;
DMA_Handle_t fft_dmarx_handle;
DMA_Handle_t fft_dmatx_handle;

/*
 *  FFT�������
 *  parameter1  init: FFT������깽¤�ΤؤΥݥ���
 *  return FFT�ϥ�ɥ�ؤΥݥ��󥿡�NULL�ǥ��顼
 */
FFT_Handle_t *
fft_init(const FFT_Init_t *init)
{
	FFT_Handle_t *hfft = &fft_handle;
	DMA_Handle_t *hdmarx = &fft_dmarx_handle;
	DMA_Handle_t *hdmatx = &fft_dmatx_handle;

	if(init == NULL)
		return NULL;

	fft_point_t point = FFT_512;
	switch(init->point_num)
	{
		case 512:
			point = FFT_512;
			break;
		case 256:
			point = FFT_256;
			break;
		case 128:
			point = FFT_128;
			break;
		case 64:
			point = FFT_64;
			break;
		default:
			return NULL;
			break;
	}

	memcpy(&hfft->Init, init, sizeof(FFT_Init_t));

	sil_orw_mem((uint32_t *)(TADR_SYSCTL_BASE+TOFF_SYSCTL_CLK_EN_CENT), SYSCTL_CLK_EN_PERI_FFT_CLK_EN);
	sil_orw_mem((uint32_t *)(TADR_SYSCTL_BASE+TOFF_SYSCTL_PERI_RESET), SYSCTL_PERI_RESET_FFT_RESET);
	sil_dly_nse(10000);
	sil_andw_mem((uint32_t *)(TADR_SYSCTL_BASE+TOFF_SYSCTL_PERI_RESET), SYSCTL_PERI_RESET_FFT_RESET);

	sil_modw_mem((uint32_t *)(FFT_BASE_ADDR+TADR_FFT_CTRL), FFT_CTRL_POINT, point);
	sil_modw_mem((uint32_t *)(FFT_BASE_ADDR+TADR_FFT_CTRL), FFT_CTRL_MODE, ((init->direction) << 3));
	sil_modw_mem((uint32_t *)(FFT_BASE_ADDR+TADR_FFT_CTRL), FFT_CTRL_SHIFT, ((init->shift) << 4));
	sil_orw_mem((uint32_t *)(FFT_BASE_ADDR+TADR_FFT_CTRL), FFT_CTRL_DMA_SEND);
	sil_orw_mem((uint32_t *)(FFT_BASE_ADDR+TADR_FFT_CTRL), FFT_CTRL_ENABLE);
	sil_andw_mem((uint32_t *)(FFT_BASE_ADDR+TADR_FFT_CTRL), FFT_CTRL_INPUT_MODE);
	sil_andw_mem((uint32_t *)(FFT_BASE_ADDR+TADR_FFT_CTRL), FFT_CTRL_DATA_MODE);

	hdmarx->chnum = init->RxDMAChannel;
	hdmarx->xfercallback = NULL;
	hdmarx->errorcallback = NULL;
	hdmarx->Init.Request = DMA_SELECT_FFT_RX_REQ;		/* DMA���� */
	hdmarx->Init.Direction = DMA_PERIPH_TO_MEMORY;		/* DMAž������ */
	hdmarx->Init.SrcMultBlock = DMAC_MULTBLOCK_CONT;	/* �������ޥ���֥�å������� */
	hdmarx->Init.DrcMultBlock = DMAC_MULTBLOCK_CONT;	/* �ǥ��ƥ��͡������ޥ���֥�å������� */
	hdmarx->Init.SrcHandShake = DMAC_HS_HARDWARE;		/* �������ϥ�ɥ������� */
	hdmarx->Init.DrcHandShake = DMAC_HS_SOFTWARE;		/* �ǥ��ƥ��͡������ϥ�ɥ������� */
	hdmarx->Init.SrcHwhsPol = DMAC_HWHS_POLARITY_LOW;	/* �������ϡ��ɥ������ϥ�ɥ����������� */
	hdmarx->Init.DrcHwhsPol = DMAC_HWHS_POLARITY_LOW;	/* �ǥ��ƥ��͡������ϡ��ɥ������ϥ�ɥ����������� */
	hdmarx->Init.Priority = 4;							/* ͥ���� */
	hdmarx->Init.SrcMaster = DMAC_MASTER1;				/* �������ޥ��������� */
	hdmarx->Init.DstMaster = DMAC_MASTER2;				/* �ǥ��ƥ��͡������ޥ��������� */
	hdmarx->Init.SrcInc = DMAC_ADDR_NOCHANGE;			/* ���������󥯥�������� */
	hdmarx->Init.DstInc = DMAC_ADDR_INCREMENT;			/* �ǥ��ƥ��͡�����󥤥󥯥�������� */
	hdmarx->Init.SrcTransWidth = DMAC_TRANS_WIDTH_64;	/* ������ž���� */
	hdmarx->Init.DstTransWidth = DMAC_TRANS_WIDTH_64;	/* �ǥ��ƥ��͡������ž���� */
	hdmarx->Init.SrcBurstSize = DMAC_MSIZE_4;			/* �������С����ȥ����� */
	hdmarx->Init.DstBurstSize = DMAC_MSIZE_4;			/* �ǥ��ƥ��͡������С����ȥ����� */
	hdmarx->Init.IocBlkTrans = 0;						/* IOC�֥�å�ž�� */
	hdmarx->localdata = (void *)hfft;

	dma_init(hdmarx);
	hfft->hdmarx = hdmarx;

	hdmatx->chnum = init->TxDMAChannel;
	hdmatx->xfercallback = NULL;
	hdmatx->errorcallback = NULL;
	hdmatx->Init.Request = DMA_SELECT_FFT_TX_REQ;		/* DMA���� */
	hdmatx->Init.Direction = DMA_MEMORY_TO_PERIPH;		/* DMAž������ */
	hdmatx->Init.SrcMultBlock = DMAC_MULTBLOCK_CONT;	/* �������ޥ���֥�å������� */
	hdmatx->Init.DrcMultBlock = DMAC_MULTBLOCK_CONT;	/* �ǥ��ƥ��͡������ޥ���֥�å������� */
	hdmatx->Init.SrcHandShake = DMAC_HS_SOFTWARE;		/* �������ϥ�ɥ������� */
	hdmatx->Init.DrcHandShake = DMAC_HS_HARDWARE;		/* �ǥ��ƥ��͡������ϥ�ɥ������� */
	hdmatx->Init.SrcHwhsPol = DMAC_HWHS_POLARITY_LOW;	/* �������ϡ��ɥ������ϥ�ɥ����������� */
	hdmatx->Init.DrcHwhsPol = DMAC_HWHS_POLARITY_LOW;	/* �ǥ��ƥ��͡������ϡ��ɥ������ϥ�ɥ����������� */
	hdmatx->Init.Priority = 4;							/* ͥ���� */
	hdmatx->Init.SrcMaster = DMAC_MASTER1;				/* �������ޥ��������� */
	hdmatx->Init.DstMaster = DMAC_MASTER2;				/* �ǥ��ƥ��͡������ޥ��������� */
	hdmatx->Init.SrcInc = DMAC_ADDR_INCREMENT;			/* ���������󥯥�������� */
	hdmatx->Init.DstInc = DMAC_ADDR_NOCHANGE;			/* �ǥ��ƥ��͡�����󥤥󥯥�������� */
	hdmatx->Init.SrcTransWidth = DMAC_TRANS_WIDTH_64;	/* ������ž���� */
	hdmatx->Init.DstTransWidth = DMAC_TRANS_WIDTH_64;	/* �ǥ��ƥ��͡������ž���� */
	hdmatx->Init.SrcBurstSize = DMAC_MSIZE_4;			/* �������С����ȥ����� */
	hdmatx->Init.DstBurstSize = DMAC_MSIZE_4;			/* �ǥ��ƥ��͡������С����ȥ����� */
	hdmatx->Init.IocBlkTrans = 0;						/* IOC�֥�å�ž�� */
	hdmatx->localdata = NULL;

	dma_init(hdmatx);
	hfft->hdmatx = hdmatx;
	return hfft;
}

/*
 *  DMAž�����ϴؿ�
 */
DMA_Handle_t *
fft_dmac_set_single_mode(DMA_Handle_t *hdma,
						  const void *src, void *dest,
						  size_t block_size)
{
	dma_reset(hdma);
	dma_start(hdma, (uintptr_t)src, (uintptr_t)dest, block_size);
	return hdma;
}

/*
 *  FFT-DMAž����λ�Ԥ�
 */
ER
fft_dmac_wait_idle(DMA_Handle_t * hdma)
{
	FFT_Handle_t *hfft = (FFT_Handle_t *)hdma->localdata;
	ER ercd = E_OK;
	int tick = DMA_TRS_TIMEOUT;

	while((hdma->status == DMA_STATUS_BUSY) && tick > 0){
		if(hfft != NULL && hfft->Init.semdmaid != 0){
	 		ercd = twai_sem(hfft->Init.semdmaid, 5);
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
 *  FFT
 */
void fft_complex_uint16_dma(FFT_Handle_t * hfft, const uint64_t *input, uint64_t *output)
{
	fft_dmac_set_single_mode(hfft->hdmarx, (void *)(FFT_BASE_ADDR+TADR_FFT_OUTPUT_FIFO), output,
		hfft->Init.point_num>>1);
	fft_dmac_set_single_mode(hfft->hdmatx, input, (void *)(FFT_BASE_ADDR+TADR_FFT_INPUT_FIFO),
		hfft->Init.point_num>>1);
	fft_dmac_wait_idle(hfft->hdmarx);
}
