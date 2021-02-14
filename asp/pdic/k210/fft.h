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
 *  @(#) $Id: fft.h 699 2020-12-26 17:44:27Z fukuen $
 */
#ifndef _FFT_H
#define _FFT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TADR_FFT_INPUT_FIFO		0x0000
#define TADR_FFT_CTRL			0x0008
  #define FFT_CTRL_POINT			0x00000007
  #define FFT_CTRL_MODE				0x00000008
  #define FFT_CTRL_SHIFT			0x00001FF0
  #define FFT_CTRL_ENABLE			0x00002000
  #define FFT_CTRL_DMA_SEND			0x00004000
  #define FFT_CTRL_INPUT_MODE		0x00018000
  #define FFT_CTRL_DATA_MODE		0x00020000
#define TADR_FFT_FIFO_CTRL		0x0010
#define TADR_FFT_INTR_MASK		0x0018
#define TADR_FFT_INTR_CLEAR 	0x0020
#define TADR_FFT_STATUS			0x0028
#define TADR_FFT_STATUS_RAW		0x0030
#define TADR_FFT_OUTPUT_FIFO	0x0038

typedef struct _complex_hard
{
    int16_t real;
    int16_t imag;
} complex_hard_t;

typedef struct _fft_data
{
    int16_t I1;
    int16_t R1;
    int16_t I2;
    int16_t R2;
} fft_data_t;

typedef enum _fft_point
{
    FFT_512,
    FFT_256,
    FFT_128,
    FFT_64,
} fft_point_t;

typedef enum _fft_direction
{
    FFT_DIR_BACKWARD,
    FFT_DIR_FORWARD,
    FFT_DIR_MAX,
} fft_direction_t;

typedef enum _fft_shift
{
    FFT_FORWARD_SHIFT  = 0x0U,
    FFT_BACKWARD_SHIFT = 0x1ffU
} fft_shift_t;

/*
 *  FFT ���������깽¤��
 */
typedef struct
{
	int32_t               TxDMAChannel;		/* FFT TxDMA�����ͥ� */
	int32_t               RxDMAChannel;		/* FFT RxDMA�����ͥ� */
    uint16_t              shift;
    fft_direction_t       direction;
    size_t                point_num;
	int                   semdmaid;			/* FFT DMA�̿��ѥ��ޥե��� */
}FFT_Init_t;

/*
 *  FFT�ϥ�ɥ����
 */
typedef struct __FFT_Handle {
	FFT_Init_t            Init;				/* FFT communication parameters */
	uint64_t              *input;		    /* Pointer to FFT Tx transfer Buffer */
	uint64_t              *output;		    /* Pointer to FFT Rx transfer Buffer */
	DMA_Handle_t          *hdmatx;		    /* FFT Tx DMA handle parameters */
	DMA_Handle_t          *hdmarx;		    /* FFT Rx DMA handle parameters */
}FFT_Handle_t;

void fft_complex_uint16_dma(FFT_Handle_t * hfft, const uint64_t *input, uint64_t *output);

#ifdef __cplusplus
}
#endif

#endif /* _FFT_H */
