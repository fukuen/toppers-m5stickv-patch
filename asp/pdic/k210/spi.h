/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2008-2011 by Embedded and Real-Time Systems Laboratory
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2015-2019 by TOPPERS PROJECT Educational Working Group.
 * 
 *  ��L���쌠�҂́C�ȉ���(1)�`(4)�̏����𖞂����ꍇ�Ɍ���C�{�\�t�g�E�F
 *  �A�i�{�\�t�g�E�F�A�����ς������̂��܂ށD�ȉ������j���g�p�E�����E��
 *  �ρE�Ĕz�z�i�ȉ��C���p�ƌĂԁj���邱�Ƃ𖳏��ŋ�������D
 *  (1) �{�\�t�g�E�F�A���\�[�X�R�[�h�̌`�ŗ��p����ꍇ�ɂ́C��L�̒���
 *      ���\���C���̗��p��������щ��L�̖��ۏ؋K�肪�C���̂܂܂̌`�Ń\�[
 *      �X�R�[�h���Ɋ܂܂�Ă��邱�ƁD
 *  (2) �{�\�t�g�E�F�A���C���C�u�����`���ȂǁC���̃\�t�g�E�F�A�J���Ɏg
 *      �p�ł���`�ōĔz�z����ꍇ�ɂ́C�Ĕz�z�ɔ����h�L�������g�i���p
 *      �҃}�j���A���Ȃǁj�ɁC��L�̒��쌠�\���C���̗��p��������щ��L
 *      �̖��ۏ؋K����f�ڂ��邱�ƁD
 *  (3) �{�\�t�g�E�F�A���C�@��ɑg�ݍ��ނȂǁC���̃\�t�g�E�F�A�J���Ɏg
 *      �p�ł��Ȃ��`�ōĔz�z����ꍇ�ɂ́C���̂����ꂩ�̏����𖞂�����
 *      �ƁD
 *    (a) �Ĕz�z�ɔ����h�L�������g�i���p�҃}�j���A���Ȃǁj�ɁC��L�̒�
 *        �쌠�\���C���̗��p��������щ��L�̖��ۏ؋K����f�ڂ��邱�ƁD
 *    (b) �Ĕz�z�̌`�Ԃ��C�ʂɒ�߂���@�ɂ���āCTOPPERS�v���W�F�N�g��
 *        �񍐂��邱�ƁD
 *  (4) �{�\�t�g�E�F�A�̗��p�ɂ�蒼�ړI�܂��͊ԐړI�ɐ����邢���Ȃ鑹
 *      �Q������C��L���쌠�҂����TOPPERS�v���W�F�N�g��Ɛӂ��邱�ƁD
 *      �܂��C�{�\�t�g�E�F�A�̃��[�U�܂��̓G���h���[�U����̂����Ȃ闝
 *      �R�Ɋ�Â�����������C��L���쌠�҂����TOPPERS�v���W�F�N�g��
 *      �Ɛӂ��邱�ƁD
 * 
 *  �{�\�t�g�E�F�A�́C���ۏ؂Œ񋟂���Ă�����̂ł���D��L���쌠�҂�
 *  ���TOPPERS�v���W�F�N�g�́C�{�\�t�g�E�F�A�Ɋւ��āC����̎g�p�ړI
 *  �ɑ΂���K�������܂߂āC�����Ȃ�ۏ؂��s��Ȃ��D�܂��C�{�\�t�g�E�F
 *  �A�̗��p�ɂ�蒼�ړI�܂��͊ԐړI�ɐ����������Ȃ鑹�Q�Ɋւ��Ă��C��
 *  �̐ӔC�𕉂�Ȃ��D
 * 
 *  @(#) $Id: spi.h 698 2019-12-28 18:11:30Z roi $
 */
/*
 * 
 *  K210 SPI�f�o�C�X�h���C�o�̊O���錾
 *
 */

#ifndef _SPI_H_
#define _SPI_H_

#ifdef __cplusplus
 extern "C" {
#endif

/*
 *  SPI�|�[�g��`
 */
#define SPI1_PORTID             1
#define SPI2_PORTID             2
#define SPI3_PORTID             3
#define SPI4_PORTID             4
#define NUM_SPIPORT             4


/*
 *  SPI��Ԓ�`
 */
#define SPI_STATUS_RESET        0x0000	/* SPI���g�p��� */
#define SPI_STATUS_READY        0x0001	/* SPI���f�B��� */
#define SPI_STATUS_ERROR        0x0002	/* SPI�G���[��� */
#define SPI_STATUS_BUSY         0x0004	/* SPI������ */

/*
 *  SPI�]�����[�h
 */
#define SPI_XMODE_TX            0x0000	/* ���M���[�h */
#define SPI_XMODE_RX            0x0001	/* ��M���[�h */
#define SPI_XMODE_TXRX          0x0002	/* ����M���[�h */

/*
 *  SPI�G���[��`
 */
#define SPI_ERROR_NONE          0x00000000	/* No error */
#define SPI_ERROR_MODF          0x00000001	/* MODF error */
#define SPI_ERROR_CRC           0x00000002	/* CRC error */
#define SPI_ERROR_OVR           0x00000004	/* OVR error */
#define SPI_ERROR_FRE           0x00000008	/* FRE error */
#define SPI_ERROR_DMA           0x00000010	/* DMA transfer error */
#define SPI_ERROR_TIMEOUT       0x00000020

/*
 *  SPI���[�N���[�h��`
 */
#define SPI_WORK_MODE_0         0x00000000
#define SPI_WORK_MODE_1         0x00000001
#define SPI_WORK_MODE_2         0x00000002
#define SPI_WORK_MODE_3         0x00000003

/*
 *  SPI�t���[���t�H�[�}�b�g��`
 */
#define SPI_FF_STANDARD         0x00000000
#define SPI_FF_DUAL             0x00000001
#define SPI_FF_QUAD             0x00000002
#define SPI_FF_OCTAL            0x00000003

/*
 *  SPI�C���X�g���N�V�����A�h���X���[�h
 */
#define SPI_AITM_STANDARD        0x00000000
#define SPI_AITM_ADDR_STANDARD   0x00000001
#define SPI_AITM_AS_FRAME_FORMAT 0x00000002


/*
 *  SPI�]�����[�h��`
 */
#define SPI_TMOD_TRANS_RECV     0x00000000
#define SPI_TMOD_TRANS          0x00000001
#define SPI_TMOD_RECV           0x00000002
#define SPI_TMOD_EEROM          0x00000003

/*
 *  SPI�]���f�[�^����`
 */
#define SPI_TRANS_CHAR          0x01
#define SPI_TRANS_SHORT         0x02
#define SPI_TRANS_INT           0x04

/*
 *  SPI CS�I���`
 */
#define SPI_CHIP_SELECT_0       0x00
#define SPI_CHIP_SELECT_1       0x01
#define SPI_CHIP_SELECT_2       0x02
#define SPI_CHIP_SELECT_3       0x03
#define SPI_CHIP_SELECT_MAX     4

/*
 *  SPI �ݒ菉���ݒ�\����
 */
typedef struct
{
	uint32_t              WorkMode;
	uint32_t              FrameFormat;
    uint32_t              DataSize;			/* SPI�]���f�[�^�T�C�Y */
	uint32_t              Prescaler;		/* SPI�N���b�N�����ݒ� */
	uint32_t              SignBit;			/* SPI MSB/LSB�ݒ� */
	uint32_t              InstLength;		/* SPI Instraction Length */
	uint32_t              AddrLength;		/* SPI Address Length */
	uint32_t              WaitCycles;		/* SPI WaitCycles */
	uint32_t              IATransMode;		/* SPI �]�����[�h */
	int32_t               SclkPin;			/* SPI SCLK-PIN */
	int32_t               MosiPin;			/* SPI MOSI-PIN */
	int32_t               MisoPin;			/* SPI MISO-PIN */
	int32_t               SsPin;			/* SPI Slave Select-PIN */
	int32_t               SsNo;				/* SPI Slave Select-Number */
	int32_t               TxDMAChannel;		/* SPI TxDMA�`�����l�� */
	int32_t               RxDMAChannel;		/* SPI RxDMA�`�����l�� */
	int                   semid;			/* SPI �ʐM�p�Z�}�t�H�l */
	int                   semlock;			/* SPI ���b�N�Z�}�t�H�l */
	int                   semdmaid;			/* SPI DMA�ʐM�p�Z�}�t�H�l */
}SPI_Init_t;

/*
 *  SPI�n���h��
 */
typedef struct _SPI_Handle_t
{
	unsigned long         base;				/* SPI registers base address */
	SPI_Init_t            Init;				/* SPI communication parameters */
	uint8_t               spi_num;
	uint8_t               dfs_offset;
	uint8_t               frf_offset;
	uint8_t               work_mode_offset;
	uint8_t               tmod_offset;
	uint8_t               dummy[3];
	uint8_t               *pTxBuffPtr;		/* Pointer to SPI Tx transfer Buffer */
	uint32_t              TxXferSize;		/* SPI Tx transfer size */
	uint32_t              TxXferCount;		/* SPI Tx Transfer Counter */
	uint8_t               *pRxBuffPtr;		/* Pointer to SPI Rx transfer Buffer */
	uint32_t              RxXferSize;		/* SPI Rx transfer size */
	uint32_t              RxXferCount;		/* SPI Rx Transfer Counter */
	DMA_Handle_t          *hdmatx;			/* SPI Tx DMA handle parameters */
	DMA_Handle_t          *hdmarx;			/* SPI Rx DMA handle parameters */
	uint16_t              xmode;			/* SPI Transfar mode */
	volatile uint16_t     status;			/* SPI communication state */
	volatile uint32_t     ErrorCode;		/* SPI Error code */
}SPI_Handle_t;


extern SPI_Handle_t *spi_init(ID port, const SPI_Init_t *init);
extern ER spi_deinit(SPI_Handle_t *hspi);
extern ER spi_core_transmit(SPI_Handle_t *hspi, int8_t ss_no, uint8_t *pdata, uint16_t length);
extern ER spi_core_transmit_fill(SPI_Handle_t *hspi, int8_t ss_no, const uint32_t *tx_buff, size_t tx_len);
extern ER spi_core_receive(SPI_Handle_t *hspi, int8_t ss_no, void *rx_buff, size_t rx_len);
extern ER spi_core_transrecv(SPI_Handle_t *hspi, int8_t ss_no, const uint8_t *tx_buf, uint8_t *rx_buf, size_t len);
extern ER spi_eerom_transrecv(SPI_Handle_t *hspi, int8_t ss_no, uint8_t *tx_buf, size_t tx_len, uint8_t *rx_buf, size_t rx_len);
extern ER spi_transmit(SPI_Handle_t *hspi, uint8_t *pdata, uint16_t length);
extern ER spi_receive(SPI_Handle_t *hspi, uint8_t *pdata, uint16_t length);
extern ER spi_transrecv(SPI_Handle_t *hspi, uint8_t *ptxData, uint8_t *prxData, uint16_t length);
extern ER spi_wait(SPI_Handle_t *hspi, uint32_t timeout);
extern void spi_handler(SPI_Handle_t *hspi);
extern void spi_isr(intptr_t exinf);
extern DMA_Handle_t *spi_dmac_set_single_mode(SPI_Handle_t *hspi, 
                          uint8_t rtx,
						  int8_t  ss_no,
                          const void *src, void *dest, uint8_t src_inc,
                          uint8_t dest_inc,
                          uint8_t dmac_burst_size,
                          uint8_t dmac_trans_width,
                          size_t block_size);
extern ER spi_dmac_wait_done(DMA_Handle_t * hdma);


#ifdef __cplusplus
}
#endif

#endif	/* _SPI_H_ */

