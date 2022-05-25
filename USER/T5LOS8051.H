#ifndef __T5LOS8051_H__
#define __T5LOS8051_H__

/* COMMON BYTE Register  */
sfr P0              = 0x80;                     /*P0�˿�*/
sfr P1              = 0x90;                     /*P1�˿�*/
sfr P2              = 0xA0;                     /*P2�˿�*/
sfr P3              = 0xB0;                     /*P3�˿�(ֻ��P3.0-P3.3��Ч)*/
sfr PSW             = 0xD0;                     /*����״̬��*/
sfr ACC             = 0xE0;                     /*�ۼ���*/
sfr B               = 0xF0;                     /*�˳�������*/
sfr SP              = 0x81;                     /*��ջָ��*/
sfr DPL             = 0x82;                     /*����ָ��(��8λ)*/
sfr DPH             = 0x83;                     /*����ָ��(��8λ)*/
sfr PCON            = 0x87;                     /*��Դ���Ƽ�������*/
sfr TCON            = 0x88;                     /*��ʱ������*/
sfr TMOD            = 0x89;                     /*��ʱ����ʽѡ��*/
sfr TL0             = 0x8A;                     /*��ʱ��0��8λ*/
sfr TL1             = 0x8B;                     /*��ʱ��1��8λ*/
sfr TH0             = 0x8C;                     /*��ʱ��0��8λ*/
sfr TH1             = 0x8D;                     /*��ʱ��1��8λ*/
//sfr IE              = 0xA8;                     /*<REG51>�ж���������*/
//sfr IP              = 0xB8;                     /*<REG51>�ж����ȿ���*/
sfr IP0             = 0xA9;                     /*�ж����ȼ�������0*/
sfr IP1             = 0xB9;                     /*�ж����ȼ�������1*/
//sfr SCON            = 0x98;                     /*<REG51>���пڿ���*/
sfr SCON0           = 0x98;                     /*UART2���ƽӿ�*/
sfr SCON1           = 0x9B;                     /*UART3���ƽӿ�*/
sfr SBUF0           = 0x99;                     /*UART2�շ�������*/
sfr SBUF1           = 0x9C;                     /*UART3�շ�������*/

/* T5L BYTE Register Must INIT */
sfr CKCON           = 0x8E;                     /*|0x00|CPU������1Tģʽ*/
sfr T2CON           = 0xC8;                     /*|0x70|T2���ƼĴ���*/
sfr DPC             = 0x93;                     /*|0x00|DPTR�ı仯ģʽ*/
sfr PAGESEL         = 0x94;                     /*|0x01|64KB����ռ�*/
sfr D_PAGESEL       = 0x95;                     /*|0x02|32KB��RAM�ռ�*/
sfr MUX_SEL         = 0xC9;                     /*|0x60|���踴��ѡ��*/
sfr PORTDRV         = 0xF9;                     /*|0x01|IO���ģʽ����������*/
sfr RAMMODE         = 0xF8;                     /*|0x00|DGUS�����洢�����ʽӿڿ���*/

/* BIT Register */
/* DGUS �����洢�� */
sbit APP_REQ        = RAMMODE^7;                /*д1�����д����ռ�ñ�������*/
sbit APP_EN         = RAMMODE^6;                /*д1����1�ζ�д��Ӳ��ִ�к�����*/
sbit APP_RW         = RAMMODE^5;                /*1=�������洢����0=д�����洢��*/
sbit APP_ACK        = RAMMODE^4;                /*Ӧ��1=OK��0=BUSY ��Ҫ�����ȴ�*/
sbit APP_DATA3      = RAMMODE^3;                /*1=��ӦByteд�룬0=��ӦByte��д��*/
sbit APP_DATA2      = RAMMODE^2;                /*ͬ��*/
sbit APP_DATA1      = RAMMODE^1;                /*ͬ��*/
sbit APP_DATA0      = RAMMODE^0;                /*ͬ��*/
sfr ADR_H           = 0xF1;                     /*DGUS�����洢����8λ��ַ��A23:A16*/
sfr ADR_M           = 0xF2;                     /*DGUS�����洢����8λ��ַ��A15:A8*/
sfr ADR_L           = 0xF3;                     /*DGUS�����洢����8λ��ַ��A7:A0*/
sfr ADR_INC         = 0xF4;                     /*�Զ�������ADR=(��д��)ADR+ADR_INC*/
sfr DATA3           = 0xFA;                     /*DGUS�������ݽӿ�*/
sfr DATA2           = 0xFB;                     /*ͬ��*/
sfr DATA1           = 0xFC;                     /*ͬ��*/
sfr DATA0           = 0xFD;                     /*ͬ��*/

/* SFR��չ�ӿ� */
sfr EXADR           = 0xFE;                     /*��չSFR��ַ*/
sfr EXDATA          = 0xFF;                     /*��չSFR�����ݽӿ�*/

/* ��ѧ���㵥ԪMDU */
sfr MAC_MODE        = 0xE5;                     /*MACӲ���˼������ƼĴ���*/
sfr DIV_MODE        = 0xE6;                     /*DIVӲ�����������ƼĴ���*/

/* ��ʱ��0/1 */
sbit TF1            = TCON^7;                   /*T1����ж������־λ*/
sbit TR1            = TCON^6;                   /*T1���п���λ*/
sbit TF0            = TCON^5;                   /*T0����ж������־λ*/
sbit TR0            = TCON^4;                   /*T0���п���λ*/
sbit IE1            = TCON^3;                   /*�ⲿ�ж�1���жϱ�־*/
sbit IT1            = TCON^2;                   /*�ⲿ�ж�1����λ*/
sbit IE0            = TCON^1;                   /*�ⲿ�ж�0���жϱ�־*/
sbit IT0            = TCON^0;                   /*�ⲿ�ж�0����λ*/
/* ��ʱ��2 */
sbit TSF2           = T2CON^7;                  /*0=CPU��Ƶ/12��1=CPU��Ƶ/24*/
sbit TR2            = T2CON^0;                  /*0=T2�رգ�1=T2����*/
sfr TH2             = 0xCD;                     /*T2����ֵH*/
sfr TL2             = 0xCC;                     /*T2����ֵL*/
sfr TRL2H           = 0xCB;                     /*T2����װ��ֵH*/
sfr TRL2L           = 0xCA;                     /*T2����װ��ֵL*/

/* IO�� */
sfr P0MDOUT         = 0xB7;                     /*P0���������*/
sfr P1MDOUT         = 0xBC;                     /*P1���������*/
sfr P2MDOUT         = 0xBD;                     /*P2���������*/
sfr P3MDOUT         = 0xBE;                     /*P3���������*/
/* IO���� */
sbit P0_0           = P0^0;
sbit P1_0           = P1^0;
sbit P1_1           = P1^1;
sbit P3_1           = P3^1;

/* UART2 */
sbit SM0            = SCON0^7;                  /*��ʽѡ��1*/
sbit SM1            = SCON0^6;                  /*��ʽѡ��2*/
sbit SM2            = SCON0^5;                  /*���ͨ�ſ���*/
sbit REN0           = SCON0^4;                  /*���н�������*/
sbit TB80           = SCON0^3;                  /*�����ĵھ�λ*/
sbit RB80           = SCON0^2;                  /*���յĵھ�λ*/
sbit TI0            = SCON0^1;                  /*�����������ж�*/
sbit RI0            = SCON0^0;                  /*�����������ж�*/
sfr ADCON           = 0xD8;                     /*�����ʷ�����ѡ��*/
sfr SREL0H          = 0xBA;                     /*���ò����ʣ���ADCONΪ0x80ʱ*/
sfr SREL0L          = 0xAA;

/* UART3 */
sfr SREL1H          = 0xBB;                     /*���ò�����*/
sfr SREL1L          = 0x9D;

/* UART4 */
sfr SCON2T          = 0x96;                     /*UART4���Ϳ���*/
sfr SCON2R          = 0x97;                     /*UART4���տ���*/
sfr SBUF2_TX        = 0x9E;                     /*UART4�������ݽӿ�*/
sfr SBUF2_RX        = 0x9F;                     /*UART4�������ݽӿ�*/
sfr BODE2_DIV_H     = 0xD9;                     /*UART4����������*/
sfr BODE2_DIV_L     = 0xE7;

/* UART5 */
sfr SCON3T          = 0xA7;                     /*UART5���Ϳ���*/
sfr SCON3R          = 0xAB;                     /*UART5���տ���*/
sfr SBUF3_TX        = 0xAC;                     /*UART5�������ݽӿ�*/
sfr SBUF3_RX        = 0xAD;                     /*UART5�������ݽӿ�*/
sfr BODE3_DIV_H     = 0xAE;                     /*UART5����������*/
sfr BODE3_DIV_L     = 0xAF;

/* CANͨ�� */
sfr CAN_CR          = 0x8F;                     /*CAN�ӿڿ��ƼĴ���*/
sfr CAN_IR          = 0x91;                     /*CAN�ӿ��ж�״̬�Ĵ���*/
sfr CAN_ET          = 0xE8;                     /*CAN�ӿڴ������ͼĴ�����λѰַ*/

/* �ж�ϵͳ���� */
/* ����0 */
sfr IEN0            = 0xA8;
sbit EA             = IEN0^7;                   /*�ж��ܿ���λ*/
sbit ET2            = IEN0^5;                   /*T2��ʱ���ж�ʹ�ܿ���λ*/
sbit ES0            = IEN0^4;                   /*UART2�ж�ʹ�ܿ���λ*/
sbit ET1            = IEN0^3;                   /*T1��ʱ���ж�ʹ�ܿ���λ*/
sbit EX1            = IEN0^2;                   /*�ⲿ�ж�1ʹ�ܿ���λP3.1*/
sbit ET0            = IEN0^1;                   /*T0��ʱ���ж�ʹ�ܿ���λ*/
sbit EX0            = IEN0^0;                   /*�ⲿ�ж�0ʹ�ܿ���λP3.0*/
/* ����1 */
sfr IEN1            = 0xB8;
sbit ES3R           = IEN1^5;                   /*UART5�����ж�ʹ�ܿ���λ*/
sbit ES3T           = IEN1^4;                   /*UART5�����ж�ʹ�ܿ���λ*/
sbit ES2R           = IEN1^3;                   /*UART4�����ж�ʹ�ܿ���λ*/
sbit ES2T           = IEN1^2;                   /*UART4�����ж�ʹ�ܿ���λ*/
sbit ECAN           = IEN1^1;                   /*CANͨ���ж�ʹ�ܿ���λ*/
/* ����2 */
sfr IEN2            = 0x9A;                     /*UART3�ж�ʹ�ܿ���λ*/
/* ����3 */
sfr IEN3            = 0xD1;                     /*UART3�ж�ʹ�ܿ���λ*/

/* �жϱ�־ */
sfr IRCON           = 0xC0;                     /*�жϱ�־�Ĵ���4*/
sfr IRCON2          = 0xBF;
sbit TF2            = IRCON^6;                  /*T2�жϴ�����־*/

/* ����״̬�Ĵ��� */
sbit CY             = PSW^7;                    /*��λ����λ��־*/
sbit AC             = PSW^6;                    /*������λ����λ��־*/
sbit F0             = PSW^5;                    /*�û���־λ*/
sbit RS1            = PSW^4;                    /*�Ĵ�����ѡ�����λ*/
sbit RS0            = PSW^3;                    /*�Ĵ�����ѡ�����λ*/
sbit OV             = PSW^2;                    /*�����־*/
sbit F1             = PSW^1;                    /*����*/
sbit P              = PSW^0;                    /*��żУ���־λ*/

#endif