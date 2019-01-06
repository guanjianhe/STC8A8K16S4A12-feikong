
#include "config.h"
#include "STC8xxx_PWM.H"


void PWMGO(void)
{
//	int i=1;

	//������Ҫʹ�õ�PWM�����Ϊǿ����ģʽ
	P20 = 0;
	P21 = 0;
	P22 = 0;
	P23 = 0;
//	P24 = 0;
//	P25 = 0;
//	P26 = 0;
//	P27 = 0;
	P2n_push_pull(0x0f);	// PWM0-P2.0	PWM1-P2.1  PWM2-P2.2  PWM3-P2.3	PWM4-P2.4	PWM5-P2.5  PWM6-P1.6  PWM7-P1.7
	
//****************************************����ΪI/0��ʼ��******************************************
	//ʹ�ö�ʱ��2��Ϊʱ��Դ
 	
	EAXSFR();		//����XFR

	PWMCFG = 0x00;	//   7λ                     6λ             5λ    4λ    3λ    2λ    1λ    0λ 
					//   CBIF
					//1  �����������жϱ�־  ���������㴥��ADC    -      -      -      -      -      -
					//0                      ����ʱ������ADC
	
	PWMIF = 0x00;	//  7λ    6λ  5λ   4λ   3λ   2λ   1λ   0λ 
					//  C7IF  C6IF  C5IF  C4IF  C3IF  C2IF  C1IF  C0IF
					//��ӦPWM�жϱ�־
	
	PWMFDCR = 0x00;	//  7λ     6λ   5λ     4λ    3λ    2λ   1λ   0λ 
					// INVCMP  INVIO  ENFD  FLTFLIO  EFDI  FDCMP  FDIO  FDIF

	PWMCKS = 11;	//7λ6λ5λ    4λ             3λ    2λ    1λ    0λ 
					//   ��0    0-ϵͳʱ�ӷ�Ƶ          ��Ƶ�����趨
					//          1-��ʱ��2���       ʱ��=ϵͳʱ��/([3:0]+1)
	
	PWMC  = 16000;	// 15λ�Ĵ���������PWM���ڣ���ֵΪ1-32767����λ������ʱ��
	

// ����Ϊÿ��PWM����ڵ�������
	PWM0CR = 0x80;	//     7λ      6λ        5λ   4λ3λ     2λ         1λ            0λ 
					//     ENCnO    CnINI       -    Cn_S      ECnI       ECnT2SI        ECnT1SI
					//1:   ����PWM  ��ʼ�ߵ�ƽ       IOѡ��    �����ж�   ����T2���ж�   ����T1���ж�
					//0:   ��ֹPWM  ��ʼ�͵�ƽ       IOѡ��    ��ֹ�ж�   ��ֹT2���ж�   ��ֹT1���ж�
	PWM1CR = 0x80; 
	PWM2CR = 0x80; 
	PWM3CR = 0x80; 
//	PWM4CR = 0x80; 
//	PWM5CR = 0x80;
//	PWM6CR = 0x80;	
//	PWM7CR = 0x80;
/*	
	PWM0HLD = 0x00;
	PWM1HLD = 0x00;
	PWM2HLD = 0x00;
	PWM3HLD = 0x00;
	PWM4HLD = 0x00;
	PWM5HLD = 0x00;
	PWM6HLD = 0x00;
	PWM7HLD = 0x00;
*/
	PWM0T1 = 4000;
	PWM1T1 = 4000;
	PWM2T1 = 4000;
	PWM3T1 = 4000;
//	PWM4T1 = 4000;
//	PWM5T1 = 4000;
//	PWM6T1 = 4000;
//	PWM7T1 = 4000;

	PWM0T2 = 2000;
	PWM1T2 = 2000;
	PWM2T2 = 2000;
	PWM3T2 = 2000;
//	PWM4T2 = 2000;
//	PWM5T2 = 2000;
//	PWM6T2 = 2000;
//	PWM7T2 = 2000;

	PWMCR = 0x80;	//����PWM

	EAXRAM();		//�ָ�����XRAM

}
