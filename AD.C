
#include "config.h"
#include "AD.H"			

int	xdata Battery = 1200;
u16 xdata ADC0_Last = 6400;
u16 xdata adc0;

#define ADC_START	(1<<6)	/* 自动清0 */
#define ADC_FLAG	(1<<5)	/* 软件清0 */

#define	ADC_SPEED	3		/* 0~15, ADC转换时间(CPU时钟数) = (n+1)*32  ADCCFG */
#define	RES_FMT		(1<<5)	/* ADC结果格式 0: 左对齐, ADC_RES: D11 D10 D9 D8 D7 D6 D5 D4, ADC_RESL: D3 D2 D1 D0 0 0 0 0 */
							/* ADCCFG      1: 右对齐, ADC_RES: 0 0 0 0 D11 D10 D9 D8, ADC_RESL: D7 D6 D5 D4 D3 D2 D1 D0 */



//*********************************初始化A/D转换*************************************************
void adc_init()
{
	P0n_pure_input(0x04);	//将P0.2作为模拟量输入
	ADC_CONTR = 0x80;		//ADC on
	ADCCFG = RES_FMT + ADC_SPEED;
}

//========================================================================
// 函数: u16	Get_ADC12bitResult(u8 channel)
// 描述: 查询法读一次ADC结果.
// 参数: channel: 选择要转换的ADC.
// 返回: 10位ADC结果.
// 版本: V1.0, 2016-4-28
//========================================================================
u16	Get_ADC12bitResult(u8 channel)	//channel = 0~7
{
	u8	i;
	ADC_RES = 0;
	ADC_RESL = 0;

	ADC_CONTR = 0x80 | ADC_START | channel; 
	NOP(10);			//
	i = 250;
	do
	{
		if((ADC_CONTR & ADC_FLAG) != 0)	//ADC结束
		{
			ADC_CONTR &= ~ADC_FLAG;
			return	((u16)ADC_RES * 256 + (u16)ADC_RESL);
		}
	}
	while(--i);
	return 0;
}

//**********************************电压处理***************************************
void AD(void)
{
	ADC0_Last = ((ADC0_Last * 3) >> 2) + Get_ADC12bitResult(10);

	adc0 = ADC0_Last >> 2;
	Battery = (int)((float)ADC0_Last * 0.08904f);	//78M05输出4.964V做ADC基准
}
