#include "oled.h"
#include "oledfont.h"
#include "delay.h"

/* ================================================================
 *  软件 I2C 底层 — 仅 OLED 使用，合并至此减少文件数
 *  引脚由 SysConfig $name="OLED_I2C" 生成的宏决定
 * ================================================================ */

#define SDA_OUT()   {                                                       \
                         DL_GPIO_initDigitalOutput(OLED_I2C_SDA_IOMUX);     \
                         DL_GPIO_setPins(OLED_I2C_PORT, OLED_I2C_SDA_PIN);  \
                         DL_GPIO_enableOutput(OLED_I2C_PORT, OLED_I2C_SDA_PIN); \
                     }

#define SDA_IN()    { DL_GPIO_initDigitalInput(OLED_I2C_SDA_IOMUX); }

#define SDA_GET()   ( ( ( DL_GPIO_readPins(OLED_I2C_PORT, OLED_I2C_SDA_PIN) & OLED_I2C_SDA_PIN ) > 0 ) ? 1 : 0 )

#define SDA(x)      ( (x) ? (DL_GPIO_setPins(OLED_I2C_PORT, OLED_I2C_SDA_PIN))    \
                           : (DL_GPIO_clearPins(OLED_I2C_PORT, OLED_I2C_SDA_PIN)) )

#define SCL(x)      ( (x) ? (DL_GPIO_setPins(OLED_I2C_PORT, OLED_I2C_SCL_PIN))    \
                           : (DL_GPIO_clearPins(OLED_I2C_PORT, OLED_I2C_SCL_PIN)) )

static void I2C_Start(void)
{
        SDA_OUT();
        SCL(1);
        SDA(0);

        SDA(1);
        delay_us(5);
        SDA(0);
        delay_us(5);

        SCL(0);
}

static void I2C_Stop(void)
{
        SDA_OUT();
        SCL(0);
        SDA(0);

        SCL(1);
        delay_us(5);
        SDA(1);
        delay_us(5);
}

static unsigned char I2C_WaitAck(void)
{
        char ack = 0;
        unsigned char ack_flag = 10;
        SCL(0);
        SDA(1);
        SDA_IN();

        SCL(1);
        while( (SDA_GET()==1) && ( ack_flag ) )
        {
                ack_flag--;
                delay_us(5);
        }

        if( ack_flag <= 0 )
        {
                I2C_Stop();
                return 1;
        }
        else
        {
                SCL(0);
                SDA_OUT();
        }
        return ack;
}

static void Send_Byte(uint8_t dat)
{
        int i = 0;
        SDA_OUT();
        SCL(0);

        for( i = 0; i < 8; i++ )
        {
                SDA( (dat & 0x80) >> 7 );
                delay_us(1);
                SCL(1);
                delay_us(5);
                SCL(0);
                delay_us(5);
                dat<<=1;
        }
}

/* ================================================================
 *  OLED 驱动
 * ================================================================ */

uint8_t OLED_GRAM[144][8];
extern void delay_ms(uint32_t ms);

void OLED_ColorTurn(uint8_t i)
{
	if(i==0) OLED_WR_Byte(0xA6,OLED_CMD);
	if(i==1) OLED_WR_Byte(0xA7,OLED_CMD);
}

void OLED_DisplayTurn(uint8_t i)
{
	if(i==0)
	{
		OLED_WR_Byte(0xC8,OLED_CMD);
		OLED_WR_Byte(0xA1,OLED_CMD);
	}
	if(i==1)
	{
		OLED_WR_Byte(0xC0,OLED_CMD);
		OLED_WR_Byte(0xA0,OLED_CMD);
	}
}

void OLED_WR_Byte(uint8_t dat, uint8_t mode)
{
    I2C_Start();
    Send_Byte(0x78);
    if (I2C_WaitAck()) { I2C_Stop(); return; }
    Send_Byte(mode ? 0x40 : 0x00);
    if (I2C_WaitAck()) { I2C_Stop(); return; }
    Send_Byte(dat);
    if (I2C_WaitAck()) { I2C_Stop(); return; }
    I2C_Stop();
}

void OLED_DisPlay_On(void)
{
	OLED_WR_Byte(0x8D,OLED_CMD);
	OLED_WR_Byte(0x14,OLED_CMD);
	OLED_WR_Byte(0xAF,OLED_CMD);
}

void OLED_DisPlay_Off(void)
{
	OLED_WR_Byte(0x8D,OLED_CMD);
	OLED_WR_Byte(0x10,OLED_CMD);
	OLED_WR_Byte(0xAF,OLED_CMD);
}

void OLED_Refresh(void)
{
	uint8_t i,n;
	for(i=0;i<8;i++)
	{
	   OLED_WR_Byte(0xb0+i,OLED_CMD);
	   OLED_WR_Byte(0x00,OLED_CMD);
	   OLED_WR_Byte(0x10,OLED_CMD);
	   for(n=0;n<128;n++)
		 OLED_WR_Byte(OLED_GRAM[n][i],OLED_DATA);
	}
}

void OLED_Clear(void)
{
	uint8_t i,n;
	for(i=0;i<8;i++)
	{
	   for(n=0;n<128;n++)
		{
			 OLED_GRAM[n][i]=0;
		}
	}
	OLED_Refresh();
}

void OLED_DrawPoint(uint8_t x,uint8_t y)
{
	uint8_t i,m,n;
	i=y/8;
	m=y%8;
	n=1<<m;
	OLED_GRAM[x][i]|=n;
}

void OLED_ClearPoint(uint8_t x,uint8_t y)
{
	uint8_t i,m,n;
	i=y/8;
	m=y%8;
	n=1<<m;
	OLED_GRAM[x][i]=~OLED_GRAM[x][i];
	OLED_GRAM[x][i]|=n;
	OLED_GRAM[x][i]=~OLED_GRAM[x][i];
}

void OLED_DrawLine(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2)
{
	uint8_t i,k,k1,k2;
	if((x1<0)||(x2>128)||(y1<0)||(y2>64)||(x1>x2)||(y1>y2))return;
	if(x1==x2)
	{
		for(i=0;i<(y2-y1);i++) OLED_DrawPoint(x1,y1+i);
	}
	else if(y1==y2)
	{
		for(i=0;i<(x2-x1);i++) OLED_DrawPoint(x1+i,y1);
	}
	else
	{
		k1=y2-y1;
		k2=x2-x1;
		k=k1*10/k2;
		for(i=0;i<(x2-x1);i++) OLED_DrawPoint(x1+i,y1+i*k/10);
	}
}

void OLED_DrawCircle(uint8_t x,uint8_t y,uint8_t r)
{
	int a = 0, b = r, num;
	while(2 * b * b >= r * r)
	{
		OLED_DrawPoint(x + a, y - b);
		OLED_DrawPoint(x - a, y - b);
		OLED_DrawPoint(x - a, y + b);
		OLED_DrawPoint(x + a, y + b);
		OLED_DrawPoint(x + b, y + a);
		OLED_DrawPoint(x + b, y - a);
		OLED_DrawPoint(x - b, y - a);
		OLED_DrawPoint(x - b, y + a);
		a++;
		num = (a * a + b * b) - r*r;
		if(num > 0) { b--; a--; }
	}
}

void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t size1)
{
	uint8_t i,m,temp,size2,chr1;
	uint8_t y0=y;
	size2=(size1/8+((size1%8)?1:0))*(size1/2);
	chr1=chr-' ';
	for(i=0;i<size2;i++)
	{
		if(size1==12) {temp=asc2_1206[chr1][i];}
		else if(size1==16) {temp=asc2_1608[chr1][i];}
		else if(size1==24) {temp=asc2_2412[chr1][i];}
		else return;
		for(m=0;m<8;m++)
		{
			if(temp&0x80)OLED_DrawPoint(x,y);
			else OLED_ClearPoint(x,y);
			temp<<=1;
			y++;
			if((y-y0)==size1)
			{
				y=y0;
				x++;
				break;
			}
		}
	}
}

void OLED_ShowString(uint8_t x,uint8_t y,uint8_t *chr,uint8_t size1)
{
	while((*chr>=' ')&&(*chr<='~'))
	{
		OLED_ShowChar(x,y,*chr,size1);
		x+=size1/2;
		if(x>128-size1)
		{
			x=0;
			y+=size1;
		}
		chr++;
	}
}

uint32_t OLED_Pow(uint8_t m,uint8_t n)
{
	uint32_t result=1;
	while(n--) result*=m;
	return result;
}

void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size1)
{
	uint8_t t,temp;
	for(t=0;t<len;t++)
	{
		temp=(num/OLED_Pow(10,len-t-1))%10;
		if(temp==0) OLED_ShowChar(x+(size1/2)*t,y,'0',size1);
		else OLED_ShowChar(x+(size1/2)*t,y,temp+'0',size1);
	}
}

void OLED_ShowChinese(uint8_t x,uint8_t y,uint8_t num,uint8_t size1)
{
	uint8_t i,m,n=0,temp,chr1;
	uint8_t x0=x,y0=y;
	uint8_t size3=size1/8;
	while(size3--)
	{
		chr1=num*size1/8+n;
		n++;
		for(i=0;i<size1;i++)
		{
			if(size1==16) {temp=Hzk1[chr1][i];}
			else if(size1==24) {temp=Hzk2[chr1][i];}
			else if(size1==32) {temp=Hzk3[chr1][i];}
			else if(size1==64) {temp=Hzk4[chr1][i];}
			else return;
			for(m=0;m<8;m++)
			{
				if(temp&0x01)OLED_DrawPoint(x,y);
				else OLED_ClearPoint(x,y);
				temp>>=1;
				y++;
			}
			x++;
			if((x-x0)==size1) {x=x0;y0=y0+8;}
			y=y0;
		}
	}
}

void OLED_WR_BP(uint8_t x,uint8_t y)
{
	OLED_WR_Byte(0xb0+y,OLED_CMD);
	OLED_WR_Byte(((x&0xf0)>>4)|0x10,OLED_CMD);
	OLED_WR_Byte((x&0x0f)|0x01,OLED_CMD);
}

void OLED_ShowPicture(uint8_t x0,uint8_t y0,uint8_t x1,uint8_t y1,uint8_t BMP[])
{
	uint32_t j=0;
	uint8_t x=0,y=0;
	if(y%8==0)y=0;
	else y+=1;
	for(y=y0;y<y1;y++)
	{
		 OLED_WR_BP(x0,y);
		 for(x=x0;x<x1;x++)
		 {
			 OLED_WR_Byte(BMP[j],OLED_DATA);
			 j++;
		 }
	}
}

void OLED_Init(void)
{
	SDA_OUT();
	delay_ms(100);

	OLED_WR_Byte(0xAE,OLED_CMD);
	OLED_WR_Byte(0x00,OLED_CMD);
	OLED_WR_Byte(0x10,OLED_CMD);
	OLED_WR_Byte(0x40,OLED_CMD);
	OLED_WR_Byte(0x81,OLED_CMD);
	OLED_WR_Byte(0xCF,OLED_CMD);
	OLED_WR_Byte(0xA1,OLED_CMD);
	OLED_WR_Byte(0xC8,OLED_CMD);
	OLED_WR_Byte(0xA6,OLED_CMD);
	OLED_WR_Byte(0xA8,OLED_CMD);
	OLED_WR_Byte(0x3f,OLED_CMD);
	OLED_WR_Byte(0xD3,OLED_CMD);
	OLED_WR_Byte(0x00,OLED_CMD);
	OLED_WR_Byte(0xd5,OLED_CMD);
	OLED_WR_Byte(0x80,OLED_CMD);
	OLED_WR_Byte(0xD9,OLED_CMD);
	OLED_WR_Byte(0xF1,OLED_CMD);
	OLED_WR_Byte(0xDA,OLED_CMD);
	OLED_WR_Byte(0x12,OLED_CMD);
	OLED_WR_Byte(0xDB,OLED_CMD);
	OLED_WR_Byte(0x40,OLED_CMD);
	OLED_WR_Byte(0x20,OLED_CMD);
	OLED_WR_Byte(0x02,OLED_CMD);
	OLED_WR_Byte(0x8D,OLED_CMD);
	OLED_WR_Byte(0x14,OLED_CMD);
	OLED_WR_Byte(0xA4,OLED_CMD);
	OLED_WR_Byte(0xA6,OLED_CMD);
	OLED_WR_Byte(0xAF,OLED_CMD);
	OLED_Clear();
}
