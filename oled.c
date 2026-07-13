#include "oled.h"
#include "oledfont.h"

uint8_t OLED_GRAM[144][8];
extern void delay_ms(uint32_t ms);

//反显函数
void OLED_ColorTurn(uint8_t i)
{
	if(i==0) OLED_WR_Byte(0xA6,OLED_CMD);//正常显示
	if(i==1) OLED_WR_Byte(0xA7,OLED_CMD);//反色显示
}

//屏幕旋转180度
void OLED_DisplayTurn(uint8_t i)
{
	if(i==0)
	{
		OLED_WR_Byte(0xC8,OLED_CMD);//正常显示
		OLED_WR_Byte(0xA1,OLED_CMD);
	}
	if(i==1)
	{
		OLED_WR_Byte(0xC0,OLED_CMD);//反转显示
		OLED_WR_Byte(0xA0,OLED_CMD);
	}
}

// ========== GPIO 模拟 I2C 底层函数 ==========
static inline void I2C_SDA_H(void) { DL_GPIO_setPins(OLED_I2C_PORT, OLED_I2C_SDA_PIN); }
static inline void I2C_SDA_L(void) { DL_GPIO_clearPins(OLED_I2C_PORT, OLED_I2C_SDA_PIN); }
static inline void I2C_SCL_H(void) { DL_GPIO_setPins(OLED_I2C_PORT, OLED_I2C_SCL_PIN); }
static inline void I2C_SCL_L(void) { DL_GPIO_clearPins(OLED_I2C_PORT, OLED_I2C_SCL_PIN); }

// SDA 切换为输入（读取模式）
static void I2C_SDA_Input(void)
{
    DL_GPIO_disableOutput(OLED_I2C_PORT, OLED_I2C_SDA_PIN);
    DL_GPIO_initDigitalInputFeatures(OLED_I2C_SDA_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
}

// SDA 切换回输出模式
static void I2C_SDA_Output(void)
{
    DL_GPIO_initDigitalOutput(OLED_I2C_SDA_IOMUX);
    DL_GPIO_enableOutput(OLED_I2C_PORT, OLED_I2C_SDA_PIN);
}

static inline uint8_t I2C_SDA_Read(void)
{
    return (DL_GPIO_readPins(OLED_I2C_PORT, OLED_I2C_SDA_PIN) != 0) ? 1 : 0;
}

static inline void I2C_Delay(void)
{
    // 80MHz 下约 2.5μs，I2C 100kHz 足够
    delay_cycles(200);
}

// I2C 起始条件
static void I2C_Start(void)
{
    I2C_SDA_Output();
    I2C_SDA_H();
    I2C_SCL_H();
    I2C_Delay();
    I2C_SDA_L();
    I2C_Delay();
    I2C_SCL_L();
    I2C_Delay();
}

// I2C 停止条件
static void I2C_Stop(void)
{
    I2C_SDA_Output();
    I2C_SDA_L();
    I2C_SCL_H();
    I2C_Delay();
    I2C_SDA_H();
    I2C_Delay();
}

// I2C 发送一个字节，返回 ACK (0=ACK, 1=NACK)
static uint8_t I2C_WriteByte(uint8_t dat)
{
    uint8_t i;
    I2C_SDA_Output();
    for (i = 0; i < 8; i++) {
        if (dat & 0x80) I2C_SDA_H();
        else            I2C_SDA_L();
        dat <<= 1;
        I2C_SCL_H();
        I2C_Delay();
        I2C_SCL_L();
        I2C_Delay();
    }
    // 读 ACK
    I2C_SDA_Input();
    I2C_SCL_H();
    I2C_Delay();
    uint8_t ack = I2C_SDA_Read();
    I2C_SCL_L();
    I2C_Delay();
    return ack;
}

// I2C 总线恢复：发 9 个时钟脉冲释放 SDA
static void I2C_Recovery(void)
{
    uint8_t i;
    I2C_SDA_Output();
    I2C_SDA_H();
    for (i = 0; i < 9; i++) {
        I2C_SCL_H();
        I2C_Delay();
        I2C_SCL_L();
        I2C_Delay();
    }
    I2C_Stop();
}

void OLED_WR_Byte(uint8_t dat, uint8_t mode)
{
    I2C_Start();
    I2C_WriteByte(0x78);            // OLED I2C 地址 (0x3C << 1)
    I2C_WriteByte(mode ? 0x40 : 0x00); // 控制字节: 0x00=命令, 0x40=数据
    I2C_WriteByte(dat);
    I2C_Stop();
}

//开启OLED显示 
void OLED_DisPlay_On(void)
{
	OLED_WR_Byte(0x8D,OLED_CMD);//电荷泵使能
	OLED_WR_Byte(0x14,OLED_CMD);//开启电荷泵
	OLED_WR_Byte(0xAF,OLED_CMD);//点亮屏幕
}

//关闭OLED显示 
void OLED_DisPlay_Off(void)
{
	OLED_WR_Byte(0x8D,OLED_CMD);//电荷泵使能
	OLED_WR_Byte(0x10,OLED_CMD);//关闭电荷泵
	OLED_WR_Byte(0xAF,OLED_CMD);//关闭屏幕
}

//更新显存到OLED	
void OLED_Refresh(void)
{
	uint8_t i,n;
	for(i=0;i<8;i++)
	{
	   OLED_WR_Byte(0xb0+i,OLED_CMD); //设置行起始地址
	   OLED_WR_Byte(0x00,OLED_CMD);   //设置低列起始地址
	   OLED_WR_Byte(0x10,OLED_CMD);   //设置高列起始地址
	   for(n=0;n<128;n++)
		 OLED_WR_Byte(OLED_GRAM[n][i],OLED_DATA);
	}
}

//清屏函数
void OLED_Clear(void)
{
	uint8_t i,n;
	for(i=0;i<8;i++)
	{
	   for(n=0;n<128;n++)
		{
			 OLED_GRAM[n][i]=0;//清除所有数据
		}
	}
	OLED_Refresh();//更新显示
}

//画点 
void OLED_DrawPoint(uint8_t x,uint8_t y)
{
	uint8_t i,m,n;
	i=y/8;
	m=y%8;
	n=1<<m;
	OLED_GRAM[x][i]|=n;
}

//清除一个点
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

//画线
void OLED_DrawLine(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2)
{
	uint8_t i,k,k1,k2;
	if((x1<0)||(x2>128)||(y1<0)||(y2>64)||(x1>x2)||(y1>y2))return;
	if(x1==x2)    //画竖线
	{
		for(i=0;i<(y2-y1);i++) OLED_DrawPoint(x1,y1+i);
	}
	else if(y1==y2)   //画横线
	{
		for(i=0;i<(x2-x1);i++) OLED_DrawPoint(x1+i,y1);
	}
	else      //画斜线
	{
		k1=y2-y1;
		k2=x2-x1;
		k=k1*10/k2;
		for(i=0;i<(x2-x1);i++) OLED_DrawPoint(x1+i,y1+i*k/10);
	}
}

//画圆
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

//显示字符
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

//显示字符串
void OLED_ShowString(uint8_t x,uint8_t y,uint8_t *chr,uint8_t size1)
{
	while((*chr>=' ')&&(*chr<='~'))
	{
		OLED_ShowChar(x,y,*chr,size1);
		x+=size1/2;
		if(x>128-size1)  //换行
		{
			x=0;
			y+=size1; // 修复了原代码的y+=2的bug
		}
		chr++;
	}
}

//m^n
uint32_t OLED_Pow(uint8_t m,uint8_t n)
{
	uint32_t result=1;
	while(n--) result*=m;
	return result;
}

//显示数字
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

//显示汉字
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

//配置写入数据的起始位置
void OLED_WR_BP(uint8_t x,uint8_t y)
{
	OLED_WR_Byte(0xb0+y,OLED_CMD);//设置行起始地址
	OLED_WR_Byte(((x&0xf0)>>4)|0x10,OLED_CMD);
	OLED_WR_Byte((x&0x0f)|0x01,OLED_CMD);
}

//显示图片
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

//OLED的初始化
void OLED_Init(void)
{
	// 先恢复 I2C 总线（防止复位后 SDA 被 OLED 拉低卡住）
	I2C_Recovery();

	// 4针OLED没有RST引脚，直接延时等待屏幕内部RC电路上电复位完成
	delay_ms(100);
	
	OLED_WR_Byte(0xAE,OLED_CMD);//--turn off oled panel
	OLED_WR_Byte(0x00,OLED_CMD);//---set low column address
	OLED_WR_Byte(0x10,OLED_CMD);//---set high column address
	OLED_WR_Byte(0x40,OLED_CMD);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
	OLED_WR_Byte(0x81,OLED_CMD);//--set contrast control register
	OLED_WR_Byte(0xCF,OLED_CMD);// Set SEG Output Current Brightness
	OLED_WR_Byte(0xA1,OLED_CMD);//--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
	OLED_WR_Byte(0xC8,OLED_CMD);//Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
	OLED_WR_Byte(0xA6,OLED_CMD);//--set normal display
	OLED_WR_Byte(0xA8,OLED_CMD);//--set multiplex ratio(1 to 64)
	OLED_WR_Byte(0x3f,OLED_CMD);//--1/64 duty
	OLED_WR_Byte(0xD3,OLED_CMD);//-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
	OLED_WR_Byte(0x00,OLED_CMD);//-not offset
	OLED_WR_Byte(0xd5,OLED_CMD);//--set display clock divide ratio/oscillator frequency
	OLED_WR_Byte(0x80,OLED_CMD);//--set divide ratio, Set Clock as 100 Frames/Sec
	OLED_WR_Byte(0xD9,OLED_CMD);//--set pre-charge period
	OLED_WR_Byte(0xF1,OLED_CMD);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
	OLED_WR_Byte(0xDA,OLED_CMD);//--set com pins hardware configuration
	OLED_WR_Byte(0x12,OLED_CMD);
	OLED_WR_Byte(0xDB,OLED_CMD);//--set vcomh
	OLED_WR_Byte(0x40,OLED_CMD);//Set VCOM Deselect Level
	OLED_WR_Byte(0x20,OLED_CMD);//-Set Page Addressing Mode (0x00/0x01/0x02)
	OLED_WR_Byte(0x02,OLED_CMD);//
	OLED_WR_Byte(0x8D,OLED_CMD);//--set Charge Pump enable/disable
	OLED_WR_Byte(0x14,OLED_CMD);//--set(0x10) disable
	OLED_WR_Byte(0xA4,OLED_CMD);// Disable Entire Display On (0xa4/0xa5)
	OLED_WR_Byte(0xA6,OLED_CMD);// Disable Inverse Display On (0xa6/a7) 
	OLED_WR_Byte(0xAF,OLED_CMD);
	OLED_Clear();
}