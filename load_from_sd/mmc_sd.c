#include "mmc_sd.h"			   


//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供學習使用，未經作者許可，不得用於其它任何用途
//ALIENTEK MiniSTM32開發板
//SD卡 驅動代碼	   
//正點原子@ALIENTEK
//技術論壇:www.openedv.com
//修改日期:2014/3/13
//版本：V1.0
//版權所有，盜版必究。
//Copyright(C) 廣州市星翼電子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////
//
void spi1_set_speed(u8 prescaler)
{
#ifdef STM32F407
  SPI1->CR1 &= 0xffc7;
  SPI1->CR1 |= prescaler;
  SPI_Cmd(SPI1, ENABLE);
#endif
}

uint8_t SPI1_ReadWriteByte(uint8_t data)
{
#ifdef STM32F407
  SPI1->DR = data; // write data to be transmitted to the SPI data register
  while( !(SPI1->SR & SPI_I2S_FLAG_TXE) ); // wait until transmit complete
  while( !(SPI1->SR & SPI_I2S_FLAG_RXNE) ); // wait until receive complete
  while( SPI1->SR & SPI_I2S_FLAG_BSY ); // wait until SPI is not busy anymore
  return SPI1->DR; // return received data from SPI data register
#endif
}
					   
u8  SD_Type=0;//SD卡的類型 
////////////////////////////////////移植修改區///////////////////////////////////
//移植時候的介面
//data:要寫入的數據
//返回值:讀到的數據
u8 SD_SPI_ReadWriteByte(u8 data)
{
#ifdef STM32F407
	return SPI1_ReadWriteByte(data);
#endif
}	  
//SD卡初始化的時候,需要低速
void SD_SPI_SpeedLow(void)
{
#ifdef STM32F407
  //SPI1_SetSpeed(SPI_BaudRatePrescaler_256);//設置到低速模式	
  spi1_set_speed(SPI_BaudRatePrescaler_256);
#endif
}
//SD卡正常工作的時候,可以高速了
void SD_SPI_SpeedHigh(void)
{
#ifdef STM32F407
  //SPI1_SetSpeed(SPI_BaudRatePrescaler_2);//設置到高速模式	
  spi1_set_speed(SPI_BaudRatePrescaler_2);
#endif
}

void init_spi1(void)
{
#ifdef STM32F407
  GPIO_InitTypeDef GPIO_InitStruct;
  SPI_InitTypeDef SPI_InitStruct;
	
  // enable clock for used IO pins
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  
  /* configure pins used by SPI1
   * PA5 = SCK
   * PA6 = MISO
   * PA7 = MOSI
   */
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6 | GPIO_Pin_5;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStruct);
	
  // connect SPI1 pins to SPI alternate function
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
  
  // enable clock for used IO pins
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
  
  /* Configure the chip select pin
     in this case we will use PA4 */
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  GPIOA->BSRRL |= GPIO_Pin_4; // set PA4 high

  // enable peripheral clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
  
  /* configure SPI1 in Mode 0 
   * CPOL = 0 --> clock is low when idle
   * CPHA = 0 --> data is sampled at the first edge
   */
  SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // set to full duplex mode, seperate MOSI and MISO lines
  SPI_InitStruct.SPI_Mode = SPI_Mode_Master;     // transmit in master mode, NSS pin has to be always high
  SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b; // one packet of data is 8 bits wide
  SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;        // clock is low when idle
  SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;      // data sampled at first edge
  //SPI_InitStruct.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set; // set the NSS management to internal and pull internal NSS high
  SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256; // SPI frequency is APB2 frequency / 4
  SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;// data is transmitted MSB first
  SPI_InitStruct.SPI_CRCPolynomial = 7;

  SPI_Init(SPI1, &SPI_InitStruct); 
  
  SPI_Cmd(SPI1, ENABLE); // enable SPI1
#endif
}


//SPI硬件層初始化
void SD_SPI_Init(void)
{
#ifdef STM32F407
  //設置硬件上與SD卡相關聯的控制引腳輸出
	//禁止其他外設(NRF/W25Q64)對SD卡產生影響

#if 0
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA, ENABLE );	 //PORTA時鐘使能 

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4;//PA2.3.4 推輓 	n_3|GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_SetBits(GPIOA,GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4);//PA2.3.4上拉 

  SPI1_Init();
#endif
  init_spi1();
	//SD_CS=1;
  SD_CS(1);
#endif
}
///////////////////////////////////////////////////////////////////////////////////
//取消選擇,釋放SPI匯流排
void SD_DisSelect(void)
{
#ifdef STM32F407
	SD_CS(1);
 	SD_SPI_ReadWriteByte(0xff);//提供額外的8個時鐘
#endif
}
//選擇sd卡,並且等待卡準備OK
//返回值:0,成功;1,失敗;
u8 SD_Select(void)
{
#ifdef STM32F407
	SD_CS(0);
	if(SD_WaitReady()==0)return 0;//等待成功
	SD_DisSelect();
#endif
	return 1;//等待失敗
}
//等待卡準備好
//返回值:0,準備好了;其他,錯誤代碼
u8 SD_WaitReady(void)
{
	u32 t=0;
	do
	{
		if(SD_SPI_ReadWriteByte(0XFF)==0XFF)return 0;//OK
		t++;		  	
	}while(t<0XFFFFFF);//等待 
	return 1;
}
//等待SD卡回應
//Response:要得到的回應值
//返回值:0,成功得到了該回應值
//    其他,得到回應值失敗
u8 SD_GetResponse(u8 Response)
{
	u16 Count=0xFFFF;//等待次數	   						  
	while ((SD_SPI_ReadWriteByte(0XFF)!=Response)&&Count)Count--;//等待得到準確的回應  	  
	if (Count==0)return MSD_RESPONSE_FAILURE;//得到回應失敗   
	else return MSD_RESPONSE_NO_ERROR;//正確回應
}
//從sd卡讀取一個數據包的內容
//buf:數據緩存區
//len:要讀取的數據長度.
//返回值:0,成功;其他,失敗;	
u8 SD_RecvData(u8*buf,u16 len)
{			  	  
	if(SD_GetResponse(0xFE))return 1;//等待SD卡發回數據起始令牌0xFE
    while(len--)//開始接收數據
    {
        *buf=SPI1_ReadWriteByte(0xFF);
        buf++;
    }
    //下面是2個偽CRC（dummy CRC）
    SD_SPI_ReadWriteByte(0xFF);
    SD_SPI_ReadWriteByte(0xFF);									  					    
    return 0;//讀取成功
}
//向sd卡寫入一個數據包的內容 512位元組
//buf:數據緩存區
//cmd:指令
//返回值:0,成功;其他,失敗;	
u8 SD_SendBlock(u8*buf,u8 cmd)
{	
	u16 t;		  	  
	if(SD_WaitReady())return 1;//等待準備失效
	SD_SPI_ReadWriteByte(cmd);
	if(cmd!=0XFD)//不是結束指令
	{
		for(t=0;t<512;t++)SPI1_ReadWriteByte(buf[t]);//提高速度,減少函數傳參時間
	    SD_SPI_ReadWriteByte(0xFF);//忽略crc
	    SD_SPI_ReadWriteByte(0xFF);
		t=SD_SPI_ReadWriteByte(0xFF);//接收響應
		if((t&0x1F)!=0x05)return 2;//響應錯誤									  					    
	}						 									  					    
    return 0;//寫入成功
}

//向SD卡發送一個命令
//輸入: u8 cmd   命令 
//      u32 arg  命令參數
//      u8 crc   crc校驗值	   
//返回值:SD卡返回的響應															  
u8 SD_SendCmd(u8 cmd, u32 arg, u8 crc)
{
    u8 r1;	
	u8 Retry=0; 
	SD_DisSelect();//取消上次片選
	if(SD_Select())return 0XFF;//片選失效 
	//發送
    SD_SPI_ReadWriteByte(cmd | 0x40);//分別寫入命令
    SD_SPI_ReadWriteByte(arg >> 24);
    SD_SPI_ReadWriteByte(arg >> 16);
    SD_SPI_ReadWriteByte(arg >> 8);
    SD_SPI_ReadWriteByte(arg);	  
    SD_SPI_ReadWriteByte(crc); 
	if(cmd==CMD12)SD_SPI_ReadWriteByte(0xff);//Skip a stuff byte when stop reading
    //等待響應，或超時退出
	Retry=0X1F;
	do
	{
		r1=SD_SPI_ReadWriteByte(0xFF);
	}while((r1&0X80) && Retry--);	 
	//返回狀態值
    return r1;
}		    																			  
//獲取SD卡的CID信息，包括製造商信息
//輸入: u8 *cid_data(存放CID的內存，至少16Byte）	  
//返回值:0：NO_ERR
//		 1：錯誤														   
u8 SD_GetCID(u8 *cid_data)
{
    u8 r1;	   
    //發CMD10命令，讀CID
    r1=SD_SendCmd(CMD10,0,0x01);
    if(r1==0x00)
	{
		r1=SD_RecvData(cid_data,16);//接收16個位元組的數據	 
    }
	SD_DisSelect();//取消片選
	if(r1)return 1;
	else return 0;
}																				  
//獲取SD卡的CSD信息，包括容量和速度信息
//輸入:u8 *cid_data(存放CID的內存，至少16Byte）	    
//返回值:0：NO_ERR
//		 1：錯誤														   
u8 SD_GetCSD(u8 *csd_data)
{
    u8 r1;	 
    r1=SD_SendCmd(CMD9,0,0x01);//發CMD9命令，讀CSD
    if(r1==0)
	{
    	r1=SD_RecvData(csd_data, 16);//接收16個位元組的數據 
    }
	SD_DisSelect();//取消片選
	if(r1)return 1;
	else return 0;
}  
//獲取SD卡的總扇區數（扇區數）   
//返回值:0： 取容量出錯 
//       其他:SD卡的容量(扇區數/512位元組)
//每扇區的位元組數必為512，因為如果不是512，則初始化不能通過.														  
u32 SD_GetSectorCount(void)
{
    u8 csd[16];
    u32 Capacity;  
    u8 n;
	u16 csize;  					    
	//取CSD信息，如果期間出錯，返回0
    if(SD_GetCSD(csd)!=0) return 0;	    
    //如果為SDHC卡，按照下面方式計算
    if((csd[0]&0xC0)==0x40)	 //V2.00的卡
    {	
		csize = csd[9] + ((u16)csd[8] << 8) + 1;
		Capacity = (u32)csize << 10;//得到扇區數	 		   
    }else//V1.XX的卡
    {	
		n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
		csize = (csd[8] >> 6) + ((u16)csd[7] << 2) + ((u16)(csd[6] & 3) << 10) + 1;
		Capacity= (u32)csize << (n - 9);//得到扇區數   
    }
    return Capacity;
}
//初始化SD卡
u8 SD_Initialize(void)
{
    u8 r1;      // 存放SD卡的返回值
    u16 retry;  // 用來進行超時計數
    u8 buf[4];  
	u16 i;

	SD_SPI_Init();		//初始化IO
 	SD_SPI_SpeedLow();	//設置到低速模式 
 	for(i=0;i<10;i++)SD_SPI_ReadWriteByte(0XFF);//發送最少74個脈衝
	retry=20;
	do
	{
		r1=SD_SendCmd(CMD0,0,0x95);//進入IDLE狀態
	}while((r1!=0X01) && retry--);
 	SD_Type=0;//預設無卡
	if(r1==0X01)
	{
		if(SD_SendCmd(CMD8,0x1AA,0x87)==1)//SD V2.0
		{
			for(i=0;i<4;i++)buf[i]=SD_SPI_ReadWriteByte(0XFF);	//Get trailing return value of R7 resp
			if(buf[2]==0X01&&buf[3]==0XAA)//卡是否支持2.7~3.6V
			{
				retry=0XFFFE;
				do
				{
					SD_SendCmd(CMD55,0,0X01);	//發送CMD55
					r1=SD_SendCmd(CMD41,0x40000000,0X01);//發送CMD41
				}while(r1&&retry--);
				if(retry&&SD_SendCmd(CMD58,0,0X01)==0)//鑒別SD2.0卡版本開始
				{
					for(i=0;i<4;i++)buf[i]=SD_SPI_ReadWriteByte(0XFF);//得到OCR值
					if(buf[0]&0x40)SD_Type=SD_TYPE_V2HC;    //檢查CCS
					else SD_Type=SD_TYPE_V2;   
				}
			}
		}else//SD V1.x/ MMC	V3
		{
			SD_SendCmd(CMD55,0,0X01);		//發送CMD55
			r1=SD_SendCmd(CMD41,0,0X01);	//發送CMD41
			if(r1<=1)
			{		
				SD_Type=SD_TYPE_V1;
				retry=0XFFFE;
				do //等待退出IDLE模式
				{
					SD_SendCmd(CMD55,0,0X01);	//發送CMD55
					r1=SD_SendCmd(CMD41,0,0X01);//發送CMD41
				}while(r1&&retry--);
			}else//MMC卡不支持CMD55+CMD41識別
			{
				SD_Type=SD_TYPE_MMC;//MMC V3
				retry=0XFFFE;
				do //等待退出IDLE模式
				{											    
					r1=SD_SendCmd(CMD1,0,0X01);//發送CMD1
				}while(r1&&retry--);  
			}
			if(retry==0||SD_SendCmd(CMD16,512,0X01)!=0)SD_Type=SD_TYPE_ERR;//錯誤的卡
		}
	}
	SD_DisSelect();//取消片選
	//SD_SPI_SpeedHigh();//高速
	if(SD_Type)return 0;
	else if(r1)return r1; 	   
	return 0xaa;//其他錯誤
}

#ifndef STM32F407
u8 disk_image_read(u8 *buf, u32 sector, u8 cnt, FILE *fst)
{
  u32 read_len = cnt * 512;
  u32 rlen = 0;
  printf("read sector: %d, read_len: %d\n", sector, read_len);

  fseek(fst, sector * 512 , SEEK_SET);
  rlen = fread(buf, 1, read_len, fst);
  printf("rlen: %d\n", rlen);
  //print_packet(buf, 512);
  return 0;
}
#endif

//讀SD卡
//buf:數據緩存區
//sector:扇區
//cnt:扇區數
//返回值:0,ok;其他,失敗.
u8 SD_ReadDisk(u8*buf,u32 sector,u8 cnt)
{
	u8 r1;
	if(SD_Type!=SD_TYPE_V2HC)sector <<= 9;//轉換為位元組地址
	if(cnt==1)
	{
		r1=SD_SendCmd(CMD17,sector,0X01);//讀命令
		if(r1==0)//指令發送成功
		{
			r1=SD_RecvData(buf,512);//接收512個位元組	   
		}
	}else
	{
		r1=SD_SendCmd(CMD18,sector,0X01);//連續讀命令
		do
		{
			r1=SD_RecvData(buf,512);//接收512個位元組	 
			buf+=512;  
		}while(--cnt && r1==0); 	
		SD_SendCmd(CMD12,0,0X01);	//發送停止命令
	}   
	SD_DisSelect();//取消片選
	return r1;//
}
//寫SD卡
//buf:數據緩存區
//sector:起始扇區
//cnt:扇區數
//返回值:0,ok;其他,失敗.
u8 SD_WriteDisk(u8*buf,u32 sector,u8 cnt)
{
	u8 r1;
	if(SD_Type!=SD_TYPE_V2HC)sector *= 512;//轉換為位元組地址
	if(cnt==1)
	{
		r1=SD_SendCmd(CMD24,sector,0X01);//讀命令
		if(r1==0)//指令發送成功
		{
			r1=SD_SendBlock(buf,0xFE);//寫512個位元組	   
		}
	}else
	{
		if(SD_Type!=SD_TYPE_MMC)
		{
			SD_SendCmd(CMD55,0,0X01);	
			SD_SendCmd(CMD23,cnt,0X01);//發送指令	
		}
 		r1=SD_SendCmd(CMD25,sector,0X01);//連續讀命令
		if(r1==0)
		{
			do
			{
				r1=SD_SendBlock(buf,0xFC);//接收512個位元組	 
				buf+=512;  
			}while(--cnt && r1==0);
			r1=SD_SendBlock(0,0xFD);//接收512個位元組 
		}
	}   
	SD_DisSelect();//取消片選
	return r1;//
}	

