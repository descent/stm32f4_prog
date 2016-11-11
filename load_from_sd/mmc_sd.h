#ifndef _MMC_SD_H_
#define _MMC_SD_H_		 

#include "stm32f4xx_usart.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_gpio.h"

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
 						    	 
// SD卡類型定義  
#define SD_TYPE_ERR     0X00
#define SD_TYPE_MMC     0X01
#define SD_TYPE_V1      0X02
#define SD_TYPE_V2      0X04
#define SD_TYPE_V2HC    0X06	   
// SD卡指令表  	   
#define CMD0    0       //卡複位
#define CMD1    1
#define CMD8    8       //命令8 ，SEND_IF_COND
#define CMD9    9       //命令9 ，讀CSD數據
#define CMD10   10      //命令10，讀CID數據
#define CMD12   12      //命令12，停止數據傳輸
#define CMD16   16      //命令16，設置SectorSize 應返回0x00
#define CMD17   17      //命令17，讀sector
#define CMD18   18      //命令18，讀Multi sector
#define CMD23   23      //命令23，設置多sector寫入前預先擦除N個block
#define CMD24   24      //命令24，寫sector
#define CMD25   25      //命令25，寫Multi sector
#define CMD41   41      //命令41，應返回0x00
#define CMD55   55      //命令55，應返回0x01
#define CMD58   58      //命令58，讀OCR信息
#define CMD59   59      //命令59，使能/禁止CRC，應返回0x00
//數據寫入回應字意義
#define MSD_DATA_OK                0x05
#define MSD_DATA_CRC_ERROR         0x0B
#define MSD_DATA_WRITE_ERROR       0x0D
#define MSD_DATA_OTHER_ERROR       0xFF
//SD卡回應標記字
#define MSD_RESPONSE_NO_ERROR      0x00
#define MSD_IN_IDLE_STATE          0x01
#define MSD_ERASE_RESET            0x02
#define MSD_ILLEGAL_COMMAND        0x04
#define MSD_COM_CRC_ERROR          0x08
#define MSD_ERASE_SEQUENCE_ERROR   0x10
#define MSD_ADDRESS_ERROR          0x20
#define MSD_PARAMETER_ERROR        0x40
#define MSD_RESPONSE_FAILURE       0xFF
 							   						 	 
//這部分應根據具體的連線來修改!
//MiniSTM32開發板使用的是PA3作為SD卡的CS腳.
//#define	SD_CS  PAout(3) 	//SD卡片選引腳					    	  
#define	SD_CS(value) \
if (value == 0) \
  /* set PA4 (CS) low  */ \
  GPIOA->BSRRH |= GPIO_Pin_4; \
else \
  GPIOA->BSRRL |= GPIO_Pin_4; // set PA4 (CS) high



extern u8  SD_Type;			//SD卡的類型
//函數申明區 
u8 SD_SPI_ReadWriteByte(u8 data);
void SD_SPI_SpeedLow(void);
void SD_SPI_SpeedHigh(void);
u8 SD_WaitReady(void);							//等待SD卡準備
u8 SD_GetResponse(u8 Response);					//獲得相應
u8 SD_Initialize(void);							//初始化
u8 SD_ReadDisk(u8*buf,u32 sector,u8 cnt);		//讀塊
u8 SD_WriteDisk(u8*buf,u32 sector,u8 cnt);		//寫塊
u32 SD_GetSectorCount(void);   					//讀扇區數
u8 SD_GetCID(u8 *cid_data);                     //讀SD卡CID
u8 SD_GetCSD(u8 *csd_data);                     //讀SD卡CSD
 
#endif

