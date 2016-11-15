/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2013        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control module to the FatFs module with a defined API.        */
/*-----------------------------------------------------------------------*/

#include "diskio.h"		/* FatFs lower layer API */
//#include "flash.h"
//#include "malloc.h"		

#include "mmc_sd.h"
#include <stdlib.h>

#ifndef STM32F407
#include <stdio.h>
#endif

#define mymalloc malloc
#define myfree free

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供學習使用，未經作者許可，不得用於其它任何用途
//ALIENTEK STM32開發板
//FATFS disio.c 驅動代碼	   
//正點原子@ALIENTEK
//技術論壇:www.openedv.com
//修改日期:2014/3/14
//版本：V1.0
//版權所有，盜版必究。
//Copyright(C) 廣州市星翼電子科技有限公司 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 

#define SD_CARD	 0  //SD卡,卷標為0
#define EX_FLASH 1	//外部flash,卷標為1
#define FILE_IMAGE_01 2
#define FILE_IMAGE_02 3

#define FLASH_SECTOR_SIZE 	512			  
//對於W25Q64 
//前4.8M位元組給fatfs用,4.8M位元組後~4.8M+100K給用戶用,4.9M以後,用於存放字型檔,字型檔占用3.09M.		 			    
u16	    FLASH_SECTOR_COUNT= 9832;	//4.8M位元組,預設為W25Q64
#define FLASH_BLOCK_SIZE   	8     	//每個BLOCK有8個扇區

FILE *fs_01=0;
FILE *fs2=0;

//初始化磁碟
DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber (0..) */
)
{
  printf("yyy");
	u8 res=0;	    
	switch(pdrv)
	{
		case SD_CARD://SD卡
                  printf("init sd card\n");
#ifdef STM32F407
			res = SD_Initialize();//SD_Initialize() 
		 	if(res)//STM32 SPI的bug,在sd卡操作失敗的時候如果不執行下面的語句,可能導致SPI讀寫異常
			{
				SD_SPI_SpeedLow();
				SD_SPI_ReadWriteByte(0xff);//提供額外的8個時鐘
				SD_SPI_SpeedHigh();
			}
#endif
  			break;
#if 0
		case EX_FLASH://外部flash
			SPI_Flash_Init();
			if(SPI_FLASH_TYPE==W25Q64)FLASH_SECTOR_COUNT=9832;	//W25Q64
			else FLASH_SECTOR_COUNT=0;							//其他
 			break;
#endif
                case FILE_IMAGE_01:
		//case EX_FLASH://外部flash
                {
#ifndef STM32F407
  printf("FILE_IMAGE_01\n");
  const char *fn = "fat32.img";

  if (fs_01 == 0 )
  {
    fs_01 = fopen(fn, "r");
    if (fs_01 == NULL)
    {
      perror("open imagefile.img error\n");
      exit(1);
    }
    printf("the 1st open %s ok\n", fn);
  }
  else
  {
    printf("already open %s ok\n", fn);
  }

  return 0;
#endif
                  break;
                }
                case FILE_IMAGE_02:
                {
#ifndef STM32F407
  const char *fn = "fat12.img";

  if (fs2 == 0 )
  {
  fs2 = fopen(fn, "r");
  if (fs2 == NULL)
  {
    perror("open fat12.img error\n");
    exit(1);
  }
  printf("open %s ok\n", fn);
  }
  else
  {
    printf("%s already open\n", fn);
  }

  return 0;
#endif
                }
		default:
			res=1; 
	}		 
	if(res)return  STA_NOINIT;
	else return 0; //初始化成功
}  

//獲得磁碟狀態
DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber (0..) */
)
{ 
	return 0;
} 

//讀扇區
//drv:磁碟編號0~9
//*buff:數據接收緩衝首地址
//sector:扇區地址
//count:需要讀取的扇區數
DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address (LBA) */
	UINT count		/* Number of sectors to read (1..128) */
)
{
	u8 res=0; 
    if (!count)return RES_PARERR;//count不能等於0，否則返回參數錯誤		 	 
	switch(pdrv)
	{
		case SD_CARD://SD卡
                        #ifdef STM32F407
			res=SD_ReadDisk(buff,sector,count);	 
		 	if(res)//STM32 SPI的bug,在sd卡操作失敗的時候如果不執行下面的語句,可能導致SPI讀寫異常
			{
				SD_SPI_SpeedLow();
				SD_SPI_ReadWriteByte(0xff);//提供額外的8個時鐘
				SD_SPI_SpeedHigh();
			}
                        #else
                        #endif


			break;
                #if 0
		case EX_FLASH://外部flash
			for(;count>0;count--)
			{
				SPI_Flash_Read(buff,sector*FLASH_SECTOR_SIZE,FLASH_SECTOR_SIZE);
				sector++;
				buff+=FLASH_SECTOR_SIZE;
			}
			res=0;
			break;
                #endif
                case FILE_IMAGE_01:
                {
                        printf("call disk_image_read()\n");
                        disk_image_read(buff, sector, count, fs_01);
                        printf("call disk_image_read() ok\n");
                        return RES_OK;	 
                  break;
                }
                case FILE_IMAGE_02:
                {
                        printf("call disk_image_read()\n");
                        disk_image_read(buff, sector, count, fs2);
                        printf("call disk_image_read() ok\n");
                        return RES_OK;	 
                  break;
                }
		default:
			res=1; 
	}
   //處理返回值，將SPI_SD_driver.c的返回值轉成ff.c的返回值
    if(res==0x00)return RES_OK;	 
    else return RES_ERROR;	   
}

//寫扇區
//drv:磁碟編號0~9
//*buff:發送數據首地址
//sector:扇區地址
//count:需要寫入的扇區數
#if _USE_WRITE
DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber (0..) */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Sector address (LBA) */
	UINT count			/* Number of sectors to write (1..128) */
)
{
	u8 res=0;  
    if (!count)return RES_PARERR;//count不能等於0，否則返回參數錯誤		 	 
	switch(pdrv)
	{
		case SD_CARD://SD卡
			res=SD_WriteDisk((u8*)buff,sector,count);
			break;
                        #if 0
		case EX_FLASH://外部flash
			for(;count>0;count--)
			{										    
				SPI_Flash_Write((u8*)buff,sector*FLASH_SECTOR_SIZE,FLASH_SECTOR_SIZE);
				sector++;
				buff+=FLASH_SECTOR_SIZE;
			}
			res=0;
			break;
                        #endif
		default:
			res=1; 
	}
    //處理返回值，將SPI_SD_driver.c的返回值轉成ff.c的返回值
    if(res == 0x00)return RES_OK;	 
    else return RES_ERROR;	
}
#endif


//其他表參數的獲得
 //drv:磁碟編號0~9
 //ctrl:控制代碼
 //*buff:發送/接收緩衝區指針
#if _USE_IOCTL
DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
#ifdef STM32F407

	DRESULT res;						  			     
	if(pdrv==SD_CARD)//SD卡
	{
	    switch(cmd)
	    {
		    case CTRL_SYNC:
				//SD_CS=0;
                                SD_CS(0);

		        if(SD_WaitReady()==0)res = RES_OK; 
		        else res = RES_ERROR;	  
				//SD_CS=1;
				SD_CS(1);
		        break;	 
		    case GET_SECTOR_SIZE:
		        *(WORD*)buff = 512;
		        res = RES_OK;
		        break;	 
		    case GET_BLOCK_SIZE:
		        *(WORD*)buff = 8;
		        res = RES_OK;
		        break;	 
		    case GET_SECTOR_COUNT:
		        *(DWORD*)buff = SD_GetSectorCount();
		        res = RES_OK;
		        break;
		    default:
		        res = RES_PARERR;
		        break;
	    }
	}else if(pdrv==EX_FLASH)	//外部FLASH  
	{
	    switch(cmd)
	    {
		    case CTRL_SYNC:
				res = RES_OK; 
		        break;	 
		    case GET_SECTOR_SIZE:
		        *(WORD*)buff = FLASH_SECTOR_SIZE;
		        res = RES_OK;
		        break;	 
		    case GET_BLOCK_SIZE:
		        *(WORD*)buff = FLASH_BLOCK_SIZE;
		        res = RES_OK;
		        break;	 
		    case GET_SECTOR_COUNT:
		        *(DWORD*)buff = FLASH_SECTOR_COUNT;
		        res = RES_OK;
		        break;
		    default:
		        res = RES_PARERR;
		        break;
	    }
	}else res=RES_ERROR;//其他的不支持
    return res;
#endif
}
#endif
//獲得時間
//User defined function to give a current time to fatfs module      */
//31-25: Year(0-127 org.1980), 24-21: Month(1-12), 20-16: Day(1-31) */                                                                                                                                                                                                                                          
//15-11: Hour(0-23), 10-5: Minute(0-59), 4-0: Second(0-29 *2) */                                                                                                                                                                                                                                                
DWORD get_fattime (void)
{				 
	return 0;
}			 
//動態分配內存
void *ff_memalloc (UINT size)			
{
	return (void*)mymalloc(size);
}
//釋放內存
void ff_memfree (void* mf)		 
{
	myfree(mf);
}
