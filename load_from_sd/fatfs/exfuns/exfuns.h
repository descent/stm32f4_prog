#ifndef __EXFUNS_H
#define __EXFUNS_H 			   
#include "type.h"
#include "ff.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供學習使用，未經作者許可，不得用於其它任何用途
//ALIENTEK STM32開發板
//FATFS 擴展代碼	   
//正點原子@ALIENTEK
//技術論壇:www.openedv.com
//修改日期:2014/3/14
//版本：V1.0
//版權所有，盜版必究。
//Copyright(C) 廣州市星翼電子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////

extern FATFS *fs[2];  
extern FIL *file;	 
extern FIL *ftemp;	 
extern UINT br,bw;
extern FILINFO fileinfo;
extern DIR dir;
extern u8 *fatbuf;//SD卡數據緩存區


//f_typetell返回的類型定義
//根據表FILE_TYPE_TBL獲得.在exfuns.c裡面定義
#define T_BIN		0X00	//bin檔案
#define T_LRC		0X10	//lrc檔案
#define T_NES		0X20	//nes檔案
#define T_TEXT		0X30	//.txt檔案
#define T_C			0X31	//.c檔案
#define T_H			0X32    //.h檔案
#define T_FLAC		0X4C	//flac檔案
#define T_BMP		0X50	//bmp檔案
#define T_JPG		0X51	//jpg檔案
#define T_JPEG		0X52	//jpeg檔案		 
#define T_GIF		0X53	//gif檔案  

 
u8 exfuns_init(void);							//申請內存
u8 f_typetell(u8 *fname);						//識別檔案類型
u8 exf_getfree(u8 *drv,u32 *total,u32 *free);	//得到磁碟總容量和剩餘容量
u32 exf_fdsize(u8 *fdname);						//得到檔案夾大小			 																		   
#endif


