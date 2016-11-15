//#include "string.h"
#include <string.h>
#include "exfuns.h"
#include "fattester.h"	
//#include <stdlib.h>
//#include "malloc.h"
//#include "usart.h"
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

//#define mymalloc malloc

 //檔案類型列表
const u8 *FILE_TYPE_TBL[6][13]=
{
{"BIN"},			//BIN檔案
{"LRC"},			//LRC檔案
{"NES"},			//NES檔案
{"TXT","C","H"},	//文本檔案
{"MP1","MP2","MP3","MP4","M4A","3GP","3G2","OGG","ACC","WMA","WAV","MID","FLAC"},//音樂檔案
{"BMP","JPG","JPEG","GIF"},//圖片檔案
};
///////////////////////////////公共檔案區,使用malloc的時候////////////////////////////////////////////
FATFS *fs[_VOLUMES];  		//邏輯磁碟工作區.	 
FIL *file;	  		//檔案1
FIL *ftemp;	  		//檔案2.
UINT br,bw;			//讀寫變數
FILINFO fileinfo;	//檔案信息
DIR dir;  			//目錄

#define FIL_POOL_SIZE 5

u8 *fatbuf;			//SD卡數據緩存區
///////////////////////////////////////////////////////////////////////////////////////
//為exfuns申請內存
//返回值:0,成功
//1,失敗
u8 exfuns_init(void)
{
  static FATFS fs_pool[_VOLUMES];
  static FIL fil_pool[FIL_POOL_SIZE];
  static u8 fat_buf_poll[512];

  for (int i=0 ; i < _VOLUMES ; ++i)
  {
    //fs[i]=(FATFS*)mymalloc(sizeof(FATFS));
    fs[i] = &fs_pool[i];
    //printf("%d fs[i]: %p\n", i, fs[i]);
  }
  file = &fil_pool[0];		//為file申請內存
  ftemp = &fil_pool[1];		//為ftemp申請內存
  fatbuf = fat_buf_poll;		        //為fatbuf申請內存
  return 0;

	//fs[0]=(FATFS*)mymalloc(sizeof(FATFS));	//為磁碟0工作區申請內存	
	//fs[1]=(FATFS*)mymalloc(sizeof(FATFS));	//為磁碟1工作區申請內存
	//file=(FIL*)mymalloc(sizeof(FIL));		//為file申請內存
	//ftemp=(FIL*)mymalloc(sizeof(FIL));		//為ftemp申請內存


	//fatbuf = (u8*)mymalloc(512);		        //為fatbuf申請內存
	//if(fs[0]&&fs[1]&&file&&ftemp&&fatbuf)return 0;  //申請有一個失敗,即失敗.
	//else return 1;	
}


#if 0
//將小寫字母轉為大寫字母,如果是數字,則保持不變.
u8 char_upper(u8 c)
{
	if(c<'A')return c;//數字,保持不變.
	if(c>='a')return c-0x20;//變為大寫.
	else return c;//大寫,保持不變
}	      
//報告檔案的類型
//fname:檔案名
//返回值:0XFF,表示無法識別的檔案類型編號.
//		 其他,高四位表示所屬大類,低四位表示所屬小類.
u8 f_typetell(u8 *fname)
{
	u8 tbuf[5];
	u8 *attr='\0';//尾碼名
	u8 i=0,j;
	while(i<250)
	{
		i++;
		if(*fname=='\0')break;//偏移到了最後了.
		fname++;
	}
	if(i==250)return 0XFF;//錯誤的字元串.
 	for(i=0;i<5;i++)//得到尾碼名
	{
		fname--;
		if(*fname=='.')
		{
			fname++;
			attr=fname;
			break;
		}
  	}
	strcpy((char *)tbuf,(const char*)attr);//copy
 	for(i=0;i<4;i++)tbuf[i]=char_upper(tbuf[i]);//全部變為大寫 
	for(i=0;i<6;i++)
	{
		for(j=0;j<13;j++)
		{
			if(*FILE_TYPE_TBL[i][j]==0)break;//此組已經沒有可對比的成員了.
			if(strcmp((const char *)FILE_TYPE_TBL[i][j],(const char *)tbuf)==0)//找到了
			{
				return (i<<4)|j;
			}
		}
	}
	return 0XFF;//沒找到		 			   
}	 
#endif

//得到磁碟剩餘容量
//drv:磁碟編號("0:"/"1:")
//total:總容量	 （單位KB）
//free:剩餘容量	 （單位KB）
//返回值:0,正常.其他,錯誤代碼
u8 exf_getfree(u8 *drv,u32 *total,u32 *free)
{
	FATFS *fs1;
	u8 res;
    u32 fre_clust=0, fre_sect=0, tot_sect=0;
    //得到磁碟信息及空閒簇數量
    res =(u32)f_getfree((const TCHAR*)drv, (DWORD*)&fre_clust, &fs1);
    if(res==0)
	{											   
	    tot_sect=(fs1->n_fatent-2)*fs1->csize;	//得到總扇區數
	    fre_sect=fre_clust*fs1->csize;			//得到空閒扇區數	   
#if _MAX_SS!=512				  				//扇區大小不是512位元組,則轉換為512位元組
		tot_sect*=fs1->ssize/512;
		fre_sect*=fs1->ssize/512;
#endif	  
		*total=tot_sect>>1;	//單位為KB
		*free=fre_sect>>1;	//單位為KB 
 	}
	return res;
}		   
/////////////////////////////////////////////////////////////////////////////////////////////////////////////




















