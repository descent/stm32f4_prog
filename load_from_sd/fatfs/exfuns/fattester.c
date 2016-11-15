#include "fattester.h"	 
//#include "mmc_sd.h"
//#include "usmart.h"
#include "exfuns.h"
#include "ff.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供學習使用，未經作者許可，不得用於其它任何用途
//ALIENTEK STM32開發板
//FATFS 測試代碼	   
//正點原子@ALIENTEK
//技術論壇:www.openedv.com
//修改日期:2014/3/14
//版本：V1.0
//版權所有，盜版必究。
//Copyright(C) 廣州市星翼電子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////
    
//為磁碟註冊工作區	 
//path:磁碟路徑，比如"0:"、"1:"
//mt:0，不立即註冊（稍後註冊）；1，立即註冊
//返回值:執行結果
u8 mf_mount(u8* path,u8 mt)
{		   
	return f_mount(fs[0],(const TCHAR*)path,mt); 
}
//打開路徑下的檔案
//path:路徑+檔案名
//mode:打開模式
//返回值:執行結果
u8 mf_open(u8*path,u8 mode)
{
	u8 res;	 
	res=f_open(file,(const TCHAR*)path,mode);//打開檔案夾
	return res;
} 
//關閉檔案
//返回值:執行結果
u8 mf_close(void)
{
	f_close(file);
	return 0;
}
//讀出數據
//len:讀出的長度
//返回值:執行結果
u8 mf_read(u16 len)
{
	u16 i,t;
	u8 res=0;
	u16 tlen=0;
	printf("\r\nRead file data is:\r\n");
	for(i=0;i<len/512;i++)
	{
		res=f_read(file,fatbuf,512,&br);
		if(res)
		{
			printf("Read Error:%d\r\n",res);
			break;
		}else
		{
			tlen+=br;
			for(t=0;t<br;t++)printf("%c",fatbuf[t]); 
		}
	}
	if(len%512)
	{
		res=f_read(file,fatbuf,len%512,&br);
		if(res)	//讀數據出錯了
		{
			printf("\r\nRead Error:%d\r\n",res);   
		}else
		{
			tlen+=br;
			for(t=0;t<br;t++)printf("%c",fatbuf[t]); 
		}	 
	}
	if(tlen)printf("\r\nReaded data len:%d\r\n",tlen);//讀到的數據長度
	printf("Read data over\r\n");	 
	return res;
}
//寫入數據
//dat:數據緩存區
//len:寫入長度
//返回值:執行結果
u8 mf_write(u8*dat,u16 len)
{			    
	u8 res;	   					   

	printf("\r\nBegin Write file...\r\n");
	printf("Write data len:%d\r\n",len);	 
	res=f_write(file,dat,len,&bw);
	if(res)
	{
		printf("Write Error:%d\r\n",res);   
	}else printf("Writed data len:%d\r\n",bw);
	printf("Write data over.\r\n");
	return res;
}

//打開目錄
 //path:路徑
//返回值:執行結果
u8 mf_opendir(u8* path)
{
	return f_opendir(&dir,(const TCHAR*)path);	
}
//關閉目錄 
//返回值:執行結果
u8 mf_closedir(void)
{
	return f_closedir(&dir);	
}
//打讀取檔案夾
//返回值:執行結果
u8 mf_readdir(void)
{
	u8 res;
	char *fn;			 
#if _USE_LFN
  TCHAR long_name_pool[_MAX_LFN * 2 + 1];

  fileinfo.lfsize = _MAX_LFN * 2 + 1;
  fileinfo.lfname = long_name_pool;

  //fileinfo.lfsize = _MAX_LFN * 2 + 1;
  //fileinfo.lfname = mymalloc(fileinfo.lfsize);
#endif		  
	res=f_readdir(&dir,&fileinfo);//讀取一個檔案的信息
	if(res!=FR_OK||fileinfo.fname[0]==0)
	{
		//myfree(fileinfo.lfname);
		return res;//讀完了.
	}
#if _USE_LFN
	fn=*fileinfo.lfname ? fileinfo.lfname : fileinfo.fname;
#else
	fn=fileinfo.fname;;
#endif	
	printf("\r\n DIR info:\r\n");

	printf("dir.id:%d\r\n",dir.id);
	printf("dir.index:%d\r\n",dir.index);
	printf("dir.sclust:%d\r\n",dir.sclust);
	printf("dir.clust:%d\r\n",dir.clust);
	printf("dir.sect:%d\r\n",dir.sect);	  

	printf("\r\n");
	printf("File Name is:%s\r\n",fn);
	printf("File Size is:%d\r\n",fileinfo.fsize);
	printf("File data is:%d\r\n",fileinfo.fdate);
	printf("File time is:%d\r\n",fileinfo.ftime);
	printf("File Attr is:%d\r\n",fileinfo.fattrib);
	printf("\r\n");
	//myfree(fileinfo.lfname);
	return 0;
}			 

 //遍歷檔案
 //path:路徑
 //返回值:執行結果
u8 mf_scan_files(u8 * path)
{
	FRESULT res;	  
    char *fn;   /* This function is assuming non-Unicode cfg. */
#if _USE_LFN
  TCHAR long_name_pool[_MAX_LFN * 2 + 1];

  fileinfo.lfsize = _MAX_LFN * 2 + 1;
  fileinfo.lfname = long_name_pool;
 	//fileinfo.lfsize = _MAX_LFN * 2 + 1;
	//fileinfo.lfname = mymalloc(fileinfo.lfsize);
#endif		  

    res = f_opendir(&dir,(const TCHAR*)path); //打開一個目錄
    if (res == FR_OK) 
	{	
		printf("\r\n"); 
		while(1)
		{
	        res = f_readdir(&dir, &fileinfo);                   //讀取目錄下的一個檔案
	        if (res != FR_OK || fileinfo.fname[0] == 0) break;  //錯誤了/到末尾了,退出
	        //if (fileinfo.fname[0] == '.') continue;             //忽略上級目錄
#if _USE_LFN
        	fn = *fileinfo.lfname ? fileinfo.lfname : fileinfo.fname;
#else							   
        	fn = fileinfo.fname;
#endif	                                              /* It is a file. */
			printf("%s/", path);//打印路徑	
			printf("%s\r\n",  fn);//打印檔案名	  
		} 
    }	  
	//myfree(fileinfo.lfname);
    return res;	  
}
//顯示剩餘容量
//drv:盤符
//返回值:剩餘容量(位元組)
u32 mf_showfree(u8 *drv)
{
	FATFS *fs1;
	u8 res;
    u32 fre_clust=0, fre_sect=0, tot_sect=0;
    //得到磁碟信息及空閒簇數量
    res = f_getfree((const TCHAR*)drv,(DWORD*)&fre_clust, &fs1);
    if(res==0)
	{											   
	    tot_sect = (fs1->n_fatent - 2) * fs1->csize;//得到總扇區數
	    fre_sect = fre_clust * fs1->csize;			//得到空閒扇區數	   
#if _MAX_SS!=512
		tot_sect*=fs1->ssize/512;
		fre_sect*=fs1->ssize/512;
#endif	  
		if(tot_sect<20480)//總容量小於10M
		{
		    /* Print free space in unit of KB (assuming 512 bytes/sector) */
		    printf("\r\n磁碟總容量:%d KB\r\n"
		           "可用空間:%d KB\r\n",
		           tot_sect>>1,fre_sect>>1);
		}else
		{
		    /* Print free space in unit of KB (assuming 512 bytes/sector) */
		    printf("\r\n磁碟總容量:%d MB\r\n"
		           "可用空間:%d MB\r\n",
		           tot_sect>>11,fre_sect>>11);
		}
	}
	return fre_sect;
}		    
//檔案讀寫指針偏移
//offset:相對首地址的偏移量
//返回值:執行結果.
u8 mf_lseek(u32 offset)
{
	return f_lseek(file,offset);
}
//讀取檔案當前讀寫指針的位置.
//返回值:位置
u32 mf_tell(void)
{
	return f_tell(file);
}
//讀取檔案大小
//返回值:檔案大小
u32 mf_size(void)
{
	return f_size(file);
} 
//創建目錄
//pname:目錄路徑+名字
//返回值:執行結果
u8 mf_mkdir(u8*pname)
{
	return f_mkdir((const TCHAR *)pname);
}
//格式化
//path:磁碟路徑，比如"0:"、"1:"
//mode:模式
//au:簇大小
//返回值:執行結果
u8 mf_fmkfs(u8* path,u8 mode,u16 au)
{
	return f_mkfs((const TCHAR*)path,mode,au);//格式化,drv:盤符;mode:模式;au:簇大小
} 
//刪除檔案/目錄
//pname:檔案/目錄路徑+名字
//返回值:執行結果
u8 mf_unlink(u8 *pname)
{
	return  f_unlink((const TCHAR *)pname);
}

//修改檔案/目錄名字(如果目錄不同,還可以移動檔案哦!)
//oldname:之前的名字
//newname:新名字
//返回值:執行結果
u8 mf_rename(u8 *oldname,u8* newname)
{
	return  f_rename((const TCHAR *)oldname,(const TCHAR *)newname);
}
//獲取盤符（磁碟名字）
//path:磁碟路徑，比如"0:"、"1:"  
void mf_getlabel(u8 *path)
{
	u8 buf[20];
	u32 sn=0;
	u8 res;
	res=f_getlabel ((const TCHAR *)path,(TCHAR *)buf,(DWORD*)&sn);
	if(res==FR_OK)
	{
		printf("\r\n磁碟%s 的盤符為:%s\r\n",path,buf);
		printf("磁碟%s 的序列號:%X\r\n\r\n",path,sn); 
	}else printf("\r\n獲取失敗，錯誤碼:%X\r\n",res);
}
//設置盤符（磁碟名字），最長11個字元！！，支持數字和大寫字母組合以及漢字等
//path:磁碟號+名字，比如"0:ALIENTEK"、"1:OPENEDV"  
void mf_setlabel(u8 *path)
{
	u8 res;
	res=f_setlabel ((const TCHAR *)path);
	if(res==FR_OK)
	{
		printf("\r\n磁碟盤符設置成功:%s\r\n",path);
	}else printf("\r\n磁碟盤符設置失敗，錯誤碼:%X\r\n",res);
} 

//從檔案裡面讀取一段字元串
//size:要讀取的長度
void mf_gets(u16 size)
{
 	TCHAR* rbuf;
	rbuf=f_gets((TCHAR*)fatbuf,size,file);
	if(*rbuf==0)return  ;//沒有數據讀到
	else
	{
		printf("\r\nThe String Readed Is:%s\r\n",rbuf);  	  
	}			    	
}
//需要_USE_STRFUNC>=1
//寫一個字元到檔案
//c:要寫入的字元
//返回值:執行結果
u8 mf_putc(u8 c)
{
	return f_putc((TCHAR)c,file);
}
//寫字元串到檔案
//c:要寫入的字元串
//返回值:寫入的字元串長度
u8 mf_puts(u8*c)
{
	return f_puts((TCHAR*)c,file);
}













