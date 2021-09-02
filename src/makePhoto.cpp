//-----------------------------------------------------------------//
// Notes       | main console program to show how to use           //
//             | class Cpco_camera and Cpco_com to grab            //
//             | images from a pco usb camera                      //
//             |                                                   //
//-----------------------------------------------------------------//
//NOW USED libAcq_DualCLHSx1AreaRAW
//export LD_LIBRARY_PATH=/home/codac-dev/eclipse-workspace/pcoImage/lib64:$LD_LIBRARY_PATH
//export LD_LIBRARY_PATH=/home/codac-dev/eclipse-workspace/pcoImage/genicam/bin/Linux64_x64:$LD_LIBRARY_PATH
//export LD_LIBRARY_PATH=/home/codac-dev/pco.fg.5.7/lib64:$LD_LIBRARY_PATH
//export LD_LIBRARY_PATH=/home/codac-dev/pco.fg.5.7/genicam/bin/Linux64_x64:$LD_LIBRARY_PATH
// libAcq_SingleCLHSx2AreaRAW or libAcq_DualCLHSx1AreaRAW
#include <iostream>
#include <iomanip>
#include <runtimer.h>
#include <vector>

Timer tm1, tm2, tm3 , tm4;
unsigned int countPhoto = 0;

#include "VersionNo.h"
#include "Cpco_com.h"
#include "Cpco_grab_clhs.h"
#include "file12.h"

using namespace std;

#define BUFNUM 4


int image_nr_from_timestamp(void* buf,int shift);
DWORD grab_single(CPco_grab_clhs* grabber,char * filename);
DWORD get_image(CPco_grab_clhs* grabber,char* filename,WORD Segment,DWORD ImageNr);
DWORD grab_count_single(CPco_grab_clhs* grabber,int count);
DWORD grab_count_wait(CPco_grab_clhs* grabber,int count);

void get_number(char* number,int len);
void get_text(char* text,int len);
void get_hexnumber(int* num,int len);


CPco_Log mylog("pco_camera_grab.log");

const char tb[3][3]={"ns","us","ms"};
const char tmode[4][20]={"Auto","SW-Trig","Ext-Exp. Start","Ext-Exp. Ctrl"};




int main(int argc, char* argv[])
{
  DWORD err;
  CPco_com* camera;
  CPco_grab_clhs* grabber;

  int help=0;
  int board=0;
  char infostr[100];
  char number[20];

  int x;
  char c;
  int ima_count=99999;
  int loop_count=1;
  int PicTimeOut=1; //10 seconds - 10000

  WORD act_recstate,act_align;
  DWORD exp_time,delay_time,pixelrate;
  WORD exp_timebase,del_timebase;
  DWORD width,height,secs,nsecs;
  WORD triggermode;
  WORD binhorz,binvert;
  WORD wRoiX0, wRoiY0, wRoiX1, wRoiY1;
  SC2_Camera_Description_Response description;
  double freq;
  SHORT ccdtemp,camtemp,pstemp;
  WORD camtype;
  DWORD serialnumber;
  int shift;
  int loglevel=0x000FF0FF;

  mylog.set_logbits(loglevel);
  //printf("Logging set to 0x%08x\n",mylog.get_logbits());

  camera= new CPco_com_clhs();
  if(camera==NULL)
  {
   cout << "ERROR: Cannot create camera object\n";
   return -1;
  }

  if(loglevel>0)
   camera->SetLog(&mylog);

  printf("Try to open Camera\n");
  err=camera->Open_Cam(board);
  if(err!=PCO_NOERROR)
  {
   printf("ERROR: 0x%x in Open_Cam\n",err);
   delete camera;
   return -1;
  }

  err=camera->PCO_GetCameraType(&camtype,&serialnumber);
  if(err!=PCO_NOERROR)
  {
   printf("ERROR: 0x%x in PCO_GetCameraType\n",err);
   camera->Close_Cam();
   delete camera;
   return -1;
  }

  printf("Grabber is CPco_grab_clhs\n");
  grabber=new CPco_grab_clhs((CPco_com_clhs*)camera);

  if(loglevel>0)
   grabber->SetLog(&mylog);

  printf("Try to open Grabber\n");
  err=grabber->Open_Grabber(board);
  if(err!=PCO_NOERROR)
  {
   printf("ERROR: 0x%x in Open_Grabber",err);
   delete grabber;
   camera->Close_Cam();
   delete camera;
   return -1;
  }





  //SETTING
  err=grabber->Set_Grabber_Timeout(PicTimeOut);
  if(err!=PCO_NOERROR)
   printf("error 0x%x in Set_Grabber_Timeout",err);

  err=camera->PCO_GetCameraDescriptor(&description);
  if(err!=PCO_NOERROR)
   printf("PCO_GetCameraDescriptor() Error 0x%x\n",err);

  err=camera->PCO_GetInfo(1,infostr,sizeof(infostr));
  if(err!=PCO_NOERROR)
   printf("PCO_GetInfo() Error 0x%x\n",err);
  else
  {
   cout << "\nCamera Name is: " << infostr << "\n";
   printf("Camera Typ is : 0x%04x\n",camtype);
   printf("Camera Serial : %d\n\n",serialnumber);
  }

  err=camera->PCO_SetCameraToCurrentTime();
  if(err!=PCO_NOERROR)
   printf("PCO_SetCameraToCurrentTime() Error 0x%x\n",err);


  err=camera->PCO_GetTemperature(&ccdtemp,&camtemp,&pstemp);
  if(err!=PCO_NOERROR)
   printf("PCO_GetTemperature() Error 0x%x\n",err);
  else
  {
   printf("\ncurrent temperatures\n");
   printf("Camera:      %d°C\n",camtemp);
   if(ccdtemp != (SHORT)(-32768))
    printf("Sensor:      %d°C\n",ccdtemp);
   if(pstemp != (SHORT)(-32768))
    printf("PowerSupply: %d°C\n\n",pstemp);
  }


//set RecordingState to STOP
  err=camera->PCO_SetRecordingState(0);
  if(err!=PCO_NOERROR)
   printf("PCO_SetRecordingState() Error 0x%x\n",err);



//start from a known state
  err=camera->PCO_ResetSettingsToDefault();
  if(err!=PCO_NOERROR)
   printf("PCO_ResetSettingsToDefault() Error 0x%x\n",err);

//time in left up angle
  err=camera->PCO_SetTimestampMode(2);
  if(err!=PCO_NOERROR)
   printf("PCO_SetTimestampMode() Error 0x%x\n",err);

  //Set roi
      unsigned short int new_wRoiX0 = 1;//961;
      unsigned short int new_wRoiX1 = 2048;//1088;
      unsigned short int new_wRoiY0 = 961;
      unsigned short int new_wRoiY1 = 1088;
      err=camera->PCO_SetROI(new_wRoiX0, new_wRoiY0, new_wRoiX1, new_wRoiY1);
      printf("Set ROI %d-%d %d-%d returned 0x%x\n",new_wRoiX0, new_wRoiX1, new_wRoiY0, new_wRoiY1,err);

//set camera timebase to us
  exp_time    	= 600; //10000
  delay_time  	= 0;
  exp_timebase	= 1;
  del_timebase	= 1;

  err=camera->PCO_SetTimebase(del_timebase,exp_timebase);//"ns","us","ms"
  if(err!=PCO_NOERROR)
   printf("PCO_SetTimebase() Error 0x%x\n",err);

  err=camera->PCO_SetDelayExposure(delay_time,exp_time);
  if(err!=PCO_NOERROR)
   printf("PCO_SetDelayExposure() Error 0x%x\n",err);

  if(description.wNumADCsDESC>1)
  {
   err=camera->PCO_SetADCOperation(2);
   if(err!=PCO_NOERROR)
    printf("PCO_SetADCOperation() Error 0x%x\n",err);
  }

  err=camera->PCO_GetPixelRate(&pixelrate);
  if(err!=PCO_NOERROR)
   printf("PCO_GetPixelrate() Error 0x%x\n",err);
  else
   printf("actual PixelRate: %d\n",pixelrate);
  printf("possible PixelRates:\n");
  for(x=0;x<4;x++)
  {
   if(description.dwPixelRateDESC[x]!=0)
   {
    printf("%d: %d\n",x,description.dwPixelRateDESC[x]);
   }
  }

  err=camera->PCO_SetBitAlignment(BIT_ALIGNMENT_LSB);
  if(err!=PCO_NOERROR)
   printf("PCO_SetBitAlignment() Error 0x%x\n",err);


//prepare Camera for recording
  err=camera->PCO_ArmCamera();
  if(err!=PCO_NOERROR)
   printf("PCO_ArmCamera() Error 0x%x\n",err);

/*
  printf("second PCO_ArmCamera\n");
  err=camera->PCO_ArmCamera();
  if(err!=PCO_NOERROR)
   printf("PCO_ArmCamera() Error 0x%x\n",err);
*/

  err=camera->PCO_GetBitAlignment(&act_align);
  if(err!=PCO_NOERROR)
   printf("PCO_GetBitAlignment() Error 0x%x\n",err);

  shift=0;
  if(act_align!=BIT_ALIGNMENT_LSB)
  {
   shift=16-description.wDynResDESC;
   printf("BitAlignment MSB shift %d\n",shift);
  }

  err=camera->PCO_GetTriggerMode(&triggermode);
  if(err!=PCO_NOERROR)
   printf("PCO_GetGetTriggermode() Error 0x%x\n",err);
  else
   printf("actual Triggermode: %d %s\n",triggermode,tmode[triggermode]);

  err=camera->PCO_GetBinning(&binhorz,&binvert);
  if(err!=PCO_NOERROR)
   printf("PCO_GetBinning() Error 0x%x\n",err);
  else
   printf("actual Binning: %dx%d\n",binhorz,binvert);

  err=camera->PCO_GetROI(&wRoiX0, &wRoiY0, &wRoiX1, &wRoiY1);
  if(err!=PCO_NOERROR)
   printf("PCO_GetROI() Error 0x%x\n",err);
  else
   printf("actual ROI: %d-%d, %d-%d\n",wRoiX0,wRoiX1,wRoiY0,wRoiY1);


  err=camera->PCO_GetActualSize(&width,&height);
  if(err!=PCO_NOERROR)
   printf("PCO_GetActualSize() Error 0x%x\n",err);

  printf("Actual Resolution %d x %d\n",width,height);

  err=grabber->PostArm();
  if(err!=PCO_NOERROR)
   printf("grabber->PostArm() Error 0x%x\n",err);

/*is done from PostArm()
  err=grabber->SetBitAlignment(act_align);
  err=grabber->Set_Grabber_Size(width,height);
  if(err!=PCO_NOERROR)
   printf("Set_Grabber_Size() Error 0x%x\n",err);

*/

  err=camera->PCO_SetRecordingState(1);
  if(err!=PCO_NOERROR)
   printf("PCO_SetRecordingState() Error 0x%x\n",err);

  c=' ';
  while(c!='x')
  {
   int ch;
   c=' ';

   printf("\n");
   camera->PCO_GetRecordingState(&act_recstate);
   camera->PCO_GetDelayExposure(&delay_time,&exp_time);
   camera->PCO_GetCOCRuntime(&secs,&nsecs);
   freq=nsecs;
   freq/=1000000000;
   freq+=secs;
   freq=1/freq;
   printf(" actual recording state %s actual freq: %.3lfHz time/pic: %.2lfms  datarate:%.2lfMB/sec \n",act_recstate ? "RUN" : "STOP",freq,1000/freq,(freq*width*height*2/(1024*1024)));
   printf("\n");
   printf("x to close camera and program   actual values\n");
   printf("l to set loop_count              loop_count      %d\n",loop_count);
   printf("c to set imagecount              imagecount      %d\n",ima_count);
   printf("t to set picture timeout         timeout         %dms\n",PicTimeOut);
   printf("e to set exposure time           exposuretime    %d%s\n",exp_time,tb[exp_timebase]);
   printf("d to set delay time              delaytime       %d%s\n",delay_time,tb[del_timebase]);
   printf("p to set camera pixelrate        pixelrate       %dHz\n",pixelrate);
   printf("a to set triggermode             triggermode     %s\n",tmode[triggermode]);
   printf("b to set binning                 binning         %dx%d\n",binhorz,binvert);
   printf("\n");
   printf("0 to set recording state to OFF\n");
   printf("1 to set recording state to ON\n");
   printf("2 Single Acquire_Image (recording camera)\n");
   printf("3 Single Get_Image (recording camera or camera recorder buffer) \n");
   printf("4 Loop Acquire_Image\n");
   printf("5 Loop Wait_Next_Image\n");

   fflush(stdin);

   for( x = 0; (x < 2) &&  ((ch = getchar()) != EOF)
                        && (ch != '\n'); x++ )
    c=(char)ch;



   if(c=='0')
   {
    err=camera->PCO_SetRecordingState(0);
    if(err==PCO_NOERROR)
     printf("\nrecoding state is set to STOP\n");
   }
   else if(c=='1')
   {
    err=camera->PCO_SetRecordingState(1);
    if(err==PCO_NOERROR)
     printf("\nrecoding state is set to RUN\n");
   }
   else if(c=='2')
   {
	   char filename[300] = "test.tif";
	      tm1.reset("Start Acq '3'");/////////////////////////////////////////////////////////////////////////

    if(act_recstate==0)
     printf("\nStart Camera before grabbing\n");
    else
    {
     //char filename[300];
     //printf("enter filename ...<CR>: \n");
     //printf("if filename has suffix .b16 save image in b16 format\n");
     //printf("if filename has suffix .tif save image in tif format\n");
     //printf("if nothing is entered, no file will be saved\n");
     //get_text(filename,300);

    tm2.reset("Start get_image()");////////////////////////////////////////////////////////////////////////////

     if(strlen(filename))
      grab_single(grabber,filename);
     else
      grab_single(grabber,NULL);

    }
    printf("\n");
    tm4.reset("END");////////////////////////////////////////////////////////////////////////////

        cout << tm1 << "\n";
        cout << tm2 << "\n";
        cout << tm3 << "\n";
        cout << tm4 << "\n";
   }
   else if(c=='3')
   {
    int Segment,image_number;
    char filename[300] = "test.tif";
    tm1.reset("Start Acq '3'");/////////////////////////////////////////////////////////////////////////

    Segment=image_number=0;

    if(act_recstate==0)
    {
     DWORD dwValid,dwMax;

     Segment=image_number=1;
     if(description.dwGeneralCaps1&GENERALCAPS1_NO_RECORDER)
     {
      printf("camera does not support image readout from Segments\n");
      continue;
     }
     printf("enter Segment   [%01d]   ...<CR>: ",Segment);
     get_number(number,2);
     if(strlen(number))
      Segment=atoi(number);

     err=camera->PCO_GetNumberOfImagesInSegment(Segment,&dwValid,&dwMax);
     if(err!=PCO_NOERROR)
     {
      printf("no information available for Segment %d\n",Segment);
      continue;
     }


     printf("enter image number (valid %d max %d) ...<CR>: ",dwValid,dwMax);
     get_number(number,10);
     if(strlen(number))
      image_number=atoi(number);

     if(dwValid==0)
     {
      printf("no images available in Segment %d\n",Segment);
      continue;
     }


     if(image_number>(int)dwValid)
     {
      printf("try again with valid image number\n");
      continue;
     }
    }

    printf("enter filename ...<CR>: \n");
    printf("if filename has suffix .b16 save image in b16 format\n");
    printf("if filename has suffix .tif save image in tif format\n");
    printf("if nothing is entered, no file will be saved\n");
    //get_text(filename,300);///////////////////////////////////////////////////////////////////
    tm2.reset("Start get_image()");////////////////////////////////////////////////////////////////////////////
    if(strlen(filename))
     get_image(grabber,filename,Segment,image_number);
    else
     get_image(grabber,NULL,Segment,image_number);
    printf("\n");


    tm4.reset("END");////////////////////////////////////////////////////////////////////////////

    cout << tm1 << "\n";
    cout << tm2 << "\n";
    cout << tm3 << "\n";
    cout << tm4 << "\n";
   }
   else if(c=='4')
   {
	tm1.reset("start 100 ima");
    if(act_recstate==0)
     printf("\nStart Camera before grabbing\n");
    else
     grab_count_single(grabber,ima_count);
    double tmp = tm1.elapsed();
    cout << left << "\n";
    cout << setw(35) << "start 100 ima" << tmp << "\n";

    cout << setw(35) << "if all 100 im right" <<  100/tmp << " Hz\n";
    printf("\n");
   }
   else if(c=='5')
   {
	 for (int i=0;i<300; i++){
		 cout << "try No " << i;
		 if(act_recstate==0)
		 		printf("\nStart Camera before grabbing\n");
		 	else
		 		grab_count_wait(grabber,ima_count);
		 	printf("\n");
	 }

   }
  }


  grabber->Close_Grabber();
  delete grabber;

  camera->Close_Cam();
  delete camera;

  printf("Any key CR to close application\n");
  getchar();

  return 0;
}


DWORD grab_single(CPco_grab_clhs* grabber,char* filename)
{
 int err;
 unsigned int w,h,l;
 int ima_num;
 WORD *picbuf;

 err=grabber->Get_actual_size(&w,&h,NULL);
 if(err!=PCO_NOERROR)
 {
  printf("\ngrab_single Get_actual_size error 0x%x\n",err);
  return err;
 }

 picbuf=(WORD*)malloc(w*h*sizeof(WORD));
 if(picbuf==NULL)
 {
  printf("\ngrab_single cannot allocate buffer\n");
  return PCO_ERROR_NOMEMORY | PCO_ERROR_APPLICATION;
 }

 err=grabber->Acquire_Image(picbuf);
 if(err!=PCO_NOERROR)
  printf("\ngrab_single Acquire_Image error 0x%x\n",err);

 if(err==PCO_NOERROR)
 {
  int min,max,v;
  WORD* adr;

  ima_num=image_nr_from_timestamp(picbuf,0);
    printf("\ngrab_single done successful, timestamp image_nr: %d\n",ima_num);

    tm3.reset("Start after acq");////////////////////////////////////////////////////////////////////////////

  max=0;
  min=0xFFFF;
  adr=(WORD*)picbuf;
  l=w*20; //skip first lines with timestamp
  for(;l<w*h;l++)
  {
   v=*(adr+l);
   if(v<min)
    min=v;
   if(v>max)
    max=v;
  }
  printf("grab_single pixels min_value: %d max_value %d\n",min,max);

  if(filename!=NULL)
  {
   char *txt;
   do
   {
    txt=strchr(filename,'.');
   }
   while((txt)&&(strlen(txt)>4));

   if(txt==NULL)
   {
    txt=filename;
    strcat(txt,".b16");
   }

   if(strstr(txt,"b16"))
   {
    store_b16(filename,w,h,0,picbuf);
    printf("b16 image saved to %s\n",filename);
   }
   else if(strstr(txt,"tif"))
   {
    store_tif(filename,w,h,0,picbuf);
    printf("tif image saved to %s\n",filename);
   }
  }
 }
 free(picbuf);

 return err;
}

DWORD get_image(CPco_grab_clhs* grabber,char* filename,WORD Segment,DWORD ImageNr)
{
 int err;
 unsigned int w,h,l;
 int ima_num;
 WORD *picbuf;

 grabber->Get_actual_size(&w,&h,NULL);
 picbuf=(WORD*)malloc(w*h*sizeof(WORD));
 if(picbuf==NULL)
 {
  printf("\nget_image cannot allocate buffer\n");
  return PCO_ERROR_NOMEMORY | PCO_ERROR_APPLICATION;
 }

 err=grabber->Get_Image(Segment,ImageNr,picbuf);
 if(err!=PCO_NOERROR)
  printf("\nget_image Acquire_Image error 0x%x\n",err);

 if(err==PCO_NOERROR)
 {
  ima_num=image_nr_from_timestamp(picbuf,0);
  printf("\nget_image done successful, timestamp image_nr: %d\n",ima_num);
  tm3.reset("Start after acq");////////////////////////////////////////////////////////////////////////////
  if(filename!=NULL)
  {
   char *txt;
   int min,max,v;

   max=0;
   min=0xFFFF;
   l=w*20; //skip first lines with timestamp
   for(;l<w*h;l++)
   {
    v=*(picbuf+l);
    if(v<min)
     min=v;
    if(v>max)
    max=v;
   }
   printf("get_image pixels min_value: %d max_value %d\n",min,max);

   do
   {
    txt=strchr(filename,'.');
   }
   while((txt)&&(strlen(txt)>4));

   if(txt==NULL)
   {
    txt=filename;
    strcat(txt,".b16");
   }

   if(strstr(txt,"b16"))
   {
    store_b16(filename,w,h,0,picbuf);
    printf("b16 image saved to %s\n",filename);
   }
   else if(strstr(txt,"tif"))
   {
    store_tif(filename,w,h,0,picbuf);
    printf("tif image saved to %s\n",filename);
   }
  }
 }

 free(picbuf);

 return err;
}


DWORD grab_count_single(CPco_grab_clhs* grabber,int count)
{
 int err,i;
 int picnum,buf_nr,first_picnum,lost;
 unsigned int w,h,bp;
 WORD *picbuf[BUFNUM];
 double tim,freq;

 picnum=1;
 err=grabber->Get_actual_size(&w,&h,&bp);
 if(err!=PCO_NOERROR)
 {
  printf("\ngrab_count_single Get_actual_size error 0x%x\n",err);
  return err;
 }
 printf("\n");
 lost=first_picnum=0;

 memset(picbuf,0,sizeof(WORD*)*BUFNUM);
 for(i=0;i< BUFNUM;i++)
 {
  picbuf[i]=(WORD*)malloc(w*h*sizeof(WORD));
  if(picbuf[i]==NULL)
  {
   printf("\nget_count_single image cannot allocate buffer %d\n",i);
   err=PCO_ERROR_NOMEMORY | PCO_ERROR_APPLICATION;
   break;
  }
 }

 for(i=0;i<count;i++)
 {
  buf_nr=i%BUFNUM;
  err=grabber->Acquire_Image(picbuf[buf_nr]);
  if(err!=PCO_NOERROR)
  {
   printf("\nAcquire_Image error 0x%x\n",err);
   break;
  }
  else
  {
   picnum=image_nr_from_timestamp(picbuf[buf_nr],0);
   printf("%05d. Image to %d  ts_nr: %05d",i+1,buf_nr,picnum);
   if(i==0)
   {
    first_picnum=picnum;
    mylog.start_time_mess();
   }
   else if((first_picnum+i)!=picnum)
   {
    printf(" %05d != %05d\n",first_picnum+i,picnum);
    first_picnum=picnum-i;
    lost++;
   }

   if((count<=10)||(i<3))
    printf("\n");
   else
    printf("\n");//\r
   fflush(stdout);
  }
 }
 i--;
 tim=mylog.stop_time_mess();
 freq=i*1000;
 freq/=tim;
 printf("\n%05d images grabbed time %dms time/pic: %.3fms lost %d freq: %.2fHz %.2fMB/sec",i+1,(int)tim,tim/i,lost,freq,(freq*w*h*2)/(1024*1024));


 for(i=0;i< BUFNUM;i++)
 {
  if(picbuf[i]!=NULL)
   free(picbuf[i]);
 }

 return err;
}


DWORD grab_count_wait(CPco_grab_clhs* grabber,int count)
{
 int err,i,timeout;
 int picnum,buf_nr,first_picnum,lost;
 unsigned int w,h,bp;
 WORD *picbuf[BUFNUM];
 double tim,freq;
 int c;

 picnum=1;
 err=grabber->Get_actual_size(&w,&h,&bp);
 if(err!=PCO_NOERROR)
 {
  printf("\ngrab_count_wait Get_actual_size error 0x%x\n",err);
  return err;
 }
 printf("\n");
 lost=first_picnum=0;

 grabber->Get_Grabber_Timeout(&timeout);

 memset(picbuf,0,sizeof(WORD*)*BUFNUM);
 for(i=0;i< BUFNUM;i++)
 {
  picbuf[i]=(WORD*)malloc(w*h*sizeof(WORD));
  if(picbuf[i]==NULL)
  {
   printf("\ngrab_count_wait cannot allocate buffer %d\n",i);
   err=PCO_ERROR_NOMEMORY | PCO_ERROR_APPLICATION;
   break;
  }
 }
 if(err!=PCO_NOERROR)
 {
  for(i=0;i< BUFNUM;i++)
  {
   if(picbuf[i]!=NULL)
    free(picbuf[i]);
  }
  return err;
 }

 err=grabber->Start_Acquire();
 if(err!=PCO_NOERROR)
 {
  printf("\ngrab_count_wait Start_Acquire error 0x%x\n",err);
  return err;
 }

 for(i=0;i<count;i++)
 {
  buf_nr=i%BUFNUM;
  err=grabber->Wait_For_Next_Image(picbuf[buf_nr],timeout);
  if(err!=PCO_NOERROR)
  {
   printf("\nWait_For_Next_Image error 0x%x\n",err);
   break;
  }
  else
  {
   picnum=image_nr_from_timestamp(picbuf[buf_nr],0);
   //printf("%05d. Image to %d  ts_nr: %05d",i+1,buf_nr,picnum);////////////
   if(i==0)
   {
    first_picnum=picnum;
    mylog.start_time_mess();
   }
   else if((first_picnum+i)!=picnum)
   {
    //printf(" %05d != %05d\n",first_picnum+i,picnum);//////////////
    first_picnum=picnum-i;
    lost++;
   }

   /*if((count<=10)||(i<3))
    //printf("\n");//////////////////////////////
   else
    //printf("\n");//\r////////////////////////*/
   fflush(stdout);
  }
 }

 i--;
 tim=mylog.stop_time_mess();
 freq=i*1000;
 freq/=tim;
 printf("\n %05d images grabbed in %dms lost images %d\n",i+1,(int)tim,lost);
 printf(" freq: %.2fHz time/pic: %.3fms  %.2fMB/sec",freq,tim/i,(freq*w*h*2)/(1024*1024));

 err=grabber->Stop_Acquire();
 if(err!=PCO_NOERROR)
 {
  printf("\ngrab_count_wait Stop_Acquire error 0x%x\n",err);
 }

 for(i=0;i< BUFNUM;i++)
 {
  if(picbuf[i]!=NULL)
   free(picbuf[i]);
 }

 return err;
}


int image_nr_from_timestamp(void *buf,int shift)
{
  unsigned short *b;
  int y;
  int image_nr=0;
  b=(unsigned short *)(buf);

  y=100*100*100;
  for(;y>0;y/=100)
  {
   *b>>=shift;
   image_nr+= (((*b&0x00F0)>>4)*10 + (*b&0x000F))*y;
   b++;
  }
  return image_nr;
}


void get_number(char *number,int len)
{
   int ret_val;
   int x=0;

   while(((ret_val=getchar())!=10)&&(x<len-1))
   {
    if(isdigit(ret_val))
     number[x++]=ret_val;
   }
   number[x]=0;
}

void get_text(char *text,int len)
{
   int ret_val;
   int x=0;

   while(((ret_val=getchar())!=10)&&(x<len-1))
   {
    if(isprint(ret_val))
     text[x++]=ret_val;
   }
   text[x]=0;
}


void get_hexnumber(int *num,int len)
{
  int ret_val;
  int c=0;
  int cmd=0;
  while(((ret_val=getchar())!=10)&&(len > 0))
  {
   if(isxdigit(ret_val))
   {
    if(ret_val<0x3A)
     cmd=(ret_val-0x30)+cmd*0x10;
    else if(ret_val<0x47)
     cmd=(ret_val-0x41+0x0A)+cmd*0x10;
    else
     cmd=(ret_val-0x61+0x0A)+cmd*0x10;
    len--;
    c++;
   }
  }
  if(c>0)
   *num=cmd;
  else
   *num=-1;
}
