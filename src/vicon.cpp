
#include <sys/time.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <math.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "udp.h"
#include "vicon.h"


int telem_read(udp_struct *udp, unsigned char *ptr, int cnt){
  int i, length;
	//length = udpServer_Receive(udp, (char*) ptr, cnt);
  length=read(udp->s, (void*) ptr, cnt);

  if (length==-1){
    udp->socket_errors=errno;
     return 0;
  }

  if (length==0) return 0;
/*
	 for (i=0;i<length; i++)
		 printf("%d ", ptr[i]) ;

	 printf("\n") ;
	 */
  return length;
}

int frame_read(FRAME_DATA *frame, int index, int cnt){
  int i, ch_avail, ch_read;

  ch_avail=frame->packet_length-frame->packet_index;

  if (ch_avail<cnt)
    ch_read=ch_avail;
  else
    ch_read=cnt;

  for (i=0;i<ch_read;i++){
    frame->buff[index++]=frame->sbuff[frame->packet_index++];
  }

  return ch_read;
}

unsigned short get_telemu16(unsigned char *buff){
  return (*buff*0x100+*(buff+1));
}

short get_telem16(unsigned char *buff){
  char msb;

  msb=(char) (*buff);
  return (msb*0x100+*(buff+1));
}

unsigned int get_telemu32(unsigned char *buff){
  unsigned int temp;
  unsigned char *temp2;
  int i;
  temp2=(unsigned char*) &temp;
	    ;
  for (i=0;i<4;i++) temp2[i]=buff[i];

  return temp;
}

unsigned int get_telemu32_reverse(unsigned char *buff){
  unsigned int temp;
  unsigned char *temp2;
  int i;
  temp2=(unsigned char*) &temp;
	    ;
  for (i=0;i<4;i++) temp2[3-i]=buff[i];

  return temp;
}

int get_telem32(unsigned char *buff){
  int temp;
  unsigned char *temp2;
  int i;
  temp2=(unsigned char*) &temp;

  for (i=0;i<4;i++) temp2[i]=buff[i];

  return temp;
}

int get_telem32_reverse(unsigned char *buff){
  int temp;
  unsigned char *temp2;
  int i;
  temp2=(unsigned char*) &temp;

  for (i=0;i<4;i++) temp2[3-i]=buff[i];

  return temp;
}

long long int get_long(unsigned char *buff){
  long long int temp;
  unsigned char *temp2;
  int i;
  temp2=(unsigned char*) &temp;

  for (i=0;i<8;i++) temp2[i]=buff[i];

  return temp;
}

int get_double(double *num, unsigned char *buff){
  double temp;
  unsigned char *temp2;
  int i;
  temp2=(unsigned char*) &temp;

  for (i=0;i<8;i++) temp2[i]=buff[i];

  if (!finite(temp)) return 0;

  *num=temp;

  return 1;
}


int get_float(float *num, unsigned char *buff){
  float temp;
  unsigned char *temp2;
  int i;
  temp2=(unsigned char*) &temp;

  for (i=0;i<4;i++) temp2[i]=buff[i];

  if (!finite(temp)) return 0;

  *num=temp;

  return 1;
}



int get_float_reverse(float *num, unsigned char *buff){
  float temp;
  unsigned char *temp2;
  int i ;
  temp2=(unsigned char*) &temp;

  for (i=0;i<4;i++) temp2[3-i]=buff[i];

  if (!finite(temp)) return 0;

  *num=temp;

  return 1;
}


int init_frame(FRAME_DATA *frame, unsigned int length){
  frame->rx_chars=0;
  frame->rx_packets=0;
  frame->checksum_errors=0;
  frame->buff_index=0;
  frame->synch=0;

  if (length<=FRAME_BUFF_SIZE){
    frame->length=length;
  }
  else{
    return -1;
  }

  return 0;
}


void reset_frame(FRAME_DATA *frame){
  frame->synch=0;
  frame->buff_index=0;
}

int get_header(FRAME_DATA* frame, char synch_ident){

// Note buffer on autopilot uses signed char so that
// comparison between header characters greater than
// 0x80 eg 0xAA will fail

  // search for synch characters
  int len, counter;

  counter=0; frame->buff[0]=0; len=0;

  // search for first header character
  // if 100 characters searched without success return empty-handed
  // to stop locking up system indefinately
  while ((!len)&&(counter++<100)){
    len=frame_read(frame, 0, 1); // read one character
    if (len==0) return 0;  // only continue while there are characters to read

    frame->rx_chars++;
    if (frame->buff[0]!=synch_ident) len=0; // keep looking if unsuccesfull

  }

 // printf("%d %d ",buff[0], counter ) ;

  if (frame->buff[0]!=synch_ident) return 0;

  return 1;
}

int check_for_frames(FRAME_DATA *frame, char synch_ident){
/************************************************************/
/* Returns 1 if a complete (& valid) command frame has been */
/* loaded and is ready to execute. Command Structure is:    */
/* Header e.g.0x55  	        			    */
/* buff[length-1] Checksum				    */
/*                                                          */
/* Note: fd should be of the form xtreme_fd[XTREME_XX_CHAN] */
/************************************************************/
  int rx_len=0;
  int i;
  unsigned char checksum;
  int bytes_to_read=0;

  if (!frame->synch){
    if (get_header(frame, synch_ident)){
      frame->synch=1;
      frame->buff_index=0; // header not included
			//printf("H ") ;
    }
    else{
      reset_frame(frame);
      return 0;
    }
  }

  bytes_to_read=frame->length-frame->buff_index;

  if (bytes_to_read>0){
    rx_len=frame_read(frame, frame->buff_index, bytes_to_read);
    frame->buff_index+=rx_len;
    frame->rx_chars+=rx_len;
  }

  if (frame->buff_index==frame->length){
    checksum=0;
    for (i=0;i<frame->length-1;i++)
      checksum+=frame->buff[i];
    frame->checksumA=checksum;
    frame->checksumB=frame->buff[frame->length-1];
    if (frame->checksumA!=frame->checksumB){
      frame->checksum_errors++;
      //rtl_printf("|%x %x", checksum,frame->buff[frame->length-1] ) ;
      reset_frame(frame);
      return -1;
    }
    else{
      frame->rx_packets++;
      reset_frame(frame);
      return 1;  // Complete command frame loaded
    }
  }
  else{
    return 0;  // Frame still loading
  }
}


void load_vicon_packet(unsigned char *buff, PVA_DATA *pva){
  get_float(&pva->position[0], buff+2);
  get_float(&pva->position[1], buff+6);
  get_float(&pva->position[2], buff+10);

  get_float(&pva->orientation[0], buff+14);
  get_float(&pva->orientation[1], buff+18);
  get_float(&pva->orientation[2], buff+22);
  get_float(&pva->orientation[3], buff+22+4);	

  pva->index=get_telemu16(buff+26);
}

void get_vicon_packet(FRAME_DATA *frame, udp_struct *udp, PVA_DATA *pva){
  int return_code=0;
  static unsigned int Vcount=0;
  unsigned char buf[1024];
  int i;

  frame->packet_length=telem_read(udp, frame->sbuff, 512);
  frame->packet_index=0;

  while (frame->packet_index < frame->packet_length){
    return_code=check_for_frames(frame,HEADER_CHAR);
    if (return_code>0){
      load_vicon_packet(frame->buff, pva);
    }
  }
/*
	if ((Vcount%20)==0) {
		printf("%d %d %d %d %f %f %f ", Vcount, frame->rx_packets, frame->checksum_errors, pva->index, pva->pos[0], pva->pos[1], pva->pos[2]) ;
		//printf("%d ", frame->buff_index ) ;
		//for (i=0;i<6;i++) printf("%d ",frame->buff[i]) ;
		printf("\n") ;
	}
	*/

  Vcount++;
}
