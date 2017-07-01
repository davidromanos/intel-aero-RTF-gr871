class PVA_DATA
{
public:
    float position[3];
    float orientation[4];
    unsigned int index;
};

/*
typedef struct {
	float pos[3] ;
	float vel[3] ;
	float angles[3] ;
	float rates[3] ;
	float accel[3] ;
	unsigned int index ;
} PVA_DATA ;
*/

#define FRAME_BUFF_SIZE 255
#define SOCKET_BUFF_SIZE 1024
#define HEADER_CHAR 0x55


class FRAME_DATA
{
public:
    unsigned char buff[FRAME_BUFF_SIZE];
    unsigned char sbuff[SOCKET_BUFF_SIZE];
    unsigned int buff_index;
    unsigned int packet_length;
    unsigned int packet_index;
    unsigned short length;
    unsigned short id;
    unsigned char synch;
    unsigned short checksum_errors;
    unsigned int rx_chars ;
    unsigned int rx_packets;
    unsigned char checksumA;
    unsigned char checksumB;
};
/*
typedef struct {
  unsigned char buff[FRAME_BUFF_SIZE];
  unsigned char sbuff[SOCKET_BUFF_SIZE];
  unsigned int buff_index ;
  unsigned int packet_length ;
  unsigned int packet_index ;
  unsigned short length ;
  unsigned short id ;
  unsigned char synch ;
  unsigned short checksum_errors ;
  unsigned int rx_chars ;
  unsigned int rx_packets ;
  unsigned char checksumA ;
  unsigned char checksumB ;
} FRAME_data ;
*/
int init_frame(FRAME_DATA *frame, unsigned int length);
void get_vicon_packet(FRAME_DATA *frame, udp_struct *udp, PVA_DATA *pva);


