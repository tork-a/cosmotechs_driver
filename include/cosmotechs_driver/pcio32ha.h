// header file of module pcidrv.o 
#define OUT_P 	   1
//#define IN_P  	   2
#define IN_P  	   12
#define MEMOUT_P   3
#define MEMIN_P    4
#define INTRSET_P  5
#define INTRESET_P 6
#define IN_P1  	   7
#define MEMIN_P1   8
#define WAIT_INTERRUPT 9
#define GET_RESOURCE   13

struct pmc_struct 
{
	unsigned int bsn;
	unsigned int axis;
	unsigned int adr;
	unsigned int com;
	unsigned long data;
	unsigned char d;
	unsigned int  i;
};


struct int_wait{
	unsigned short timeout;
	int err;

};


typedef struct{
	unsigned char bus;		//PCIバス番号
	unsigned char dev;		//PCIデバイス番号
	unsigned char func;		//PCIファンクション番号
	unsigned char baseclass;	//基本クラス
	unsigned char subclass;		//サブクラス
	unsigned char programif;
	unsigned char revision;
	unsigned char irq;		//Interrupt
	unsigned char Bsn;		//Bsn
	unsigned int  Mem_base;		//Memory Base Address
	unsigned int  Io_base;		//IO Base Address
} pciresource;


//************PCIO32H ORIGINAL Define ************
#define PCIO32H_MAX_SLOTS 16  //PCIO32H MAX SLOTS
#define PCIO32H_MAX_PORT   7

//IO map
#define PCIO32H_PORT_1		0
#define PCIO32H_PORT_2		1
#define PCIO32H_PORT_3		2
#define PCIO32H_PORT_4		3
#define PCIO32H_BSN_SWITCH	4


//ERROR CODE--------------------------------------------------------------------
#define PCIO32H_SUCCESS			0  // SUCCESS
#define PCIO32H_ERR_SYSTEM		1  // SYSTEM ERROR
#define PCIO32H_ERR_NO_DEVICE		2  // NO DEVICE
#define PCIO32H_ERR_IN_USE		3  // DEVICE USE
#define PCIO32H_ERR_INVALID_BSN		4  // INVALID BSN
#define PCIO32H_ERR_INVALID_PORT	6  // INVALID PORT
#define PCIO32H_ERR_PARAMETER		7  // PARAMETER ERROR
#define PCIO32H_ERR_PROC		8  // FUNC FAIL
#define PCIO32H_ERR_USER_HANDLER	9  // USER HANDLER RUNNING


void	ErrorSet( unsigned short Number, unsigned short Error );
unsigned long Pcio32hwGetLastError( unsigned short wBsn );
int Pcio32hwGetLibVersion( unsigned char *pbLibVersion);
int Pcio32hwGetDrvVersion( unsigned char *pbDrvVersion);
int Pcio32hwCreate( unsigned short wBsn );
void Pcio32hwClose( void );
int Pcio32hwGetResource( unsigned short wBsn, pciresource *pres );
int Pcio32hwInPort( unsigned short wBsn,unsigned short wPort,unsigned char *pbData);
int Pcio32hwOutPort( unsigned short wBsn,unsigned short wPort,unsigned char bData);
int Pcio32hwInPortW( unsigned short wBsn,unsigned short wPort,unsigned short *pwData);
int Pcio32hwOutPortW( unsigned short wBsn,unsigned short wPort,unsigned short wData);
int Pcio32hwInPortDW( unsigned short wBsn,unsigned long *pdwData);
int Pcio32hwOutPortDW( unsigned short wBsn,unsigned long dwData);
int Pcio32hwInPortBit( unsigned short wBsn,unsigned char wBit,unsigned char *pbONOFF);
int Pcio32hwOutPortBit( unsigned short wBsn,unsigned char wBit,unsigned char bONOFF);
