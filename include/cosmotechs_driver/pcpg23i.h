// header file of module pcpg28iodrv.o

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
 
#define OUT_P 	   1
#define IN_P  	   12
#define MEMOUT_P   3
#define MEMIN_P    4
#define INTRSET_P  5
#define INTRESET_P 6
#define IN_P1  	   7
#define MEMIN_P1   8
#define WAIT_INTERRUPT 9

#define PCPG23I_MAX_SLOTS 16
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


//**********************************************
//
//  PMC842 PORT ADDRESS
//
//**********************************************
#define PMC_DATA_PORT0  0   //2^24 - 2^31
#define PMC_DATA_PORT1  1   //2^16 - 2^23
#define PMC_DATA_PORT2  2   //2^8  - 2^15
#define PMC_DATA_PORT3  3   //2^0  - 2^7
#define PMC_COM_PORT    4   //Commnad Write PORT
#define PMC_MODE1_WRITE	5   //MODE1 WRITE
#define PMC_MODE2_WRITE 6   //MODE2 WRITE
#define PMC_UNSIG_WRITE 7   //UNIVERSAL SIGNAL WRITE

#define PMC_DRIVE_READ  4   //Drive Status Read
#define PMC_END_READ	5   //End Status Read
#define PMC_MECH_READ   6   //Mechanical Signeal Read
#define PMC_UNSIG_READ  7   //UNIVERSAL SIGNAL Read


//************PCPG23I ORIGINAL Define ************
#define PCPG23I_MAX_AXIS   8  //PCPG23I MAX
#define PCPG23I_MAX_SLOTS 16  //PCPG23I MAX SLOTS
#define PCPG23I_MAX_PORT   7


//ERROR CODE--------------------------------------------------------------------
#define PCPG23I_SUCCESS			    0  // SUCCESS
#define PCPG23I_ERR_INVALID_BSN		4  // INVALID BSN
#define PCPG23I_ERR_INVALID_AXIS		50 // INVALID AXIS
#define PCPG23I_ERR_PARAMETER		7  // PARAMETER ERROR

#define		BAS_RNG	(32768000.0 / 32768.0)

//============================================
// Commands
//===========================================
#define PCPG23I_RANGE_WRITE						0x00
#define PCPG23I_RANGE_READ						0x01
#define PCPG23I_START_STOP_SPEED_DATA_WRITE		0x02
#define PCPG23I_START_STOP_SPEED_DATA_READ		0x03
#define PCPG23I_OBJECT_SPEED_DATA_WRITE			0x04
#define PCPG23I_OBJECT_SPEED_DATA_READ			0x05
#define PCPG23I_RATE1_DATA_WRITE					0x06
#define PCPG23I_RATE1_DATA_READ					0x07
#define PCPG23I_RATE2_DATA_WRITE					0x08
#define PCPG23I_RATE2_DATA_READ					0x09
#define PCPG23I_SLOW_DOWN_REAR_PULSE_WRITE		0x10
#define PCPG23I_SLOW_DOWN_REAR_PULSE_READ		0x11
#define PCPG23I_NOW_SPEED_DATA_READ				0x12
#define PCPG23I_DRIVE_PULSE_COUNTER_READ			0x13
#define PCPG23I_PRESET_PULSE_DATA_OVERRIDE		0x14
#define PCPG23I_PRESET_PULSE_DATA_READ			0x15
#define PCPG23I_DEVIATION_DATA_READ				0x16
#define PCPG23I_INPOSITION_WAIT_MODE1_SET		0x17
#define PCPG23I_INPOSITION_WAIT_MODE2_SET		0x18
#define PCPG23I_INPOSITION_WAIT_MODE_RESET		0x19
#define PCPG23I_ALARM_STOP_ENABLE_MODE_SET		0x1a
#define PCPG23I_ALARM_STOP_ENABLE_MODE_RESET		0x1b
#define PCPG23I_INTERRUPT_ENABLE_MODE_SET		0x1c
#define PCPG23I_INTERRUPT_ENABLE_MODE_RESET		0x1d
#define PCPG23I_SLOW_DOWN_STOP					0x1e
#define PCPG23I_EMERGENCY_STOP					0x1f
#define PCPG23I_PLUS_PRESET_PULSE_DRIVE			0x20
#define PCPG23I_MINUS_PRESET_PULSE_DRIVE			0x21
#define PCPG23I_PLUS_CONTINUOUS_DRIVE			0x22
#define PCPG23I_MINUS_CONTINUOUS_DRIVE			0x23
#define PCPG23I_PLUS_SIGNAL_SEARCH1_DRIVE		0x24
#define PCPG23I_MINUS_SIGNAL_SEARCH1_DRIVE		0x25
#define PCPG23I_PLUS_SIGNAL_SEARCH2_DRIVE		0x26
#define PCPG23I_MINUS_SIGNAL_SEARCH2_DRIVE		0x27
#define PCPG23I_INTERNAL_COUNTER_WRITE			0x28
#define PCPG23I_INTERNAL_COUNTER_READ			0x29
#define PCPG23I_INTERNAL_COMPARATE_DATA_WRITE	0x2a
#define PCPG23I_INTERNAL_COMPARATE_DATA_READ		0x2b
#define PCPG23I_EXTERNAL_COUNTER_WRITE			0x2c
#define PCPG23I_EXTERNAL_COUNTER_READ			0x2d
#define PCPG23I_EXTERNAL_COMPARATE_DATA_WRITE	0x2e
#define PCPG23I_EXTERNAL_COMPARATE_DATA_READ		0x2f
#define PCPG23I_INTERNAL_PRE_SCALE_DATA_WRITE	0x30
#define PCPG23I_INTERNAL_PRE_SCALE_DATA_READ		0x31
#define PCPG23I_EXTERNAL_PRE_SCALE_DATA_WRITE	0x32
#define PCPG23I_EXTERNAL_PRE_SCALE_DATA_READ		0x33
#define PCPG23I_CLEAR_SIGNAL_SELECT				0x34
#define PCPG23I_ONE_TIME_CLEAR_REQUEST			0x35
#define PCPG23I_FULL_TIME_CLEAR_REQUEST			0x36
#define PCPG23I_CLEAR_REQUEST_RESET				0x37
#define PCPG23I_REVERSE_COUNT_MODE_SET			0x38
#define PCPG23I_REVERSE_COUNT_MODE_RESET			0x39
#define PCPG23I_NO_OPERATION						0x3a
#define PCPG23I_STRAIGHT_ACCELERATE_MODE_SET		0x84
#define PCPG23I_US_STRAIGHT_ACCELERATE_MODE_SET	0x85
#define PCPG23I_S_CURVE_ACCELERATE_MODE_SET		0x86
#define PCPG23I_US_S_CURVE_ACCELERATE_MODE_SET	0x87
#define PCPG23I_SW1_DATA_WRITE					0x88
#define PCPG23I_SW1_DATA_READ					0x89
#define PCPG23I_SW2_DATA_WRITE					0x8a
#define PCPG23I_SW2_DATA_READ					0x8b
#define PCPG23I_SLOW_DOWN_LIMIT_ENABLE_MODE_SET	0x8c
#define PCPG23I_SLOW_DOWN_LIMIT_ENABLE_MODE_RESET 	0x8d
#define PCPG23I_EMERGENCY_LIMIT_ENABLE_MODE_SET	    0x8e
#define PCPG23I_EMERGENCY_LIMIT_ENABLE_MODE_RESET 	0x8f
#define PCPG23I_INITIAL_CLEAR					0x90
#define PCPG23I_MODE1_READ						0xa0
#define PCPG23I_MODE2_READ						0xa1
#define PCPG23I_STATUS_READ						0xa2
#define PCPG23I_SLOW_DOWN_STOP2					0xa3
#define PCPG23I_RISE_PULSE_COUNTER_READ			0xa6
#define PCPG23I_BI_PHASE_PULSE_SELECT_DATA_WRITE	0xaa
#define PCPG23I_BI_PHASE_PULSE_SELECT_DATA_READ	0xab
#define PCPG23I_SIGNAL_SERH2_REAR_PULSE_DATA_WRITE	0xac
#define PCPG23I_SIGNAL_SERH2_REAR_PULSE_DATA_READ	0xad
#define PCPG23I_MANUAL_PULSE_MODE_WRITE			0xc0
#define PCPG23I_MANUAL_PULSE_MODE_READ			0xc1
#define PCPG23I_SOFT_SYNC_MODE_WRITE				0xc2
#define PCPG23I_SOFT_SYNC_MODE_READ				0xc3
#define PCPG23I_SOFT_SYNC_EXECUTE				0xc4

#define MAX_AXIS 			8
#define PCPG23I_CAUTION		2
#define	BAS_RNG	 			(32768000.0 / 32768.0)
#define	BAS_RTF	 			(32768000.0 / 8.0)
#define	BAS_RTF2			(32768000.0 / 16.0)
#define	RATEDAT				(8.0 / 32768000.0)
#define	PCPG23I_STRAIGHT_MOVE		-1

typedef struct{
	double			dLoSpd[MAX_AXIS];
	double			dHiSpd[MAX_AXIS];
	double			dObjSpd[MAX_AXIS];
	short			sUpDnSpd[MAX_AXIS];
	double			dSCurve[MAX_AXIS];
} PCPG23I_SPD_PARAM, *pPCPG23I_SPD_PARAMS;

#ifdef __cplusplus
}
#endif /* __cplusplus */
