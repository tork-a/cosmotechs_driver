/*****************************************************************************
 * PCPG23iw.h
 *
 * PCPG-23I制御
 * Copyright CosmoTechs Co.,Ltd.
 *
 * Date 2016.02.05
 *****************************************************************************/
#include "pcpg23i.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define PCPG23I_DLL_NAME		"Pcpg23i.dll"		// DLL 名
#define PCPG23I_ERR_WRAPDLL	((DWORD)-1)		        // ラッパー関数内エラー

typedef int BOOL;
typedef void VOID;
typedef unsigned char BYTE;
typedef unsigned char* PBYTE;
typedef unsigned short WORD;
typedef unsigned short* PWORD;
typedef unsigned long DWORD;
typedef unsigned long* PDWORD;
typedef pciresource PPCPG23IR;

BOOL Pcpg23iwDllOpen();
BOOL Pcpg23iwDllClose();
BOOL Pcpg23iwGetLibVersion( PBYTE pbLibVer );
BOOL Pcpg23iwGetDrvVersion( PBYTE pbDrvVer );
DWORD Pcpg23iwGetLastError( WORD wBsn );
BOOL Pcpg23iwCreate( WORD wBsn );
BOOL Pcpg23iwClose( WORD wBsn );
BOOL Pcpg23iwGetResource(WORD wBsn, PPCPG23IR pres);
BOOL Pcpg23iwInPort( WORD wBsn, WORD wAxis ,WORD wPort, PBYTE pbData );
BOOL Pcpg23iwOutPort( WORD wBsn, WORD wAxis ,WORD wPort, BYTE bData );
BOOL Pcpg23iwGetDriveStatus( WORD wBsn, WORD wAxis, PBYTE pbStatus);
BOOL Pcpg23iwGetEndStatus( WORD wBsn, WORD wAxis, PBYTE pbStatus);
BOOL Pcpg23iwGetMechanicalSignal( WORD wBsn, WORD wAxis, PBYTE pbSignal );
BOOL Pcpg23iwGetUniversalSignal( WORD wBsn, WORD wAxis, PBYTE pbSignal );
BOOL Pcpg23iwMode1Write( WORD wBsn, WORD wAxis, BYTE bMode );
BOOL Pcpg23iwMode2Write( WORD wBsn, WORD wAxis, BYTE bMode );
BOOL Pcpg23iwUniversalSignalWrite( WORD wBsn, WORD wAxis, BYTE bSignal );
BOOL Pcpg23iwDataRead( WORD wBsn, WORD wAxis, BYTE bCmd, PBYTE pbData );
BOOL Pcpg23iwDataHalfRead( WORD wBsn, WORD wAxis, BYTE bCmd, PWORD pwData );
BOOL Pcpg23iwDataFullRead( WORD wBsn, WORD wAxis, BYTE bCmd, PDWORD pdwData );
BOOL Pcpg23iwCommandWrite( WORD wBsn, WORD wAxis, BYTE bCmd );
BOOL Pcpg23iwDataWrite( WORD wBsn, WORD wAxis, BYTE bCmd, BYTE bData );
BOOL Pcpg23iwDataHalfWrite( WORD wBsn, WORD wAxis, BYTE bCmd, WORD wData );
BOOL Pcpg23iwDataFullWrite( WORD wBsn, WORD wAxis, BYTE bCmd, DWORD dwData );
  //BOOL Pcpg23iwIRQCallBack( WORD wBsn, DWORD dwType, DWORD dwBusNo, DWORD dwLevel, DWORD dwMode, void (WINAPI *lhandler)(void));
  //BOOL Pcpg23iwIRQCallBackT( WORD wBsn, DWORD dwType, DWORD dwBusNo, DWORD dwLevel, DWORD dwMode, void (WINAPI *lhandler)(void));
BOOL Pcpg23iwFreeIRQ( WORD wBsn, DWORD dwLevel );
BOOL Pcpg23iwFreeIRQT( WORD wBsn, DWORD dwLevel );
VOID Pcpg23iwExLock();
VOID Pcpg23iwExUnLock();
BOOL Pcpg23iwMemOutPort( WORD wBsn, WORD wAdr, BYTE bData );
BOOL Pcpg23iwMemInPort( WORD wBsn, WORD wAdr, PBYTE pbData );

BOOL Pcpg23iwGetInternalCounter( WORD wBsn, WORD wAxis, PDWORD pdwData );
BOOL Pcpg23iwSetInternalCounter( WORD wBsn, WORD wAxis, DWORD dwData );
BOOL Pcpg23iwGetDrivePulseCounter( WORD wBsn, WORD wAxis, PDWORD pdwData );
BOOL Pcpg23iwGetNowSpeedData( WORD wBsn, WORD wAxis, PWORD pwData ); 

BOOL Pcpg23iwSpeedParameterRead(WORD wBsn, WORD wAxis, double *pdLowSpeed, double *pdHighSpeed, short *psAccTime, double *pdSRate);
BOOL Pcpg23iwSpeedRead(WORD wBsn, WORD wAxis, double *pdObjSpeed);
WORD Pcpg23iwSpeedParameterWrite(WORD wBsn, WORD wAxis, double dLowSpeed, double dHighSpeed, short sAccTime, double dSRate);
WORD Pcpg23iwSpeedWrite(WORD wBsn, WORD wAxis, double dObjSpeed);

BOOL Pcpg23iwEmergencyStop( WORD wBsn, WORD wAxis );
BOOL Pcpg23iwSlowStop( WORD wBsn, WORD wAxis );
BOOL Pcpg23iwMoveExec( WORD wBsn, WORD wAxis, long lData );
BOOL Pcpg23iwGetFrequency( WORD wBsn, WORD wAxis, WORD wKind, double *pdData );
BOOL Pcpg23iwStartSignalWrite( WORD wBsn, WORD wAxis, WORD wAxisStart );
BOOL Pcpg23iwSignalSearch( WORD wBsn, WORD wAxis, WORD wDir, WORD wKind, WORD wSignal );

BOOL Pcpg23iwOrg_Return( WORD wBsn, WORD wAxis, BYTE bTimer );
int	 Pcpg23iwOrg_End( WORD wBsn, WORD wAxis );
int	 Pcpg23iwOrg_WatchX( WORD wBsn, WORD wAxis );
int	 Pcpg23iwOrg_Sqnc0( WORD wBsn, WORD wAxis );
int	 Pcpg23iwOrg_Sqnc1( WORD wBsn, WORD wAxis );
int	 Pcpg23iwOrg_Sqnc2( WORD wBsn, WORD wAxis );
int	 Pcpg23iwOrg_Sqnc3( WORD wBsn, WORD wAxis );

BOOL Pcpg23iwGetSwitchValue(WORD wBsn,PDWORD pdwSwitchValue);
  //BOOL Pcpg23iwIRQCallBack1(WORD wBsn,DWORD dwType,DWORD dwBusNo,DWORD dwLevel,DWORD dwMode,void (WINAPI *)(void));
BOOL Pcpg23iwFreeIRQ1(WORD wBsn,DWORD dwLevel);
  //BOOL Pcpg23iwIRQCallBack2(WORD wBsn,DWORD dwType,DWORD dwBusNo,DWORD dwLevel,DWORD dwMode,void (WINAPI *)(void));
BOOL Pcpg23iwFreeIRQ2(WORD wBsn,DWORD dwLevel);

#ifdef __cplusplus
}
#endif /* __cplusplus */
