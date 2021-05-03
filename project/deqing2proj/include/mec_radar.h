#ifndef  __MEC_RADAR_H__
#define  __MEC_RADAR_H__

#include <string>
#include <vector>
#include "time.h"
#include <stdio.h>
#include <fstream>
#include "radar_log.h"

typedef  unsigned char      BYTE;
typedef  unsigned short     WORD;
typedef  unsigned long      DWORD;
typedef  unsigned long long DDWORD;

#define INVALID_SOCKET          -1
#define SOCKET_ERROR	        -1
#define LOOP_BUF_SIZE_MIN       40          //ÿ֡����С���ȣ�40
#define LOOP_BUF_SIZE           9256        //ÿ֡����󳤶ȣ�40+256*36=9256
#define RADAR_PROTO_VER         "1.7.0.3"

typedef enum
{
    REAL_TARGET_DATA    = 0x01,
    POINT_CLOUD_DATA    = 0x02,
    TRAFFIC_FLOW_DATA   = 0x03
};
typedef struct
{
    WORD        wYear;                  // ��
    BYTE        bMonth;                 // ��
    BYTE        bDay;                   // ��
    BYTE        bHour;                  // ʱ
    BYTE        bMinute;                // ��
    BYTE        bSecond;                // ��
    WORD        wMilliSec;              // ����
    BYTE        bWeek;                  // ���� ��0-7��ʾ��һ�����գ�
}TTime;

typedef struct  
{
    WORD        wTargetId;              // Ŀ����
    BYTE        bLaneId;                // Ŀ����������
    BYTE        bTargetType;            // Ŀ������
    double      dblTargetLength;        // Ŀ�곤��
    double      dblTargetWidth;         // Ŀ����
    double      dblTargetHeight;        // Ŀ��߶�
    DWORD       dwReserve;              // �����ֶ�
    double      dblTargetCourseAngle;   // Ŀ��ƫ����
    double      dblXCoordinates;        // X����
    double      dblYCoordinates;        // Y����
    double      dblXaxisVelocity;       // X���ٶ�
    double      dblYaxisVelocity;       // Y���ٶ�
    double      dblVelocity;            // ����
    double      dblAcceleration;        // �˶�������ٶ�
    double      dblXaxisAcceleration;   // X����ٶ�
    double      dblYaxisAcceleration;   // Y����ٶ�
    double      dblTargetLongitude;     // Ŀ������㾭��
    double      dblTargetLatitude;      // Ŀ�������γ��
}TObjectData;

typedef struct
{
    double      dblDistance;            // �������
    double      dblSpeed;               // �����ٶ�
    double      dblAngle;               // �Ƕ�
}TPointCloudData;

typedef struct
{
    WORD        wStatPeriod;            // ͳ������
    WORD        wLaneId;                // Ŀ����������
    double      dblMonitorPos;          // ���λ��
    WORD        SmallTrafficFlow;       // С������
    WORD        GrandeTrafficFlow;      // ������
    WORD        VentiTrafficFlow;       // ��������
    WORD        TotalTrafficFlow;       // �ܳ�����
    double      dblAverageSpeed;        // ƽ������
    double      dblTimeHeadway;         // ��ͷʱ��
    double      dblTimeOccupancy;       // ������Ȧʱ��ռ����
    double      dblMaxQueueLength;      // ����Ŷӳ���
}TTrafficFlowData;



typedef struct  
{
    std::string strRadarId;             // �״����кţ�Э��涨�20�ַ�
    BYTE        bDataType;              // �������ͣ�0x01-Object 0x02-Cluster
    TTime       tDataTime;              // ����ʱ�䣺��ȷ��ms
    DWORD       dwDataLen;              // ���ݳ��ȣ�pbData����(���ݸ������������ݻ��������)
    void*       pbData;                 // �������ݣ���������(TObjectData*) ���� ��������(BYTE*)
}TRadarData;


class CLoopBuf
{
public:
    CLoopBuf();
    virtual ~CLoopBuf();

    void PutToLoopBuf(BYTE* pbBuf, WORD wLen);
    WORD RxFromLoopBuf(BYTE* pbRxBuf, WORD wBufSize);
    void DeleteFromLoopBuf(WORD wLen);
    WORD GetLoopBufLen();

private:
    WORD  m_wRxHead;
    WORD  m_wRxTail;
    BYTE  m_bLoopBuf[LOOP_BUF_SIZE];    //���յ�ѭ��������
};

typedef struct                          //��SOCKET���յ����ֽ���������Ч֡����Ҫ�õ��ı���
{
    WORD 	    m_wRxPtr; 
    WORD 	    m_nRxCnt; 
    WORD 	    m_nRxStep; 
    WORD 	    m_wRxFrmLen;
    WORD 	    m_wRxtry;
    CLoopBuf    m_cLoopBuff;
    BYTE        m_bRxBuf[LOOP_BUF_SIZE]; //Э��һ֡���֧��256��Ŀ�꣺256*36+40=9256
}TParseState;

typedef void (*pfnRadarDataCB)(TRadarData*, WORD);
extern bool RadarProtoInit(pfnRadarDataCB pfnFunc, WORD wPort);
extern void RadarProtoRun(LogWriter &L);


class CMecRadarPro
{
    typedef struct {
        BYTE        bType;
        WORD        wDataUnitLen;
        WORD        wDataUnitCnt;
        void        (CMecRadarPro::* pmfPtr)(TRadarData&, WORD, BYTE*);
    }TDataUnitType;

public:
    CMecRadarPro();
    virtual ~CMecRadarPro();
    bool    Init(pfnRadarDataCB pfnFunc, WORD wPort);
    void    Run(LogWriter &L);
    std::ofstream ouF;
    time_t start;



protected:

private:
    int     IsValidDataType(BYTE bType);
    int     GetLocalNormalIp(char* ip);
    int     IsValidIpAddress(const char* ip_address);
    BYTE    checkSum(BYTE* data, int start, int len);
    bool    VeryRxFrm(BYTE* pbBuff, WORD wLen);
    int     Receive(int m_Socket, BYTE* pbRxBuf, WORD wBufSize);
    void    HandleFrm(std::pair<int, TParseState>* pPairTemp,LogWriter &L);
    int     Parse(std::pair<int, TParseState>* pPairTemp, BYTE* pbSrcBuf, WORD len);
    int     RecvAndParseData(std::pair<int, TParseState>* pPairTemp,LogWriter &L);
    void    AcceptNewConect(LogWriter &L);
    void    RecvSocketData(LogWriter &L);
    void    HandleFrmType01(TRadarData& tData, WORD wUnitLen, BYTE* pbBuff);
    void    HandleFrmType02(TRadarData& tData, WORD wUnitLen, BYTE* pbBuff);
    void    HandleFrmType03(TRadarData& tData, WORD wUnitLen, BYTE* pbBuff);

    bool            m_fIsInit;    
    int             m_iSock;        
    pfnRadarDataCB  m_pfnDataCB;
    std::vector <std::pair<int, TParseState>> m_vecSock;
    TDataUnitType   m_tDataUnit[10];
};




static inline WORD WPA_GET_BE16(const BYTE* a)
{
    return (a[0] << 8) | a[1];
}

static inline WORD WPA_GET_LE16(const BYTE* a)
{
    return (a[1] << 8) | a[0];
}

static inline DWORD WPA_GET_BE24(const BYTE* a)
{
    return (a[0] << 16) | (a[1] << 8) | a[2];
}
static inline DWORD WPA_GET_LE24(const BYTE* a)
{
    return (a[2] << 16) | (a[1] << 8) | a[0];
}
static inline DWORD WPA_GET_BE32(const BYTE* a)
{
    return (a[0] << 24) | (a[1] << 16) | (a[2] << 8) | a[3];
}
static inline DWORD WPA_GET_LE32(const BYTE* a)
{
    return (a[3] << 24) | (a[2] << 16) | (a[1] << 8) | a[0];
}

#endif

