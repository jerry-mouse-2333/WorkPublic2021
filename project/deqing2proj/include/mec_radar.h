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
#define LOOP_BUF_SIZE_MIN       40          //每帧的最小长度：40
#define LOOP_BUF_SIZE           9256        //每帧的最大长度：40+256*36=9256
#define RADAR_PROTO_VER         "1.7.0.3"

typedef enum
{
    REAL_TARGET_DATA    = 0x01,
    POINT_CLOUD_DATA    = 0x02,
    TRAFFIC_FLOW_DATA   = 0x03
};
typedef struct
{
    WORD        wYear;                  // 年
    BYTE        bMonth;                 // 月
    BYTE        bDay;                   // 日
    BYTE        bHour;                  // 时
    BYTE        bMinute;                // 分
    BYTE        bSecond;                // 秒
    WORD        wMilliSec;              // 毫秒
    BYTE        bWeek;                  // 星期 （0-7表示周一到周日）
}TTime;

typedef struct  
{
    WORD        wTargetId;              // 目标编号
    BYTE        bLaneId;                // 目标所属车道
    BYTE        bTargetType;            // 目标类型
    double      dblTargetLength;        // 目标长度
    double      dblTargetWidth;         // 目标宽度
    double      dblTargetHeight;        // 目标高度
    DWORD       dwReserve;              // 保留字段
    double      dblTargetCourseAngle;   // 目标偏航角
    double      dblXCoordinates;        // X坐标
    double      dblYCoordinates;        // Y坐标
    double      dblXaxisVelocity;       // X轴速度
    double      dblYaxisVelocity;       // Y轴速度
    double      dblVelocity;            // 车速
    double      dblAcceleration;        // 运动方向加速度
    double      dblXaxisAcceleration;   // X轴加速度
    double      dblYaxisAcceleration;   // Y轴加速度
    double      dblTargetLongitude;     // 目标坐标点经度
    double      dblTargetLatitude;      // 目标坐标点纬度
}TObjectData;

typedef struct
{
    double      dblDistance;            // 径向距离
    double      dblSpeed;               // 径向速度
    double      dblAngle;               // 角度
}TPointCloudData;

typedef struct
{
    WORD        wStatPeriod;            // 统计周期
    WORD        wLaneId;                // 目标所属车道
    double      dblMonitorPos;          // 监测位置
    WORD        SmallTrafficFlow;       // 小车流量
    WORD        GrandeTrafficFlow;      // 大车流量
    WORD        VentiTrafficFlow;       // 超大车流量
    WORD        TotalTrafficFlow;       // 总车流量
    double      dblAverageSpeed;        // 平均车速
    double      dblTimeHeadway;         // 车头时距
    double      dblTimeOccupancy;       // 虚拟线圈时间占有率
    double      dblMaxQueueLength;      // 最大排队长度
}TTrafficFlowData;



typedef struct  
{
    std::string strRadarId;             // 雷达序列号：协议规定最长20字符
    BYTE        bDataType;              // 数据类型：0x01-Object 0x02-Cluster
    TTime       tDataTime;              // 数据时间：精确到ms
    DWORD       dwDataLen;              // 数据长度：pbData长度(数据个数：对象数据或点云数据)
    void*       pbData;                 // 数据内容：对象数据(TObjectData*) 或者 点云数据(BYTE*)
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
    BYTE  m_bLoopBuf[LOOP_BUF_SIZE];    //接收的循环缓冲区
};

typedef struct                          //从SOCKET接收到的字节流里捡出有效帧所需要用到的变量
{
    WORD 	    m_wRxPtr; 
    WORD 	    m_nRxCnt; 
    WORD 	    m_nRxStep; 
    WORD 	    m_wRxFrmLen;
    WORD 	    m_wRxtry;
    CLoopBuf    m_cLoopBuff;
    BYTE        m_bRxBuf[LOOP_BUF_SIZE]; //协议一帧最大支持256个目标：256*36+40=9256
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

