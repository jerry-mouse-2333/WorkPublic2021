#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>

#include <iomanip>
#include <sstream>
#include <string.h>

#include "mec_radar.h"

#define RADAR_LCO_START					0
#define RADAR_LCO_FRAME_LEN			    4
#define RADAR_LCO_FRAME_TYPE			6
#define RADAR_LCO_CRC			        7
#define RADAR_LCO_DEV_ID			    8
#define RADAR_LCO_FRAME_TIME			28
#define RADAR_LCO_FRAME_DATA			36

std::string Dump2Hex(const char* pbuf, unsigned int len);


CMecRadarPro g_MecRadar;
bool RadarProtoInit(pfnRadarDataCB pfnFunc, WORD wPort)
{
    return g_MecRadar.Init(pfnFunc, wPort);
}
void RadarProtoRun(LogWriter &L)
{
    g_MecRadar.Run(L);  
}

CMecRadarPro::CMecRadarPro()
{
    m_fIsInit       = false;
    m_iSock         = INVALID_SOCKET;

    m_tDataUnit[0].bType = REAL_TARGET_DATA;
    m_tDataUnit[0].wDataUnitLen = 36;
    m_tDataUnit[0].wDataUnitCnt = 256;
    m_tDataUnit[0].pmfPtr = &CMecRadarPro::HandleFrmType01;
    m_tDataUnit[1].bType = POINT_CLOUD_DATA;
    m_tDataUnit[1].wDataUnitLen = 8;
    m_tDataUnit[1].wDataUnitCnt = 256;
    m_tDataUnit[1].pmfPtr = &CMecRadarPro::HandleFrmType02;
    m_tDataUnit[2].bType = TRAFFIC_FLOW_DATA;
    m_tDataUnit[2].wDataUnitLen = 22;
    m_tDataUnit[2].wDataUnitCnt = 256;
    m_tDataUnit[2].pmfPtr = &CMecRadarPro::HandleFrmType03;
}

CMecRadarPro::~CMecRadarPro()
{

}

int CMecRadarPro::IsValidDataType(BYTE bType)
{
    WORD wDataCnt = (sizeof(m_tDataUnit) / sizeof(m_tDataUnit[0]));
    for (int i = 0; i < wDataCnt; i++)
    {
        if (m_tDataUnit[i].bType == bType)
            return i;
    }

    return -1;
}

int CMecRadarPro::IsValidIpAddress(const char* ip_address) 
{
    struct sockaddr_in sa;
    int result = inet_pton(AF_INET, ip_address, &(sa.sin_addr));
    return result;
}

bool CMecRadarPro::Init(pfnRadarDataCB pfnFunc, WORD wPort)
{
    if ((!m_fIsInit) && (pfnFunc!=NULL) && (wPort > 0))//��δ��ʼ��
    {        
        m_pfnDataCB     = pfnFunc;
        
        m_iSock = socket(PF_INET, SOCK_STREAM, 0);
        if (m_iSock == INVALID_SOCKET)
        {
            printf("SockSvrThread: fail to create socket.\r\n");
            return false;
        }

        unsigned int arg = 1;
        if (ioctl(m_iSock, FIONBIO, (unsigned int*)&arg) != 0)
        {
            printf("CSocketIf::InitSock: ioctl fail.\r\n");
            close(m_iSock);
            return false;
        }

        int on = 1;
        if (setsockopt(m_iSock, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)) < 0)
        {
            printf("setsockopt error:%s\n", strerror(errno));
            close(m_iSock);
            return false;
        }

        struct  sockaddr_in local_addr;
        local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        local_addr.sin_family = AF_INET;
        local_addr.sin_port = htons(wPort);

        if (bind(m_iSock, (struct sockaddr*)&local_addr, sizeof(local_addr)) != 0)
        {
            close(m_iSock);
            printf("SockSvrThread: fail to bind socket\n");
            return false;
        }

        if (listen(m_iSock, 1) != 0)
        {
            close(m_iSock);
            printf("SockSvrThread: fail to listen socket.\r\n");
            return false;
        }

        printf("radar proto ver: %s\r\n", RADAR_PROTO_VER);
        printf("Tcp Server Start Listen: prot = %d\r\n", wPort);
        m_fIsInit = true;
    }
    
    start = time(NULL);
    ouF.open("/root/project/src/raw.dat", std::ofstream::binary);
    return true;
}

int CMecRadarPro::GetLocalNormalIp(char* ip)
{    
#if 0
    struct ifaddrs* ifAddrStruct;
    void* tmpAddrPtr = NULL;
    getifaddrs(&ifAddrStruct);
    while (ifAddrStruct != NULL) 
    {
        if (ifAddrStruct->ifa_addr->sa_family == AF_INET) 
        {
            tmpAddrPtr = &((struct sockaddr_in*)ifAddrStruct->ifa_addr)->sin_addr;
            inet_ntop(AF_INET, tmpAddrPtr, ip, INET_ADDRSTRLEN);
            printf("%s IP Address:%s\n", ifAddrStruct->ifa_name, ip);
            if (strcmp(ip, "127.0.0.1") != 0)
            {
                printf("GetLocalIp Success: %s\r\n", ip);
                break;
            }
        }
        ifAddrStruct = ifAddrStruct->ifa_next;
    }

    freeifaddrs(ifAddrStruct);
#endif
    return 0;
    
}

void CMecRadarPro::AcceptNewConect(LogWriter &L)
{
    //2021-04-02 WJJ
    std::pair<int,vector<string>>sock2ip;
    struct  sockaddr_in remote_addr;
    int addrlen = sizeof(remote_addr);
    int iNewSock = accept(m_iSock, (struct sockaddr*)&remote_addr, (socklen_t*)&addrlen);
    if (iNewSock != INVALID_SOCKET)
    {
        DWORD dwRemoteIP = ntohl(remote_addr.sin_addr.s_addr);
        unsigned int arg = 1;
        ioctl(iNewSock, FIONBIO, (unsigned int*)&arg);

        TParseState tState;
        tState.m_wRxPtr     = 0;
        tState.m_nRxCnt     = 0;
        tState.m_nRxStep    = 0;
        tState.m_wRxFrmLen  = 0;
        tState.m_wRxtry     = 0;
        memset(tState.m_bRxBuf, 0x00, sizeof(tState.m_bRxBuf));
        // m_vecSock.push_back({ iNewSock, tState}); //ά������TCP���ӵ�vector
        if(((dwRemoteIP >> 24) & 0xff) == 172)
        {
                m_vecSock.push_back({ iNewSock, tState}); //??��??TCPl???ector

                //2021-04-02 WJJ ����ǵ�һ�γ��֣���ʼ��ÿ��socket��IP��state
                //����ǶϿ���������ɾ��֮ǰ��
                if (L.sock_ip_maps.find(iNewSock) != L.sock_ip_maps.end()){
                    L.sock_ip_maps.erase(iNewSock);
                }
                sock2ip.first = iNewSock;
                sock2ip.second.push_back("null");
                sock2ip.second.push_back("null");
                L.sock_ip_maps.insert(sock2ip);
        }
        printf("Tcp Server: accept client %d.%d.%d.%d : %d\n", (dwRemoteIP >> 24) & 0xff, (dwRemoteIP >> 16) & 0xff, (dwRemoteIP >> 8) & 0xff, dwRemoteIP & 0xff, ntohs(remote_addr.sin_port));
        printf("Tcp Client: socket=%d  (total connect count=%d)\r\n", iNewSock, m_vecSock.size());
    }
}

int CMecRadarPro::Receive(int m_Socket, BYTE* pbRxBuf, WORD wBufSize)
{
    int len = 0;
    if (m_Socket != INVALID_SOCKET)
    {
        len = recv(m_Socket, (char*)pbRxBuf, wBufSize, 0);
    }

    if (len == 0)
    {
        printf("socket: %d has been closed (receive = %d)\r\n", m_Socket, len);
        return -1; //socket�ӿ��Ѿ�ʧЧ
    }
    else if (len == SOCKET_ERROR)
    {
        int iLastErr = errno;
        if (iLastErr == EWOULDBLOCK)
            return 0;
        else if (iLastErr == 0)
            return 0;
        else
        {
            printf("socket: %d has been closed (receive = %d)\r\n", m_Socket, len);
            return -1;//socket�ӿ��Ѿ�ʧЧ
        }
    }
    else
    {
        //printf("socked receive bytes: %d   %d\r\n", m_Socket, len);
    
        //Dump2Hex((const char*)pbRxBuf, len);

        //printf("%s", Dump2Hex((const char*)pbRxBuf, len).c_str());

        return len;
    }
}


BYTE CMecRadarPro::checkSum(BYTE* data, int start, int len)
{
    int sum = 0;
    for (int i = start; i < len; i++)
    {
        sum += *(data + i);
    }

    unsigned char ret = sum & 0xFF;
    return ret;
}


bool CMecRadarPro::VeryRxFrm(BYTE* pbBuff, WORD wLen)
{
    if ((pbBuff != NULL) && (wLen >= LOOP_BUF_SIZE_MIN))
    {
        //��ͷУ��
        if (WPA_GET_BE32(pbBuff+ RADAR_LCO_START) != 0x55AA55BB)
        {
            printf("frame head error\r\n");
            return false;
        }

        //����У��
        BYTE bDataType = pbBuff[RADAR_LCO_FRAME_TYPE];
        int index = IsValidDataType(bDataType);
        if (index < 0)
        {
            printf("data type error\r\n");
            return false;
        }

        //����У��
        int wDataLen = WPA_GET_BE16(pbBuff + RADAR_LCO_FRAME_LEN);
        if (((wDataLen + 8 - 40) % m_tDataUnit[index].wDataUnitLen) != 0)
        {
            printf("data unit len error\r\n");
            return false;
        }

        //CRCУ��
        if (checkSum(pbBuff + RADAR_LCO_DEV_ID, 0, wDataLen -4) != pbBuff[RADAR_LCO_CRC])
        {
            printf("check sum error\r\n");
            return false;
        }

        //���У��
        //ʱ��У��
        //����У��
        //��βУ��
        if (WPA_GET_BE32(pbBuff + wLen - 4) != 0x55CC55DD)
        {
            printf("frame tail error\r\n");
            return false;
        }

        return true;
    }

    return false;
}

int CMecRadarPro::Parse(std::pair<int, TParseState>* pPairTemp, BYTE* pbSrcBuf, WORD len)
{       
    int i = 0;
    BYTE* pbBlock = pbSrcBuf;
    BYTE* pbBuff  = pPairTemp->second.m_bRxBuf;

    for (i = 0; i < len; i++)
    {
        BYTE b = *pbBlock++;
        switch (pPairTemp->second.m_nRxStep)
        {
        case 0:
            if (b == 0x55)
            {
	            memset(pPairTemp->second.m_bRxBuf, 0x00, LOOP_BUF_SIZE);
	            pPairTemp->second.m_bRxBuf[0]   = 0x55;
	            pPairTemp->second.m_wRxPtr      = 1;    // ���׶� �Ѿ�����
	            pPairTemp->second.m_nRxCnt      = 3;    // �½׶� ��Ҫ����
	            pPairTemp->second.m_nRxStep     = 1;
            }
            break;

        case 1:
            pPairTemp->second.m_bRxBuf[pPairTemp->second.m_wRxPtr++] = b;
            pPairTemp->second.m_nRxCnt--;
            if (pPairTemp->second.m_nRxCnt == 0)    
            {
                if ((pbBuff[1] == 0xAA) && (pbBuff[2] == 0x55) && (pbBuff[3] == 0xBB))
                {
                    pPairTemp->second.m_nRxCnt  = 2;    
                    pPairTemp->second.m_nRxStep = 2;
                }
                else
                {
                    pPairTemp->second.m_nRxStep = 0;
                    printf("case 1 over: failed\r\n");
                }
            }
            break;

        case 2:
            pPairTemp->second.m_bRxBuf[pPairTemp->second.m_wRxPtr++] = b;
            pPairTemp->second.m_nRxCnt--;
            if (pPairTemp->second.m_nRxCnt == 0)
            {
                pPairTemp->second.m_wRxFrmLen = (WPA_GET_BE16(pbBuff+4) + 8); //(((WORD)pbBuff[4] << 8) | pbBuff[5]) + 8;
                WORD wTmp = pPairTemp->second.m_wRxFrmLen;
                if ((wTmp>= LOOP_BUF_SIZE_MIN) && (wTmp <= LOOP_BUF_SIZE))
                {
                    pPairTemp->second.m_nRxCnt  = (pPairTemp->second.m_wRxFrmLen - 6);
                    pPairTemp->second.m_nRxStep = 3;
                }
                else
                {
                    pPairTemp->second.m_nRxStep = 0;
                    printf("case 2 over: failed\r\n");
                }
            }
            break;

        case 3:
            pPairTemp->second.m_bRxBuf[pPairTemp->second.m_wRxPtr++] = b;
            pPairTemp->second.m_nRxCnt--;
            if (pPairTemp->second.m_nRxCnt == 0)
            {
                pPairTemp->second.m_nRxStep = 0;
                if (VeryRxFrm(pbBuff, pPairTemp->second.m_wRxFrmLen))
                {
                    return (i + 1);
                }

                printf("case 3 over: failed\r\n");
            }
            break;

        default:
            pPairTemp->second.m_nRxStep = 0;
            break;
        }
    } 

    return -len;
}

void CMecRadarPro::HandleFrmType01(TRadarData & tData, WORD wUnitLen, BYTE* pbBuff)
{
    TObjectData* ptData = new TObjectData[tData.dwDataLen];
    if (ptData != NULL)
    {
        for (int i = 0; i < tData.dwDataLen; i++)
        {   
            ptData[i].wTargetId             = WPA_GET_BE16(pbBuff);                                      //Ŀ����            
            ptData[i].bLaneId               = pbBuff[2];                                                 //Ŀ����������            
            ptData[i].bTargetType           = pbBuff[3];                                                 //Ŀ������            
            ptData[i].dblTargetLength       = (pbBuff[4] * 0.1);                                         //Ŀ�곤��           
            ptData[i].dblTargetWidth        = (pbBuff[5] * 0.1);                                         //Ŀ����            
            ptData[i].dblTargetHeight       = (pbBuff[6] * 0.1);                                         //Ŀ��߶�            
            ptData[i].dwReserve             = WPA_GET_BE24(pbBuff  + 7);                                 //����            
            ptData[i].dblTargetCourseAngle  = ((WPA_GET_BE16(pbBuff  + 10)) * 0.1);                      //Ŀ��ƫ����            
            ptData[i].dblXCoordinates       = ((signed short int)(WPA_GET_BE16(pbBuff  + 12)) * 0.01);   //X ����           
            ptData[i].dblYCoordinates       = ((unsigned short int)(WPA_GET_BE16(pbBuff  + 14)) * 0.01);   //Y ����            
            ptData[i].dblXaxisVelocity      = ((signed short int)(WPA_GET_BE16(pbBuff  + 16)) * 0.01);   //X ���ٶ�            
            ptData[i].dblYaxisVelocity      = ((signed short int)(WPA_GET_BE16(pbBuff  + 18)) * 0.01);   //Y ���ٶ�            
            ptData[i].dblVelocity           = ((signed short int)(WPA_GET_BE16(pbBuff  + 20)) * 0.01);   //����            
            ptData[i].dblAcceleration       = ((signed short int)(WPA_GET_BE16(pbBuff  + 22)) * 0.01);   //Ŀ���˶�������ٶ�            
            ptData[i].dblXaxisAcceleration  = ((signed short int)(WPA_GET_BE16(pbBuff  + 24)) * 0.01);   //X ����ٶ�            
            ptData[i].dblYaxisAcceleration  = ((signed short int)(WPA_GET_BE16(pbBuff  + 26)) * 0.01);   //Y ����ٶ�            
            ptData[i].dblTargetLongitude    = ((signed int)(WPA_GET_BE32(pbBuff  + 28)) * 0.0000001);    //Ŀ������㾭��            
            ptData[i].dblTargetLatitude     = ((signed int)(WPA_GET_BE32(pbBuff  + 32)) * 0.0000001);    //Ŀ�������γ��

            pbBuff += wUnitLen;
        }

        tData.pbData = ptData;
    }
}

void CMecRadarPro::HandleFrmType02(TRadarData& tData, WORD wUnitLen, BYTE* pbBuff)
{
    TPointCloudData* ptData = new TPointCloudData[tData.dwDataLen];
    if (ptData != NULL)
    {
        for (int i = 0; i < tData.dwDataLen; i++)
        {
            ptData[i].dblDistance   = (WPA_GET_BE32(pbBuff) * 0.1);
            ptData[i].dblSpeed      = ((signed short int)(WPA_GET_BE16(pbBuff + 4)) * 0.1);
            ptData[i].dblAngle      = ((signed short int)(WPA_GET_BE16(pbBuff + 6)) * 0.1);

            pbBuff += wUnitLen;
        }

        tData.pbData = ptData;
    }
}

void CMecRadarPro::HandleFrmType03(TRadarData& tData, WORD wUnitLen, BYTE* pbBuff)
{
    TTrafficFlowData* ptData = new TTrafficFlowData[tData.dwDataLen];
    if (ptData != NULL)
    {
        for (int i = 0; i < tData.dwDataLen; i++)
        {
            ptData[i].wStatPeriod         = WPA_GET_BE16(pbBuff);
            ptData[i].wLaneId             = WPA_GET_BE16(pbBuff+2);
            ptData[i].dblMonitorPos       = ((WPA_GET_BE16(pbBuff + 4)) * 0.1);
            ptData[i].SmallTrafficFlow    = WPA_GET_BE16(pbBuff + 6);
            ptData[i].GrandeTrafficFlow   = WPA_GET_BE16(pbBuff + 8);
            ptData[i].VentiTrafficFlow    = WPA_GET_BE16(pbBuff + 10);
            ptData[i].TotalTrafficFlow    = WPA_GET_BE16(pbBuff + 12);
            ptData[i].dblAverageSpeed     = ((WPA_GET_BE16(pbBuff + 14)) * 0.1);
            ptData[i].dblTimeHeadway      = ((WPA_GET_BE16(pbBuff + 16)) * 0.1);
            ptData[i].dblTimeOccupancy    = ((WPA_GET_BE16(pbBuff + 18)) * 0.1);
            ptData[i].dblMaxQueueLength   = ((WPA_GET_BE16(pbBuff + 20)) * 0.1);

            pbBuff += wUnitLen;
        }

        tData.pbData = ptData;
    }
}

//2021-04-02 WJJ add sock_ip_map: socket to radar_ip (m_vecSock.first(i.e. pPairTemp.first) map to strRadarId(i.e. radarIP))
void CMecRadarPro::HandleFrm(std::pair<int, TParseState>* pPairTemp,LogWriter &L)
{
    //������������ô����һ���Ϸ�֡����������У��֮��ļ�鹤��
    BYTE* pbBuff    = pPairTemp->second.m_bRxBuf;
    BYTE  bDataType = pbBuff[RADAR_LCO_FRAME_TYPE];
    int   iIndex    = IsValidDataType(bDataType);
    WORD  wUnitLen  = m_tDataUnit[iIndex].wDataUnitLen;
    WORD  wUnitCnt  = ((pPairTemp->second.m_wRxFrmLen - 40) / wUnitLen);

    // printf("wUnitLen = %d wUnitCnt = %d\r\n", wUnitLen, wUnitCnt);
    TRadarData   tRadarDatas;
    tRadarDatas.strRadarId.assign((char*)(pbBuff + RADAR_LCO_DEV_ID), 20);

    //2021-04-02 WJJ ���ÿ��socket��Ӧ���״�IP,���ó�ʼ״̬Ϊ����
    std::string device_name = tRadarDatas.strRadarId.substr(0,tRadarDatas.strRadarId.find_first_of('\0'));
    map<int, vector<string>>::iterator iter;
    iter = L.sock_ip_maps.find(pPairTemp->first);
    if (iter != L.sock_ip_maps.end()){           //����ǰ�״��socket��map���Ǵ��ڵģ���ô������
        iter->second[0] =  device_name;
        iter->second[1] = "1";
    }

    tRadarDatas.bDataType           = pbBuff[RADAR_LCO_FRAME_TYPE];
    tRadarDatas.tDataTime.wYear     = (pbBuff[RADAR_LCO_FRAME_TIME] + 2000);
    tRadarDatas.tDataTime.bMonth    = (pbBuff[RADAR_LCO_FRAME_TIME + 1] & 0x0F);
    tRadarDatas.tDataTime.bDay      = (pbBuff[RADAR_LCO_FRAME_TIME + 2] & 0x1F);
    tRadarDatas.tDataTime.bHour     = (pbBuff[RADAR_LCO_FRAME_TIME + 3] & 0x1F);
    tRadarDatas.tDataTime.bMinute   = (pbBuff[RADAR_LCO_FRAME_TIME + 4] & 0x3F);
    tRadarDatas.tDataTime.bSecond   = (pbBuff[RADAR_LCO_FRAME_TIME + 5] & 0x3F);
    tRadarDatas.tDataTime.wMilliSec = (WPA_GET_BE16(pbBuff + RADAR_LCO_FRAME_TIME + 6) & 0x03FF);
    tRadarDatas.dwDataLen           = wUnitCnt;

    //2021-04-02 WJJ ���¹���״̬
    
    if (wUnitCnt == 0){
        if (iter->second[0] == device_name){
            iter->second[1] = "2";
        }
    }

    tRadarDatas.pbData              = NULL;

    if (wUnitCnt > 0)
        (this->*(m_tDataUnit[iIndex].pmfPtr))(tRadarDatas, wUnitLen, pbBuff+RADAR_LCO_FRAME_DATA);

    if (m_pfnDataCB != NULL)
    {
        m_pfnDataCB(&tRadarDatas, 1);
    }

    if (tRadarDatas.pbData != NULL)
        delete[] tRadarDatas.pbData;



#if 0
    if (wUnitCnt > 0)
    {
        if (wUnitLen == 36)
        {
            ptObjAry = new TObjectData[wUnitCnt];
            if (ptObjAry != NULL)
            {
                for (int i = 0; i < wUnitCnt; i++)
                {
                    pbBuff = (pPairTemp->second.m_bRxBuf + (i * 36));
                    //Ŀ����
                    ptObjAry[i].wTargetId = WPA_GET_BE16(pbBuff + RADAR_LCO_FRAME_DATA);
                    //Ŀ����������
                    ptObjAry[i].bLaneId = pbBuff[RADAR_LCO_FRAME_DATA + 2];
                    //Ŀ������
                    ptObjAry[i].bTargetType = pbBuff[RADAR_LCO_FRAME_DATA + 3];
                    //Ŀ�곤��
                    ptObjAry[i].dblTargetLength = (pbBuff[RADAR_LCO_FRAME_DATA + 4] * 0.1);
                    //Ŀ����
                    ptObjAry[i].dblTargetWidth = (pbBuff[RADAR_LCO_FRAME_DATA + 5] * 0.1);
                    //Ŀ��߶�
                    ptObjAry[i].dblTargetHeight = (pbBuff[RADAR_LCO_FRAME_DATA + 6] * 0.1);
                    //����
                    ptObjAry[i].dwReserve = WPA_GET_BE24(pbBuff + RADAR_LCO_FRAME_DATA + 7);
                    //Ŀ��ƫ����
                    ptObjAry[i].dblTargetCourseAngle = ((WPA_GET_BE16(pbBuff + RADAR_LCO_FRAME_DATA + 10)) * 0.1);
                    //X ����
                    ptObjAry[i].dblXCoordinates = ((signed short int)(WPA_GET_BE16(pbBuff + RADAR_LCO_FRAME_DATA + 12)) * 0.01);
                    //Y ����
                    ptObjAry[i].dblYCoordinates = ((signed short int)(WPA_GET_BE16(pbBuff + RADAR_LCO_FRAME_DATA + 14)) * 0.01);
                    //X ���ٶ�
                    ptObjAry[i].dblXaxisVelocity = ((signed short int)(WPA_GET_BE16(pbBuff + RADAR_LCO_FRAME_DATA + 16)) * 0.01);
                    //Y ���ٶ�
                    ptObjAry[i].dblYaxisVelocity = ((signed short int)(WPA_GET_BE16(pbBuff + RADAR_LCO_FRAME_DATA + 18)) * 0.01);
                    //����
                    ptObjAry[i].dblVelocity = ((signed short int)(WPA_GET_BE16(pbBuff + RADAR_LCO_FRAME_DATA + 20)) * 0.01);
                    //Ŀ���˶�������ٶ�
                    ptObjAry[i].dblAcceleration = ((signed short int)(WPA_GET_BE16(pbBuff + RADAR_LCO_FRAME_DATA + 22)) * 0.01);
                    //X ����ٶ�
                    ptObjAry[i].dblXaxisAcceleration = ((signed short int)(WPA_GET_BE16(pbBuff + RADAR_LCO_FRAME_DATA + 24)) * 0.01);
                    //Y ����ٶ�
                    ptObjAry[i].dblYaxisAcceleration = ((signed short int)(WPA_GET_BE16(pbBuff + RADAR_LCO_FRAME_DATA + 26)) * 0.01);
                    //Ŀ������㾭��
                    ptObjAry[i].dblTargetLongitude = ((signed int)(WPA_GET_BE32(pbBuff + RADAR_LCO_FRAME_DATA + 28)) * 0.0000001);
                    //Ŀ�������γ��
                    ptObjAry[i].dblTargetLatitude = ((signed int)(WPA_GET_BE32(pbBuff + RADAR_LCO_FRAME_DATA + 32)) * 0.0000001);
                }

                tRadarDatas.pbData = ptObjAry;
            }
        }
        else
        {
            ptCldAry = new TPointCloudData[wUnitCnt];
            if (ptCldAry != NULL)
            {
                for (int i = 0; i < wUnitCnt; i++)
                {
                    pbBuff = (pPairTemp->second.m_bRxBuf + (i * 8));
                    ptCldAry[i].dblDistance = (WPA_GET_BE32(pbBuff + RADAR_LCO_FRAME_DATA) * 0.1);
                    ptCldAry[i].dblSpeed = ((signed short int)(WPA_GET_BE16(pbBuff + RADAR_LCO_FRAME_DATA + 4)) * 0.1);
                    ptCldAry[i].dblAngle = ((signed short int)(WPA_GET_BE16(pbBuff + RADAR_LCO_FRAME_DATA + 6)) * 0.1);
                }

                tRadarDatas.pbData = ptCldAry;
            }
        }
    }
#endif



}

int CMecRadarPro::RecvAndParseData(std::pair<int, TParseState>* pPairTemp,LogWriter &L)
{
    //1.�ӽӿ�ȡ����Ƭ�����ݣ����Ƶ�ѭ��������
    BYTE  bBuf[4096] = {0x00};
    int  len = Receive(pPairTemp->first, bBuf, sizeof(bBuf)-10);
    if (len < 0)
    {
        printf("socket happen errors: socket=%d\r\n", pPairTemp->first);
        return -1;
    }

    if (len > 0)
        pPairTemp->second.m_cLoopBuff.PutToLoopBuf(bBuf, len);
    len = pPairTemp->second.m_cLoopBuff.GetLoopBufLen();
    if (len <= 0)
    {
        //printf("buff have no datas\r\n");
        return 0;
    }

    //2.��ѭ���������в�ͣ�������� �����е����˵����socket�ӿ�Ŀǰ�����������vec��ɾ����
    while (len > 0)
    {
        len = pPairTemp->second.m_cLoopBuff.RxFromLoopBuf(bBuf, LOOP_BUF_SIZE - 10);
        if (len > 0)
        {
            int nScanLen = Parse(pPairTemp, bBuf, len);
            if (nScanLen > 0)//�ɹ�����һ֡����
            {
                pPairTemp->second.m_cLoopBuff.DeleteFromLoopBuf(nScanLen);
                HandleFrm(pPairTemp,L);
            }
            else //��������֡����
            {
                pPairTemp->second.m_cLoopBuff.DeleteFromLoopBuf(nScanLen);
                break;
            }
        }
        else
        {
            break;
        }
    }

    return 0;
}

void CMecRadarPro::RecvSocketData(LogWriter &L)
{
    std::vector<std::pair<int, TParseState>>::iterator it;
    for (it = m_vecSock.begin(); it != m_vecSock.end();)  //���������Ѿ��������ӵ�TCP�ͻ���
    {
        if (RecvAndParseData(&*it,L) < 0)
        {
            int tmp = it->first;
            it = m_vecSock.erase(it);

            //2021-04-02 WJJ  ���������״�״̬Ϊ3
            L.sock_ip_maps.find(tmp)->second[1] = "0";
            printf("remove invalid socket: %d (total connect cnt=%d)\r\n", tmp, m_vecSock.size());

            continue;
        }

        ++it;
    }
}

void CMecRadarPro::Run(LogWriter &L)
{
    if (!m_fIsInit)
    {
        printf("error: not init!!!\r\n");
        return;
    }

    AcceptNewConect(L);
    RecvSocketData(L);
}



CLoopBuf::CLoopBuf()
{
    m_wRxHead = 0;
    m_wRxTail = 0;
    memset(m_bLoopBuf, 0x00, sizeof(m_bLoopBuf));
}

CLoopBuf::~CLoopBuf()
{

}


WORD CLoopBuf::GetLoopBufLen()
{
    if (m_wRxHead >= m_wRxTail)
        return m_wRxHead - m_wRxTail;
    else
        return m_wRxHead + LOOP_BUF_SIZE - m_wRxTail;
}

void CLoopBuf::PutToLoopBuf(BYTE* pbBuf, WORD wLen)
{
    WORD WBuffLen = 0;
    if (wLen > LOOP_BUF_SIZE)
    {
        printf("CProto::PutToLoopBuf over,wLen>LOOP_BUF_SIZE.\n");
        return;
    }
    WBuffLen = (m_wRxHead + LOOP_BUF_SIZE - m_wRxTail) % LOOP_BUF_SIZE;
    if (wLen + WBuffLen > LOOP_BUF_SIZE)
    {
        printf("CProto::PutToLoopBuf over,(wLen+WBuffLen)>LOOP_BUF_SIZE.\n");
        return;
    }

    if (m_wRxHead + wLen <= LOOP_BUF_SIZE)
    {
        memcpy(&m_bLoopBuf[m_wRxHead], pbBuf, wLen);
    }
    else
    {
        memcpy(&m_bLoopBuf[m_wRxHead], pbBuf, LOOP_BUF_SIZE - m_wRxHead);
        memcpy(m_bLoopBuf, pbBuf + LOOP_BUF_SIZE - m_wRxHead, wLen - (LOOP_BUF_SIZE - m_wRxHead));
    }

    m_wRxHead += wLen;
    if (m_wRxHead >= LOOP_BUF_SIZE)
        m_wRxHead -= LOOP_BUF_SIZE;
}

WORD CLoopBuf::RxFromLoopBuf(BYTE* pbRxBuf, WORD wBufSize)
{
    WORD wRetLen;
    if (m_wRxHead != m_wRxTail)   //������Ϊ���ջ��������ڵ���ѭ��������
    {
        if (m_wRxHead > m_wRxTail)
        {
            wRetLen = m_wRxHead - m_wRxTail;
            if (wRetLen > wBufSize)
                wRetLen = wBufSize;

            memcpy(pbRxBuf, &m_bLoopBuf[m_wRxTail], wRetLen);
            return wRetLen;
        }
        else
        {
            //��������ݵ�ѭ���ѭ��������ת����һ�㻺������˳��
            wRetLen = LOOP_BUF_SIZE - m_wRxTail;
            if (wRetLen >= wBufSize)
            {
                memcpy(pbRxBuf, &m_bLoopBuf[m_wRxTail], wBufSize);
                return wBufSize;
            }
            else
            {
                memcpy(pbRxBuf, &m_bLoopBuf[m_wRxTail], wRetLen);

                if (wRetLen + m_wRxHead > wBufSize)
                {
                    memcpy(pbRxBuf + wRetLen, m_bLoopBuf, wBufSize - wRetLen);
                    return wBufSize;
                }
                else
                {
                    memcpy(pbRxBuf + wRetLen, m_bLoopBuf, m_wRxHead);
                    return wRetLen + m_wRxHead;
                }
            }
        }

        //����ֻ��peek����pop
    }
    else
    {
        return 0;
    }
}

void CLoopBuf::DeleteFromLoopBuf(WORD wLen)
{
    WORD wLeft = GetLoopBufLen();
    if (wLen > wLeft)
        wLen = wLeft;

    m_wRxTail = (m_wRxTail + wLen) % LOOP_BUF_SIZE;
}

std::string Dump2Hex(const char* pbuf, unsigned int len)
{

    #if 1 //���ԣ������������
    {    

        g_MecRadar.ouF.write(pbuf, len);


        if( time(NULL) - g_MecRadar.start  >= 300 )
            g_MecRadar.ouF.close();
    }
    #endif
       
    std::stringstream	ss_;
    const unsigned int	nperlen = 16;
    const unsigned char* ptr = (const unsigned char*)pbuf;
    unsigned int			ii_, jj_, curpos, lines_ = (len + nperlen - 1) / nperlen;

    ss_.fill('0');
    //ss_ << "Dump2Hex(" << len << "):" << std::endl;
    for (ii_ = 0; ii_ < lines_; ++ii_) {
        ss_ << std::setw(4) << ii_ << ": ";
        for (jj_ = 0; jj_ < nperlen; ++jj_) {
            curpos = ii_ * nperlen + jj_;
            if (curpos >= len) {
                ss_ << "   ";
            }
            else {
                ss_ << std::setw(2) << std::hex << (unsigned)ptr[curpos] << std::dec << ' ';
            }
            if (7 == jj_) {
                ss_ << "- ";
            }
        }
        ss_ << " ";
        for (jj_ = 0; jj_ < nperlen; ++jj_) {
            curpos = ii_ * nperlen + jj_;
            if (curpos >= len) {
                break;
            }

            if (isprint(ptr[curpos])) {
                ss_ << (char)ptr[curpos] << ' ';
            }
            else {
                ss_ << ". ";
            }
            if (7 == jj_) {
                ss_ << "- ";
            }
        }
        ss_ << std::endl;
    }

    return ss_.str();
}
