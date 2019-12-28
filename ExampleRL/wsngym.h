#ifndef MY_GYM_ENTITY_H
#define MY_GYM_ENTITY_H

#include "ns3/stats-module.h"
#include "ns3/opengym-module.h"
#include "ns3/lr-wpan-mac.h"
#include <ns3/lr-wpan-module.h>

namespace ns3{

class WSNGym:public OpenGymEnv
{
public:
    WSNGym();
    WSNGym(uint32_t nodeId);
    WSNGym(uint32_t nodeId, bool randData);
    virtual ~WSNGym();
    virtual void DoDispose();

    virtual Ptr<OpenGymSpace> GetActionSpace();
    virtual Ptr<OpenGymSpace> GetObservationSpace();
    virtual bool GetGameOver();
    virtual Ptr<OpenGymDataContainer> GetObservation();
    virtual float GetReward();
    virtual std::string GetExtraInfo();
    virtual bool ExecuteActions(Ptr<OpenGymDataContainer> action);

    //static void ScheduleNextStateRead(double envStepTime);
    void GetCongestionParams(std::multimap<uint16_t, MeshRootData>&);
    //void SetDeviceContainer(NetDeviceContainer &device);
protected:
    uint32_t m_nodeId;
    uint32_t m_numActions;

private:
    uint32_t m_lqt;
    std::multimap<uint16_t, MeshRootData> *meshPtr;
    bool firstTime = true;
    uint32_t j = 0;
    uint32_t low = 0;
    //float m_congestionData[3];
    float m_queueLength;
    float m_pktArrivalRate;
    float m_delay;
    // game over?
    bool m_isGameOver;
    float m_reward;
    // actions
    float m_queueWeight;
    float m_arrivalRateWeight;
    float m_delayWeight;
    uint32_t m_totalPacketDropped;
    uint32_t m_prevTPD;
    uint16_t m_maxQueue;

};

}

#endif