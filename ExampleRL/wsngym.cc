#include "wsngym.h"
#include "ns3/object.h"
#include "ns3/core-module.h"
#include "ns3/node-list.h"
#include "ns3/log.h"
#include <sstream>
#include <iostream>
#include <vector>
#include <random>
#include <map>


namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("WSNGym");

WSNGym::WSNGym()
{
    NS_LOG_FUNCTION(this);
    SetOpenGymInterface(OpenGymInterface::Get()); // ?
    m_numActions = 3;

}

WSNGym::WSNGym(uint32_t nodeId)
{
    NS_LOG_FUNCTION(this);
    SetOpenGymInterface(OpenGymInterface::Get()); // ?
    m_nodeId = nodeId;
}

WSNGym::WSNGym(uint32_t nodeId, bool randData)
{
    NS_LOG_FUNCTION(this);
    SetOpenGymInterface(OpenGymInterface::Get()); // ?
    m_nodeId = nodeId;
}

void
WSNGym::DoDispose ()
{
  NS_LOG_FUNCTION (this);
}

WSNGym::~WSNGym()
{
    NS_LOG_FUNCTION(this);
}

// Define action space: coefficients in LQM function
Ptr<OpenGymSpace>
WSNGym::GetActionSpace()
{
    /*uint32_t parameterNum = 3;
    // queue length weight
    // arrival rate weight
    // delay weight
    uint32_t low = 0;
    uint32_t high = 10;
    std::vector<uint32_t> shape = {parameterNum,};
    std::string dtype = TypeNameGet<uint32_t>();
    Ptr<OpenGymBoxSpace> box = CreateObject<OpenGymBoxSpace>(low, high, shape, dtype);
    NS_LOG_UNCOND ("GetActionSpace: " << box);
    return box;*/
    NS_LOG_FUNCTION (this);
    Ptr<OpenGymDiscreteSpace> space = CreateObject<OpenGymDiscreteSpace>(20);
    NS_LOG_UNCOND ("GetActionSpace: " << space);
    return space;
}

// Define Observation data
Ptr<OpenGymSpace>
WSNGym::GetObservationSpace()
{
    NS_LOG_FUNCTION(this);
    // 1: normalized queue length
    // 2: packet arrival rate
    // 3: delay
    uint32_t parameterNum = 3;
    float low = 0.0;
    float high = 1.0;
    std::vector<uint32_t> shape = {parameterNum,};
    std::string type = TypeNameGet<uint32_t>();
    Ptr<OpenGymBoxSpace> box = CreateObject<OpenGymBoxSpace>(low, high, shape, type);
    return box;
}

// Obtain observation data
Ptr<OpenGymDataContainer>
WSNGym::GetObservation()
{

    uint32_t parameterNum = 3; // 3 for now
    /*float X = 100.0;
    for (unsigned i = 0; i < parameterNum; i++)
    {
        float r1 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/X));
        //std::cout << "i: " << i << "r1: " << r1 << std::endl;
        m_congestionData[i] = r1;
    }*/
    std::vector<uint32_t> shape = {parameterNum,};
    Ptr<OpenGymBoxContainer<float>> box = CreateObject<OpenGymBoxContainer<float>>(shape);
    /*std::cout << m_congestionData[0] << std::endl;
    std::cout << m_congestionData[1] << std::endl;
    std::cout << m_congestionData[2] << std::endl;*/
    float avgQueue = 0;
    float avgArrivalRate = 0;
    float avgDelay = 0;
    uint32_t count = 0;
    std::multimap<uint16_t, MeshRootData>::const_iterator i = meshPtr->begin();
    for (;i != meshPtr->end(); ++i)
    {
        if(i->first <= 3)
        {
            avgQueue += i->second.m_queueSize;
            avgArrivalRate += i->second.m_arrivalRate;
            avgDelay += i->second.m_avgDelay;
            count++;
        }
        //std::cout << "-" <<avgQueue << "-" << avgArrivalRate << "-" << avgDelay;
        
    }
    m_queueLength = avgQueue / float(count);
    m_pktArrivalRate = avgArrivalRate / float(count);
    m_delay = avgDelay / float(count);
    std::cout << "-" <<m_queueLength << "-" << m_pktArrivalRate << "-" << m_delay;
    box->AddValue(m_queueLength);
    box->AddValue(m_pktArrivalRate);
    box->AddValue(m_delay);
    return box;
}

bool
WSNGym::ExecuteActions(Ptr<OpenGymDataContainer> action)
{
    /*Ptr<OpenGymBoxContainer<uint32_t>> box = DynamicCast<OpenGymBoxContainer<uint32_t>>(action);
    m_queueWeight = box->GetValue(0);
    m_arrivalRateWeight = box->GetValue(1);
    m_delayWeight = box->GetValue(2);
    NS_LOG_INFO ("MyExecuteActions: " << action);
    return true;*/
    NS_LOG_FUNCTION (this);
    Ptr<OpenGymDiscreteContainer> discrete = DynamicCast<OpenGymDiscreteContainer>(action);
    uint32_t lqt = discrete->GetValue();
    m_lqt = lqt;
    for (uint32_t i = 0; i < NodeList::GetNNodes(); i++)
    {
        Ptr<Node> node = NodeList::GetNode(i);
        Ptr<LrWpanNetDevice> device = node->GetDevice(0)->GetObject<LrWpanNetDevice>();
        device->GetMac()->SetLQT(m_lqt);
    }
    /*for (NetDeviceContainer::Iterator i= m_deviceContainer.Begin(); i != m_deviceContainer.End (); i++)
    {
        Ptr<NetDevice> d = *i;
        Ptr<LrWpanNetDevice> device = d->GetObject<LrWpanNetDevice>();
        device->GetMac()->SetLQT(m_lqt);
    }*/
    NS_LOG_UNCOND ("LQT: " << m_lqt);
    return true;
}

float
WSNGym::GetReward()
{
    float totReward = 0;
    NS_LOG_INFO("MyGetReward: " << m_reward);
    if(m_queueLength > 0.9)
        totReward += -80.0;
    else if(m_queueLength > 0.5)
        totReward += -40.0;
    else if(m_queueLength > 0.2)
        totReward += 10.0;
    else
        totReward += 30.0;

    if(m_pktArrivalRate > 0.9)
        totReward += -80.0;
    else if(m_pktArrivalRate > 0.5)
        totReward += -40.0;
    else if(m_pktArrivalRate > 0.2)
        totReward += 10.0;
    else
        totReward += 30.0;
    
    if(m_delay > 0.9)
        totReward += -80.0;
    else if(m_delay > 0.5)
        totReward += -40.0;
    else if(m_delay > 0.2)
        totReward += 10.0;
    else
        totReward += 30.0;
    
    m_reward = totReward;
    return m_reward;
}

// optional
std::string
WSNGym::GetExtraInfo()
{
    NS_LOG_FUNCTION (this);
    std::string myInfo = "AB info";
    NS_LOG_UNCOND("MyGetExtraInfo: " << myInfo);
    return myInfo;
}

// game over when reached a certain limit of congestion?
bool
WSNGym::GetGameOver()
{
    m_isGameOver = false;
    if(m_queueLength > 0.9 && m_pktArrivalRate > 0.9 && m_delay > 0.9)
        m_isGameOver = true;

    return m_isGameOver;
}
/*
static void 
WSNGym::ScheduleNextStateRead(double envStepTime)
{
  Simulator::Schedule (Seconds(envStepTime), &WSNGym::ScheduleNextStateRead, envStepTime, this);
  Notify();
}
*/
void
WSNGym::GetCongestionParams(std::multimap<uint16_t, MeshRootData> &congestionParams)
{
    meshPtr = &congestionParams;
    /*float avgQueue = 0;
    float avgArrivalRate = 0;
    float avgDelay = 0;
    uint32_t count = 0;
    std::multimap<uint16_t, MeshRootData>::const_iterator i = congestionParams.begin();
    for (;i != congestionParams.end(); ++i)
    {
        if(i->first <= 3)
        {
            avgQueue += i->second.m_queueSize;
            avgArrivalRate += i->second.m_arrivalRate;
            avgDelay += i->second.m_avgDelay;
            count++;
        }
        std::cout << "-" <<avgQueue << "-" << avgArrivalRate << "-" << avgDelay;
        getchar();
        
    }
    m_queueLength = avgQueue / float(count);
    m_queueLength = avgQueue / float(count);
    m_queueLength = avgQueue / float(count);*/
}
/*void
WSNGym::SetDeviceContainer(NetDeviceContainer &device)
{
    m_deviceContainer = &device;   
}*/
}