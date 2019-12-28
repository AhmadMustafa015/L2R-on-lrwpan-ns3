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
    Ptr<OpenGymDiscreteSpace> space = CreateObject<OpenGymDiscreteSpace>(6);
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
    std::vector<uint32_t> shape = {parameterNum,};
    Ptr<OpenGymBoxContainer<float>> box = CreateObject<OpenGymBoxContainer<float>>(shape);
    float avgQueue = 0;
    float avgArrivalRate = 0;
    float avgDelay = 0;
    uint32_t count = 0;
    Ptr<LrWpanNetDevice> device;
    for (uint32_t i = 1; i < NodeList::GetNNodes(); i++)
    {
        Ptr<Node> node = NodeList::GetNode(i);
        device = node->GetDevice(0)->GetObject<LrWpanNetDevice>();
        
        if (device->GetMac()->GetDepth() <= 3)
        {
            avgQueue += device->GetMac()->GetQueueSize();
            
            uint32_t temp2 = device->GetMac()->GetArrivalRate();
            float *temp22 = reinterpret_cast<float*>(&temp2);
            avgArrivalRate += *temp22;

            uint32_t temp3 = device->GetMac()->GetAvgDelay();
            float *temp33 = reinterpret_cast<float*>(&temp3);
            avgDelay += *temp33;
            count++;
        }
        m_totalPacketDropped =  device->GetMac()->GetPacketDroppedByQueue();
    }
    m_queueLength = avgQueue / float(count);
    m_queueLength = m_queueLength; // float(device->GetMac()->GetMaxQueueSize());
    m_maxQueue = device->GetMac()->GetMaxQueueSize();
    m_pktArrivalRate = float(count) / avgArrivalRate ;
    m_delay = avgDelay / float(count);

    /*
    float avgQueue = 0;
    float avgArrivalRate = 0;
    float avgDelay = 0;
    uint32_t count = 5;
    std::multimap<uint16_t, MeshRootData>::const_iterator i = meshPtr->begin();
    j = 0;
    for (;i != meshPtr->end(); ++i)
    {
        if(i->first <= 3 &&  low <= j && j < low+count)
        {
            avgQueue += i->second.m_queueSize;
            avgArrivalRate += i->second.m_arrivalRate;
            avgDelay += i->second.m_avgDelay;
            //count++;
        }
        //std::cout << "-" <<avgQueue << "-" << avgArrivalRate << "-" << avgDelay;
        ++j;
    }
    low += count;
    m_queueLength = avgQueue / float(count);
    m_pktArrivalRate =  float(count) / avgArrivalRate;
    m_delay = avgDelay / float(count);*/
    //std::cout << "-" <<m_queueLength << "-" << m_pktArrivalRate << "-" << m_delay;
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
    return true;
    min: 4-8
    max: 15-20
    */
    NS_LOG_FUNCTION (this);
    Ptr<OpenGymDiscreteContainer> discrete = DynamicCast<OpenGymDiscreteContainer>(action);
    uint8_t lqt = discrete->GetValue();
    //m_lqt = lqt + 4;
    for (uint32_t i = 0; i < NodeList::GetNNodes(); i++)
    {
        Ptr<Node> node = NodeList::GetNode(i);
        Ptr<LrWpanNetDevice> device = node->GetDevice(0)->GetObject<LrWpanNetDevice>();
        Ptr<LrWpanCsmaCa> csma = device->GetCsmaCa();
        csma->SetMacMaxBE(lqt+4);
        csma->SetMacMinBE(lqt+2);
        csma->SetMacMaxCSMABackoffs(lqt+1);
        //device->GetMac()->SetMacMaxFrameRetries(lqt);
        device->GetMac()->SetLQT(lqt+1);
    }
    /*for (NetDeviceContainer::Iterator i= m_deviceContainer.Begin(); i != m_deviceContainer.End (); i++)
    {
        Ptr<NetDevice> d = *i;
        Ptr<LrWpanNetDevice> device = d->GetObject<LrWpanNetDevice>();
        device->GetMac()->SetLQT(m_lqt);
    }*/
    NS_LOG_UNCOND ("MinBE: " << lqt + 4);
    return true;
}

float
WSNGym::GetReward()
{
    float totReward = 0;
    NS_LOG_INFO("MyGetReward: " << m_reward);
    /*if(m_queueLength - m_maxQueue  > 0)
        totReward -= 1.0 * (m_queueLength - m_maxQueue);
    else if(m_queueLength > 0.5)
        totReward += -40.0;
    else if(m_queueLength > 0.2)
        totReward += 40.0;
    else
        totReward += 30.0;*/

    if(m_pktArrivalRate > 4)
        totReward += -80.0;
    else if(m_pktArrivalRate > 2)
        totReward += -40.0;
    else if(m_pktArrivalRate > 1)
        totReward += 10.0;
    else
        totReward += 30.0;
    
    if(m_delay > .02)
        totReward += -80.0;
    else if(m_delay > .01)
        totReward += -40.0;
    else if(m_delay < .003)
        totReward += 10.0;
    else
        totReward += 30.0;
    if(m_totalPacketDropped < m_prevTPD)
        totReward += 1 * (m_prevTPD - m_totalPacketDropped);
    else
        totReward -= 1 * (m_totalPacketDropped-m_prevTPD);    
    m_prevTPD = m_totalPacketDropped;
    m_reward = totReward;
    return m_reward;
}

// optional
std::string
WSNGym::GetExtraInfo()
{
    NS_LOG_FUNCTION (this);
    std::string myInfo = "AB info";
    //NS_LOG_UNCOND("MyGetExtraInfo: " << myInfo);
    return myInfo;
}

// game over when reached a certain limit of congestion?
bool
WSNGym::GetGameOver()
{
    m_isGameOver = false;
    if(m_queueLength > 0.9 && m_pktArrivalRate > 0.9 && m_delay > 0.9)
        m_isGameOver = false;

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