#include "ns3/log.h"
#include "ns3/address.h"
#include "ns3/node.h"
#include "ns3/nstime.h"
#include "ns3/data-rate.h"
#include "ns3/random-variable-stream.h"
#include "ns3/socket.h"
#include "ns3/simulator.h"
#include "ns3/packet.h"
#include "ns3/uinteger.h"
#include "ns3/trace-source-accessor.h"
#include "l2r-application.h"
#include "ns3/string.h"
#include "ns3/pointer.h"
#include <ns3/lr-wpan-module.h>
//onOffApplication
namespace ns3 {

//NS_LOG_COMPONENT_DEFINE ("l2rApplication");

//NS_OBJECT_ENSURE_REGISTERED (l2rApplication);
TypeId l2rapplication::GetTypeId (void)
{
  static TypeId tid = TypeId ("l2rapplication")
    .SetParent<Application> ()
    .SetGroupName ("Applications")
    .AddConstructor<l2rapplication> ()
    .AddAttribute ("OnTime", "A RandomVariableStream used to pick the duration of the 'On' state.",
                   StringValue ("ns3::ConstantRandomVariable[Constant=5]"),
                   MakePointerAccessor (&l2rapplication::m_onTime),
                   MakePointerChecker <RandomVariableStream>())
    .AddAttribute ("OffTime", "A RandomVariableStream used to pick the duration of the 'Off' state.",
                   StringValue ("ns3::ConstantRandomVariable[Constant=0]"),
                   MakePointerAccessor (&l2rapplication::m_offTime),
                   MakePointerChecker <RandomVariableStream>())
    ;
  return tid;
}
l2rapplication::l2rapplication()
:m_residualBits (0),
  m_lastStartTime (Seconds (0)),
  m_totBytes (0)
{
  m_cbrRate = 1000;
  m_pktSize = 20;
  m_maxBytes = 0; 
  m_totalPacketsSend = 0;
  m_totalPacketsDroped = 0;
}
void
l2rapplication::Setup(Ptr<NetDevice> dev, Ptr<RandomVariableStream> on,Ptr<RandomVariableStream> off)
{
  m_netDevice = dev;
  m_onTime = on;
  m_offTime = off;
}
l2rapplication::~l2rapplication()
{
}
void
l2rapplication::CancelEvents()
{
  if (m_sendEvent.IsRunning ())
  { // Cancel the pending send packet event
    // Calculate residual bits since last packet sent
    Time delta (Simulator::Now () - m_lastStartTime);
    int64x64_t bits = delta.To (Time::S) * m_cbrRate.GetBitRate ();
    m_residualBits += bits.GetHigh ();
  }
  Simulator::Cancel (m_sendEvent);
  Simulator::Cancel (m_startStopEvent);
}
void 
l2rapplication::StartApplication () // Called at time specified by Start
{
  // Insure no pending event
  CancelEvents ();
  // If we are not yet connected, there is nothing to do here
  // The ConnectionComplete upcall will start timers at that time
  //if (!m_connected) return;
  ScheduleStartEvent ();
}

void 
l2rapplication::SendPacket () //ToDo
{
  Ptr<LrWpanNetDevice> device = m_netDevice->GetObject<LrWpanNetDevice> ();  
  Ptr<Packet> packet = Create<Packet> (m_pktSize);
  m_totBytes += m_pktSize;
  L2R_Header L2R_DataHeader;
  L2R_DataHeader.SetSrcMacAddress(device->GetMac ()->GetShortAddress());
  L2R_DataHeader.SetMsgType(DataHeader);
  L2R_DataHeader.SetDepth(device->GetMac ()->GetDepth());
  L2R_DataHeader.SetPQM(device->GetMac ()->GetPqm());
  L2R_DataHeader.SetQueueSize(device->GetMac ()->GetQueueSize());
  L2R_DataHeader.SetDelay(device->GetMac ()->GetAvgDelay());
  L2R_DataHeader.SetArrivalRate(device->GetMac ()->GetArrivalRate());
  ++device->GetMac ()-> m_totalPacketSentByNode;
  ++m_totalPacketsSend;
  packet->AddHeader (L2R_DataHeader); //serialize is called here
  McpsDataRequestParams params;
  params.m_dstPanId = device->GetMac ()->GetPanId();
  params.m_srcAddrMode = SHORT_ADDR;
  params.m_dstAddrMode = SHORT_ADDR;
  params.m_dstAddr = device->GetMac ()->OutputRoute();
  params.m_msduHandle = 0;
  params.m_txOptions = TX_OPTION_ACK;
  device->GetMac ()->UpdateDelay(packet->GetUid(), Simulator::Now ());
  device->GetMac ()->OutputTree(packet,Simulator::Now (),params);
  //device->GetMac ()->m_l2rQueue.insert(std::make_pair(packet->GetUid(),packet));
  device->GetMac ()->IncQueue();
  //m_delayForEachPacket.insert(std::make_pair (m_txPkt->GetUid(),Simulator::Now ()));
  m_totalPacketSendUid.insert(std::make_pair(packet->GetUid(), Simulator::Now ().GetSeconds()));
  Simulator::ScheduleNow (&LrWpanMac::McpsDataRequest,device->GetMac (),
                             params, packet);
  m_lastStartTime = Simulator::Now ();
  m_residualBits = 0;
  ScheduleNextTx ();
}
void 
l2rapplication::StopApplication () // Called at time specified by Stop
{
  CancelEvents ();
}
void 
l2rapplication::ScheduleStopEvent ()
{  // Schedules the event to stop sending data (switch to "Off" state)
  Time onInterval = Seconds (m_onTime->GetValue ());
  //NS_LOG_LOGIC ("stop at " << onInterval);
  m_startStopEvent = Simulator::Schedule (onInterval, &l2rapplication::StopSending, this);
}
void 
l2rapplication::StopSending ()
{
  CancelEvents ();
  ScheduleStartEvent ();
}
void 
l2rapplication::SetMaxBytes (uint64_t maxBytes)
{
  //NS_LOG_FUNCTION (this << maxBytes);
  m_maxBytes = maxBytes;
}
int64_t 
l2rapplication::AssignStreams (int64_t stream)
{
  //NS_LOG_FUNCTION (this << stream);
  m_onTime->SetStream (stream);
  m_offTime->SetStream (stream + 1);
  return 2;
}
void 
l2rapplication::ScheduleStartEvent ()
{
  // Schedules the event to start sending data (switch to the "On" state)
  Time offInterval = Seconds (m_offTime->GetValue ());
  //NS_LOG_LOGIC ("start at " << offInterval);
  m_startStopEvent = Simulator::Schedule (offInterval, &l2rapplication::StartSending, this);
}
void 
l2rapplication::StartSending ()
{
  m_lastStartTime = Simulator::Now ();
  ScheduleNextTx ();  // Schedule the send packet event
  ScheduleStopEvent ();
}
void 
l2rapplication::ScheduleNextTx ()
{
  if (m_maxBytes == 0 || m_totBytes < m_maxBytes)
    {
      uint32_t bits = m_pktSize * 8 - m_residualBits;
      //NS_LOG_LOGIC ("bits = " << bits);
      Time nextTime (Seconds (bits /
                              static_cast<double>(m_cbrRate.GetBitRate ()))); // Time till next packet
      //NS_LOG_LOGIC ("nextTime = " << nextTime);
      m_sendEvent = Simulator::Schedule (nextTime,
                                         &l2rapplication::SendPacket, this);
    }
  else
    { // All done, cancel any pending events
      StopApplication ();
    }
}
void
l2rapplication::TotalPacketPrint()
{
  std::cout <<"Total Packet Sent: " <<m_totalPacketsSend << " Total Packets Delayed No Enough Memory: "<< m_totalPacketsDroped  << std::endl;
}
void 
l2rapplication::SetPacketSize(uint32_t pktSize)
{
  m_pktSize = pktSize;
}
void
l2rapplication::SendBurst()
{
  for(uint8_t i = 0; i < 10; i++)
  {
    Ptr<LrWpanNetDevice> device = m_netDevice->GetObject<LrWpanNetDevice> ();  
    Ptr<Packet> packet = Create<Packet> (m_pktSize);
    L2R_Header L2R_DataHeader;
    L2R_DataHeader.SetSrcMacAddress(device->GetMac ()->GetShortAddress());
    L2R_DataHeader.SetMsgType(DataHeader);
    L2R_DataHeader.SetDepth(device->GetMac ()->GetDepth());
  L2R_DataHeader.SetPQM(device->GetMac ()->GetPqm());
  L2R_DataHeader.SetQueueSize(device->GetMac ()->GetQueueSize());
  L2R_DataHeader.SetDelay(device->GetMac ()->GetAvgDelay());
  L2R_DataHeader.SetArrivalRate(device->GetMac ()->GetArrivalRate());
  ++device->GetMac ()-> m_totalPacketSentByNode;
  packet->AddHeader (L2R_DataHeader); //serialize is called here
  McpsDataRequestParams params;
  params.m_dstPanId = device->GetMac ()->GetPanId();
  params.m_srcAddrMode = SHORT_ADDR;
  params.m_dstAddrMode = SHORT_ADDR;
  params.m_dstAddr = device->GetMac ()->OutputRoute();
  params.m_msduHandle = 0;
  params.m_txOptions = TX_OPTION_ACK;
  Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable> ();
  device->GetMac ()->UpdateDelay(packet->GetUid(), Simulator::Now ());
  device->GetMac ()->OutputTree(packet,Simulator::Now (),params);
  //device->GetMac ()->m_l2rQueue.insert(std::make_pair(packet->GetUid(),packet));
  Simulator::Schedule (MicroSeconds(var->GetValue (1, 1000)),&LrWpanMac::McpsDataRequest,device->GetMac (),
                             params, packet);  
  }                             
}
void
l2rapplication::PrintEndtoEndDelay()
{
  for (std::map<uint64_t, double>::iterator i = m_totalPacketSendUid.begin (); i != m_totalPacketSendUid.end (); i++) //ToDo
  {

    std::ofstream out4 ("EndtoEndDelay_Nodes.csv", std::ios::app);
    out4 << i->first << "," << i->second
         << std::endl;
    out4.close ();
  }
}

}