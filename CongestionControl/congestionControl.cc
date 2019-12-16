#include <ns3/log.h>
#include <ns3/core-module.h>
#include <ns3/lr-wpan-module.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/propagation-delay-model.h>
#include <ns3/simulator.h>
#include <ns3/single-model-spectrum-channel.h>
#include <ns3/constant-position-mobility-model.h>
#include <ns3/packet.h>
#include "ns3/mobility-module.h"
#include <iostream>
#include "ns3/netanim-module.h"
#include "vector"
#include "string"
#include "ns3/node.h"
#include "ns3/node-list.h"
#include <math.h>
#include "l2r-application.h"
#include <iomanip>
#include "ns3/opengym-module.h"
#include "wsngym.h"
using namespace ns3;

#define PI 3.14159265
bool verbose = false;
AnimationInterface * pAnim = 0;
NetDeviceContainer devContainer;
LrWpanHelper lrWpanHelper;
//Ptr<LrWpanCsmaCa> csmaCa = CreateObject<LrWpanCsmaCa> ();
std::string CSVfileName = "CongestionControl.csv";
void modify (const Mac16Address &sender,const uint16_t &depth, const uint16_t &pqm,const Mac16Address &receiver);
/// RGB structure
struct rgb {
  uint8_t r; ///< red
  uint8_t g; ///< green
  uint8_t b; ///< blue
};
struct rgb colors [] = {
                        {255, 0 , 0}, // Red
                        { 0, 255, 0}, // Blue
                        { 0, 0, 255 },  // Green
                        {255, 125, 0},
                        {255, 0, 127},
                        {255, 255, 0},
                        {0, 255, 255},
                        {255, 0, 255}
                        };
static void DataIndication (McpsDataIndicationParams params, Ptr<Packet> p)
{
}
static void L2rUpdateTcie(McpsDataIndicationParams params, uint16_t depth, uint16_t pqm, Mac16Address receiver)
{
  Mac16Address senderAdd = params.m_srcAddr;
  modify(senderAdd,depth,pqm,receiver);
}
static void congestionVsTime ()
{
  uint32_t avgDroppedPacket = 0;
  double timeNow = Simulator::Now ().GetSeconds ();
  for (NetDeviceContainer::Iterator i= devContainer.Begin(); i != devContainer.End (); i++)
  {
    Ptr<NetDevice> d = *i;
    if(d->GetNode()->GetId () == 0)
      continue;
    Ptr<LrWpanNetDevice> device = d->GetObject<LrWpanNetDevice> ();
    avgDroppedPacket += device->GetMac ()->GetPacketDroppedByQueue();
    
  }
  //avgDroppedPacket /= (50-1);
  std::ofstream out2 ("congestionVsTime_total.csv", std::ios::app);
  out2 << timeNow << "," << avgDroppedPacket
      << std::endl; 
  out2.close ();
  Simulator::Schedule(Seconds(1),congestionVsTime);
}

/*static void DataConfirm (McpsDataConfirmParams params)
{
  std::cout << "LrWpanMcpsDataConfirmStatus = " << params.m_status << std::endl;
}
*/
static void ReceivePacket (MeshRootData para,Mac16Address srcAddress)
{
  std::ofstream out (CSVfileName.c_str (), std::ios::app);
  out << (Simulator::Now ()).GetSeconds ()<< "," << srcAddress << "," << para.m_queueSize 
      << "," << para.m_arrivalRate << "," << para.m_avgDelay << std::endl;
  out.close ();
}
void ScheduleNextStateRead(double envStepTime, Ptr<OpenGymInterface> openGymInterface)
{
  Simulator::Schedule (Seconds(envStepTime), &ScheduleNextStateRead, envStepTime, openGymInterface);
  openGymInterface->NotifyCurrentState();
}
/*static void StateChangeNotification (std::string context, Ptr<Packet> p)
{
  std::cout << "drop Packet number: " << p->GetUid() << std::endl;
}*/
class CongestionControl
{
public:
  CongestionControl ();
  /**
   * Run function
   * \param nNodes The total number of nodes
   * \param nSinks The total number of mesh root
   * \param totalTime The total simulation time
   * \param periodicUpdateInterval The routing update interval
   * \param dataStart The data transmission start time
   * \param printRoutes print the routes if true
   * \param CSVfileName The CSV file name
   */
  void CaseRun (uint32_t nWifis,
                uint32_t nSinks,
                double totalTime,
                uint8_t periodicUpdateInterval,
                double dataStart,
                bool printRoutes,
                std::string CSVfileName,
                bool enableTracing,
                bool enablePcap,
                uint32_t meshNodeId,
                uint32_t txRange
                );
private:
  uint32_t m_nNodes; ///< total number of nodes
  uint32_t m_nSinks; ///< number of receiver nodes
  double m_totalTime; ///< total simulation time (in seconds)
  uint32_t m_periodicUpdateInterval; ///< routing update interval
  double m_dataStart; ///< time to start data transmissions (seconds)
  uint32_t bytesTotal; ///< total bytes received by all nodes
  uint32_t packetsReceived; ///< total packets received by all nodes
  bool m_printRoutingTable; ///< print routing table
  std::string m_CSVfileName; ///< CSV file name
  bool m_enableTracing;
  bool m_enablePcap;
  uint32_t m_meshNodeId;
  Ptr<SingleModelSpectrumChannel> channel;
  NodeContainer ch;
  uint32_t m_distanceBtwNodes;
  uint32_t m_packetSize;
  uint64_t m_maxTxBytePerNode;
  uint16_t m_maxQueueSize;
  double m_sensingPeriod;
  static uint64_t m_totalPhyDrop;
  //l2rapplication m_applicationContainer;

private:
  /// Create and initialize all nodes
  void CreateNodes ();
  /**
   * Create and initialize all devices
   * \param tr_name The trace file name
   */
  void CreateDevices (std::string tr_name);
  /// Create data sinks and sources
  void InstallApplications ();
  /// Setup mobility model
  void SetupMobility ();
  /// NetAnim Interface
  /// Data Indecation callback
  /// L2R Receive TC-IE callback 
  /// Data Confirm callback
  /// State change callback
   /**
   * Packet receive function
   * \param packet
   */
  ///check Throughput
  void CheckThroughput ();
  static void PhyRxDrop (CongestionControl *cc,Ptr<LrWpanNetDevice> device, Ptr<const Packet> packet);

};
int main (int argc, char *argv[])
{
  CongestionControl congestionControl;
  uint32_t nNodes = 50;
  uint32_t nSinks = 1;
  double totalTime = 100;
  uint8_t periodicUpdateInterval = 101;
  double dataStart = 2.0;
  bool printRoutingTable = true;
  uint32_t meshNodeId = 0;
  bool enableTracing = true;
  bool enablePcap = false;
  uint32_t distanceBtwNodes = 79; 
  //double onTime = 1;
  //double offTime = 1;

  CommandLine cmd;
  cmd.AddValue ("nNodes", "Number of wifi nodes[Default:30]", nNodes);
  cmd.AddValue ("totalTime", "Total Simulation time[Default:100]", totalTime);
  cmd.AddValue ("periodicUpdateInterval", "Periodic Interval Time[Default=15]", periodicUpdateInterval);
  cmd.AddValue ("dataStart", "Time at which nodes start to transmit data[Default=5.0]", dataStart);
  cmd.AddValue ("printRoutingTable", "print routing table for nodes[Default:1]", printRoutingTable);
  cmd.AddValue ("CSVfileName", "The name of the CSV output file name[Default:CongestionControl.csv]", CSVfileName);
  cmd.AddValue ("meshNodeId", "The node which will be the mesh root[Default:0]", meshNodeId);
  cmd.AddValue ("verbose", "turn on all log components", verbose);
  cmd.AddValue ("enableTracing", "Output Tracing file[Default:0]", enableTracing);
  cmd.AddValue ("Enable Pcap", "Output Pcap packet tracing file[Default:0]", enablePcap);
  cmd.AddValue ("distanceBtwNodes", "Distance Between Nodes[Default:80]", distanceBtwNodes);
  cmd.Parse (argc, argv);
  //LogComponentEnable ("LrWpanMac", LOG_LEVEL_ALL);
  //LogComponentEnable ("LrWpanPhy", LOG_LEVEL_ALL);
  std::ofstream out (CSVfileName.c_str ());
  out << "SimulationSecond," <<
  "Sender Mac Addr," <<
  "Normalized Queue Length," <<
  "Arrival Rate Moving Avg," <<
  "Avg Delay," <<
  std::endl;
  out.close ();
  std::ofstream out2 ("congestionVsTime_Total.csv");
  out2.close();
  SeedManager::SetSeed (167);

  congestionControl = CongestionControl();
  congestionControl.CaseRun (nNodes, nSinks, totalTime, periodicUpdateInterval,
                              dataStart, printRoutingTable, CSVfileName, enableTracing, 
                              enablePcap, meshNodeId, distanceBtwNodes);
  return 0;
}

CongestionControl::CongestionControl()
  :bytesTotal (0),
  packetsReceived (0)
{
}
void
CongestionControl::CheckThroughput ()
{
}
void
CongestionControl::CaseRun(uint32_t nNodes, uint32_t nSinks, 
                          double totalTime, uint8_t periodicUpdateInterval,
                          double dataStart, bool printRoutes, std::string CSVfileName, 
                          bool enableTracing, bool enablePcap, uint32_t meshNodeId, uint32_t distanceBtwNodes)
{
  m_nNodes = nNodes;
  m_nSinks = nSinks;
  m_totalTime = totalTime;
  m_periodicUpdateInterval = periodicUpdateInterval;
  m_dataStart = dataStart;
  m_printRoutingTable = printRoutes;
  m_CSVfileName = CSVfileName;
  m_enableTracing = enableTracing;
  m_enablePcap = enablePcap;
  m_meshNodeId = meshNodeId;
  m_distanceBtwNodes = distanceBtwNodes;
  m_packetSize = 20;
  m_maxTxBytePerNode = 0;
  m_maxQueueSize = 15;
  m_sensingPeriod = 2;
  m_totalPhyDrop = 0;
  uint32_t openGymPort = 5555;
  //double envStepTime = 0.5;

  std::stringstream ss;
  ss << m_nNodes;
  std::string t_nodes = ss.str ();
  std::stringstream ss3;
  ss3 << m_totalTime;
  std::string m_TotalTime = ss3.str ();
  std::string tr_name = "L2R_" + t_nodes + "Nodes_" + m_TotalTime + "SimTime";
  std::cout << "Trace file generated is " << tr_name << ".tr\n";

  Ptr<OpenGymInterface> openGymInterface = CreateObject<OpenGymInterface>(openGymPort);
  Ptr<WSNGym> myWSNGym = CreateObject<WSNGym>();
  myWSNGym->SetOpenGymInterface(openGymInterface);

  openGymInterface->SetGetActionSpaceCb( MakeCallback (&WSNGym::GetActionSpace, myWSNGym));
  openGymInterface->SetGetObservationSpaceCb( MakeCallback (&WSNGym::GetObservationSpace, myWSNGym));
  openGymInterface->SetGetGameOverCb( MakeCallback (&WSNGym::GetGameOver, myWSNGym));
  openGymInterface->SetGetObservationCb( MakeCallback (&WSNGym::GetObservation, myWSNGym));
  openGymInterface->SetGetRewardCb( MakeCallback (&WSNGym::GetReward, myWSNGym));
  openGymInterface->SetGetExtraInfoCb( MakeCallback (&WSNGym::GetExtraInfo, myWSNGym));
  openGymInterface->SetExecuteActionsCb( MakeCallback (&WSNGym::ExecuteActions, myWSNGym));

  CreateNodes ();
  SetupMobility ();
  CreateDevices (tr_name);

    /*Ptr<LrWpanNetDevice> device = d->GetObject<LrWpanNetDevice> ();
    //std::cout << "Node: " << nodeID << "Send data packet to: ";
    L2R_Header dataHeader;
    float delay1 = 5.45;
    float arrivalRate = 6.45;
    uint16_t QueueS = 7;
    uint32_t* pInt1 = reinterpret_cast<uint32_t*>(&delay1);
    uint32_t* pInt2 = reinterpret_cast<uint32_t*>(&arrivalRate);
    dataHeader.SetMsgType(DataHeader);
    dataHeader.SetDelay(*pInt1);
    dataHeader.SetArrivalRate(*pInt2);
    dataHeader.SetQueueSize(QueueS);
    dataHeader.SetSrcMacAddress(Mac16Address("00:08"));
    Ptr<Packet> p0 = Create<Packet> (); //Zero payload packet
    p0->AddHeader (dataHeader); //serialize is called here
    McpsDataRequestParams params;
    params.m_dstPanId = 10;
    params.m_srcAddrMode = SHORT_ADDR;
    params.m_dstAddrMode = SHORT_ADDR;
    params.m_dstAddr = Mac16Address("00:07");
    std::cout << params.m_dstAddr << std::endl;
    params.m_msduHandle = 0; //ToDo underStand the msduhandle from standard
    params.m_txOptions = TX_OPTION_ACK; 
    for(uint8_t i =0; i <2; i++) 
    {
      std::cout << "Sending Data Packet From: " << "00:08" << "To: " << "00:07" << std::endl;
      Simulator::ScheduleWithContext (1, Seconds (8),
                                      &LrWpanMac::McpsDataRequest,
                                      devContainer.Get(7)->GetObject<LrWpanNetDevice> ()->GetMac (), params, p0);
    }
    std::cout << "Data Rate: " << d->GetObject<LrWpanNetDevice> ()->GetPhy ()->GetDataOrSymbolRate(true) <<std::endl;*/
  InstallApplications ();
  Ptr<OutputStreamWrapper> routeTree = Create<OutputStreamWrapper> ((tr_name + "_routeTree" + ".routes"), std::ios::out);
  for (NetDeviceContainer::Iterator i= devContainer.Begin(); i != devContainer.End (); i++)
  {
    Ptr<NetDevice> d = *i;
    Ptr<LrWpanNetDevice> device = d->GetObject<LrWpanNetDevice> ();
    device->GetMac ()->outputRoutesTree(routeTree);
    uint32_t nodeID = d->GetNode ()->GetId ();
    if(nodeID == meshNodeId)
    {
      myWSNGym->GetCongestionParams(device->GetMac()->m_meshRootData);
    }
  }
  std::string animFile = tr_name + ".xml";
  pAnim = new AnimationInterface (animFile); //Mandatory
  //pAnim->EnablePacketMetadata (); //Optional
  //Simulator::Schedule(Seconds(5.0), &ScheduleNextStateRead, envStepTime, openGymInterface);
  Simulator::Stop (Seconds (m_totalTime));
  Simulator::Schedule(Seconds(m_dataStart + 1),congestionVsTime);
  Simulator::Run ();
  uint32_t totalPacketSent = 0;
  uint32_t totalPacketDroped = 0;
  uint32_t internalLoad = 0;
  std::cout << "Animation Trace file created:" << animFile.c_str ()<< std::endl;
  for(uint32_t i = 1; i < m_nNodes; i++)
  {
    //Ptr<l2rapplication> app = ch.Get (i)->GetApplication(0)->GetObject<l2rapplication> ();
    //app->TotalPacketPrint();

    totalPacketSent += ch.Get (i)->GetDevice (0)->GetObject<LrWpanNetDevice> ()->GetMac ()->GetTotalPacketSentByNode ();
    std::cout << "Total Packet Sent by Node: " << i << " = "
              << ch.Get (i)->GetDevice (0)->GetObject<LrWpanNetDevice> ()->GetMac ()->GetTotalPacketSentByNode ()
              <<"\tTotal Packet Droped: " <<ch.Get (i)->GetDevice (0)->GetObject<LrWpanNetDevice> ()->GetMac ()->GetTotalPacketDroppedByQueue()
              << std::endl;
    totalPacketDroped += ch.Get (i)->GetDevice (0)->GetObject<LrWpanNetDevice> ()->GetMac ()-> GetTotalPacketDroppedByQueue();
    internalLoad += ch.Get (i)->GetDevice (0)->GetObject<LrWpanNetDevice> ()->GetMac ()-> GetInternalLoad();
  }
  std::cout << "Total Packet Sent By All Nodes = " << totalPacketSent  <<std::endl
            << "Total Packet Dropped By All Nodes (Congestion) = " << totalPacketDroped <<std::endl
            << "Total Internal Load: " << internalLoad << std::endl;
  std::cout << "Total Packet Received by Sink = " 
            << ch.Get(m_meshNodeId)->GetDevice (0)->GetObject<LrWpanNetDevice> ()->GetMac ()->GetTotalPacketRxByMeshRoot() << std::endl;
  myWSNGym->NotifySimulationEnd();
  Simulator::Destroy ();
  //m_applicationContainer->TotalPacketPrint();
  delete pAnim;
}
void 
CongestionControl::CreateNodes()
{
  //NS_LOG_INFO ("Create nodes.");
  ch.Create (m_nNodes);
}
void
CongestionControl::SetupMobility ()
{
  // Install mobility
  MobilityHelper mobility;
  ObjectFactory pos;
  pos.SetTypeId ("ns3::RandomRectanglePositionAllocator");
  Ptr <UniformRandomVariable> x= CreateObject <UniformRandomVariable>();
  Ptr<ListPositionAllocator> taPositionAlloc = CreateObject<ListPositionAllocator> ();
  int xPos, yPos;
  uint32_t nodeCount;
  float radius, thetaRad;
  xPos = yPos = 0;
  radius = m_distanceBtwNodes;
  thetaRad = (m_distanceBtwNodes / radius);

  std::cout << "Adding node at position (" << xPos << ", " << yPos << ")" << std::endl;
  taPositionAlloc->Add (Vector (xPos, yPos, 0.0)); // mesh root?

  for (nodeCount = 1; nodeCount < m_nNodes; nodeCount++)
  {
    thetaRad += (m_distanceBtwNodes / radius)* (1 + x->GetValue()*0.1);
    xPos = radius * sin(thetaRad) + x->GetValue()*10; 
    yPos = radius * cos(thetaRad) + x->GetValue()*10;

    std::cout << "Adding node at position (" << xPos << ", " << yPos << ")" <<std::endl;
    taPositionAlloc->Add (Vector (xPos, yPos, 0.0));

    if (thetaRad > (2 * PI - (m_distanceBtwNodes / radius)))
    {
      radius += m_distanceBtwNodes;
      thetaRad = 0;
    }
  }
  //Ptr <PositionAllocator> taPositionAlloc = pos.Create ()->GetObject <PositionAllocator> ();
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.SetPositionAllocator (taPositionAlloc);
  mobility.Install (ch);
}
void
CongestionControl::CreateDevices (std::string tr_name)
{
  channel = CreateObject<SingleModelSpectrumChannel> ();
  //Ptr<LogDistancePropagationLossModel> propModel = CreateObject<LogDistancePropagationLossModel> ();
  Ptr<RangePropagationLossModel> propModel = CreateObject<RangePropagationLossModel> ();

  //Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel> ();
  channel->AddPropagationLossModel (propModel);
  //channel->SetPropagationDelayModel (delayModel);

  lrWpanHelper.SetChannel(channel);
  // Add and install the LrWpanNetDevice for each node
  devContainer = lrWpanHelper.Install(ch);
  lrWpanHelper.AssociateToPan (devContainer, 10);

  std::cout << "Created " << devContainer.GetN() << " devices" << std::endl;
  std::cout << "There are " << ch.GetN() << " nodes" << std::endl;

  int temp = 0;
  for (NetDeviceContainer::Iterator i= devContainer.Begin(); i != devContainer.End (); i++)
  {
    Ptr<NetDevice> d = *i;
    Ptr<LrWpanNetDevice> device = d->GetObject<LrWpanNetDevice> ();
    uint32_t nodeID = d->GetNode ()->GetId ();
    device->GetPhy ()->TraceConnectWithoutContext ("PhyRxDrop", MakeBoundCallback (&CongestionControl::PhyRxDrop, this, device));
    temp++; 
    //uint32_t nodeID = d->GetNode ()->GetId ();
    device->GetMac ()->SetMaxQueueSize(m_maxQueueSize);
    //csmaCa->SetUnSlottedCsmaCa ();
    //csmaCa->SetMac(device->GetMac ());
    //device->GetMac ()->SetCsmaCa (csmaCa);
    //McpsDataConfirmCallback cb0;
    //cb0 = MakeCallback (&DataConfirm);
    //device->GetMac ()->SetMcpsDataConfirmCallback (cb0);
    //device->GetMac ()->
    McpsDataIndicationCallback cb1;
    L2rReceiveUpdateCallback cb3;
    cb1 = MakeCallback (&DataIndication);
    device->GetMac ()->SetMcpsDataIndicationCallback (cb1);
    cb3 = MakeCallback (&L2rUpdateTcie);
    device->GetMac ()->SetL2rReceiveUpdateCallback(cb3);
    if(nodeID == m_meshNodeId)
      {
        meshRootRxMsgCallback cb2;
        cb2 = MakeCallback (&ReceivePacket);
        device->GetMac ()->SetMeshRootRxMsgUpdateCallback(cb2);
      }
  }
  //lrWpanHelper.EnableAsciiInternal(stram, "")
  if(m_enableTracing == true)
  {
    // Tracing
    AsciiTraceHelper ascii;
    Ptr<OutputStreamWrapper> stream = ascii.CreateFileStream (tr_name + ".tr");
    lrWpanHelper.EnableAsciiAll (stream);
  }
  if(m_enablePcap == true)
    lrWpanHelper.EnablePcapAll (std::string (tr_name), true);
  Packet::EnablePrinting ();
  devContainer.Get(m_meshNodeId)->GetObject<LrWpanNetDevice> ()->GetMac ()->L2R_AssignL2RProtocolForSink(true, 0xffff, m_periodicUpdateInterval);
  devContainer.Get(m_meshNodeId)->GetObject<LrWpanNetDevice> ()->GetMac ()->L2R_SendTopologyDiscovery();
  if (m_printRoutingTable)
  {
    Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper> ((tr_name + ".routes"), std::ios::out);
    for (NetDeviceContainer::Iterator i= devContainer.Begin(); i != devContainer.End (); i++)
    { 
      Ptr<NetDevice> d = *i;
      Ptr<LrWpanNetDevice> device = d->GetObject<LrWpanNetDevice> ();
      Simulator::Schedule (Seconds(1.01), &LrWpanMac::PrintRoutingTable,device->GetMac (),
                          d->GetNode(), routingStream,Time::S);
    }  
  }
  if (verbose)
  {
    lrWpanHelper.EnableLogComponents ();
  }
  //End of the CreateDevice function
}
void
CongestionControl::InstallApplications()
{  

  uint8_t temp = 0;
  for (NetDeviceContainer::Iterator i= devContainer.Begin(); i != devContainer.End (); i++)
  {
    Ptr<NetDevice> d = *i;
    if(d->GetNode()->GetId () == m_meshNodeId)
      continue;
    Ptr<ConstantRandomVariable> rvg = CreateObject<ConstantRandomVariable> ();
    Ptr<UniformRandomVariable> on = CreateObject<UniformRandomVariable> ();
    Ptr<UniformRandomVariable> off = CreateObject<UniformRandomVariable> ();
    Ptr<l2rapplication> app = CreateObject<l2rapplication> ();
    ch.Get (d->GetNode()->GetId ())->AddApplication (app);
    on->SetAttribute ("Min", DoubleValue (0.2));
    on->SetAttribute ("Max", DoubleValue (1.2));
    off->SetAttribute ("Min", DoubleValue (0.2));
    off->SetAttribute ("Max", DoubleValue (1.2));
    app->Setup(d,on, off);
    Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable> ();
    app->SetStartTime (Seconds (var->GetValue (m_dataStart, m_dataStart +2)));
    app->SetStopTime (Seconds (m_totalTime));
    app->SetPacketSize(m_packetSize);
    app->SetMaxBytes(0);
    app->AssignStreams(var->GetValue (1, m_nNodes));
    //app->SendBurst();
    //Simulator::Schedule(Seconds(20),&l2rapplication::SendBurst,app);
    //Simulator::Schedule(Seconds(40),&l2rapplication::SendBurst,app);
    ++temp;

  
    /*L2R_Header L2R_DataHeader;
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
    params.m_txOptions = TX_OPTION_NONE;
    device->GetMac ()->UpdateDelay(packet->GetUid(), Simulator::Now ());
    device->GetMac ()->OutputTree(packet,Simulator::Now (),params);
    Simulator::ScheduleNow (&LrWpanMac::McpsDataRequest,device->GetMac (),
                             params, packet);  
  m_lastStartTime = Simulator::Now ();
  m_residualBits = 0;*/
  }
  
}

void modify (const Mac16Address &sender,const uint16_t &depth, const uint16_t &pqm,const Mac16Address &receiver)
{
  /*std::ostringstream oss;
  oss << "Update:" << Simulator::Now ().GetSeconds ();
  pAnim->UpdateLinkDescription (0, 1, oss.str ());
  pAnim->UpdateLinkDescription (0, 2, oss.str ());
  pAnim->UpdateLinkDescription (0, 3, oss.str ());
  pAnim->UpdateLinkDescription (0, 4, oss.str ());
  pAnim->UpdateLinkDescription (0, 5, oss.str ());
  pAnim->UpdateLinkDescription (0, 6, oss.str ());
  pAnim->UpdateLinkDescription (1, 7, oss.str ());
  pAnim->UpdateLinkDescription (1, 8, oss.str ());
  pAnim->UpdateLinkDescription (1, 9, oss.str ());
  pAnim->UpdateLinkDescription (1, 10, oss.str ());
  pAnim->UpdateLinkDescription (1, 11, oss.str ());
  */
  std::ostringstream node0Oss;
  /*static uint32_t index = 0;
  index = depth %3;
  struct rgb color = colors[index];*/
  uint32_t nodeID;
  for (NetDeviceContainer::Iterator i= devContainer.Begin(); i != devContainer.End (); i++)
  {  
    Ptr<NetDevice> d = *i;
    Ptr<LrWpanNetDevice> device = d->GetObject<LrWpanNetDevice> ();
    nodeID = d->GetNode ()->GetId ();
    pAnim->UpdateNodeSize(nodeID, 15,15);
    if(receiver == device->GetAddress ())
    {
      node0Oss << nodeID <<"," << depth <<"," << pqm <<"," << receiver;
      // Every update change the node description for nodes
      pAnim->UpdateNodeDescription (nodeID, node0Oss.str ());
      // Every update change the color for nodes if receive update from mesh Root
      pAnim->UpdateNodeColor (nodeID, colors[depth].r, colors[depth].g,colors[depth].b); 
    }
  } 
}
void CongestionControl::PhyRxDrop (CongestionControl *cc,Ptr<LrWpanNetDevice> device, Ptr<const Packet> packet)
{
  /*std::ostringstream os;
  packet->Print (os);
  std::cout << std::setiosflags (std::ios::fixed) << std::setprecision (9) << "[" << Simulator::Now ().GetSeconds () << "] " << device->GetMac ()->GetShortAddress () << " PhyRxDrop: " << os.str () << std::endl;
*/
++m_totalPhyDrop;
}

uint64_t CongestionControl::m_totalPhyDrop = 0;
