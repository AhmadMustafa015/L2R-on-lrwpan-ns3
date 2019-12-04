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
using namespace ns3;

#define PI 3.14159265
bool verbose = false;
AnimationInterface * pAnim = 0;
NetDeviceContainer devContainer;
LrWpanHelper lrWpanHelper;
void modify (const Mac16Address &sender,const uint16_t &depth, const uint16_t &pqm,const Mac16Address &receiver);
/// RGB structure
struct rgb {
  uint8_t r; ///< red
  uint8_t g; ///< green
  uint8_t b; ///< blue
};
struct rgb colors [] = {
                        {31, 40 , 8}, // Red
                        { 2, 71, 93 }, // Blue
                        { 89, 3, 80 }  // Green
                        };
static void DataIndication (McpsDataIndicationParams params, Ptr<Packet> p)
{
}
static void L2rUpdateTcie(McpsDataIndicationParams params, uint16_t depth, uint16_t pqm, Mac16Address receiver)
{
  Mac16Address senderAdd = params.m_srcAddr;
  modify(senderAdd,depth,pqm,receiver);
}

/*static void DataConfirm (McpsDataConfirmParams params)
{
  std::cout << "LrWpanMcpsDataConfirmStatus = " << params.m_status << std::endl;
}
*/
static void StateChangeNotification (std::string context, Time now, LrWpanPhyEnumeration oldState, LrWpanPhyEnumeration newState)
{
}
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
  std::vector<l2rapplication> m_applicationContainer;

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
  void ReceivePacket (MeshRootData para,Mac16Address srcAddress);
  ///check Throughput
  void CheckThroughput ();
};
int main (int argc, char *argv[])
{
  CongestionControl congestionControl;
  uint32_t nNodes = 30;
  uint32_t nSinks = 1;
  double totalTime = 100;
  uint8_t periodicUpdateInterval = 50;
  double dataStart = 5.0;
  bool printRoutingTable = true;
  std::string CSVfileName = "CongestionControl.csv";
  uint32_t meshNodeId = 0;
  bool enableTracing = false;
  bool enablePcap = true;
  uint32_t distanceBtwNodes = 80; 

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
  std::ofstream out (CSVfileName.c_str ());
  out << "SimulationSecond," <<
  "Sender Mac Addr," <<
  "Normalized Queue Length," <<
  "Arrival Rate Moving Avg," <<
  "Avg Delay," <<
  std::endl;
  out.close ();
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
CongestionControl::ReceivePacket (MeshRootData para,Mac16Address srcAddress)
{
  std::ofstream out (m_CSVfileName.c_str (), std::ios::app);
  out << (Simulator::Now ()).GetSeconds ()<< "," << srcAddress << "," << para.m_queueSize 
      << "," << para.m_arrivalRate << "," << para.m_avgDelay << std::endl;
  out.close ();
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

  std::stringstream ss;
  ss << m_nNodes;
  std::string t_nodes = ss.str ();
  std::stringstream ss3;
  ss3 << m_totalTime;
  std::string m_TotalTime = ss3.str ();
  std::string tr_name = "L2R_" + t_nodes + "Nodes_" + m_TotalTime + "SimTime";
  std::cout << "Trace file generated is " << tr_name << ".tr\n";

  CreateNodes ();
  SetupMobility ();
  CreateDevices (tr_name);

    Ptr<NetDevice> d = devContainer.Get (4);
    Ptr<LrWpanNetDevice> device = d->GetObject<LrWpanNetDevice> ();
    //std::cout << "Node: " << nodeID << "Send data packet to: ";
    /*L2R_Header dataHeader;
    dataHeader.SetMsgType(DataHeader);
    Ptr<Packet> p0 = Create<Packet> (); //Zero payload packet
    p0->AddHeader (dataHeader); //serialize is called here
    McpsDataRequestParams params;
    params.m_dstPanId = 10;
    params.m_srcAddrMode = SHORT_ADDR;
    params.m_dstAddrMode = SHORT_ADDR;
    params.m_dstAddr = Mac16Address("00:04");
    std::cout << params.m_dstAddr << std::endl;
    params.m_msduHandle = 0; //ToDo underStand the msduhandle from standard
    params.m_txOptions = TX_OPTION_ACK; 
    for(uint8_t i =0; i <10; i++) 
    {
      std::cout << "Sending Data Packet From: " << "00:05" << "To: " << "00:04" << std::endl;
      Simulator::ScheduleWithContext (1, Seconds (8),
                                      &LrWpanMac::McpsDataRequest,
                                      devContainer.Get(4)->GetObject<LrWpanNetDevice> ()->GetMac (), params, p0);
    }
    std::cout << "Data Rate: " << d->GetObject<LrWpanNetDevice> ()->GetPhy ()->GetDataOrSymbolRate(true) <<std::endl;
  //InstallApplications ();*/
  std::string animFile = tr_name + ".xml";
  pAnim = new AnimationInterface (animFile); //Mandatory
  //pAnim->EnablePacketMetadata (); //Optional
  Simulator::Stop (Seconds (m_totalTime));
  Simulator::Run ();
  std::cout << "Animation Trace file created:" << animFile.c_str ()<< std::endl;
  Simulator::Destroy ();
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
    thetaRad += (m_distanceBtwNodes / radius)* (1 + x->GetValue()*0.3);
    xPos = radius * sin(thetaRad) + x->GetValue()*30; 
    yPos = radius * cos(thetaRad) + x->GetValue()*30;

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
  Ptr<LogDistancePropagationLossModel> propModel = CreateObject<LogDistancePropagationLossModel> ();
  Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel> ();
  channel->AddPropagationLossModel (propModel);
  channel->SetPropagationDelayModel (delayModel);

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
    device->GetPhy ()->TraceConnect ("TrxState", std::string ("phy" + temp), MakeCallback (&StateChangeNotification));
    temp++; 
    uint32_t nodeID = d->GetNode ()->GetId ();
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
        cb3 = MakeCallback (&ReceivePacket);
        device->GetMac ()->SetMeshRootRxMsgUpdateCallback(cb3);
      }
  }
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
  devContainer.Get(m_meshNodeId)->GetObject<LrWpanNetDevice> ()->GetMac ()->L2R_AssignL2RProtocolForSink(true, 8, m_periodicUpdateInterval);
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
    //Ptr<LrWpanNetDevice> device = d->GetObject<LrWpanNetDevice> ();
    m_applicationContainer.push_back(l2rapplication(d));
    Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable> ();
    m_applicationContainer[temp].Start(Seconds (var->GetValue (m_dataStart, m_dataStart +1)));
    m_applicationContainer[temp].Stop (Seconds (m_totalTime));
    ++temp;
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
  static uint32_t index = 0;
  index = depth %3;
  struct rgb color = colors[index];
  uint32_t nodeID;
  for (NetDeviceContainer::Iterator i= devContainer.Begin(); i != devContainer.End (); i++)
  {  
    Ptr<NetDevice> d = *i;
    Ptr<LrWpanNetDevice> device = d->GetObject<LrWpanNetDevice> ();
    nodeID = d->GetNode ()->GetId ();
    pAnim->UpdateNodeSize(nodeID, 15,15);
    if(receiver == device->GetAddress ())
    {
      node0Oss << "N:" << nodeID <<" D:" << depth <<" PQM: " << pqm <<" MAC:" << receiver;
      // Every update change the node description for nodes
      pAnim->UpdateNodeDescription (nodeID, node0Oss.str ());
      // Every update change the color for nodes if receive update from mesh Root
      pAnim->UpdateNodeColor (nodeID, (color.r * (depth+1) %255), (color.r * (depth+2) %255),(color.r * (depth+3) %255)); 
    }
  } 
}

