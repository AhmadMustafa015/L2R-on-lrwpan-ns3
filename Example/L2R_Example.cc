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
#include "ns3/opengym-module.h"
#include "wsngym.h"
#include <math.h>
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
                        { 255, 0, 0 }, // Red
                        { 0, 255, 0 }, // Blue
                        { 0, 0, 255 }  // Green
                        };
static void DataIndication (McpsDataIndicationParams params, Ptr<Packet> p)
{
}
static void L2rUpdateTcie(McpsDataIndicationParams params, uint16_t depth, uint16_t pqm, Mac16Address receiver)
{
  Mac16Address senderAdd = params.m_srcAddr;
  modify(senderAdd,depth,pqm,receiver);
}

static void DataConfirm (McpsDataConfirmParams params)
{
  std::cout << "LrWpanMcpsDataConfirmStatus = " << params.m_status << std::endl;
}

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
                uint32_t openGymPort
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
  uint32_t m_openGymPort;
  Ptr<SingleModelSpectrumChannel> channel;
  NodeContainer ch;

private:
  /// Instantiate OpenGymInterface
  void CreateOpenGymInterface();
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
  void ReceivePacket (Ptr <Packet> packet);
  ///check Throughput
  void CheckThroughput ();
  //OnOffApplication
  /**
   * \brief Cancel all pending events.
   */
  //void CancelEvents ();
  //EventId         m_sendEvent;    //!< Event id of pending "send packet" event
};
int main (int argc, char *argv[])
{
  uint32_t openGymPort = 5555;
  CongestionControl congestionControl;
  uint32_t nNodes = 30;
  uint32_t nSinks = 1;
  double totalTime = 100;
  uint8_t periodicUpdateInterval = 15;
  double dataStart = 5.0;
  bool printRoutingTable = true;
  std::string CSVfileName = "CongestionControl.csv";
  uint32_t meshNodeId = 0;
  bool enableTracing = false;
  bool enablePcap = false;

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
  cmd.AddValue ("openGymPort", "OpenGymInterface port[Default:5555]", openGymPort);
  cmd.Parse (argc, argv);
  LogComponentEnable ("LrWpanMac", LOG_LEVEL_ALL);
  std::ofstream out (CSVfileName.c_str ());
  out << "SimulationSecond," <<
  "ReceiveRate," <<
  "PacketsReceived," <<
  "NumberOfSinks," <<
  std::endl;
  out.close ();
  SeedManager::SetSeed (167);

  congestionControl = CongestionControl();
  congestionControl.CaseRun (nNodes, nSinks, totalTime, periodicUpdateInterval,
                              dataStart, printRoutingTable, CSVfileName, 
                              enableTracing, enablePcap, meshNodeId, openGymPort);
  return 0;
}

CongestionControl::CongestionControl()
  :bytesTotal (0),
   packetsReceived (0)
{
}
void
CongestionControl::ReceivePacket (Ptr <Packet> packet)
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
                          bool enableTracing, bool enablePcap, uint32_t meshNodeId, uint32_t openGymPort)
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
  m_openGymPort = openGymPort;

  std::stringstream ss;
  ss << m_nNodes;
  std::string t_nodes = ss.str ();
  std::stringstream ss3;
  ss3 << m_totalTime;
  std::string m_TotalTime = ss3.str ();
  std::string tr_name = "L2R_" + t_nodes + "Nodes_" + m_TotalTime + "SimTime";
  std::cout << "Trace file generated is " << tr_name << ".tr\n";

  CreateOpenGymInterface();
  CreateNodes ();
  SetupMobility ();
  CreateDevices (tr_name);
  //InstallApplications ();
  std::string animFile = tr_name + ".xml";
  pAnim = new AnimationInterface (animFile); //Mandatory
  //pAnim->EnablePacketMetadata (); //Optional
  Simulator::Stop (Seconds (m_totalTime));
  Simulator::Run ();
  openGymInterface->NotifySimulationEnd();
  std::cout << "Animation Trace file created:" << animFile.c_str ()<< std::endl;
  Simulator::Destroy ();
  delete pAnim;
}
void
CongestionControl::CreateOpenGymInterface(uint32_t openGymPort)
{
  Ptr<OpenGymInterface> openGymInterface;
  openGymInterface = OpenGymInterface::Get(openGymPort);

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
  int xPos, yPos, txRange;
  uint32_t nodeCount;
  float radius, thetaRad;
  xPos = yPos = 0;
  txRange = 90;
  radius = txRange;
  thetaRad = (txRange / radius);

  std::cout << "Adding node at position (" << xPos << ", " << yPos << ")" << std::endl;
  taPositionAlloc->Add (Vector (xPos, yPos, 0.0)); // mesh root?

  for (nodeCount = 1; nodeCount < m_nNodes; nodeCount++)
  {
    thetaRad += (txRange / radius)* (1 + x->GetValue()*0.3);
    xPos = radius * sin(thetaRad) + x->GetValue()*30; 
    yPos = radius * cos(thetaRad) + x->GetValue()*30;

    std::cout << "Adding node at position (" << xPos << ", " << yPos << ")" <<std::endl;
    taPositionAlloc->Add (Vector (xPos, yPos, 0.0));

    if (thetaRad > (2 * PI - (txRange / radius)))
    {
      radius += txRange;
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
    McpsDataConfirmCallback cb0;
    cb0 = MakeCallback (&DataConfirm);
    device->GetMac ()->SetMcpsDataConfirmCallback (cb0);
    //device->GetMac ()->
    McpsDataIndicationCallback cb1;
    L2rReceiveUpdateCallback cb3;
    cb1 = MakeCallback (&DataIndication);
    device->GetMac ()->SetMcpsDataIndicationCallback (cb1);
    cb3 = MakeCallback (&L2rUpdateTcie);
    device->GetMac ()->SetL2rReceiveUpdateCallback(cb3);
  }
  if(m_enableTracing == true)
  {
    // Tracing
    AsciiTraceHelper ascii;
    Ptr<OutputStreamWrapper> stream = ascii.CreateFileStream (tr_name + ".tr");
    lrWpanHelper.EnableAsciiAll (stream);
    if(m_enablePcap == true)
      lrWpanHelper.EnablePcapAll (std::string (tr_name), true);
  }
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
  index++;
  if (index == 3) 
    index = 0;
  struct rgb color = colors[index];
  for (NetDeviceContainer::Iterator i= devContainer.Begin(); i != devContainer.End (); i++)
  {  
    Ptr<NetDevice> d = *i;
    Ptr<LrWpanNetDevice> device = d->GetObject<LrWpanNetDevice> ();
    if(receiver == device->GetAddress ())
    {
      uint32_t nodeID = d->GetNode ()->GetId ();
      node0Oss << "N:" << nodeID <<" D:" << depth <<" PQM: " << pqm <<" MAC:" << receiver;
      // Every update change the node description for nodes
      pAnim->UpdateNodeDescription (nodeID, node0Oss.str ());
      // Every update change the color for nodes if receive update from mesh Root
      pAnim->UpdateNodeColor (nodeID, color.r, color.g, color.b); 
    }
  }  
}

//onOffApplication
/*void
CongestionControl::CancelEvents()
{
  if (m_sendEvent.IsRunning ())
  { // Cancel the pending send packet event
    // Calculate residual bits since last packet sent
    Time delta (Simulator::Now () - m_lastStartTime);
    int64x64_t bits = delta.To (Time::S) * m_cbrRate.GetBitRate ();
    m_residualBits += bits.GetHigh ();
  }
}*/
