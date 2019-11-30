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
using namespace ns3;


static void DataIndication (McpsDataIndicationParams params, Ptr<Packet> p)
{
  //NS_LOG_UNCOND ("Received packet of size " << p->GetSize ());
  
  //To do send another packet after successfully recieve the packet
  //this->McpsDataRequest(params, p);
}

static void DataConfirm (McpsDataConfirmParams params)
{
  //NS_LOG_UNCOND ("LrWpanMcpsDataConfirmStatus = " << params.m_status);
  std::cout << "LrWpanMcpsDataConfirmStatus = " << params.m_status << std::endl;

}

static void StateChangeNotification (std::string context, Time now, LrWpanPhyEnumeration oldState, LrWpanPhyEnumeration newState)
{
  /*NS_LOG_UNCOND (context << " state change at " << now.GetSeconds ()
                         << " from " << LrWpanHelper::LrWpanPhyEnumerationPrinter (oldState)
                         << " to " << LrWpanHelper::LrWpanPhyEnumerationPrinter (newState));*/
  /*std::cout << context<< " state change at " << now.GetSeconds ()
                         << " from " << LrWpanHelper::LrWpanPhyEnumerationPrinter (oldState)
                         << " to " << LrWpanHelper::LrWpanPhyEnumerationPrinter (newState)
                         <<std::endl;*/
}

int main (int argc, char *argv[])
{
  bool verbose = false;
  bool extended = false;
  bool printRoutingTable = true;
  int nNodes = 20;
  double sTotalTime = 80;
  uint8_t tcieInterval = 7;
  int xMax = 200; //max x direction
  int yMax = 200; //max y direction
  //int nSinks = 1;
  //LogComponentEnable ("LrWpanPhy",LOG_LEVEL_ALL);
  LogComponentEnable ("LrWpanMac",LOG_LEVEL_ALL);
  std::stringstream ss;
  ss << nNodes;
  std::string t_nodes = ss.str ();

  std::stringstream ss3;
  ss3 << sTotalTime;
  std::string m_TotalTime = ss3.str ();
  std::string tr_name = "L2R_" + t_nodes + "Nodes_" + m_TotalTime + "SimTime";
  CommandLine cmd;

  cmd.AddValue ("verbose", "turn on all log components", verbose);
  cmd.AddValue ("extended", "use extended addressing", extended);

  cmd.Parse (argc, argv);

  LrWpanHelper lrWpanHelper;
  if (verbose)
  {
    lrWpanHelper.EnableLogComponents ();
  }
  //NS_LOG_INFO ("Create nodes.");
  NodeContainer ch;
  ch.Create (nNodes);

  // Set seed for random numbers
  SeedManager::SetSeed (167);

  // Install mobility
  MobilityHelper mobility;
  ObjectFactory pos;
  pos.SetTypeId ("ns3::RandomRectanglePositionAllocator");
  pos.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max="+std::to_string(xMax)+"]"));
  pos.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max="+std::to_string(yMax)+"]"));
  /*Ptr <UniformRandomVariable> x= CreateObject <UniformRandomVariable>();
  Ptr<ListPositionAllocator> taPositionAlloc = CreateObject<ListPositionAllocator> ();
  int nodeCount, xPos, yPos;
  xPos = 0;
  yPos = 0;
  for (nodeCount=1; nodeCount<=nNodes; nodeCount++)
  {
    xPos=((int)(x->GetValue()*1000))%100+1; yPos=((int)(x->GetValue()*1000))%200+1;
    
    std::cout << "Adding node at position (" << xPos << ", " << yPos << ")" << std::endl;
    taPositionAlloc->Add (Vector (xPos, yPos, 0.0));
    //xPos = xPos + 50;
    //yPos = yPos + 50;
  }*/
  Ptr <PositionAllocator> taPositionAlloc = pos.Create ()->GetObject <PositionAllocator> ();
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.SetPositionAllocator (taPositionAlloc);
  mobility.Install (ch);

  //NS_LOG_INFO ("Create channels.");
  Ptr<SingleModelSpectrumChannel> channel = CreateObject<SingleModelSpectrumChannel> ();
  Ptr<LogDistancePropagationLossModel> propModel = CreateObject<LogDistancePropagationLossModel> ();
  Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel> ();
  channel->AddPropagationLossModel (propModel);
  channel->SetPropagationDelayModel (delayModel);

  lrWpanHelper.SetChannel(channel);
  // Add and install the LrWpanNetDevice for each node
  NetDeviceContainer devContainer = lrWpanHelper.Install(ch);
  lrWpanHelper.AssociateToPan (devContainer, 10);

  std::cout << "Created " << devContainer.GetN() << " devices" << std::endl;
  std::cout << "There are " << ch.GetN() << " nodes" << std::endl;  


  
  // Trace state changes in the phy
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
    cb1 = MakeCallback (&DataIndication);
    device->GetMac ()->SetMcpsDataIndicationCallback (cb1);
    
  }

  // Tracing
  //lrWpanHelper.EnablePcapAll (std::string ("lr-wpan-data"), true);
  //AsciiTraceHelper ascii;
  //Ptr<OutputStreamWrapper> stream = ascii.CreateFileStream ("lr-wpan-data.tr");
  //lrWpanHelper.EnableAsciiAll (stream);

  // The below should trigger two callbacks when end-to-end data is working
  // 1) DataConfirm callback is called
  // 2) DataIndication callback is called with value of 50
  Packet::EnablePrinting ();

  // instantiate a header.
  /*Ptr<Packet> p0 = Create<Packet> (50);
  //sending process
  McpsDataRequestParams params;
  params.m_dstPanId = 10;
  if (!extended)
    {
      params.m_srcAddrMode = SHORT_ADDR;
      params.m_dstAddrMode = SHORT_ADDR;
      params.m_dstAddr = Mac16Address ("00:03");
    }
  params.m_msduHandle = 0;
  params.m_txOptions = TX_OPTION_ACK;
//  dev0->GetMac ()->McpsDataRequest (params, p0);
  Simulator::ScheduleWithContext (1, Seconds (0.0),
                                  &LrWpanMac::McpsDataRequest,
                                  devContainer.Get(0)->GetObject<LrWpanNetDevice> ()->GetMac (), params, p0);

  Ptr<Packet> p2 = Create<Packet> (60);  // 60 bytes of dummy data
  //p2->AddHeader (sourceHeader);
  if (!extended)
  {
    params.m_dstAddr = Mac16Address ("ff:ff");
  }
  Simulator::ScheduleWithContext (1, Seconds (1.0),
                                  &LrWpanMac::McpsDataRequest,
                                  devContainer.Get(0)->GetObject<LrWpanNetDevice> ()->GetMac (), params, p2);*/

  devContainer.Get(1)->GetObject<LrWpanNetDevice> ()->GetMac ()->L2R_AssignL2RProtocolForSink(true, 8, tcieInterval);
  devContainer.Get(1)->GetObject<LrWpanNetDevice> ()->GetMac ()->L2R_SendTopologyDiscovery();
  /*Simulator::ScheduleWithContext(1,MicroSeconds(6),
                                &LrWpanMac::L2R_Start,
                                devContainer.Get(1)->GetObject<LrWpanNetDevice> ()->GetMac ());
 *///devContainer.Get(1)->GetObject<LrWpanNetDevice> ()->GetMac ()->L2R_Start();
  /*if (printRoutingTable)
  {
    Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper> ((tr_name + ".routes"), std::ios::out);
    lrWpanHelper.PrintRoutingTableAllAt (ch,Seconds (tcieInterval + 1), routingStream,Time::S);
  }*/

  if (printRoutingTable)
  {
    Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper> ((tr_name + ".routes"), std::ios::out);
    for (NetDeviceContainer::Iterator i= devContainer.Begin(); i != devContainer.End (); i++)
    { 
      Ptr<NetDevice> d = *i;
      Ptr<LrWpanNetDevice> device = d->GetObject<LrWpanNetDevice> ();
      Simulator::Schedule (Seconds (tcieInterval + 1), &LrWpanMac::PrintRoutingTable,device->GetMac (),
                          d->GetNode(), routingStream,Time::S);
    }  
  }
  Simulator::Stop (Seconds (sTotalTime));
  Simulator::Run ();
  AnimationInterface anim ("l2r-routing.xml");

  Simulator::Destroy ();
  return 0;
}
