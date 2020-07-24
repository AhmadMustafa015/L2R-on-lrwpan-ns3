# L2R-on-lrwpan-ns3
##ABSTRACT
Wireless sensor networks have proven to be a valuable resource in recent times. For example, it can be used in disaster situations, pollution monitoring, and medical applications. However, these networks suffer from several challenges, including network congestion, which occurs when the load of the system is increased. Congestion can reduce the amount of data received, causes packet drops, and negatively affects the latency of this system. In this project, a congestion control algorithm is introduced to alleviate congestion and mitigate its negative effects. The algorithm is tested over the recently-standardized IEEE 802.15.10 network released in 2017. Machine learning was employed, specifically reinforcement learning, in order to create a learning algorithm that interacts with the wireless sensor network and change its parameters accordingly. As will be shown, this has improved congestion considerably. The algorithm was tested using the well-known NS-3 simulator, and reinforcement learning was executed with the help of Google’s TensorFlow, and OpenAI Gym.

##3.1 Writing the new IEEE 802.15.10 standard in NS-3
After spending hours searching for any implementation or open-source code, none were found for the standard. So, we decided to write the code from scratch. As we mentioned in the previous chapter, IEEE 802.15.10 is required to be merged with the IEEE 802.15.4 standard. Writing the code for the new standard was a real challenge, not considering also the complexity of the NS-3 simulator. Before we decided to go the hard way, we try to use other MANET routing protocols, but we found that these protocols aren’t designed to fit the requirement of WSNs, such as Destination-Sequenced Distance Vector routing (DSDV). After simulation using NS-3, we found that LR-WPAN drops most of the packets because DSDV is designed such that each node broadcasts its routing table which increases the load on the system [20]. We will address the procedure for writing the standard in the next subsections.
IEEE 802.15.10 Introduced three types of routing P2P, Upstream, and DS. In our system design, we wrote the Upstream routing part of the standard. Since we want to simulate the traffic goes from devices to the mesh root.
In upstream routing, each device selects next hop from its local NT, as shown in Table 3. Each device doesn’t know the entire path to the mesh root. Instead, it chooses next hop based on ancestor available in local NT. That’s for Upstream routing required to have at least one ancestor in the local NT. Devices may also route a frame through a sibling.
Figure 11 shows an example of Upstream routing between A and M, where M is a mesh root. The smaller the PQM value the better the path to the mesh root. The plain and dashed arrows represent the Upstream route for different LQT 3 and 5, respectively. When LQT is set to 5, a device forwards the frame to the ancestor with LQM less than or equal LQT. Device A has two ancestors with LQM less than or equal to 5, B and E. If there are multiple ancestors with LQM satisfying LQT then the device chooses the ancestor with the lowest PQM value (better path). Since the PQM over the link from A to E (10) is less than the PQM over the link from A to B (11) then device A chooses E as a its next hop. Then the path to the mesh root is A-E-M. On the other hand, when LQT is set to 3, device A doesn’t have any ancestor with LQM equal to or less than LQT. When a device doesn't have any ancestor satisfying the LQM-LQT condition then, the frame is forwarded to the ancestor with the lowest LQM value. As seen in the example device A forwards the frame to the ancestor with the lowest LQM which is device B. Then the path to the mesh root is A-B-E-M.
 
The outgoing frame is processed according to the algorithm in Figure 12. The device adds L2R IE and MHR to the frame and transmits the frame through the physical layer.
 
Figure 13 illustrates the algorithm used to deal with the incoming frame from the physical layer. If the frame passes the MAC filtering at the MAC layer, the frame is delivered to the L2R sublayer. If DA matches the device’s address (usually mesh root address) the frame is delivered to the next higher layer. Otherwise, the frame is forwarded again to the next hop device.
 
Figure 13: Receiver frame filtering
In the following subsections, we will discuss the procedure to write the new IEEE 802.15.10 standard over IEEE 802.15.4 standard.
3.1.1 Mesh discovery procedure
The next higher layer of a joining device issues the L2RLME-PAN-SCAN.request primitive to request the broadcast of an enhanced beacon request (EBR) with an L2R Discovery (L2R-D) IE where the content field is set as described in sections 5.1.2.1.1 or 5.1.2.1.2 of the standard [15].
The code below shows the procedure to send the topology discovery message by mesh root. As seen only mesh root can initiate the topology discovery message.

Listing 1 (C++)
1.	void  
2.	LrWpanMac::L2R_SendTopologyDiscovery()  
3.	{  
4.	  if(!m_isSink)  
5.	  {  
6.	    NS_ABORT_MSG ("only Sink can start the topology");  
7.	    return;  
8.	  }  
9.	  else  
10.	  {  
11.	    NS_LOG_FUNCTION ("Sending Topology Discovery Msg ");  
12.	    L2R_Header L2R_DIE;  
13.	    m_rootAddress = m_shortAddress;  
14.	    L2R_DIE.SetMeshRootAddress(m_rootAddress);  
15.	    L2R_DIE.SetMsgType(L2R_D_IE);  
16.	    Ptr<Packet> p0 = Create<Packet> ();  
17.	    p0->AddHeader (L2R_DIE);  
18.	    McpsDataRequestParams params;  
19.	    params.m_dstPanId = this->GetPanId();  
20.	    params.m_srcAddrMode = SHORT_ADDR;  
21.	    params.m_dstAddrMode = SHORT_ADDR;  
22.	    params.m_dstAddr = "ff:ff";  
23.	    params.m_msduHandle = 0;  
24.	    params.m_txOptions = TX_OPTION_NONE;  
25.	    Simulator::ScheduleNow (&LrWpanMac::McpsDataRequest,this,  
26.	                             params, p0);  
27.	    L2R_Start(); 
28.	  }  
29.	}  
Since we only simulate the algorithm on a single PAN, we only consider to add mesh root address in the L2R-D IE. 
The L2R-D IE is sent in an EBR with the destination PAN Identifier and the Destination Address fields set to 0xffff. Each device receives the L2R-D IE, broadcasts the message. At the end of the broadcasting, all devices will be aware of the mesh root address.
3.1.2 L2R mesh construction and building routing table
Upstream routing is constructed based on the information in Table 2 and Table 3. Those tables are built based on the information retrieved from L2R-D IE and TC IE. Each device manages the local MT table and NT table. MT table contains information about the mesh while the local NT table contains information about neighbors.

When mesh root initiates an L2R mesh by sending the first L2R-D IE, it sends the first TC IE with MSN set to value between 0xf0 and 0xff. MSN used to determine if the received TC IE is older or newer than the saved TC IE on that device. Also, it’s used to tell the devices connected to the mesh that the mesh root is in the reinitialization phase such that each device will reset its local MT if they received TC IE with MSN between 0xf0 and 0xff after receiving the initialization TC IE from mesh root. When a device receives a TC IE, it updates the local MT and NT tables to match the information in the TC IE message.
The relevant information of the L2R Discovery (L2R-D IE) and in the TC IE is stored in an NT as shown in Table 3. Each device manages a number of NT equal to the number of neighbors of that device. Assuming device A has 4 neighbors then it will have 4 local NT. All local NTs that a device had we will refer to it as the routing table.


PQM Value	Depends on the metric ID	Value of the PQM provided by the current neighbor.
TC IE interval	0 – 63	Interval between the neighbor’s RA IE transmissions in the unit specified by interval unit. Equal to the TC IE interval if set to 0.
NLM IE interval	0 – 63	Interval between the neighbor’s NLM IE transmissions in the unit specified by interval unit. A value of 0 indicates that no NLM IE has been received from the neighbor
List of reachable destinations	Short address	List of devices that are reachable
through the neighbor.
LQM	Depends on the metric ID	Value of the LQM between the current device and the current neighbor.

The below code illustrates the steps of sending the first TC IE by mesh root, the PQM value of the mesh root is set to zero. Line 17 to 19 aren’t part of the standard that have an important part to calculate the LQT value, we will discuss these parameters in section ‎3.2 (Contributions on the IEEE 802.15.10 standard to avoid congestion).






Listing 2 (C++)
1.	void  
2.	LrWpanMac::L2R_SendPeriodicUpdate()  
3.	{  
4.	  std::map <Mac16Address, L2R_RoutingTableEntry> removedAddresses;  
5.	  m_routingTable.Purge (removedAddresses);  
6.	  if (m_isSink)  
7.	  {  
8.	    NS_LOG_FUNCTION (Seconds(Simulator::Now ()) << "Sending TC-IE by Sink ");  
9.	    L2R_Header TC_IE_H;  
10.	    TC_IE_H.SetMeshRootAddress(m_rootAddress);  
11.	    TC_IE_H.SetMsgType(TC_IE);  
12.	    TC_IE_H.SetPQM(0);  
13.	    TC_IE_H.SetMSN(m_msn);  
14.	    TC_IE_H.SetLQT(m_lqt);  
15.	    TC_IE_H.SetTCIEInterval(m_tcieInterval);  
16.	    TC_IE_H.SetDepth(m_depth);  
17.	    TC_IE_H.SetQueueSize(0);  
18.	    TC_IE_H.SetDelay(0);  
19.	    TC_IE_H.SetArrivalRate(0);  
20.	    Ptr<Packet> p0 = Create<Packet> ();  
21.	    p0->AddHeader (TC_IE_H);  
22.	    McpsDataRequestParams params;  
23.	    params.m_dstPanId = this->GetPanId();  
24.	    params.m_srcAddrMode = SHORT_ADDR;  
25.	    params.m_dstAddrMode = SHORT_ADDR;  
26.	    params.m_dstAddr = "ff:ff";  
27.	    params.m_msduHandle = 0;  
28.	    params.m_txOptions = TX_OPTION_NONE;  
29.	    Simulator::ScheduleNow (&LrWpanMac::McpsDataRequest,this,  
30.	                             params, p0);  
31.	    if(m_msn > 0xef && m_msn <= 0xff)  
32.	      m_msn = 0x00;  
33.	    else  
34.	      ++m_msn;  
35.	  m_periodicUpdateTimer.Schedule (Seconds(m_tcieInterval));     
When a device receives a TC IE from a neighbor A, B calculates the LQM value between A and B then adds this value to the PQM value of device A obtained from TC IE. Device A adds this entry to the routing table if it does not exist, or updates the entry if it’s already there and the new MSN value is greater than the previous MSN value. Each device rebroadcasts the TC IE message received from neighbors.

Before transmitting the new TC IE, device B browses its routing table, finds the neighbor with the best PQM value then sets its own PQM to that value. Assuming that the depth of the neighbor with best PQM equals D then device A sets its own depth to D+1.
 
Figure 14, Figure 15, Figure 16 and Figure 17 show an example of mesh construction. This Example is an output of actual simulation for 20 devices and one mesh root. We wrote this code for the scenario in the NS-3 simulator and generated an XML file to illustrate the behavior of the protocol as animation. As seen, when the virtual simulation time equals to 0.02, mesh root initiates the first L2R-D IE. After that phase, each device will be aware of the mesh root MAC address. After sending discovery messages, mesh root sends the first TC IE, devices within range of transmission (device 1 to 5) receive this message and update their MT based on the information received from mesh root.
At virtual simulation time, 0.052 devices 1 to 5 received the TC IE successfully from the mesh root and update their depth and PQM. As seen in Figure 17 devices with green color represent depth equal 1, blue color for a depth equal to 2, and orange color for a depth equal to 3. The four values below each node represent node ID, depth, PQM and MAC address, respectively.
When simulation time is  0.1. All devices in this scenario are initiated with their depth and PQM value and ready to start sending actual data.
  
The routing table for node 8 is shown below. The table contains all the neighbors and their MAC addresses, depths, PQM values, lifetime and TC IE intervals. Lifetime represents the total time since the last update for respective entry, this is an important value to record, in order to delete the outdated entry. TC IE interval represents the time between update messages.
