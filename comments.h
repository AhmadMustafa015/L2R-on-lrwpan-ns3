
  
  //NS_LOG_INFO ("Create channels.");
  
  // Trace state changes in the phy
 
  

  // The below should trigger two callbacks when end-to-end data is working
  // 1) DataConfirm callback is called
  // 2) DataIndication callback is called with value of 50

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

  
  /*Simulator::ScheduleWithContext(1,MicroSeconds(6),
                                &LrWpanMac::L2R_Start,
                                devContainer.Get(1)->GetObject<LrWpanNetDevice> ()->GetMac ());
 *///devContainer.Get(1)->GetObject<LrWpanNetDevice> ()->GetMac ()->L2R_Start();
  /*if (printRoutingTable)
  {
    Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper> ((tr_name + ".routes"), std::ios::out);
    lrWpanHelper.PrintRoutingTableAllAt (ch,Seconds (tcieInterval + 1), routingStream,Time::S);
  }*/
