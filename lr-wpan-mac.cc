/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 The Boeing Company
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Authors:
 *  Gary Pei <guangyu.pei@boeing.com>
 *  kwong yin <kwong-sang.yin@boeing.com>
 *  Tom Henderson <thomas.r.henderson@boeing.com>
 *  Sascha Alexander Jopen <jopen@cs.uni-bonn.de>
 *  Erwan Livolant <erwan.livolant@inria.fr>
 */
#include "lr-wpan-mac.h"
#include "lr-wpan-csmaca.h"
#include "lr-wpan-mac-header.h"
#include "lr-wpan-mac-trailer.h"
#include <ns3/simulator.h>
#include <ns3/log.h>
#include <ns3/uinteger.h>
#include <ns3/node.h>
#include <ns3/packet.h>
#include <ns3/random-variable-stream.h>
#include <ns3/double.h>
#include <iomanip>
#include "ns3/address-utils.h"
#include "lr-wpan-net-device.h"
#undef NS_LOG_APPEND_CONTEXT
#define NS_LOG_APPEND_CONTEXT                                   \
  std::clog << "[address " << m_shortAddress << "] ";

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("LrWpanMac");

NS_OBJECT_ENSURE_REGISTERED (LrWpanMac);

const uint32_t LrWpanMac::aMinMPDUOverhead = 9; // Table 85

TypeId
LrWpanMac::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::LrWpanMac")
    .SetParent<Object> ()
    .SetGroupName ("LrWpan")
    .AddConstructor<LrWpanMac> ()
    .AddAttribute ("PanId", "16-bit identifier of the associated PAN",
                   UintegerValue (),
                   MakeUintegerAccessor (&LrWpanMac::m_macPanId),
                   MakeUintegerChecker<uint16_t> ())
    .AddTraceSource ("MacTxEnqueue",
                     "Trace source indicating a packet has been "
                     "enqueued in the transaction queue",
                     MakeTraceSourceAccessor (&LrWpanMac::m_macTxEnqueueTrace),
                     "ns3::Packet::TracedCallback")
    .AddTraceSource ("MacTxDequeue",
                     "Trace source indicating a packet has was "
                     "dequeued from the transaction queue",
                     MakeTraceSourceAccessor (&LrWpanMac::m_macTxDequeueTrace),
                     "ns3::Packet::TracedCallback")
    .AddTraceSource ("MacTx",
                     "Trace source indicating a packet has "
                     "arrived for transmission by this device",
                     MakeTraceSourceAccessor (&LrWpanMac::m_macTxTrace),
                     "ns3::Packet::TracedCallback")
    .AddTraceSource ("MacTxOk",
                     "Trace source indicating a packet has been "
                     "successfully sent",
                     MakeTraceSourceAccessor (&LrWpanMac::m_macTxOkTrace),
                     "ns3::Packet::TracedCallback")
    .AddTraceSource ("MacTxDrop",
                     "Trace source indicating a packet has been "
                     "dropped during transmission",
                     MakeTraceSourceAccessor (&LrWpanMac::m_macTxDropTrace),
                     "ns3::Packet::TracedCallback")
    .AddTraceSource ("MacPromiscRx",
                     "A packet has been received by this device, "
                     "has been passed up from the physical layer "
                     "and is being forwarded up the local protocol stack.  "
                     "This is a promiscuous trace,",
                     MakeTraceSourceAccessor (&LrWpanMac::m_macPromiscRxTrace),
                     "ns3::Packet::TracedCallback")
    .AddTraceSource ("MacRx",
                     "A packet has been received by this device, "
                     "has been passed up from the physical layer "
                     "and is being forwarded up the local protocol stack.  "
                     "This is a non-promiscuous trace,",
                     MakeTraceSourceAccessor (&LrWpanMac::m_macRxTrace),
                     "ns3::Packet::TracedCallback")
    .AddTraceSource ("MacRxDrop",
                     "Trace source indicating a packet was received, "
                     "but dropped before being forwarded up the stack",
                     MakeTraceSourceAccessor (&LrWpanMac::m_macRxDropTrace),
                     "ns3::Packet::TracedCallback")
    .AddTraceSource ("Sniffer",
                     "Trace source simulating a non-promiscuous "
                     "packet sniffer attached to the device",
                     MakeTraceSourceAccessor (&LrWpanMac::m_snifferTrace),
                     "ns3::Packet::TracedCallback")
    .AddTraceSource ("PromiscSniffer",
                     "Trace source simulating a promiscuous "
                     "packet sniffer attached to the device",
                     MakeTraceSourceAccessor (&LrWpanMac::m_promiscSnifferTrace),
                     "ns3::Packet::TracedCallback")
    .AddTraceSource ("MacStateValue",
                     "The state of LrWpan Mac",
                     MakeTraceSourceAccessor (&LrWpanMac::m_lrWpanMacState),
                     "ns3::TracedValueCallback::LrWpanMacState")
    .AddTraceSource ("MacState",
                     "The state of LrWpan Mac",
                     MakeTraceSourceAccessor (&LrWpanMac::m_macStateLogger),
                     "ns3::LrWpanMac::StateTracedCallback")
    .AddTraceSource ("MacSentPkt",
                     "Trace source reporting some information about "
                     "the sent packet",
                     MakeTraceSourceAccessor (&LrWpanMac::m_sentPktTrace),
                     "ns3::LrWpanMac::SentTracedCallback")
  ;
  return tid;
}

LrWpanMac::LrWpanMac ()
:m_periodicUpdateTimer (Timer::CANCEL_ON_DESTROY),
 m_routingTable (),
 m_printRoutingTableTimer(Timer::CANCEL_ON_DESTROY)
{

  // First set the state to a known value, call ChangeMacState to fire trace source.
  m_lrWpanMacState = MAC_IDLE;
  ChangeMacState (MAC_IDLE);

  m_macRxOnWhenIdle = true;
  m_macPanId = 0;
  m_associationStatus = ASSOCIATED;
  m_selfExt = Mac64Address::Allocate ();
  m_macPromiscuousMode = false;
  m_macMaxFrameRetries = 3;
  m_retransmission = 0;
  m_numCsmacaRetry = 0;
  m_txPkt = 0;
  m_isSink = false;
  Ptr<UniformRandomVariable> uniformVar = CreateObject<UniformRandomVariable> ();
  uniformVar->SetAttribute ("Min", DoubleValue (0.0));
  uniformVar->SetAttribute ("Max", DoubleValue (255.0));
  m_macDsn = SequenceNumber8 (uniformVar->GetValue ());
  m_shortAddress = Mac16Address ("00:00");
  //AM: modified at 7/11
  //SetMac16();
  m_depth = 0;
  m_msn = 0xf0;
  m_lqt = 0xffff;
  m_pqm = 0;
  m_tcieInterval = 0;
  m_arrivalRate = Seconds(0);
  m_maxQueueSize = 5;
  m_delayCountPacket = 0;
  m_avgDelay = 0;
  m_totalPacketRxByMesh = 0;
  m_totalPacketSentByNode = 0;
  m_totalPacketDroppedByNode = 0;
  m_arrivalRateComplement = 0;
  m_internalLoad = 0;
  m_totalPacketDroppedEverySecond = 0;
  m_queueSize = 0;
}

LrWpanMac::~LrWpanMac ()
{
}

void
LrWpanMac::DoInitialize ()
{
  if (m_macRxOnWhenIdle)
    {
      m_phy->PlmeSetTRXStateRequest (IEEE_802_15_4_PHY_RX_ON);
    }
  else
    {
      m_phy->PlmeSetTRXStateRequest (IEEE_802_15_4_PHY_TRX_OFF);
    }

  Object::DoInitialize ();
}

void
LrWpanMac::DoDispose ()
{
  if (m_csmaCa != 0)
    {
      m_csmaCa->Dispose ();
      m_csmaCa = 0;
    }
  m_txPkt = 0;
  for (uint32_t i = 0; i < m_txQueue.size (); i++)
    {
      m_txQueue[i]->txQPkt = 0;
      delete m_txQueue[i];
    }
  m_txQueue.clear ();
  m_phy = 0;
  m_mcpsDataIndicationCallback = MakeNullCallback< void, McpsDataIndicationParams, Ptr<Packet> > ();
  m_mcpsDataConfirmCallback = MakeNullCallback< void, McpsDataConfirmParams > ();

  Object::DoDispose ();
}

bool
LrWpanMac::GetRxOnWhenIdle ()
{
  return m_macRxOnWhenIdle;
}

void
LrWpanMac::SetRxOnWhenIdle (bool rxOnWhenIdle)
{
  NS_LOG_FUNCTION (this << rxOnWhenIdle);
  m_macRxOnWhenIdle = rxOnWhenIdle;

  if (m_lrWpanMacState == MAC_IDLE)
    {
      if (m_macRxOnWhenIdle)
        {
          m_phy->PlmeSetTRXStateRequest (IEEE_802_15_4_PHY_RX_ON);
        }
      else
        {
          m_phy->PlmeSetTRXStateRequest (IEEE_802_15_4_PHY_TRX_OFF);
        }
    }
}

void
LrWpanMac::SetShortAddress (Mac16Address address)
{
  NS_LOG_FUNCTION (this << address);
  m_shortAddress = address;
}

void
LrWpanMac::SetExtendedAddress (Mac64Address address)
{
  NS_LOG_FUNCTION (this << address);
  m_selfExt = address;
}


Mac16Address
LrWpanMac::GetShortAddress () const
{
  NS_LOG_FUNCTION (this);
  return m_shortAddress;
}

Mac64Address
LrWpanMac::GetExtendedAddress () const
{
  //NS_LOG_FUNCTION (this);
  return m_selfExt;
}
void
LrWpanMac::McpsDataRequest (McpsDataRequestParams params, Ptr<Packet> p)
{
  NS_LOG_FUNCTION (this << p);

  McpsDataConfirmParams confirmParams;
  confirmParams.m_msduHandle = params.m_msduHandle;
  L2R_Header l2rH;
  p->PeekHeader(l2rH);
  // TODO: We need a drop trace for the case that the packet is too large or the request parameters are maleformed.
  //       The current tx drop trace is not suitable, because packets dropped using this trace carry the mac header
  //       and footer, while packets being dropped here do not have them.

  LrWpanMacHeader macHdr (LrWpanMacHeader::LRWPAN_MAC_DATA, m_macDsn.GetValue ());
  m_macDsn++;

  if (p->GetSize () > LrWpanPhy::aMaxPhyPacketSize - aMinMPDUOverhead)
    {
      // Note, this is just testing maximum theoretical frame size per the spec
      // The frame could still be too large once headers are put on
      // in which case the phy will reject it instead
      NS_LOG_ERROR (this << " packet too big: " << p->GetSize ());
      confirmParams.m_status = IEEE_802_15_4_FRAME_TOO_LONG;
      if (!m_mcpsDataConfirmCallback.IsNull ())
        {
          m_mcpsDataConfirmCallback (confirmParams);
        }
      return;
    }

  if ((params.m_srcAddrMode == NO_PANID_ADDR)
      && (params.m_dstAddrMode == NO_PANID_ADDR))
    {
      NS_LOG_ERROR (this << " Can not send packet with no Address field" );
      confirmParams.m_status = IEEE_802_15_4_INVALID_ADDRESS;
      if (!m_mcpsDataConfirmCallback.IsNull ())
        {
          m_mcpsDataConfirmCallback (confirmParams);
        }
      return;
    }

    //AM: modified on 12/12
    if(l2rH.GetMsgType() == DataHeader && m_queueSize >= m_maxQueueSize)
    {
      NS_LOG_LOGIC(this << " can't send packet queue is full: ");
      ++m_totalPacketDroppedByNode;
      return;
    }
  switch (params.m_srcAddrMode)
    {
    case NO_PANID_ADDR:
      macHdr.SetSrcAddrMode (params.m_srcAddrMode);
      macHdr.SetNoPanIdComp ();
      break;
    case ADDR_MODE_RESERVED:
      NS_ABORT_MSG ("Can not set source address type to ADDR_MODE_RESERVED. Aborting.");
      break;
    case SHORT_ADDR:
      macHdr.SetSrcAddrMode (params.m_srcAddrMode);
      macHdr.SetSrcAddrFields (GetPanId (), GetShortAddress ());
      break;
    case EXT_ADDR:
      macHdr.SetSrcAddrMode (params.m_srcAddrMode);
      macHdr.SetSrcAddrFields (GetPanId (), GetExtendedAddress ());
      break;
    default:
      NS_LOG_ERROR (this << " Can not send packet with incorrect Source Address mode = " << params.m_srcAddrMode);
      confirmParams.m_status = IEEE_802_15_4_INVALID_ADDRESS;
      if (!m_mcpsDataConfirmCallback.IsNull ())
        {
          m_mcpsDataConfirmCallback (confirmParams);
        }
      return;
    }
  switch (params.m_dstAddrMode)
    {
    case NO_PANID_ADDR:
      macHdr.SetDstAddrMode (params.m_dstAddrMode);
      macHdr.SetNoPanIdComp ();
      break;
    case ADDR_MODE_RESERVED:
      NS_ABORT_MSG ("Can not set destination address type to ADDR_MODE_RESERVED. Aborting.");
      break;
    case SHORT_ADDR:
      macHdr.SetDstAddrMode (params.m_dstAddrMode);
      macHdr.SetDstAddrFields (params.m_dstPanId, params.m_dstAddr);
      break;
    case EXT_ADDR:
      macHdr.SetDstAddrMode (params.m_dstAddrMode);
      macHdr.SetDstAddrFields (params.m_dstPanId, params.m_dstExtAddr);
      break;
    default:
      NS_LOG_ERROR (this << " Can not send packet with incorrect Destination Address mode = " << params.m_dstAddrMode);
      confirmParams.m_status = IEEE_802_15_4_INVALID_ADDRESS;
      if (!m_mcpsDataConfirmCallback.IsNull ())
        {
          m_mcpsDataConfirmCallback (confirmParams);
        }
      return;
    }

  macHdr.SetSecDisable ();
  //extract the last 3 bits in TxOptions and map to macHdr
  int b0 = params.m_txOptions & TX_OPTION_ACK;
  int b1 = params.m_txOptions & TX_OPTION_GTS;
  int b2 = params.m_txOptions & TX_OPTION_INDIRECT;
  if (b0 == TX_OPTION_ACK)
    {
      // Set AckReq bit only if the destination is not the broadcast address.
      if (!(macHdr.GetDstAddrMode () == SHORT_ADDR && macHdr.GetShortDstAddr () == "ff:ff"))
        {
          macHdr.SetAckReq ();
        }
    }
  else if (b0 == 0)
    {
      macHdr.SetNoAckReq ();
    }
  else
    {
      confirmParams.m_status = IEEE_802_15_4_INVALID_PARAMETER;
      NS_LOG_ERROR (this << "Incorrect TxOptions bit 0 not 0/1");
      if (!m_mcpsDataConfirmCallback.IsNull ())
        {
          m_mcpsDataConfirmCallback (confirmParams);
        }
      return;
    }

  //if is Slotted CSMA means its beacon enabled
  if (m_csmaCa->IsSlottedCsmaCa ())
    {
      if (b1 == TX_OPTION_GTS)
        {
          //TODO:GTS Transmission
        }
      else if (b1 == 0)
        {
          //TODO:CAP Transmission
        }
      else
        {
          NS_LOG_ERROR (this << "Incorrect TxOptions bit 1 not 0/1");
          confirmParams.m_status = IEEE_802_15_4_INVALID_PARAMETER;
          if (!m_mcpsDataConfirmCallback.IsNull ())
            {
              m_mcpsDataConfirmCallback (confirmParams);
            }
          return;
        }
    }
  else
    {
      if (b1 != 0)
        {
          NS_LOG_ERROR (this << "for non-beacon-enables PAN, bit 1 should always be set to 0");
          confirmParams.m_status = IEEE_802_15_4_INVALID_PARAMETER;
          if (!m_mcpsDataConfirmCallback.IsNull ())
            {
              m_mcpsDataConfirmCallback (confirmParams);
            }
          return;
        }
    }

  if (b2 == TX_OPTION_INDIRECT)
    {
      //TODO :indirect tx
    }
  else if (b2 == 0)
    {
      //TODO :direct tx
    }
  else
    {
      NS_LOG_ERROR (this << "Incorrect TxOptions bit 2 not 0/1");
      confirmParams.m_status = IEEE_802_15_4_INVALID_PARAMETER;
      if (!m_mcpsDataConfirmCallback.IsNull ())
        {
          m_mcpsDataConfirmCallback (confirmParams);
        }
      return;
    }

  p->AddHeader (macHdr);

  LrWpanMacTrailer macTrailer;
  // Calculate FCS if the global attribute ChecksumEnable is set.
  if (Node::ChecksumEnabled ())
    {
      macTrailer.EnableFcs (true);
      macTrailer.SetFcs (p);
    }
  p->AddTrailer (macTrailer);

  m_macTxEnqueueTrace (p);

  TxQueueElement *txQElement = new TxQueueElement;
  txQElement->txQMsduHandle = params.m_msduHandle;
  txQElement->txQPkt = p;
  if(l2rH.GetMsgType() == DataHeader)
    ++m_queueSize;
  m_txQueue.push_back (txQElement);
  CheckQueue ();
}

void
LrWpanMac::CheckQueue ()
{
  NS_LOG_FUNCTION (this);

  // Pull a packet from the queue and start sending, if we are not already sending.
  if (m_lrWpanMacState == MAC_IDLE && !m_txQueue.empty () && m_txPkt == 0 && !m_setMacState.IsRunning ())
    {
      TxQueueElement *txQElement = m_txQueue.front ();
      m_txPkt = txQElement->txQPkt;
      //AM: modified on 4/12
      L2R_Header l2rHeader;
      m_txPkt->PeekHeader(l2rHeader);
      if(!m_isSink && l2rHeader.GetMsgType() == DataHeader)
      {
        //Time now = Simulator::Now ();
        //m_delayForEachPacket.insert(std::make_pair (m_txPkt->GetUid(), now));
        std::map<uint64_t, Time>::const_iterator i = m_delayForEachPacket.find (m_txPkt->GetUid());
        if(i != m_delayForEachPacket.end())
        {
          m_delayForEachPacket.erase(i);
          //m_avgDelay = m_avgDelay + (now.GetSeconds () - i->second.GetSeconds ());
          //++m_delayCountPacket;
        }
      }
      //end
      m_setMacState = Simulator::ScheduleNow (&LrWpanMac::SetLrWpanMacState, this, MAC_CSMA);
    }
}

void
LrWpanMac::SetCsmaCa (Ptr<LrWpanCsmaCa> csmaCa)
{
  m_csmaCa = csmaCa;
}

void
LrWpanMac::SetPhy (Ptr<LrWpanPhy> phy)
{
  m_phy = phy;
}

Ptr<LrWpanPhy>
LrWpanMac::GetPhy (void)
{
  return m_phy;
}

void
LrWpanMac::SetMcpsDataIndicationCallback (McpsDataIndicationCallback c)
{
  m_mcpsDataIndicationCallback = c;
}
//AM: modified on 30/12
void
LrWpanMac::SetL2rReceiveUpdateCallback (L2rReceiveUpdateCallback c)
{
  m_l2rReceiveUpdateCallback = c;
}
void LrWpanMac::SetMeshRootRxMsgUpdateCallback (meshRootRxMsgCallback c)
{
  m_meshRxMsgCallback = c;
}
void
LrWpanMac::SetMcpsDataConfirmCallback (McpsDataConfirmCallback c)
{
  m_mcpsDataConfirmCallback = c;
}

void
LrWpanMac::PdDataIndication (uint32_t psduLength, Ptr<Packet> p, uint8_t lqi)
{
  NS_ASSERT (m_lrWpanMacState == MAC_IDLE || m_lrWpanMacState == MAC_ACK_PENDING || m_lrWpanMacState == MAC_CSMA);

  NS_LOG_FUNCTION (this << psduLength << p << (uint16_t)lqi);

  bool acceptFrame;

  // from sec 7.5.6.2 Reception and rejection, Std802.15.4-2006
  // level 1 filtering, test FCS field and reject if frame fails
  // level 2 filtering if promiscuous mode pass frame to higher layer otherwise perform level 3 filtering
  // level 3 filtering accept frame
  // if Frame type and version is not reserved, and
  // if there is a dstPanId then dstPanId=m_macPanId or broadcastPanI, and
  // if there is a shortDstAddr then shortDstAddr =shortMacAddr or broadcastAddr, and
  // if beacon frame then srcPanId = m_macPanId
  // if only srcAddr field in Data or Command frame,accept frame if srcPanId=m_macPanId

  Ptr<Packet> originalPkt = p->Copy (); // because we will strip headers
  L2R_Header l2rHeader;
  p->PeekHeader(l2rHeader);
  m_promiscSnifferTrace (originalPkt);

  m_macPromiscRxTrace (originalPkt);
  // XXX no rejection tracing (to macRxDropTrace) being performed below

  LrWpanMacTrailer receivedMacTrailer;
  p->RemoveTrailer (receivedMacTrailer);
  if (Node::ChecksumEnabled ())
    {
      receivedMacTrailer.EnableFcs (true);
    }

  // level 1 filtering
  if (!receivedMacTrailer.CheckFcs (p))
    {
      m_macRxDropTrace (originalPkt);
    }
  else
    {
      LrWpanMacHeader receivedMacHdr;
      p->RemoveHeader (receivedMacHdr);

      McpsDataIndicationParams params;
      params.m_dsn = receivedMacHdr.GetSeqNum ();
      params.m_mpduLinkQuality = lqi;
      params.m_srcPanId = receivedMacHdr.GetSrcPanId ();
      params.m_srcAddrMode = receivedMacHdr.GetSrcAddrMode ();
      switch (params.m_srcAddrMode)
        {
        case SHORT_ADDR:
          params.m_srcAddr = receivedMacHdr.GetShortSrcAddr ();
          NS_LOG_DEBUG ("Packet from " << params.m_srcAddr);
          break;
        case EXT_ADDR:
          params.m_srcExtAddr = receivedMacHdr.GetExtSrcAddr ();
          NS_LOG_DEBUG ("Packet from " << params.m_srcExtAddr);
          break;
        default:
          break;
        }
      params.m_dstPanId = receivedMacHdr.GetDstPanId ();
      params.m_dstAddrMode = receivedMacHdr.GetDstAddrMode ();
      switch (params.m_dstAddrMode)
        {
        case SHORT_ADDR:
          params.m_dstAddr = receivedMacHdr.GetShortDstAddr ();
          NS_LOG_DEBUG ("Packet to " << params.m_dstAddr);
          break;
        case EXT_ADDR:
          params.m_dstExtAddr = receivedMacHdr.GetExtDstAddr ();
          NS_LOG_DEBUG ("Packet to " << params.m_dstExtAddr);
          break;
        default:
          break;
        }

      if (m_macPromiscuousMode)
        {
          //level 2 filtering
          if (!m_mcpsDataIndicationCallback.IsNull ())
            {
              NS_LOG_DEBUG ("promiscuous mode, forwarding up");
              m_mcpsDataIndicationCallback (params, p);
            }
          else
            {
              NS_LOG_ERROR (this << " Data Indication Callback not initialised");
            }
        }
      else
        {
          //level 3 frame filtering
          acceptFrame = (receivedMacHdr.GetType () != LrWpanMacHeader::LRWPAN_MAC_RESERVED);

          if (acceptFrame)
            {
              acceptFrame = (receivedMacHdr.GetFrameVer () <= 1);
            }

          if (acceptFrame
              && (receivedMacHdr.GetDstAddrMode () > 1))
            {
              acceptFrame = receivedMacHdr.GetDstPanId () == m_macPanId
                || receivedMacHdr.GetDstPanId () == 0xffff;
            }

          if (acceptFrame
              && (receivedMacHdr.GetDstAddrMode () == 2))
            {
              acceptFrame = receivedMacHdr.GetShortDstAddr () == m_shortAddress
                || receivedMacHdr.GetShortDstAddr () == Mac16Address ("ff:ff");        // check for broadcast addrs
            }

          if (acceptFrame
              && (receivedMacHdr.GetDstAddrMode () == 3))
            {
              acceptFrame = (receivedMacHdr.GetExtDstAddr () == m_selfExt);
            }

          if (acceptFrame
              && (receivedMacHdr.GetType () == LrWpanMacHeader::LRWPAN_MAC_BEACON))
            {
              if (m_macPanId == 0xffff)
                {
                  // TODO: Accept only if the frame version field is valid
                  acceptFrame = true;
                }
              else
                {
                  acceptFrame = receivedMacHdr.GetSrcPanId () == m_macPanId;
                }
            }

          if (acceptFrame
              && ((receivedMacHdr.GetType () == LrWpanMacHeader::LRWPAN_MAC_DATA)
                  || (receivedMacHdr.GetType () == LrWpanMacHeader::LRWPAN_MAC_COMMAND))
              && (receivedMacHdr.GetSrcAddrMode () > 1))
            {
              acceptFrame = receivedMacHdr.GetSrcPanId () == m_macPanId; // \todo need to check if PAN coord
            }
                        //Added new filtering layer check queue
          if (acceptFrame && (m_isSink || (l2rHeader.GetMsgType() != DataHeader) ||(m_queueSize < (m_maxQueueSize)))) //AM: Modified at 3/12
            {
             /* if(m_txQueue.size () >= m_maxQueueSize * 0.5)
              {
                //SendNlmMsg();
              }*/
              /*std::cout <<(Simulator::Now ()).GetSeconds () <<"A packet Rx By Mac: " << params.m_dstAddr <<" From:" << params.m_srcAddr<< " Queue Size = " << m_txQueue.size () 
                        <<"LQI " << (uint16_t)lqi <<std::endl;*/
              m_macRxTrace (originalPkt);
              // \todo: What should we do if we receive a frame while waiting for an ACK?
              //        Especially if this frame has the ACK request bit set, should we reply with an ACK, possibly missing the pending ACK?

              // If the received frame is a frame with the ACK request bit set, we immediately send back an ACK.
              // If we are currently waiting for a pending ACK, we assume the ACK was lost and trigger a retransmission after sending the ACK.
              if ((receivedMacHdr.IsData () || receivedMacHdr.IsCommand ()) && receivedMacHdr.IsAckReq ()
                  && !(receivedMacHdr.GetDstAddrMode () == SHORT_ADDR && receivedMacHdr.GetShortDstAddr () == "ff:ff"))
                {
                  // If this is a data or mac command frame, which is not a broadcast,
                  // with ack req set, generate and send an ack frame.
                  // If there is a CSMA medium access in progress we cancel the medium access
                  // for sending the ACK frame. A new transmission attempt will be started
                  // after the ACK was send.
                  if (m_lrWpanMacState == MAC_ACK_PENDING)
                    {
                      m_ackWaitTimeout.Cancel ();
                      PrepareRetransmission ();
                    }
                  else if (m_lrWpanMacState == MAC_CSMA)
                    {
                      // \todo: If we receive a packet while doing CSMA/CA, should  we drop the packet because of channel busy,
                      //        or should we restart CSMA/CA for the packet after sending the ACK?
                      // Currently we simply restart CSMA/CA after sending the ACK.
                      m_csmaCa->Cancel ();
                    }
                  // Cancel any pending MAC state change, ACKs have higher priority.
                  m_setMacState.Cancel ();
                  ChangeMacState (MAC_IDLE);
                  m_setMacState = Simulator::ScheduleNow (&LrWpanMac::SendAck, this, receivedMacHdr.GetSeqNum ());
                }

              if (receivedMacHdr.IsData () && !m_mcpsDataIndicationCallback.IsNull ())
                {
                  // If it is a data frame, push it up the stack.
                  NS_LOG_DEBUG ("PdDataIndication():  Packet is for me; forwarding up");
                  //AM: modified on 4/11 5:50 AM
                  RecieveL2RPacket(params,p);
                  
                  /*
                  std::cout << "location 2 recieved packet*******************" << std::endl;
                  L2R_Header L2R_RXHeader;

                  uint32_t hData = p->PeekHeader(L2R_RXHeader);
                  uint32_t depth = L2R_RXHeader.GetDepth(); 
                  std::cout << "Successfully read " << hData << " Bytes Header " << " Node Depth = " << depth << std::endl;*/
                  //L2R_RXHeader.Print();
                  m_mcpsDataIndicationCallback (params, p);
                }
              else if (receivedMacHdr.IsAcknowledgment () && m_txPkt && m_lrWpanMacState == MAC_ACK_PENDING)
                {
                  LrWpanMacHeader macHdr;
                  m_txPkt->PeekHeader (macHdr);
                  if (receivedMacHdr.GetSeqNum () == macHdr.GetSeqNum ())
                    {
                      m_macTxOkTrace (m_txPkt);
                      // If it is an ACK with the expected sequence number, finish the transmission
                      // and notify the upper layer.
                      m_ackWaitTimeout.Cancel ();
                      if (!m_mcpsDataConfirmCallback.IsNull ())
                        {
                          //AM: modified on 4/11 5:39 AM
                          std::cout << "location 1 recieved packet*******************" << std::endl;
                          TxQueueElement *txQElement = m_txQueue.front ();
                          McpsDataConfirmParams confirmParams;
                          confirmParams.m_msduHandle = txQElement->txQMsduHandle;
                          confirmParams.m_status = IEEE_802_15_4_SUCCESS;
                          m_mcpsDataConfirmCallback (confirmParams);
                        }
                      RemoveFirstTxQElement ();
                      m_setMacState.Cancel ();
                      m_setMacState = Simulator::ScheduleNow (&LrWpanMac::SetLrWpanMacState, this, MAC_IDLE);
                    }
                  else
                    {
                      // If it is an ACK with an unexpected sequence number, mark the current transmission as failed and start a retransmit. (cf 7.5.6.4.3)
                      m_ackWaitTimeout.Cancel ();
                      if (!PrepareRetransmission ())
                        {
                          m_setMacState.Cancel ();
                          m_setMacState = Simulator::ScheduleNow (&LrWpanMac::SetLrWpanMacState, this, MAC_IDLE);
                        }
                      else
                        {
                          m_setMacState.Cancel ();
                          m_setMacState = Simulator::ScheduleNow (&LrWpanMac::SetLrWpanMacState, this, MAC_CSMA);
                        }
                    }
                }
            }
          else
            {
              if(acceptFrame)
                { 
                  if((l2rHeader.GetMsgType() == DataHeader))
                  { 
                    std::cout <<(Simulator::Now ()).GetSeconds () <<"A packet dropped By Mac: " << params.m_dstAddr <<" From:" << params.m_srcAddr<< " Queue Size = " << m_queueSize 
                            <<" exceeds the limit: " << m_maxQueueSize << std::endl;
                    ++m_totalPacketDroppedByNode;
                  }
                }
              m_macRxDropTrace (originalPkt);
              
            }
        }
    }
}

void
LrWpanMac::SendAck (uint8_t seqno)
{
  NS_LOG_FUNCTION (this << static_cast<uint32_t> (seqno));

  NS_ASSERT (m_lrWpanMacState == MAC_IDLE);

  // Generate a corresponding ACK Frame.
  LrWpanMacHeader macHdr (LrWpanMacHeader::LRWPAN_MAC_ACKNOWLEDGMENT, seqno);
  LrWpanMacTrailer macTrailer;
  Ptr<Packet> ackPacket = Create<Packet> (0);
  ackPacket->AddHeader (macHdr);
  // Calculate FCS if the global attribute ChecksumEnable is set.
  if (Node::ChecksumEnabled ())
    {
      macTrailer.EnableFcs (true);
      macTrailer.SetFcs (ackPacket);
    }
  ackPacket->AddTrailer (macTrailer);

  // Enqueue the ACK packet for further processing
  // when the transmitter is activated.
  m_txPkt = ackPacket;

  // Switch transceiver to TX mode. Proceed sending the Ack on confirm.
  ChangeMacState (MAC_SENDING);
  m_phy->PlmeSetTRXStateRequest (IEEE_802_15_4_PHY_TX_ON);
}

void
LrWpanMac::RemoveFirstTxQElement ()
{
  TxQueueElement *txQElement = m_txQueue.front ();
  Ptr<const Packet> p = txQElement->txQPkt;
  m_numCsmacaRetry += m_csmaCa->GetNB () + 1;

  Ptr<Packet> pkt = p->Copy ();
  LrWpanMacHeader hdr;
  pkt->RemoveHeader (hdr);
  if (hdr.GetShortDstAddr () != Mac16Address ("ff:ff"))
    {
      m_sentPktTrace (p, m_retransmission + 1, m_numCsmacaRetry);
    }

  txQElement->txQPkt = 0;
  delete txQElement;
  m_txQueue.pop_front ();
  m_txPkt = 0;
  m_retransmission = 0;
  m_numCsmacaRetry = 0;
  m_macTxDequeueTrace (p);
}

void
LrWpanMac::AckWaitTimeout (void)
{
  NS_LOG_FUNCTION (this);

  // TODO: If we are a PAN coordinator and this was an indirect transmission,
  //       we will not initiate a retransmission. Instead we wait for the data
  //       being extracted after a new data request command.
  if (!PrepareRetransmission ())
    {
      SetLrWpanMacState (MAC_IDLE);
    }
  else
    {
      SetLrWpanMacState (MAC_CSMA);
    }
}

bool
LrWpanMac::PrepareRetransmission (void)
{
  NS_LOG_FUNCTION (this);

  if (m_retransmission >= m_macMaxFrameRetries)
    {
      // Maximum number of retransmissions has been reached.
      // remove the copy of the packet that was just sent
      TxQueueElement *txQElement = m_txQueue.front ();
      m_macTxDropTrace (txQElement->txQPkt);
      if (!m_mcpsDataConfirmCallback.IsNull ())
        {
          McpsDataConfirmParams confirmParams;
          confirmParams.m_msduHandle = txQElement->txQMsduHandle;
          confirmParams.m_status = IEEE_802_15_4_NO_ACK;
          m_mcpsDataConfirmCallback (confirmParams);
        }
      RemoveFirstTxQElement ();
      return false;
    }
  else
    {
      m_retransmission++;
      //AM:Modified 
      ++m_internalLoad;
      m_numCsmacaRetry += m_csmaCa->GetNB () + 1;
      // Start next CCA process for this packet.
      return true;
    }
}

void
LrWpanMac::PdDataConfirm (LrWpanPhyEnumeration status)
{
  NS_ASSERT (m_lrWpanMacState == MAC_SENDING);

  NS_LOG_FUNCTION (this << status << m_txQueue.size ());

  LrWpanMacHeader macHdr;
  m_txPkt->PeekHeader (macHdr);
  L2R_Header l2rH;
  m_txPkt->PeekHeader (l2rH);
  if (status == IEEE_802_15_4_PHY_SUCCESS)
    {
      if (!macHdr.IsAcknowledgment ())
        {
          // We have just send a regular data packet, check if we have to wait
          // for an ACK.
          if (macHdr.IsAckReq ())
            {
              // wait for the ack or the next retransmission timeout
              // start retransmission timer
              Time waitTime = MicroSeconds (GetMacAckWaitDuration () * 1000 * 1000 / m_phy->GetDataOrSymbolRate (false));
              NS_ASSERT (m_ackWaitTimeout.IsExpired ());
              m_ackWaitTimeout = Simulator::Schedule (waitTime, &LrWpanMac::AckWaitTimeout, this);
              m_setMacState.Cancel ();
              m_setMacState = Simulator::ScheduleNow (&LrWpanMac::SetLrWpanMacState, this, MAC_ACK_PENDING);
              if(l2rH.GetMsgType () == DataHeader)
                --m_queueSize;
              return;
            }
          else
            {
              m_macTxOkTrace (m_txPkt);
              // remove the copy of the packet that was just sent
              if (!m_mcpsDataConfirmCallback.IsNull ())
                {
                  McpsDataConfirmParams confirmParams;
                  NS_ASSERT_MSG (m_txQueue.size () > 0, "TxQsize = 0");
                  TxQueueElement *txQElement = m_txQueue.front ();
                  confirmParams.m_msduHandle = txQElement->txQMsduHandle;
                  confirmParams.m_status = IEEE_802_15_4_SUCCESS;
                  m_mcpsDataConfirmCallback (confirmParams);
                }
                if(l2rH.GetMsgType () == DataHeader)
                  --m_queueSize;
              RemoveFirstTxQElement ();
            }
        }
      else
        {
          // We have send an ACK. Clear the packet buffer.
          m_txPkt = 0;
        }
    }
  else if (status == IEEE_802_15_4_PHY_UNSPECIFIED)
    {

      if (!macHdr.IsAcknowledgment ())
        {
          NS_ASSERT_MSG (m_txQueue.size () > 0, "TxQsize = 0");
          TxQueueElement *txQElement = m_txQueue.front ();
          m_macTxDropTrace (txQElement->txQPkt);
          if (!m_mcpsDataConfirmCallback.IsNull ())
            {
              McpsDataConfirmParams confirmParams;
              confirmParams.m_msduHandle = txQElement->txQMsduHandle;
              confirmParams.m_status = IEEE_802_15_4_FRAME_TOO_LONG;
              m_mcpsDataConfirmCallback (confirmParams);
            }
          RemoveFirstTxQElement ();
        if(l2rH.GetMsgType () == DataHeader)
          --m_queueSize;
        }
      else
        {
          NS_LOG_ERROR ("Unable to send ACK");
        }
    }
  else
    {
      // Something went really wrong. The PHY is not in the correct state for
      // data transmission.
      NS_FATAL_ERROR ("Transmission attempt failed with PHY status " << status);
    }

  m_setMacState.Cancel ();
  m_setMacState = Simulator::ScheduleNow (&LrWpanMac::SetLrWpanMacState, this, MAC_IDLE);
}

void
LrWpanMac::PlmeCcaConfirm (LrWpanPhyEnumeration status)
{
  //NS_LOG_FUNCTION (this << status);
  // Direct this call through the csmaCa object
  m_csmaCa->PlmeCcaConfirm (status);
}

void
LrWpanMac::PlmeEdConfirm (LrWpanPhyEnumeration status, uint8_t energyLevel)
{
  //NS_LOG_FUNCTION (this << status << energyLevel);

}

void
LrWpanMac::PlmeGetAttributeConfirm (LrWpanPhyEnumeration status,
                                    LrWpanPibAttributeIdentifier id,
                                    LrWpanPhyPibAttributes* attribute)
{
  NS_LOG_FUNCTION (this << status << id << attribute);
}

void
LrWpanMac::PlmeSetTRXStateConfirm (LrWpanPhyEnumeration status)
{
  NS_LOG_FUNCTION (this << status);

  if (m_lrWpanMacState == MAC_SENDING && (status == IEEE_802_15_4_PHY_TX_ON || status == IEEE_802_15_4_PHY_SUCCESS))
    {
      NS_ASSERT (m_txPkt);

      // Start sending if we are in state SENDING and the PHY transmitter was enabled.
      m_promiscSnifferTrace (m_txPkt);
      m_snifferTrace (m_txPkt);
      m_macTxTrace (m_txPkt);
      m_phy->PdDataRequest (m_txPkt->GetSize (), m_txPkt);
    }
  else if (m_lrWpanMacState == MAC_CSMA && (status == IEEE_802_15_4_PHY_RX_ON || status == IEEE_802_15_4_PHY_SUCCESS))
    {
      // Start the CSMA algorithm as soon as the receiver is enabled.
      m_csmaCa->Start ();
    }
  else if (m_lrWpanMacState == MAC_IDLE)
    {
      NS_ASSERT (status == IEEE_802_15_4_PHY_RX_ON || status == IEEE_802_15_4_PHY_SUCCESS || status == IEEE_802_15_4_PHY_TRX_OFF);
      // Do nothing special when going idle.
    }
  else if (m_lrWpanMacState == MAC_ACK_PENDING)
    {
      NS_ASSERT (status == IEEE_802_15_4_PHY_RX_ON || status == IEEE_802_15_4_PHY_SUCCESS);
    }
  else
    {
      // TODO: What to do when we receive an error?
      // If we want to transmit a packet, but switching the transceiver on results
      // in an error, we have to recover somehow (and start sending again).
      NS_FATAL_ERROR ("Error changing transceiver state");
    }
}

void
LrWpanMac::PlmeSetAttributeConfirm (LrWpanPhyEnumeration status,
                                    LrWpanPibAttributeIdentifier id)
{
  NS_LOG_FUNCTION (this << status << id);
}

void
LrWpanMac::SetLrWpanMacState (LrWpanMacState macState)
{
  NS_LOG_FUNCTION (this << "mac state = " << macState);

  McpsDataConfirmParams confirmParams;

  if (macState == MAC_IDLE)
    {
      ChangeMacState (MAC_IDLE);

      if (m_macRxOnWhenIdle)
        {
          m_phy->PlmeSetTRXStateRequest (IEEE_802_15_4_PHY_RX_ON);
        }
      else
        {
          m_phy->PlmeSetTRXStateRequest (IEEE_802_15_4_PHY_TRX_OFF);
        }

      CheckQueue ();
    }
  else if (macState == MAC_ACK_PENDING)
    {
      ChangeMacState (MAC_ACK_PENDING);
      m_phy->PlmeSetTRXStateRequest (IEEE_802_15_4_PHY_RX_ON);
    }
  else if (macState == MAC_CSMA)
    {
      NS_ASSERT (m_lrWpanMacState == MAC_IDLE || m_lrWpanMacState == MAC_ACK_PENDING);

      ChangeMacState (MAC_CSMA);
      m_phy->PlmeSetTRXStateRequest (IEEE_802_15_4_PHY_RX_ON);
    }
  else if (m_lrWpanMacState == MAC_CSMA && macState == CHANNEL_IDLE)
    {
      // Channel is idle, set transmitter to TX_ON
      ChangeMacState (MAC_SENDING);
      m_phy->PlmeSetTRXStateRequest (IEEE_802_15_4_PHY_TX_ON);
    }
  else if (m_lrWpanMacState == MAC_CSMA && macState == CHANNEL_ACCESS_FAILURE)
    {
      NS_ASSERT (m_txPkt);

      // cannot find a clear channel, drop the current packet.
      NS_LOG_DEBUG ( this << " cannot find clear channel");
      confirmParams.m_msduHandle = m_txQueue.front ()->txQMsduHandle;
      confirmParams.m_status = IEEE_802_15_4_CHANNEL_ACCESS_FAILURE;
      m_macTxDropTrace (m_txPkt);
      if (!m_mcpsDataConfirmCallback.IsNull ())
        {
          m_mcpsDataConfirmCallback (confirmParams);
        }
      // remove the copy of the packet that was just sent
      RemoveFirstTxQElement ();

      ChangeMacState (MAC_IDLE);
    }
}

LrWpanAssociationStatus
LrWpanMac::GetAssociationStatus (void) const
{
  return m_associationStatus;
}

void
LrWpanMac::SetAssociationStatus (LrWpanAssociationStatus status)
{
  m_associationStatus = status;
}

uint16_t
LrWpanMac::GetPanId (void) const
{
  return m_macPanId;
}

void
LrWpanMac::SetPanId (uint16_t panId)
{
  m_macPanId = panId;
}

void
LrWpanMac::ChangeMacState (LrWpanMacState newState)
{
  NS_LOG_LOGIC (this << " change lrwpan mac state from "
                     << m_lrWpanMacState << " to "
                     << newState);
  m_macStateLogger (m_lrWpanMacState, newState);
  m_lrWpanMacState = newState;
}

uint64_t
LrWpanMac::GetMacAckWaitDuration (void) const
{
  return m_csmaCa->GetUnitBackoffPeriod () + m_phy->aTurnaroundTime + m_phy->GetPhySHRDuration ()
         + ceil (6 * m_phy->GetPhySymbolsPerOctet ());
}

uint8_t
LrWpanMac::GetMacMaxFrameRetries (void) const
{
  return m_macMaxFrameRetries;
}

void
LrWpanMac::SetMacMaxFrameRetries (uint8_t retries)
{
  m_macMaxFrameRetries = retries;
}
//AM: modified at 7/11
/*void
LrWpanMac::AddedL2RoutingProtocol(Ptr<L2R_RoutingProtocol> routingProtocol)
{
  m_routingProtocol = routingProtocol;
}*/

//AM: modified at 4/11 6:03

L2R_Header::L2R_Header ()
{
  m_depth = 0;
  m_MSN = 0;
  m_PQM = 0;
  m_meshRootAddress = Mac16Address("00:00");
  m_TCIEInterval = 0xff;
  m_msgType = DataHeader;
  m_LQT = 0;
  m_queueSize = 0;
  m_arrivalRate = 0;
  m_avgDelay =0;
}
L2R_Header::~L2R_Header ()
{
}

TypeId
L2R_Header::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::L2R_Header")
    .SetParent<Header> ()
    .AddConstructor<L2R_Header> ()
  ;
  return tid;
}
TypeId
L2R_Header::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

void
L2R_Header::Print (std::ostream &os) const
{
  // This method is invoked by the packet printing
  // routines to print the content of my header.
  //os << "data=" << m_data << std::endl;
  //ToDo print header debends on the msg type

  os << "DestinationRootMacAddress: " << m_meshRootAddress
     << " depth: " << m_depth
     << " PQM: " << m_PQM
     << " Arrival Rate: " << m_arrivalRate
     << " NQueue Size: " << m_queueSize
     << " Delay: " << m_avgDelay;
}
uint32_t
L2R_Header::GetSerializedSize (void) const
{
  /*
    * Each L2R header will have
    * Root 16 MAC Address      : 2 octet
    * LQT                      : 0/1 Octet
    * TC IE Interval           : 0/1 Octet
    * MSN                      : 0/2 octet
    * Depth                    : 0/2 octet
    * PQM                      : 0/2 octet
    * MSG Type                 : 0/1 octet
  */
  uint32_t size = 3;
  switch (m_msgType)
  {
  case TC_IE:
    size += 20;
    break;
  case L2R_D_IE:
    break;
  case DataHeader:
    size += 14;
    break;
  case NLM_IE:
    size +=10;
    break;
  }
  return size;
}
void
L2R_Header::Serialize (Buffer::Iterator start) const
{
  // we can serialize two bytes at the start of the buffer.
  // we write them in network byte order.
  start.WriteU8(m_msgType);
  switch (m_msgType)
  {
  case TC_IE:
    WriteTo(start,m_meshRootAddress);
    start.WriteHtonU16 (m_PQM);
    start.WriteHtonU16(m_LQT);
    start.WriteU8 (m_TCIEInterval);
    start.WriteHtonU16 (m_MSN);
    start.WriteHtonU16 (m_depth);
    start.WriteHtonU16 (m_queueSize);
    start.WriteHtonU32 (m_avgDelay);
    start.WriteHtonU32 (m_arrivalRate);
    break;
  case L2R_D_IE:
    WriteTo(start,m_meshRootAddress);
    break;
  case DataHeader:
    WriteTo(start,m_srcAddress);
    start.WriteHtonU16 (m_depth);
    start.WriteHtonU16 (m_PQM);
    start.WriteHtonU16 (m_queueSize);
    start.WriteHtonU32 (m_avgDelay);
    start.WriteHtonU32 (m_arrivalRate);
    break;
  case NLM_IE:
    start.WriteHtonU16 (m_queueSize);
    start.WriteHtonU32 (m_avgDelay);
    start.WriteHtonU32 (m_arrivalRate);
    break;
  }
}
uint32_t
L2R_Header::Deserialize (Buffer::Iterator start)
{
  // we can deserialize two bytes from the start of the buffer.
  // we read them in network byte order and store them
  // in host byte order.
  Buffer::Iterator i = start;
  m_msgType = i.ReadU8();
  switch (m_msgType)
  {
  case 0:
    ReadFrom (i, m_meshRootAddress);
    m_PQM = i.ReadNtohU16 ();
    m_LQT = i.ReadNtohU16();
    m_TCIEInterval = i.ReadU8 ();
    m_MSN = i.ReadNtohU16 ();
    m_depth = i.ReadNtohU16 ();
    m_queueSize = i.ReadNtohU16 ();
    m_avgDelay = i.ReadNtohU32 ();
    m_arrivalRate = i.ReadNtohU32 ();
    break;
  case 1:
    ReadFrom (i, m_meshRootAddress);
    break;
  case 2:
    ReadFrom (i, m_srcAddress);
    m_depth = i.ReadNtohU16 ();
    m_PQM = i.ReadNtohU16 ();
    m_queueSize = i.ReadNtohU16 ();
    m_avgDelay = i.ReadNtohU32 ();
    m_arrivalRate = i.ReadNtohU32 ();
    break;
  case 3:
    m_queueSize = i.ReadNtohU16 ();
    m_avgDelay = i.ReadNtohU32 ();
    m_arrivalRate = i.ReadNtohU32 ();
    break;
  }
  uint32_t dist = i.GetDistanceFrom (start);
  // we return the number of bytes effectively read.
  return dist;
}

uint16_t 
L2R_Header::GetDepth (void) const
{
  return m_depth;
}
Mac16Address 
L2R_Header::GetMeshRootAddress (void) const
{
  return m_meshRootAddress;
}
uint16_t 
L2R_Header::GetPQM(void) const
{
  return m_PQM;
}
uint16_t 
L2R_Header::GetMSN (void) const
{
  return m_MSN;
}
uint8_t L2R_Header::GetTCIEInterval (void) const
{
  return m_TCIEInterval;
}
void L2R_Header::SetDepth (uint16_t depth)
{
  m_depth = depth;
}
void L2R_Header::SetMeshRootAddress(Mac16Address meshRootAddress)
{
  m_meshRootAddress = meshRootAddress;
}
void L2R_Header::SetPQM (uint16_t pqm)
{
  m_PQM = pqm;
}
void L2R_Header::SetMSN (uint16_t msn)
{
  m_MSN = msn;
}
void L2R_Header::SetTCIEInterval (uint8_t tcieinterval)
{
  m_TCIEInterval = tcieinterval;
}
void L2R_Header::SetMsgType (enum L2R_MsgType msgType)
{
  m_msgType = msgType;
}
void L2R_Header::SetLQT(uint16_t lqt)
{
  m_LQT = lqt;
}
void
L2R_Header::SetArrivalRate(uint32_t arrivalRate)
{
  m_arrivalRate = arrivalRate;
}
void
L2R_Header::SetQueueSize(uint16_t q)
{
  m_queueSize = q;
}
void 
L2R_Header::SetDelay(uint32_t delay)
{
  m_avgDelay = delay;
}
void 
L2R_Header::SetSrcMacAddress(Mac16Address src)
{
  m_srcAddress = src;
}
enum L2R_MsgType
L2R_Header::GetMsgType (void) const
{
  switch (m_msgType)
    {
    case 0:
      return TC_IE;
      break;
    case 1:
      return L2R_D_IE;
      break;
    case 2:
      return DataHeader;
      break;
    default:
      return DataHeader;
    }
}
uint16_t L2R_Header::GetLQT(void) const
{
  return m_LQT;
}
uint32_t
L2R_Header::GetArrivalRate(void) const
{
  return m_arrivalRate;
}
uint32_t
L2R_Header::GetDelay(void) const
{
  return m_avgDelay;
}
uint16_t
L2R_Header::GetQueueSize(void) const
{
  return m_queueSize;
}
Mac16Address
L2R_Header::GetSrcAddress(void) const
{
  return m_srcAddress;
}

//AM: modified at 6/11 6:03
//Routing Protocol
//AM: modified at 8/11
L2R_RoutingTableEntry::L2R_RoutingTableEntry (uint16_t depth,
                                      uint16_t PQM,
                                      Time lifetime,
                                      Time tcieInterval,
                                      Mac16Address nextHop,
                                      bool areChanged)
  : m_depth (depth),
    m_pqm (PQM),
    m_lifeTime (lifetime),
    m_flag (VALID),
    m_tcieInterval (tcieInterval),
    m_entriesChanged (areChanged)
{
  m_nextHop = nextHop;
  m_l2rMissedTcIe = 0; 
  m_delayPar = 1;
  m_queuePar = 1;
  m_arrivalPar = 1;
}
L2R_RoutingTableEntry::~L2R_RoutingTableEntry ()
{
}
L2R_RoutingTable::L2R_RoutingTable ()
{
  m_l2rMaxMissedTcIe = 100; 
}
void
L2R_RoutingTableEntry::Print (Ptr<OutputStreamWrapper> stream) const
{
  
  *stream->GetStream () << std::resetiosflags(std::ios::adjustfield) << std::setiosflags (std::ios::fixed) << m_nextHop << "\t\t"
                        << std::setiosflags (std::ios::left)
                        << std::setw (20) << m_depth << "\t" << std::setw (20) << m_pqm << "\t"
                        << std::setprecision (3) << m_lifeTime.GetSeconds ()
                        << "s\t\t" <<std::setprecision (3)<< m_tcieInterval.GetSeconds () << "s\t\t\t"
                        << m_queuePar << "\t\t\t" << m_arrivalPar <<"\t\t\t" << m_delayPar <<"\n";
}
bool
L2R_RoutingTable::AddRoute (L2R_RoutingTableEntry & rt)
{
  //std::pair is a struct template that provides a way to store two heterogeneous objects as a single unit
  std::pair<std::map<Mac16Address, L2R_RoutingTableEntry>::iterator, bool> result = m_mac16AddressEntry.insert (std::make_pair (
                                                                                                            rt.GetNextHop (),rt));
  return result.second;
}
bool
L2R_RoutingTable::DeleteRoute (Mac16Address dst)
{
  if (m_mac16AddressEntry.erase (dst) != 0)
    {
      // NS_LOG_DEBUG("Route erased");
      return true;
    }
  return false;
}
bool
L2R_RoutingTable::LookupRoute (Mac16Address id,
                           L2R_RoutingTableEntry & rt)
{
  if (m_mac16AddressEntry.empty ())
    {
      return false;
    }
  std::map<Mac16Address, L2R_RoutingTableEntry>::const_iterator i = m_mac16AddressEntry.find (id);
  if (i == m_mac16AddressEntry.end ())
    {
      return false;
    }
  rt = i->second;
  return true;
}
bool
L2R_RoutingTable::LookupRoute (Mac16Address id,
                           L2R_RoutingTableEntry & rt,
                           bool forRouteInput)
{
  if (m_mac16AddressEntry.empty ())
    {
      return false;
    }
  std::map<Mac16Address, L2R_RoutingTableEntry>::const_iterator i = m_mac16AddressEntry.find (id);
  if (i == m_mac16AddressEntry.end ())
    {
      return false;
    }
  /*if (forRouteInput == true && id == i->second.GetInterface ().GetBroadcast ()) //ToDo
    {
      return false;
    }*/
  rt = i->second;
  return true;
}
bool
L2R_RoutingTable::Update (L2R_RoutingTableEntry & rt)
{
  std::map<Mac16Address, L2R_RoutingTableEntry>::iterator i = m_mac16AddressEntry.find (rt.GetNextHop ());
  if (i == m_mac16AddressEntry.end ())
    {
      return false;
    }
  i->second = rt;
  return true;
}
bool
L2R_RoutingTable::GetListOfDestinationWithNextHop (std::map<Mac16Address, L2R_RoutingTableEntry> & possibleRoutes,
                             const uint16_t myDepth)
{
  possibleRoutes.clear();
  for (std::map<Mac16Address, L2R_RoutingTableEntry>::const_iterator i = m_mac16AddressEntry.begin (); i
       != m_mac16AddressEntry.end (); ++i)
    {
      if (i->second.GetDepth () < myDepth)
        {
          possibleRoutes.insert (
            std::make_pair (i->first,i->second));
        }
    }
    if(possibleRoutes.empty() == true)
      return false;
    else
      return true;
}
void
L2R_RoutingTable::GetListOfAllRoutes (std::map<Mac16Address, L2R_RoutingTableEntry> & allRoutes)
{
  for (std::map<Mac16Address, L2R_RoutingTableEntry>::iterator i = m_mac16AddressEntry.begin (); i != m_mac16AddressEntry.end (); ++i)
    {
      if (i->second.GetFlag () == VALID)
        {
          allRoutes.insert (
            std::make_pair (i->first,i->second));
        }
    }
}
void
L2R_RoutingTable::Purge (std::map<Mac16Address, L2R_RoutingTableEntry> & removedAddresses)
{
  if (m_mac16AddressEntry.empty ())
    {
      return;
    }
  for (std::map<Mac16Address, L2R_RoutingTableEntry>::iterator i = m_mac16AddressEntry.begin (); i != m_mac16AddressEntry.end (); ) //ToDo
    {
      std::map<Mac16Address, L2R_RoutingTableEntry>::iterator itmp = i;
      if (i->second.GetLifeTime () > m_holddownTime && (i->second.GetDepth () > 0)) //holding time is TCIE interval and not sink
      {
        if(i->second.GetL2rMissedTcIe() >= m_l2rMaxMissedTcIe)
        {
          removedAddresses.insert (std::make_pair (i->first,i->second));
          ++i;
          m_mac16AddressEntry.erase (itmp);
        }
        else
        {
          i->second.IncL2rMissedTcIe();
        }
      }
      else
      {
        ++i;
      }
      
    }
  return;
}
void
L2R_RoutingTable::Print (Ptr<OutputStreamWrapper> stream) const
{
  *stream->GetStream () << "\nL2R Routing table\n" << "Destination\t\tdepth\t\tPQM\t\tLifeTime\t\tTCIEInterval\t\tNormalizedQueue\t\tTimeBtwArrivalRate\t\tAvgDelay\n";
  for (std::map<Mac16Address, L2R_RoutingTableEntry>::const_iterator i = m_mac16AddressEntry.begin (); i
       != m_mac16AddressEntry.end (); ++i)
    {
      i->second.Print (stream);
    }
  *stream->GetStream () << "\n";
}
uint32_t
L2R_RoutingTable::RoutingTableSize ()
{
  return m_mac16AddressEntry.size ();
}
bool
L2R_RoutingTable::AddMacEvent (Mac16Address address,
                            EventId id)
{
  std::pair<std::map<Mac16Address, EventId>::iterator, bool> result = m_macEvents.insert (std::make_pair (address,id));
  return result.second;
}
bool
L2R_RoutingTable::DeleteMacEvent (Mac16Address address)
{
  EventId event;
  std::map<Mac16Address, EventId>::const_iterator i = m_macEvents.find (address);
  if (m_macEvents.empty () || i == m_macEvents.end ())
    {
      return false;
    }
  event = i->second;
  if (event.IsRunning ())
    {
      return false;
    }
  if (event.IsExpired ())
    {
      event.Cancel ();
      m_macEvents.erase (address);
      return true;
    }
  else
    {
      m_macEvents.erase (address);
      return true;
    }
}
bool
L2R_RoutingTable::AnyRunningEvent (Mac16Address address)
{
  EventId event;
  std::map<Mac16Address, EventId>::const_iterator i = m_macEvents.find (address);
  if (m_macEvents.empty ())
    {
      return false;
    }
  if (i == m_macEvents.end ())
    {
      return false;
    }
  event = i->second;
  if (event.IsRunning ())
    {
      return true;
    }
  else
    {
      return false;
    }
}
EventId
L2R_RoutingTable::GetEventId (Mac16Address address)
{
  std::map <Mac16Address, EventId>::const_iterator i = m_macEvents.find (address);
  if (m_macEvents.empty () || i == m_macEvents.end ())
    {
      return EventId ();
    }
  else
    {
      return i->second;
    }
}
void
LrWpanMac::L2R_AssignL2RProtocolForSink(bool isSink, uint16_t lqt, uint8_t tcieInterval)
{
  m_isSink = isSink;
  m_msn = 0xf0;
  m_lqt = lqt;
  m_tcieInterval = tcieInterval;
  m_tcieIncr = 0;
}
void
LrWpanMac::L2R_SendPeriodicUpdate()
{
  std::map <Mac16Address, L2R_RoutingTableEntry> removedAddresses;
  //m_routingTable.Purge (removedAddresses);
  if (m_isSink)
  {
    NS_LOG_FUNCTION (Seconds(Simulator::Now ()) << "Sending TC-IE by Sink ");
    L2R_Header TC_IE_H;
    TC_IE_H.SetMeshRootAddress(m_rootAddress);
    TC_IE_H.SetMsgType(TC_IE);
    TC_IE_H.SetPQM(0);
    TC_IE_H.SetMSN(m_msn);
    TC_IE_H.SetLQT(m_lqt);
    TC_IE_H.SetTCIEInterval(m_tcieInterval);
    TC_IE_H.SetDepth(m_depth);
    TC_IE_H.SetQueueSize(0);
    TC_IE_H.SetDelay(0);
    TC_IE_H.SetArrivalRate(0);
    Ptr<Packet> p0 = Create<Packet> (); //Zero payload packet
    p0->AddHeader (TC_IE_H);
    McpsDataRequestParams params;
    params.m_dstPanId = this->GetPanId();
    params.m_srcAddrMode = SHORT_ADDR;
    params.m_dstAddrMode = SHORT_ADDR;
    params.m_dstAddr = "ff:ff";
    params.m_msduHandle = 0; //ToDo underStand the msduhandle from standard
    params.m_txOptions = TX_OPTION_NONE;
    Simulator::ScheduleNow (&LrWpanMac::McpsDataRequest,this,
                             params, p0);
    if(m_msn > 0xef && m_msn <= 0xff)
      m_msn = 0x00;
    else
      ++m_msn;
  m_periodicUpdateTimer.Schedule (Seconds(m_tcieInterval));                          

  }
  else
  {
    NS_LOG_FUNCTION (Seconds(Simulator::Now ())<<"Forwording TC-IE ");
    L2R_Header TC_IE_H;
    TC_IE_H.SetMeshRootAddress(m_rootAddress);
    TC_IE_H.SetMsgType(TC_IE);
    TC_IE_H.SetPQM(m_pqm);
    TC_IE_H.SetMSN(m_msn);
    TC_IE_H.SetLQT(m_lqt);
    TC_IE_H.SetTCIEInterval(m_tcieInterval);
    TC_IE_H.SetDepth(m_depth);
    TC_IE_H.SetQueueSize(GetQueueSize());
    TC_IE_H.SetDelay(GetAvgDelay());
    TC_IE_H.SetArrivalRate(GetArrivalRate());
    Ptr<Packet> p0 = Create<Packet> (); //Zero payload packet
    p0->AddHeader (TC_IE_H);
    McpsDataRequestParams params;
    params.m_dstPanId = this->GetPanId();
    params.m_srcAddrMode = SHORT_ADDR;
    params.m_dstAddrMode = SHORT_ADDR;
    params.m_dstAddr = "ff:ff";
    params.m_msduHandle = 0; //ToDo underStand the msduhandle from standard
    params.m_txOptions = TX_OPTION_NONE;
    Simulator::ScheduleNow (&LrWpanMac::McpsDataRequest,this,
                             params, p0);
  }
  for (std::map<Mac16Address, L2R_RoutingTableEntry>::const_iterator rmItr = removedAddresses.begin (); rmItr
           != removedAddresses.end (); ++rmItr)
  {
    NS_LOG_FUNCTION ("Update for removed record is: Destination: " << rmItr->second.GetNextHop()
                                                                << " depth:" << rmItr->second.GetDepth());
  }
}
void
LrWpanMac::L2R_SendTopologyDiscovery()
{
  if(!m_isSink)
  {
    NS_ABORT_MSG ("only Sink can start the topology");
    return;
  }
  else
  {
    NS_LOG_FUNCTION ("Sending Topology Discovery Msg ");
    L2R_Header L2R_DIE;
    m_rootAddress = m_shortAddress; //change root name
    L2R_DIE.SetMeshRootAddress(m_rootAddress);
    L2R_DIE.SetMsgType(L2R_D_IE);
    Ptr<Packet> p0 = Create<Packet> (); //Zero payload packet
    p0->AddHeader (L2R_DIE); //serialize is called here
    McpsDataRequestParams params;
    params.m_dstPanId = this->GetPanId();
    params.m_srcAddrMode = SHORT_ADDR;
    params.m_dstAddrMode = SHORT_ADDR;
    params.m_dstAddr = "ff:ff";
    params.m_msduHandle = 0; //ToDo underStand the msduhandle from standard
    params.m_txOptions = TX_OPTION_NONE;
    Simulator::ScheduleNow (&LrWpanMac::McpsDataRequest,this,
                             params, p0);
    L2R_Start();
  }
}
void 
LrWpanMac::L2R_Start()
{
  m_uniformRandomVariable = CreateObject<UniformRandomVariable> ();
  m_periodicUpdateTimer.SetFunction (&LrWpanMac::L2R_SendPeriodicUpdate,this);
  if(m_isSink)
    m_periodicUpdateTimer.Schedule (MicroSeconds(10000));//m_uniformRandomVariable->GetInteger (0,2000)));
}
void LrWpanMac::RecieveL2RPacket(McpsDataIndicationParams rxParams, Ptr<Packet> p)
{
  L2R_Header L2rRxMsg;
  Ptr<Packet> originalPkt = p->Copy (); // because we will strip headers
  p->RemoveHeader(L2rRxMsg);
  //McpsDataIndicationParams rxParams;
  Mac16Address sender = rxParams.m_srcAddr;
  Mac16Address receiver = rxParams.m_dstAddr;
  
  L2R_RoutingTableEntry tableEntry;
  EventId event;
  bool tableVerifier = m_routingTable.LookupRoute (sender,tableEntry);
  
  switch (L2rRxMsg.GetMsgType())
  {
  case TC_IE:
  {
  uint16_t tempPqm = L2rRxMsg.GetPQM();
  uint16_t tempDepth = L2rRxMsg.GetDepth();
  uint16_t tempMsn  = L2rRxMsg.GetMSN();
  uint32_t tempArrivalRate = L2rRxMsg.GetArrivalRate();
  float tempQueueSize = L2rRxMsg.GetQueueSize() / float(m_maxQueueSize);
  uint32_t tempDelay = L2rRxMsg.GetDelay();
  float *arrRate = reinterpret_cast<float*>(&tempArrivalRate);
  float *entavgDelay3 = reinterpret_cast<float*>(&tempDelay);
  m_tcieInterval = L2rRxMsg.GetTCIEInterval();
  NS_LOG_FUNCTION ("Received a TC-IE packet from "
                  << sender << " to " << receiver << ". Details are: Destination: " << L2rRxMsg.GetMeshRootAddress () << ", PQM: "
                  << L2rRxMsg.GetPQM () << ", depth: " << L2rRxMsg.GetDepth ());
    if(tempMsn > 0xf0)
    {       
      if(!m_isSink && m_depth == 0)
      {
        uint16_t tempLqm = 1; //ToDo What LQM should be for node with 1 depth
        tempPqm += tempLqm;
        m_depth = tempDepth + 1;  
          L2R_RoutingTableEntry newEntry (
                tempDepth,
                tempPqm,
                Simulator::Now (), //ToDo Life Time
                Seconds(m_tcieInterval),//ToDo convert TcIE to time
                sender,
                false);
              newEntry.SetFlag (VALID);
              newEntry.SetArrivalRatePar(*arrRate);
              newEntry.SetQueuePar(tempQueueSize);
              newEntry.SetDelayPar(*entavgDelay3);
              m_routingTable.AddRoute(newEntry);
              NS_LOG_FUNCTION ("New Route added to routing tables");
              m_depth = tempDepth+1;
              m_l2rReceiveUpdateCallback(rxParams,m_depth,m_pqm,m_shortAddress);
              m_pqm = tempPqm;
        
      }
      else
      {
        if (tempDepth >= m_depth)
        {
          //if Rx msg from upstream node discard
          NS_LOG_FUNCTION ("Discard packet of size " << p->GetSize ()<<"MSN " <<tempMsn <<"Rx msg from upstream node");
          return;
        }
        else
        {
          if(tableVerifier == true) //already have this route in the entry
          {
            NS_LOG_FUNCTION ("Discard packet of size " << p->GetSize ()<<"MSN " <<tempMsn <<" already have this route in the entry");
            return;
          }
          else
          {
            uint16_t tempLqm = 1; //ToDo What LQM should be for node in initialization
            tempPqm += tempLqm;
            L2R_RoutingTableEntry newEntry (
            tempDepth,
            tempPqm,
            Simulator::Now (), //ToDo Life Time
            Seconds(m_tcieInterval),//ToDo convert TcIE to time
            sender,
            false);
            newEntry.SetFlag (VALID);
            newEntry.SetArrivalRatePar(*arrRate);
            newEntry.SetQueuePar(tempQueueSize);
            newEntry.SetDelayPar(*entavgDelay3);
            bool returnSuccessful =  m_routingTable.AddRoute (newEntry);
            NS_LOG_FUNCTION ("New Route added to routing tables" << returnSuccessful);
          }
          uint16_t minPqm = 0xffff;
          std::map<Mac16Address, L2R_RoutingTableEntry> allRoutes;
          m_routingTable.GetListOfAllRoutes(allRoutes);
          for (std::map<Mac16Address, L2R_RoutingTableEntry>::const_iterator i = allRoutes.begin (); i
            != allRoutes.end (); ++i) 
          {
            if(i->second.GetPQM () < minPqm)
            {
              minPqm = i->second.GetPQM ();
              m_pqm = minPqm;
              m_depth = i->second.GetDepth() + 1;
            }
          }
              m_l2rReceiveUpdateCallback(rxParams,m_depth,m_pqm,m_shortAddress);
          //look if the mac in the routing table Done
          //do the pqm condition Done
          //ToDo my depth Done
        } 
      }
      if(m_isSink == false)     
      {
        event =  Simulator::ScheduleNow(&LrWpanMac::L2R_SendPeriodicUpdate,this);
        m_routingTable.AddMacEvent(sender, event);
        NS_LOG_FUNCTION("EventCreated EventUID: " << event.GetUid ());
      } 
    }
    else //msn < 0xf0
    {
      if(tableVerifier == false) //new Entry
      {
        uint16_t tempLqm = 1; //ToDo What LQM should be for new nodes 
        tempPqm += tempLqm;
        L2R_RoutingTableEntry newEntry (
        tempDepth,
        tempPqm,
        Simulator::Now (),
        Seconds(m_tcieInterval),
        sender,
        false);
        newEntry.SetFlag (VALID);
        newEntry.SetArrivalRatePar(*arrRate);
        newEntry.SetQueuePar(tempQueueSize);
        newEntry.SetDelayPar(*entavgDelay3);
        bool returnSuccessful =  m_routingTable.AddRoute (newEntry);
            NS_LOG_FUNCTION ("New Route added to routing tables" << returnSuccessful);
        if(m_isSink) //code not completed here all nodes have zero pqm
        {
          m_l2rReceiveUpdateCallback(rxParams,m_depth,m_pqm,m_shortAddress);
          return;
        }
        uint16_t minPqm = 10000;
        std::map<Mac16Address, L2R_RoutingTableEntry> allRoutes;
        m_routingTable.GetListOfAllRoutes(allRoutes);
        for (std::map<Mac16Address, L2R_RoutingTableEntry>::const_iterator i = allRoutes.begin (); i
          != allRoutes.end (); ++i) 
        {
        if(i->second.GetPQM () < minPqm)
          {
            minPqm = i->second.GetPQM ();
            m_pqm = minPqm;
            m_depth = i->second.GetDepth() + 1;
          }
        }
        m_l2rReceiveUpdateCallback(rxParams,m_depth,m_pqm,m_shortAddress);
     }
     else
     {
       if((m_msn == 0xef && tempMsn < 0xef) || m_msn < tempMsn)
       {
          double arrivalRateMovingAvg = 0;
          std::queue<Time> temp_myqueue = m_arrivalRateMovingAvg;
          while (!temp_myqueue.empty())
          {
	    	    arrivalRateMovingAvg += temp_myqueue.front().GetSeconds();
		        temp_myqueue.pop();
	        }
          if(m_arrivalRateMovingAvg.size() != 0)
            arrivalRateMovingAvg = arrivalRateMovingAvg / m_arrivalRateMovingAvg.size();
         /* uint16_t tempLqm = tableEntry.GetQueuePar() * m_txQueue.size() / m_maxQueueSize +
                              tableEntry.GetArrivalPar() * arrivalRateMovingAvg +
                              tableEntry.GetDelayPar() * m_avgDelay / m_delayCountPacket;*/
          uint16_t tempLqm =  tempQueueSize * 10 +
                              1/(tempArrivalRate+1) * 10 +
                              tempDelay;                    
          //uint16_t tempLqm = 1;

          tempPqm += tempLqm;
          tableEntry.SetDepth(tempDepth);
          tableEntry.SetEntriesChanged(true);
          tableEntry.SetFlag(VALID);
          tableEntry.SetNextHop(sender);
          tableEntry.SetLifeTime(Simulator::Now ());
          tableEntry.SetPQM(tempPqm);
          tableEntry.SetArrivalRatePar(*arrRate);
          tableEntry.SetQueuePar(tempQueueSize);
          tableEntry.SetDelayPar(*entavgDelay3);
          m_routingTable.Update(tableEntry);
          NS_LOG_FUNCTION ("Received update For TcIE From " << sender);
          if(m_isSink) //code not completed here all nodes have zero pqm
          {
            m_l2rReceiveUpdateCallback(rxParams,m_depth,m_pqm,m_shortAddress);
            return;
          }
          uint16_t minPqm = 0xffff;
          std::map<Mac16Address, L2R_RoutingTableEntry> allRoutes;
          m_routingTable.GetListOfAllRoutes(allRoutes);
          for (std::map<Mac16Address, L2R_RoutingTableEntry>::const_iterator i = allRoutes.begin (); i
            != allRoutes.end (); ++i) 
          {
            if(i->second.GetPQM () < minPqm)
            {
              minPqm = i->second.GetPQM ();
              m_pqm = minPqm;
              m_depth = i->second.GetDepth() + 1;
            }
          }
          m_l2rReceiveUpdateCallback(rxParams,m_depth,m_pqm,m_shortAddress);
       }
       else
       {
        NS_LOG_FUNCTION ("Discard packet of size " << p->GetSize ()<<"MSN " <<tempMsn <<"Rx msg from upstream node");
        break;
       }
       
     }
    }
    if(m_isSink == false)
    {
      event =  Simulator::ScheduleNow(&LrWpanMac::L2R_SendPeriodicUpdate,this);
      m_routingTable.AddMacEvent(sender, event);
      NS_LOG_FUNCTION("EventCreated EventUID: " << event.GetUid ());
    }
    break;
  }
  case L2R_D_IE:
  {
    if(!m_isSink &&  m_rootAddress == Mac16Address("00:00"))
    {
      NS_LOG_FUNCTION ("Received a TC-D-IE packet from "
                  << sender << " to " << receiver << "Mesh Root Address: " << L2rRxMsg.GetMeshRootAddress ());
      m_rootAddress = L2rRxMsg.GetMeshRootAddress();
      L2R_Header L2R_DIE;
      m_rootAddress = m_shortAddress; //change root name
      L2R_DIE.SetMeshRootAddress(m_rootAddress);
      L2R_DIE.SetMsgType(L2R_D_IE);
      Ptr<Packet> p0 = Create<Packet> (); //Zero payload packet
      p0->AddHeader (L2R_DIE); //serialize is called here
      McpsDataRequestParams params;
      params.m_dstPanId = this->GetPanId();
      params.m_srcAddrMode = SHORT_ADDR;
      params.m_dstAddrMode = SHORT_ADDR;
      params.m_dstAddr = Mac16Address("ff:ff");
      params.m_msduHandle = 0; //ToDo underStand the msduhandle from standard
      params.m_txOptions = TX_OPTION_NONE;
      Simulator::ScheduleNow (&LrWpanMac::McpsDataRequest,this,
                             params, p0);
    }
    else
    {
      NS_LOG_FUNCTION ("Discard packet of size " << p->GetSize () <<"Rx msg from upstream node");
    }
    
  break;
  }
  case DataHeader:
  {
    if(m_isSink)
    {
      //std::cout<<"Mesh Successfully Received a packet with: " << std::endl;
      L2R_Header dataHeader;
      originalPkt->RemoveHeader(dataHeader);

      float ent1 = float(dataHeader.GetQueueSize())/float(m_maxQueueSize);
      uint32_t arrivalRate  = dataHeader.GetArrivalRate();
      uint32_t avdelay = dataHeader.GetDelay();
      float *ent2 = reinterpret_cast<float*>(&arrivalRate);
      float *ent3 = reinterpret_cast<float*>(&avdelay);
      /*std::cout <<"Queue Size: " << ent1 << std::endl;
      std::cout <<"Arrival Rate: " << *ent2 << std::endl;
      std::cout <<"AvgDelay: " << *ent3 << std::endl;*/

      Mac16Address srcAddress = dataHeader.GetSrcAddress();
      MeshRootData newEntry = {ent1, //number of element in the queue / queue size
                               *ent2,//avg of the msg received / time ToDo make it normalized
                               *ent3,}; //The time that the packet stay in the queue 
      
      m_meshRootData.insert (std::make_pair(dataHeader.GetDepth(), newEntry));
      ++m_totalPacketRxByMesh;
      m_meshRxMsgCallback(newEntry,srcAddress);
      *m_stream->GetStream () << Simulator::Now ().GetSeconds () <<" Sink Receive Packet number: " << originalPkt->GetUid() 
                            <<" Received From node: " << sender << " To Me: "<< m_shortAddress 
                            <<" Node Queue Size: " << m_queueSize
                            <<std::endl;
      return;
    }
    //std::cout << "New Data Received: " << std::endl;
    //std::cout << "Queue Size is: " << m_txQueue.size () << std::endl;
    Time now = Simulator::Now ();
    if (m_arrivalRateMovingAvg.size() > 20) //taking arrival rate for 20 reading
      m_arrivalRateMovingAvg.pop();
    
    if(m_arrivalRateComplement == 0)
    {
      m_arrivalRate = now;
      ++m_arrivalRateComplement;
    }
    else
    {
      m_arrivalRate = now - m_arrivalRate;
      m_arrivalRateMovingAvg.push(m_arrivalRate);
      m_arrivalRateComplement = 0;
    }
    m_delayForEachPacket.insert(std::make_pair (originalPkt->GetUid(), now));
    McpsDataRequestParams paramsSend;
    paramsSend.m_dstPanId = this->GetPanId();;
    paramsSend.m_srcAddrMode = SHORT_ADDR;
    paramsSend.m_dstAddrMode = SHORT_ADDR;
    paramsSend.m_dstAddr = this->OutputRoute ();
    paramsSend.m_msduHandle = 0; //ToDo underStand the msduhandle from standard
    paramsSend.m_txOptions = TX_OPTION_ACK;  
    *m_stream->GetStream () << now.GetSeconds () <<" Forward Packet number: " << originalPkt->GetUid() 
                            <<" Received From node: " << sender << " To Me: "<< m_shortAddress 
                            <<" Forword it to node: " << paramsSend.m_dstAddr << std::endl;
    //std::cout << "Sending Data Packet From: " << m_shortAddress << "To: " <<paramsSend.m_dstAddr << std::endl;
    ++m_internalLoad;
    Simulator::ScheduleNow(&LrWpanMac::McpsDataRequest,this, paramsSend, originalPkt);
  
  break;
  }
  case NLM_IE:
    if(tableVerifier == true)
    {
      float normalizeQueue = float(L2rRxMsg.GetQueueSize())/float(m_maxQueueSize);
      uint32_t arrivalRate  = L2rRxMsg.GetArrivalRate();
      uint32_t avdelay = L2rRxMsg.GetDelay();
      float *arrRate = reinterpret_cast<float*>(&arrivalRate);
      float *entavgDelay3 = reinterpret_cast<float*>(&avdelay);
      tableEntry.SetArrivalRatePar(*arrRate);
      tableEntry.SetDelayPar(*entavgDelay3);
      tableEntry.SetQueuePar(normalizeQueue);
      m_routingTable.Update(tableEntry);
    }
  break;
  }
}
void
LrWpanMac::SendNlmMsg()
{
  L2R_Header L2R_NLM;
  m_rootAddress = m_shortAddress; //change root name
  L2R_NLM.SetMsgType(NLM_IE);
  L2R_NLM.SetArrivalRate(GetArrivalRate());
  L2R_NLM.SetDelay(GetAvgDelay());
  L2R_NLM.SetQueueSize(m_txQueue.size());
  Ptr<Packet> p0 = Create<Packet> (); //Zero payload packet
  p0->AddHeader (L2R_NLM); //serialize is called here
  McpsDataRequestParams params;
  params.m_dstPanId = this->GetPanId();
  params.m_srcAddrMode = SHORT_ADDR;
  params.m_dstAddrMode = SHORT_ADDR;
  params.m_dstAddr = Mac16Address("ff:ff");
  params.m_msduHandle = 0; //ToDo underStand the msduhandle from standard
  params.m_txOptions = TX_OPTION_NONE;
  Simulator::ScheduleNow (&LrWpanMac::McpsDataRequest,this,
                          params, p0);
}
void
LrWpanMac::L2R_MaxMissedTcIeMsg (uint8_t maxMissed)
{
  m_routingTable.SetL2rMaxMissedTcIe(maxMissed);
}
//AB: modified on 19/11
Mac16Address
LrWpanMac::OutputRoute()
{
  std::map<uint16_t, L2R_RoutingTableEntry> PQMValues;
  std::map<Mac16Address, L2R_RoutingTableEntry> possibleRoutes;
  bool anyAvailabeAncestor;
  anyAvailabeAncestor = m_routingTable.GetListOfDestinationWithNextHop(possibleRoutes, m_depth);  // neighbours with depth < my_depth
  Mac16Address nextHopAddress = Mac16Address("00:00");
  if(anyAvailabeAncestor == true)
  {
    std::map<Mac16Address, L2R_RoutingTableEntry>::const_iterator i = possibleRoutes.begin();
    for(;i != possibleRoutes.end(); ++i)
    {
      PQMValues.insert(std::make_pair(i->second.GetPQM(), i->second));    // sorted map according to PQM, but not Depth: add later to generalize

    }

    std::map<uint16_t, L2R_RoutingTableEntry>::const_iterator j = PQMValues.begin();
    for(;j != PQMValues.end(); ++j)
    {
      if(j->second.GetLQM() <= m_lqt)  // no GetLQM function in L2R_RoutingTableEntry? Add it?
        {
          nextHopAddress = j->second.GetNextHop();
          return nextHopAddress;
        }
    }
    j = PQMValues.begin();
    uint16_t minLQM = j->second.GetLQM();
    nextHopAddress = j->second.GetNextHop();
    for(;j != PQMValues.end(); ++j)   // send to lowest LQM
    {
      if(j->second.GetLQM() < minLQM)
      {
        minLQM = j->second.GetLQM();
        nextHopAddress = j->second.GetNextHop();
      }
    }
    if (nextHopAddress == Mac16Address("00:00"))
      std::cout << "Node Doesn't send any thing no Ancecters" << std::endl;
    return nextHopAddress;
  }
  else
  {
		std::map<Mac16Address, L2R_RoutingTableEntry> allRoutes;
    m_routingTable.GetListOfAllRoutes(allRoutes);
		std::map<Mac16Address, L2R_RoutingTableEntry>::const_iterator i = allRoutes.begin ();
		uint16_t minDepth = i->second.GetDepth();
    for (; i != allRoutes.end (); ++i) 
  	{
    	if(i->second.GetDepth () < minDepth)
          {
            m_depth = i->second.GetDepth() + 1;
            nextHopAddress = i->first;
          }
    } // continiou cout mac in lower depth
    return nextHopAddress;
  }
  
}
//AM: modified on 25/11
void LrWpanMac::PrintRoutingTable (Ptr<Node> node,Ptr<OutputStreamWrapper> stream, Time::Unit unit)
{
  //Ptr<Node> node = NodeList::GetNode (0);
  if(m_isSink == true)
  {
  ++m_tcieIncr;
  for(uint8_t i = 0; i<6;i++)
    {
      *stream->GetStream () << "******************";
      if(i == 2)
        *stream->GetStream () << "TC-IE Updates number: " << m_tcieIncr;
    }
    *stream->GetStream () << std::endl;
  }
  m_nodeId = node->GetId ();
  *stream->GetStream () << "Node: " <<  m_nodeId
                        << std::resetiosflags(std::ios::adjustfield) << std::setiosflags (std::ios::fixed)
                        << ", Mac Address: " << m_shortAddress
                        << ", Depth: " << m_depth
                        << std::setprecision (4)
                        << ", Time: " << Now ().As (unit)
                        << std::setprecision (4)
                        << ", Local time: " << node->GetLocalTime ().As (unit)
                        << ", L2R Routing table";
  if(m_isSink == true)
  {
    *stream->GetStream () << ", Mesh Root" << std::endl;
  }
  else
    *stream->GetStream () << std::endl;


  m_routingTable.Print (stream);
  *stream->GetStream () << std::endl;
 
 //m_printRoutingTableTimer.SetFunction(&LrWpanMac::PrintRoutingTable,this,node,stream,unit);
 //m_printRoutingTableTimer.Schedule(Seconds(m_tcieInterval+1));
  Time TCIEInter =  Seconds(m_tcieInterval + 1) + Now();
  Simulator::Schedule (TCIEInter, &LrWpanMac::PrintRoutingTable,this,
                          node, stream,unit);
}
uint16_t
LrWpanMac::GetQueueSize(void) const
{
  return m_queueSize;
}
uint32_t 
LrWpanMac::GetArrivalRate(void) const
{
  float sendArrival = 0;
  std::queue<Time> temp_myqueue = m_arrivalRateMovingAvg;
  while (!temp_myqueue.empty())
  {
		sendArrival += temp_myqueue.front().GetSeconds();
		temp_myqueue.pop();
	}
  if(m_arrivalRateMovingAvg.size() != 0)
    sendArrival = sendArrival / m_arrivalRateMovingAvg.size();
  uint32_t* pInt = reinterpret_cast<uint32_t*>(&sendArrival);
  return *pInt;
}
uint32_t
LrWpanMac::GetAvgDelay(void)
{
  float sendDelay = 0;
  Time now = Simulator::Now ();
  for (std::map<uint64_t, Time>::iterator i = m_delayForEachPacket.begin (); i != m_delayForEachPacket.end (); ++i)
    m_avgDelay = m_avgDelay + (now.GetSeconds () - i->second.GetSeconds ());
  //m_avgDelay = m_avgDelay/ m_delayForEachPacket.size();
  if (m_delayForEachPacket.size() != 0)
    sendDelay = m_avgDelay / m_delayForEachPacket.size();
  uint32_t* pInt = reinterpret_cast<uint32_t*>(&sendDelay); //ToDo Should I deallocate this
  m_avgDelay = 0;
  return *pInt;
}
uint16_t 
LrWpanMac::GetDepth(void) const
{
  return m_depth;
}
uint16_t 
LrWpanMac::GetPqm (void) const
{
  return m_pqm;
}
uint32_t 
LrWpanMac::GetTotalPacketDroppedByQueue(void)
{
  return m_totalPacketDroppedByNode;
}
uint32_t
LrWpanMac::GetPacketDroppedByQueue(void)
{
  uint32_t temp = m_totalPacketDroppedByNode -  m_totalPacketDroppedEverySecond;
  m_totalPacketDroppedEverySecond = m_totalPacketDroppedByNode;
  return temp;
}
uint32_t 
LrWpanMac::GetTotalPacketSentByNode(void) const
{
  return m_totalPacketSentByNode;
}
uint32_t 
LrWpanMac::GetTotalPacketRxByMeshRoot(void) const
{
  return m_totalPacketRxByMesh;
}
void 
LrWpanMac::SetMaxQueueSize (uint16_t maxQueue)
{
  m_maxQueueSize = maxQueue;
}
uint16_t 
LrWpanMac::GetMaxQueueSize(void) const
{
  return m_maxQueueSize;
}
void 
LrWpanMac::UpdateDelay(uint64_t pId, Time t)
{
  m_delayForEachPacket.insert(std::make_pair (pId, t));
}
void
LrWpanMac::outputRoutesTree(Ptr<OutputStreamWrapper> stream)
{
  m_stream = stream;
}
void 
LrWpanMac::OutputTree(Ptr<Packet> p, Time t,McpsDataRequestParams params)
{
  *m_stream->GetStream () << t.GetSeconds () <<" Node Generate Packet number: " << p->GetUid() 
                            <<" Sending From: " << m_shortAddress << " To: "<< params.m_dstAddr 
                            <<" Node Queue Size: " << m_queueSize << std::endl;
} 
uint32_t 
LrWpanMac::GetInternalLoad() const
{
  return m_internalLoad;
}
} // namespace ns3