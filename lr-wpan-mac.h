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
 *  For IEEE 802.15.10:
 *  Ahmad Mustafa Abdel-Qader <ahmad.m.abdelqader@gmail.com>
 */
#ifndef LR_WPAN_MAC_H
#define LR_WPAN_MAC_H

#include <ns3/object.h>
#include <ns3/traced-callback.h>
#include <ns3/traced-value.h>
#include <ns3/mac16-address.h>
#include <ns3/mac64-address.h>
#include <ns3/sequence-number.h>
#include <ns3/lr-wpan-phy.h>
#include <ns3/event-id.h>
#include <deque>
#include <iostream>
#include <ns3/packet.h>
#include <cassert>
#include <map>
#include <sys/types.h>
#include "ns3/output-stream-wrapper.h"
#include "ns3/timer.h"
#include "ns3/random-variable-stream.h"
#include "queue"
namespace ns3 {
  //AM: modified at 4/11

class Packet;
class LrWpanCsmaCa;
class Node;

/**
 * \defgroup lr-wpan LR-WPAN models
 *
 * This section documents the API of the IEEE 802.15.4-related models.  For a generic functional description, please refer to the ns-3 manual.
 */

/**
 * \ingroup lr-wpan
 *
 * Tx options
 */
typedef enum
{
  TX_OPTION_NONE = 0,    //!< TX_OPTION_NONE
  TX_OPTION_ACK = 1,     //!< TX_OPTION_ACK
  TX_OPTION_GTS = 2,     //!< TX_OPTION_GTS
  TX_OPTION_INDIRECT = 4 //!< TX_OPTION_INDIRECT
} LrWpanTxOption;

/**
 * \ingroup lr-wpan
 *
 * MAC states
 */
typedef enum
{
  MAC_IDLE,              //!< MAC_IDLE
  MAC_CSMA,              //!< MAC_CSMA
  MAC_SENDING,           //!< MAC_SENDING
  MAC_ACK_PENDING,       //!< MAC_ACK_PENDING
  CHANNEL_ACCESS_FAILURE,//!< CHANNEL_ACCESS_FAILURE
  CHANNEL_IDLE,          //!< CHANNEL_IDLE
  SET_PHY_TX_ON          //!< SET_PHY_TX_ON
} LrWpanMacState;

namespace TracedValueCallback {

/**
 * \ingroup lr-wpan
 * TracedValue callback signature for LrWpanMacState.
 *
 * \param [in] oldValue original value of the traced variable
 * \param [in] newValue new value of the traced variable
 */
  typedef void (* LrWpanMacState)(LrWpanMacState oldValue,
                                  LrWpanMacState newValue);

}  // namespace TracedValueCallback

/**
 * \ingroup lr-wpan
 *
 * table 80 of 802.15.4
 */
typedef enum
{
  NO_PANID_ADDR = 0,
  ADDR_MODE_RESERVED = 1,
  SHORT_ADDR = 2,
  EXT_ADDR = 3
} LrWpanAddressMode;

/**
 * \ingroup lr-wpan
 *
 * table 83 of 802.15.4
 */
typedef enum
{
  ASSOCIATED = 0,
  PAN_AT_CAPACITY = 1,
  PAN_ACCESS_DENIED = 2,
  ASSOCIATED_WITHOUT_ADDRESS = 0xfe,
  DISASSOCIATED = 0xff
} LrWpanAssociationStatus;

/**
 * \ingroup lr-wpan
 *
 * Table 42 of 802.15.4-2006
 */
typedef enum
{
  IEEE_802_15_4_SUCCESS                = 0,
  IEEE_802_15_4_TRANSACTION_OVERFLOW   = 1,
  IEEE_802_15_4_TRANSACTION_EXPIRED    = 2,
  IEEE_802_15_4_CHANNEL_ACCESS_FAILURE = 3,
  IEEE_802_15_4_INVALID_ADDRESS        = 4,
  IEEE_802_15_4_INVALID_GTS            = 5,
  IEEE_802_15_4_NO_ACK                 = 6,
  IEEE_802_15_4_COUNTER_ERROR          = 7,
  IEEE_802_15_4_FRAME_TOO_LONG         = 8,
  IEEE_802_15_4_UNAVAILABLE_KEY        = 9,
  IEEE_802_15_4_UNSUPPORTED_SECURITY   = 10,
  IEEE_802_15_4_INVALID_PARAMETER      = 11
} LrWpanMcpsDataConfirmStatus;


/**
 * \ingroup lr-wpan
 *
 * MCPS-DATA.request params. See 7.1.1.1
 */
struct McpsDataRequestParams
{
  McpsDataRequestParams ()
    : m_srcAddrMode (SHORT_ADDR),
      m_dstAddrMode (SHORT_ADDR),
      m_dstPanId (0),
      m_dstAddr (),
      m_msduHandle (0),
      m_txOptions (0)
  {
  }
  LrWpanAddressMode m_srcAddrMode; //!< Source address mode
  LrWpanAddressMode m_dstAddrMode; //!< Destination address mode
  uint16_t m_dstPanId;             //!< Destination PAN identifier
  Mac16Address m_dstAddr;          //!< Destination address
  Mac64Address m_dstExtAddr;       //!< Destination extended address
  uint8_t m_msduHandle;            //!< MSDU handle
  uint8_t m_txOptions;             //!< Tx Options (bitfield)
};

/**
 * \ingroup lr-wpan
 *
 * MCPS-DATA.confirm params. See 7.1.1.2
 */
struct McpsDataConfirmParams
{
  uint8_t m_msduHandle; //!< MSDU handle
  LrWpanMcpsDataConfirmStatus m_status; //!< The status of the last MSDU transmission
};

/**
 * \ingroup lr-wpan
 *
 * MCPS-DATA.indication params. See 7.1.1.3
 */
struct McpsDataIndicationParams
{
  uint8_t m_srcAddrMode;  //!< Source address mode
  uint16_t m_srcPanId;    //!< Source PAN identifier
  Mac16Address m_srcAddr; //!< Source address
  Mac64Address m_srcExtAddr;  //!< Source extended address
  uint8_t m_dstAddrMode;  //!< Destination address mode
  uint16_t m_dstPanId;    //!< Destination PAN identifier
  Mac16Address m_dstAddr; //!< Destination address
  Mac64Address m_dstExtAddr;  //!< Destination extended address
  uint8_t m_mpduLinkQuality;  //!< LQI value measured during reception of the MPDU
  uint8_t m_dsn;          //!< The DSN of the received data frame
};
//AM: modified on 3/12
struct MeshRootData
{
  float m_queueSize;
  float m_arrivalRate;
  float m_avgDelay;
};


/**
 * \ingroup lr-wpan
 *
 * This callback is called after a McpsDataRequest has been called from
 * the higher layer.  It returns a status of the outcome of the
 * transmission request
 */
typedef Callback<void, McpsDataConfirmParams> McpsDataConfirmCallback;

/**
 * \ingroup lr-wpan
 *
 * This callback is called after a Mcps has successfully received a
 *  frame and wants to deliver it to the higher layer.
 *
 *  \todo for now, we do not deliver all of the parameters in section
 *  7.1.1.3.1 but just send up the packet.
 */
typedef Callback<void, McpsDataIndicationParams, Ptr<Packet>> McpsDataIndicationCallback;
//AM: modified on 30/12
typedef Callback<void, McpsDataIndicationParams,uint16_t ,uint16_t, Mac16Address> L2rReceiveUpdateCallback;
typedef Callback<void,MeshRootData,Mac16Address> meshRootRxMsgCallback;
/**
 * \ingroup lr-wpan
 *
 * Class that implements the LR-WPAN Mac state machine
 */

//AM: modified at 7/11

enum L2R_MsgType
{
  TC_IE = 0,
  L2R_D_IE = 1,
  DataHeader = 2,
  NLM_IE = 3
};
class L2R_Header : public Header 
{
public:
  L2R_Header ();
  virtual ~L2R_Header ();

  /**
   * Set the header data.
   * \param meshRootAddress.
   * \param depth.
   * \param PQM.
   * \param MSN.
   * \param TCIEInterval.
   */
  void SetDepth (uint16_t depth);
  void SetMeshRootAddress(Mac16Address meshRootAddress);
  void SetPQM (uint16_t pqm);
  void SetMSN (uint16_t msn);
  void SetTCIEInterval(uint8_t tcieinterval);
  void SetMsgType (enum L2R_MsgType msgType);
  void SetLQT(uint16_t lqt);
  void SetQueueSize(uint16_t queue);
  void SetDelay(uint32_t delay);
  void SetArrivalRate(uint32_t arrivalRate);
  void SetSrcMacAddress (Mac16Address srcAddress);
  /**
   * Get the header data.
   * \return The data.
   */
  uint16_t GetDepth (void) const;
  Mac16Address GetMeshRootAddress (void) const;
  uint16_t GetPQM (void) const;
  uint16_t GetMSN (void) const;
  uint8_t GetTCIEInterval(void) const;
  enum L2R_MsgType GetMsgType(void) const;
  uint16_t GetLQT(void) const;
  uint16_t GetQueueSize(void) const;
  uint32_t GetDelay(void) const;
  uint32_t GetArrivalRate(void) const;
  Mac16Address GetSrcAddress (void) const;
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual void Print (std::ostream &os) const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);
  virtual uint32_t GetSerializedSize (void) const;
private:
  uint16_t m_depth;  //!< Header data
  Mac16Address m_meshRootAddress;
  uint16_t m_PQM;
  uint8_t m_TCIEInterval;
  uint16_t m_MSN;
  uint16_t m_LQT;
  uint8_t m_msgType;
  uint16_t m_queueSize;
  uint32_t m_arrivalRate;
  uint32_t m_avgDelay; 
  Mac16Address m_srcAddress;
  
};



enum RouteFlags
{
  VALID = 0,     // !< VALID
  INVALID = 1,     // !< INVALID
};
class L2R_RoutingTableEntry
{
public:
  /**
   * c-tor
   *
   * 
   */
  L2R_RoutingTableEntry (uint16_t depth = 0, uint16_t PQM = 0,Time lifetime = Simulator::Now (), Time tcieInterval = Simulator::Now (),
                     Mac16Address nextHop = Mac16Address (), bool changedEntries = false); //added MSN

  ~L2R_RoutingTableEntry ();
  /**
   * Get destination MAC address
   * \returns the destination MAC address (usually Sink node)
   */
  Mac16Address
  GetDestination () const
  {
    return m_dst;
  }
  /**
   * Get route
   * \returns the IPv4 route
   */
  /*Ptr<Ipv4Route> //ToDo
  GetRoute () const
  {
    return 
  }*/
  /**
   * Set route
   * \param route the IPv4 route
   */
  /*void
  SetRoute (Ptr<Ipv4Route> route) //ToDo
  {
     = route;
  }*/
  /**
   * Set next hop
   * \param nextHop the Mac address of the next hop
   */
  void
  SetNextHop (Mac16Address nextHop)
  {
    m_nextHop = nextHop;
  }
  void
  SetLQM(uint16_t lqm)
  {
    m_lqm = lqm;
  }
  uint16_t 
  GetLQM() const
  {
    return m_lqm;
  }
  /**
   * Get next hop
   * \returns the Mac address of the next hop
   */
  Mac16Address
  GetNextHop () const
  {
    return m_nextHop;
  }
  /**
   * Set depth
   * \param depth the depth
   */
  void
  SetDepth (uint16_t depth)
  {
    m_depth = depth;
  }
  /**
   * Get depth
   * \returns the depth
   */
  uint32_t
  GetDepth () const
  {
    return m_depth;
  }
  /**
   * Set PQM
   * \param PQM the PQM
   */
  void
  SetPQM (uint16_t pqm)
  {
    m_pqm = pqm;
  }
  /**
   * Get PQM
   * \returns the PQM count
   */
  uint16_t
  GetPQM () const
  {
    return m_pqm;
  }
  /**
   * Set lifetime
   * \param lifeTime the lifetime value
   */
  void
  SetLifeTime (Time lifeTime)
  {
    m_lifeTime = lifeTime;
  }
  /**
   * Get lifetime
   * \returns the lifetime value
   */
  Time
  GetLifeTime () const
  {
    return (Simulator::Now () - m_lifeTime);
  }
  /**
   * Set TC IE Interval time
   * \param TC IE Interval the TC IE Interval
   */
  void
  SetTCIEInterval (Time tcieinterval)
  {
    m_tcieInterval = tcieinterval;
  }
  /**
   * Get TC IE Interval
   * \returns the TC IE Interval
   */
  Time
  GetTCIEInterval () const
  {
    return (m_tcieInterval);
  }
  /**
   * Set route flags
   * \param flag the route flags
   */
  void
  SetFlag (RouteFlags flag)
  {
    m_flag = flag;
  }
  /**
   * Get route flags
   * \returns the route flags
   */
  RouteFlags
  GetFlag () const
  {
    return m_flag;
  }
  /**
   * Set entries changed indicator
   * \param entriesChanged
   */
  void
  SetEntriesChanged (bool entriesChanged) //ToDo
  {
    m_entriesChanged = entriesChanged;
  }
  /**
   * Get entries changed
   * \returns the entries changed flag
   */
  bool
  GetEntriesChanged () const
  {
    return m_entriesChanged;
  }
  /**
   * \brief Compare destination address
   * \param destination destination node Mac address
   * \return true if equal
   */
  bool
  operator== (Mac16Address const destination) const
  {
    return m_dst == destination;
  }
  /**
   * Print routing table entry
   * \param stream the output stream
   */
  void
  Print (Ptr<OutputStreamWrapper> stream) const;
  uint8_t
  GetL2rMissedTcIe () 
  {
    return m_l2rMissedTcIe; 
  }
  void
  IncL2rMissedTcIe () 
  {
    ++m_l2rMissedTcIe; 
  }
  //algorithm Parameter
  void
  SetArrivalRatePar (float arrivalPar)
  {
    m_arrivalPar = arrivalPar;
  }
  void 
  SetQueuePar(float queuePar)
  {
    m_queuePar = queuePar;
  }
  void 
  SetDelayPar(float delayPar)
  {
    m_delayPar = delayPar;
  }
  float
  GetQueuePar()
  {
    return m_queuePar;
  }
  float
  GetArrivalPar()
  {
    return m_arrivalPar;
  }
  float
  GetDelayPar()
  {
    return m_delayPar;
  }

private:
  // Fields
  /// Destination Sequence Number
  uint16_t m_depth;
  /// Hop Count (number of hops needed to reach destination)
  uint16_t m_pqm;
  /**
   * \brief Expiration or deletion time of the route
   *	Lifetime field in the routing table plays dual role --
   *	for an active route it is the expiration time, and for an invalid route
   *	it is the deletion time.
   */
  Time m_lifeTime;
  
  /// Routing flags: valid, invalid or in search
  RouteFlags m_flag; //ToDo
  /// Time for which the node retains an update with changed metric before broadcasting it.
  /// A node does that in hope of receiving a better update.
  Time m_tcieInterval;
  /// Flag to show if any of the routing table entries were changed with the routing update.
  uint32_t m_entriesChanged;
  uint8_t m_l2rMissedTcIe;
  Mac16Address m_dst;
  Mac16Address m_nextHop;
  uint16_t m_lqm;
  //algorithm Parameters
  float m_queuePar;
  float m_delayPar;
  float m_arrivalPar;
};

/**
 * \ingroup l2r
 * \brief The Routing table used by L2R protocol
 */
class L2R_RoutingTable
{
public:
  /// c-tor
  L2R_RoutingTable ();
  /**
   * Add routing table entry if it doesn't yet exist in routing table
   * \param r routing table entry
   * \return true in success
   */
  bool
  AddRoute (L2R_RoutingTableEntry & r);
  /**
   * Delete routing table entry with destination address dst, if it exists.
   * \param dst destination address
   * \return true on success
   */
  bool
  DeleteRoute (Mac16Address dst);
  /**
   * Lookup routing table entry with destination address dst
   * \param dst destination address
   * \param rt entry with destination address dst, if exists
   * \return true on success
   */
  bool
  LookupRoute (Mac16Address dst, L2R_RoutingTableEntry & rt);
  /**
   * Lookup routing table entry with destination address dst
   * \param id destination address
   * \param rt entry with destination address dst, if exists
   * \param forRouteInput for routing input
   * \return true on success
   */
  bool
  LookupRoute (Mac16Address id, L2R_RoutingTableEntry & rt, bool forRouteInput);
  /**
   * Updating the routing Table with routing table entry rt
   * \param rt routing table entry
   * \return true on success
   */
  bool
  Update (L2R_RoutingTableEntry & rt);
  /**
   * Lookup list of addresses for which nxtHp is the next Hop address
   * \param nxtHp nexthop's address for which we want the list of destinations
   * \param dstList is the list that will hold all these destination addresses
   */
  bool
  GetListOfDestinationWithNextHop (std::map<Mac16Address, L2R_RoutingTableEntry> & possibleRoutes,
                             const uint16_t myDepth);
  /**
   * Lookup list of all addresses in the routing table
   * \param allRoutes is the list that will hold all these addresses present in the nodes routing table
   */
  void
  GetListOfAllRoutes (std::map<Mac16Address, L2R_RoutingTableEntry> & allRoutes);
  /// Delete all entries from routing table
  void
  Clear ()
  {
    m_mac16AddressEntry.clear ();
  }
  /**
   * Delete all outdated entries if Lifetime is expired
   * \param removedAddresses is the list of addresses to purge
   */
  void
  Purge (std::map<Mac16Address, L2R_RoutingTableEntry> & removedAddresses);
  /**
   * Print routing table
   * \param stream the output stream
   */
  void
  Print (Ptr<OutputStreamWrapper> stream) const;
  /**
   * Provides the number of routes present in that nodes routing table.
   * \returns the number of routes
   */
  uint32_t
  RoutingTableSize ();
  /**
  * Add an event for a destination address so that the update to for that destination is sent
  * after the event is completed.
  * \param address destination address for which this event is running.
  * \param id unique eventid that was generated.
  * \return true on success
  */
  bool
  AddMacEvent (Mac16Address address, EventId id);
  /**
  * Clear up the entry from the map after the event is completed
  * \param address destination address for which this event is running.
  * \return true on success
  */
  bool
  DeleteMacEvent (Mac16Address address);
  /**
  * Force delete an update waiting for settling time to complete as a better update to
  * same destination was received.
  * \param address destination address for which this event is running.
  * \return true on success
  */
  bool
  AnyRunningEvent (Mac16Address address);

  /**
    * Get the EcentId associated with that address.
    * \param address destination address for which this event is running.
    * \return EventId on finding out an event is associated else return NULL.
    */
  EventId
  GetEventId (Mac16Address address);

  /**
   * Get hold down time (time until an invalid route may be deleted)
   * \returns the hold down time
   */
  Time Getholddowntime () const
  {
    return m_holddownTime;
  }
  /**
   * Set hold down time (time until an invalid route may be deleted)
   * \param t the hold down time
   */
  void Setholddowntime (Time t)
  {
    m_holddownTime = t;
  }
  void SetL2rMaxMissedTcIe(uint8_t l2rMaxMissedTcIe)
  {
    m_l2rMaxMissedTcIe = l2rMaxMissedTcIe;
  }
  uint8_t GetL2rMaxMissedTcIe()
  {
    return m_l2rMaxMissedTcIe;
  }
private:
  // Fields
  /// an entry in the routing table.
  std::map<Mac16Address, L2R_RoutingTableEntry> m_mac16AddressEntry;
  /// an entry in the event table.
  std::map<Mac16Address, EventId> m_macEvents;
  /// hold down time of an expired route
  Time m_holddownTime;
  uint8_t m_l2rMaxMissedTcIe;
  

};


class LrWpanMac : public Object
{
public:
  /**
   * Get the type ID.
   *
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);

  /**
   * The minimum number of octets added by the MAC sublayer to the PSDU.
   * See IEEE 802.15.4-2006, section 7.4.1, Table 85
   */
  static const uint32_t aMinMPDUOverhead;

  /**
   * Default constructor.
   */
  LrWpanMac (void);
  virtual ~LrWpanMac (void);

  /**
   * Check if the receiver will be enabled when the MAC is idle.
   *
   * \return true, if the receiver is enabled during idle periods, false otherwise
   */
  bool GetRxOnWhenIdle (void);

  /**
   * Set if the receiver should be enabled when the MAC is idle.
   *
   * \param rxOnWhenIdle set to true to enable the receiver during idle periods
   */
  void SetRxOnWhenIdle (bool rxOnWhenIdle);

  // XXX these setters will become obsolete if we use the attribute system
  /**
   * Set the short address of this MAC.
   *
   * \param address the new address
   */
  void SetShortAddress (Mac16Address address);

  /**
   * Get the short address of this MAC.
   *
   * \return the short address
   */
  Mac16Address GetShortAddress (void) const;

  /**
   * Set the extended address of this MAC.
   *
   * \param address the new address
   */
  void SetExtendedAddress (Mac64Address address);

  /**
   * Get the extended address of this MAC.
   *
   * \return the extended address
   */
  Mac64Address GetExtendedAddress (void) const;

  /**
   * Set the PAN id used by this MAC.
   *
   * \param panId the new PAN id.
   */
  void SetPanId (uint16_t panId);

  /**
   * Get the PAN id used by this MAC.
   *
   * \return the PAN id.
   */
  uint16_t GetPanId (void) const;

  /**
   *  IEEE 802.15.4-2006, section 7.1.1.1
   *  MCPS-DATA.request
   *  Request to transfer a MSDU.
   *
   *  \param params the request parameters
   *  \param p the packet to be transmitted
   */
  void McpsDataRequest (McpsDataRequestParams params, Ptr<Packet> p);

  /**
   * Set the CSMA/CA implementation to be used by the MAC.
   *
   * \param csmaCa the CSMA/CA implementation
   */
  void SetCsmaCa (Ptr<LrWpanCsmaCa> csmaCa);

  /**
   * Set the underlying PHY for the MAC.
   *
   * \param phy the PHY
   */
  void SetPhy (Ptr<LrWpanPhy> phy);

  /**
   * Get the underlying PHY of the MAC.
   *
   * \return the PHY
   */
  Ptr<LrWpanPhy> GetPhy (void);

  /**
   * Set the callback for the indication of an incoming data packet.
   * The callback implements MCPS-DATA.indication SAP of IEEE 802.15.4-2006,
   * section 7.1.1.3.
   *
   * \param c the callback
   */
  void SetMcpsDataIndicationCallback (McpsDataIndicationCallback c);
  //AM: modified on 30/12
  void SetL2rReceiveUpdateCallback (L2rReceiveUpdateCallback c);
  void SetMeshRootRxMsgUpdateCallback (meshRootRxMsgCallback c);

  /**
   * Set the callback for the confirmation of a data transmission request.
   * The callback implements MCPS-DATA.confirm SAP of IEEE 802.15.4-2006,
   * section 7.1.1.2.
   *
   * \param c the callback
   */
  void SetMcpsDataConfirmCallback (McpsDataConfirmCallback c);

  // interfaces between MAC and PHY
  /**
   *  IEEE 802.15.4-2006 section 6.2.1.3
   *  PD-DATA.indication
   *  Indicates the transfer of an MPDU from PHY to MAC (receiving)
   *  @param psduLength number of bytes in the PSDU
   *  @param p the packet to be transmitted
   *  @param lqi Link quality (LQI) value measured during reception of the PPDU
   */
  void PdDataIndication (uint32_t psduLength, Ptr<Packet> p, uint8_t lqi);

  /**
   *  IEEE 802.15.4-2006 section 6.2.1.2
   *  Confirm the end of transmission of an MPDU to MAC
   *  @param status to report to MAC
   *  PHY PD-DATA.confirm status
   */
  void PdDataConfirm (LrWpanPhyEnumeration status);

  /**
   *  IEEE 802.15.4-2006 section 6.2.2.2
   *  PLME-CCA.confirm status
   *  @param status TRX_OFF, BUSY or IDLE
   */
  void PlmeCcaConfirm (LrWpanPhyEnumeration status);

  /**
   *  IEEE 802.15.4-2006 section 6.2.2.4
   *  PLME-ED.confirm status and energy level
   *  @param status SUCCESS, TRX_OFF or TX_ON
   *  @param energyLevel 0x00-0xff ED level for the channel
   */
  void PlmeEdConfirm (LrWpanPhyEnumeration status, uint8_t energyLevel);

  /**
   *  IEEE 802.15.4-2006 section 6.2.2.6
   *  PLME-GET.confirm
   *  Get attributes per definition from Table 23 in section 6.4.2
   *  @param status SUCCESS or UNSUPPORTED_ATTRIBUTE
   *  @param id the attributed identifier
   *  @param attribute the attribute value
   */
  void PlmeGetAttributeConfirm (LrWpanPhyEnumeration status,
                                LrWpanPibAttributeIdentifier id,
                                LrWpanPhyPibAttributes* attribute);

  /**
   *  IEEE 802.15.4-2006 section 6.2.2.8
   *  PLME-SET-TRX-STATE.confirm
   *  Set PHY state
   *  @param status in RX_ON,TRX_OFF,FORCE_TRX_OFF,TX_ON
   */
  void PlmeSetTRXStateConfirm (LrWpanPhyEnumeration status);

  /**
   *  IEEE 802.15.4-2006 section 6.2.2.10
   *  PLME-SET.confirm
   *  Set attributes per definition from Table 23 in section 6.4.2
   *  @param status SUCCESS, UNSUPPORTED_ATTRIBUTE, INVALID_PARAMETER, or READ_ONLY
   *  @param id the attributed identifier
   */
  void PlmeSetAttributeConfirm (LrWpanPhyEnumeration status,
                                LrWpanPibAttributeIdentifier id);

  /**
   * CSMA-CA algorithm calls back the MAC after executing channel assessment.
   *
   * \param macState indicate BUSY or IDLE channel condition
   */
  void SetLrWpanMacState (LrWpanMacState macState);

  /**
   * Get the current association status.
   *
   * \return current association status
   */
  LrWpanAssociationStatus GetAssociationStatus (void) const;

  /**
   * Set the current association status.
   *
   * \param status new association status
   */
  void SetAssociationStatus (LrWpanAssociationStatus status);

  //MAC sublayer constants
  /**
   * Length of a superframe slot in symbols. Defaults to 60 symbols in each
   * superframe slot.
   * See IEEE 802.15.4-2006, section 7.4.1, Table 85.
   */
  uint64_t m_aBaseSlotDuration;

  /**
   * Number of a superframe slots per superframe. Defaults to 16.
   * See IEEE 802.15.4-2006, section 7.4.1, Table 85.
   */
  uint64_t m_aNumSuperframeSlots;

  /**
   * Length of a superframe in symbols. Defaults to
   * aBaseSlotDuration * aNumSuperframeSlots in symbols.
   * See IEEE 802.15.4-2006, section 7.4.1, Table 85.
   */
  uint64_t m_aBaseSuperframeDuration;

  //MAC PIB attributes
  /**
   * The time that the device transmitted its last beacon frame, in symbol
   * periods. Only 24 bits used.
   * See IEEE 802.15.4-2006, section 7.4.2, Table 86.
   */
  uint64_t m_macBeaconTxTime;

  /**
   * Symbol boundary is same as m_macBeaconTxTime.
   * See IEEE 802.15.4-2006, section 7.4.2, Table 86.
   */
  uint64_t m_macSyncSymbolOffset;

  /**
   * Specification of how often the coordinator transmits its beacon.
   * 0 - 15 with 15 means no beacons are being sent.
   * See IEEE 802.15.4-2006, section 7.4.2, Table 86.
   */
  uint64_t m_macBeaconOrder;

  /**
   * The length of the active portion of the outgoing superframe, including the
   * beacon frame.
   * 0 - 15 with 15 means the superframe will not be active after the beacon.
   * See IEEE 802.15.4-2006, section 7.4.2, Table 86.
   */
  uint64_t m_macSuperframeOrder;

  /**
   * Indicates if MAC sublayer is in receive all mode. True mean accept all
   * frames from PHY.
   * See IEEE 802.15.4-2006, section 7.4.2, Table 86.
   */
  bool m_macPromiscuousMode;

  /**
   * 16 bits id of PAN on which this device is operating. 0xffff means not
   * asscoiated.
   * See IEEE 802.15.4-2006, section 7.4.2, Table 86.
   */
  uint16_t m_macPanId;

  /**
   * Sequence number added to transmitted data or MAC command frame, 00-ff.
   * See IEEE 802.15.4-2006, section 7.4.2, Table 86.
   */
  SequenceNumber8 m_macDsn;

  /**
   * The maximum number of retries allowed after a transmission failure.
   * See IEEE 802.15.4-2006, section 7.4.2, Table 86.
   */
  uint8_t m_macMaxFrameRetries;

  /**
   * Indication of whether the MAC sublayer is to enable its receiver during
   * idle periods.
   * See IEEE 802.15.4-2006, section 7.4.2, Table 86.
   */
  bool m_macRxOnWhenIdle;

  /**
   * Get the macAckWaitDuration attribute value.
   *
   * \return the maximum number symbols to wait for an acknowledgment frame
   */
  uint64_t GetMacAckWaitDuration (void) const;

  /**
   * Get the macMaxFrameRetries attribute value.
   *
   * \return the maximum number of retries
   */
  uint8_t GetMacMaxFrameRetries (void) const;

  /**
   * Set the macMaxFrameRetries attribute value.
   *
   * \param retries the maximum number of retries
   */
  void SetMacMaxFrameRetries (uint8_t retries);

  /**
   * TracedCallback signature for sent packets.
   *
   * \param [in] packet The packet.
   * \param [in] retries The number of retries.
   * \param [in] backoffs The number of CSMA backoffs.
   */
  typedef void (* SentTracedCallback)
    (Ptr<const Packet> packet, uint8_t retries, uint8_t backoffs);

  /**
   * TracedCallback signature for LrWpanMacState change events.
   *
   * \param [in] oldValue The original state value.
   * \param [in] newValue The new state value.
   * \deprecated The LrWpanMacState is now accessible as the
   * TracedValue \c MacStateValue. The \c MacState TracedCallback will
   * be removed in a future release.
   */
  typedef void (* StateTracedCallback)
  (LrWpanMacState oldState, LrWpanMacState newState);
  //AM: modified at 7/11
  //void AddedL2RoutingProtocol(Ptr<L2R_RoutingProtocol> routingProtocol);
  //L2R Protocol
  void L2R_AssignL2RProtocolForSink(bool isSink, uint16_t lqt, uint8_t tcieInterval);
  void RecieveL2RPacket (McpsDataIndicationParams params, Ptr<Packet> p);
  void L2R_SendPeriodicUpdate();
  void L2R_SendTopologyDiscovery();
  void L2R_MaxMissedTcIeMsg (uint8_t maxMissed);  
  void SetMaxQueueSize(uint16_t maxQueue);
  //AB: modified on 19/11
  Mac16Address OutputRoute();
  void L2R_Start (); 
  //AM: modified on 25/11
  void PrintRoutingTable (Ptr<Node> node, Ptr<OutputStreamWrapper> stream, Time::Unit unit = Time::S);
  //AM: modified 1/12
  uint16_t GetDepth(void) const;
  uint16_t GetPqm (void) const;
  uint32_t GetTotalPacketDroppedByQueue(void);
  uint32_t GetTotalPacketSentByNode(void) const;
  uint32_t GetTotalPacketRxByMeshRoot(void) const;
  uint16_t GetQueueSize(void) const;
  uint32_t GetArrivalRate(void) const;
  uint32_t GetAvgDelay (void);
  uint16_t GetMaxQueueSize(void) const;
  void UpdateDelay(uint64_t pId, Time t);
  uint32_t m_totalPacketSentByNode;
  void outputRoutesTree(Ptr<OutputStreamWrapper> stream);
  void OutputTree(Ptr<Packet> p, Time t,McpsDataRequestParams params);
  uint32_t GetInternalLoad() const;
  std::multimap<uint16_t, MeshRootData> m_meshRootData;
  void SetLQT(uint16_t lqt)
  {
    m_lqt = lqt;
  }
  void IncQueue()
  {
    ++m_queueSize;
  }
protected:
  // Inherited from Object.
  virtual void DoInitialize (void);
  virtual void DoDispose (void);

private:
  //AM: modified at 15/11
  bool m_isSink;
  uint16_t m_lqt;
  uint16_t m_msn;
  uint16_t m_depth;
  uint8_t m_tcieInterval;
  uint16_t m_pqm;
  uint32_t m_tcieIncr;
  Mac16Address m_rootAddress;
  Ptr<OutputStreamWrapper> m_stream;
  /// Timer to trigger periodic updates from a node
  Timer m_periodicUpdateTimer;
  L2R_RoutingTable m_routingTable;
  Timer m_printRoutingTableTimer;
  Ptr<UniformRandomVariable> m_uniformRandomVariable;
  uint32_t m_totalPacketRxByMesh;
  uint32_t m_totalPacketDroppedByNode;
  uint32_t m_totalPacketDroppedEverySecond;
  uint16_t m_maxQueueSize;
  uint32_t m_delayCountPacket;
  //std::multimap<Mac16Address, MeshRootData> m_meshRootData;
  uint32_t m_nodeId;
  std::map<uint64_t, Time> m_delayForEachPacket;
  float m_avgDelay;
  Time m_arrivalRate;
  std::queue<Time> m_arrivalRateMovingAvg;
  uint8_t m_arrivalRateComplement;
  void SendNlmMsg();
  uint32_t m_internalLoad;
  uint32_t m_queueSize;
  //std::multimap<uint16_t, MeshRootData> m_meshRootData;
  /*void SetLQT(uint16_t lqt)
  {
    m_lqt = lqt;
  } */ 
  //uint32_t m_nodeID;
  //void SetMac16();
  /**
   * Helper structure for managing transmission queue elements.
   */
  struct TxQueueElement
  {
    uint8_t txQMsduHandle; //!< MSDU Handle
    Ptr<Packet> txQPkt;    //!< Queued packet
  };

  /**
   * Send an acknowledgment packet for the given sequence number.
   *
   * \param seqno the sequence number for the ACK
   */
  void SendAck (uint8_t seqno);

  /**
   * Remove the tip of the transmission queue, including clean up related to the
   * last packet transmission.
   */
  void RemoveFirstTxQElement ();

  /**
   * Change the current MAC state to the given new state.
   *
   * \param newState the new state
   */
  void ChangeMacState (LrWpanMacState newState);

  /**
   * Handle an ACK timeout with a packet retransmission, if there are
   * retransmission left, or a packet drop.
   */
  void AckWaitTimeout (void);

  /**
   * Check for remaining retransmissions for the packet currently being sent.
   * Drop the packet, if there are no retransmissions left.
   *
   * \return true, if the packet should be retransmitted, false otherwise.
   */
  bool PrepareRetransmission (void);

  /**
   * Check the transmission queue. If there are packets in the transmission
   * queue and the MAC is idle, pick the first one and initiate a packet
   * transmission.
   */
  void CheckQueue (void);

  /**
   * The trace source fired when packets are considered as successfully sent
   * or the transmission has been given up.
   * Only non-broadcast packets are traced.
   *
   * The data should represent:
   * packet, number of retries, total number of csma backoffs
   *
   * \see class CallBackTraceSource
   */
  TracedCallback<Ptr<const Packet>, uint8_t, uint8_t > m_sentPktTrace;

  /**
   * The trace source fired when packets come into the "top" of the device
   * at the L3/L2 transition, when being queued for transmission.
   *
   * \see class CallBackTraceSource
   */
  TracedCallback<Ptr<const Packet> > m_macTxEnqueueTrace;

  /**
   * The trace source fired when packets are dequeued from the
   * L3/l2 transmission queue.
   *
   * \see class CallBackTraceSource
   */
  TracedCallback<Ptr<const Packet> > m_macTxDequeueTrace;

  /**
   * The trace source fired when packets are being sent down to L1.
   *
   * \see class CallBackTraceSource
   */
  TracedCallback<Ptr<const Packet> > m_macTxTrace;

  /**
   * The trace source fired when packets where successfully transmitted, that is
   * an acknowledgment was received, if requested, or the packet was
   * successfully sent by L1, if no ACK was requested.
   *
   * \see class CallBackTraceSource
   */
  TracedCallback<Ptr<const Packet> > m_macTxOkTrace;

  /**
   * The trace source fired when packets are dropped due to missing ACKs or
   * because of transmission failures in L1.
   *
   * \see class CallBackTraceSource
   */
  TracedCallback<Ptr<const Packet> > m_macTxDropTrace;

  /**
   * The trace source fired for packets successfully received by the device
   * immediately before being forwarded up to higher layers (at the L2/L3
   * transition).  This is a promiscuous trace.
   *
   * \see class CallBackTraceSource
   */
  TracedCallback<Ptr<const Packet> > m_macPromiscRxTrace;

  /**
   * The trace source fired for packets successfully received by the device
   * immediately before being forwarded up to higher layers (at the L2/L3
   * transition).  This is a non-promiscuous trace.
   *
   * \see class CallBackTraceSource
   */
  TracedCallback<Ptr<const Packet> > m_macRxTrace;

  /**
   * The trace source fired for packets successfully received by the device
   * but dropped before being forwarded up to higher layers (at the L2/L3
   * transition).
   *
   * \see class CallBackTraceSource
   */
  TracedCallback<Ptr<const Packet> > m_macRxDropTrace;

  /**
   * A trace source that emulates a non-promiscuous protocol sniffer connected
   * to the device.  Unlike your average everyday sniffer, this trace source
   * will not fire on PACKET_OTHERHOST events.
   *
   * On the transmit size, this trace hook will fire after a packet is dequeued
   * from the device queue for transmission.  In Linux, for example, this would
   * correspond to the point just before a device hard_start_xmit where
   * dev_queue_xmit_nit is called to dispatch the packet to the PF_PACKET
   * ETH_P_ALL handlers.
   *
   * On the receive side, this trace hook will fire when a packet is received,
   * just before the receive callback is executed.  In Linux, for example,
   * this would correspond to the point at which the packet is dispatched to
   * packet sniffers in netif_receive_skb.
   *
   * \see class CallBackTraceSource
   */
  TracedCallback<Ptr<const Packet> > m_snifferTrace;

  /**
   * A trace source that emulates a promiscuous mode protocol sniffer connected
   * to the device.  This trace source fire on packets destined for any host
   * just like your average everyday packet sniffer.
   *
   * On the transmit size, this trace hook will fire after a packet is dequeued
   * from the device queue for transmission.  In Linux, for example, this would
   * correspond to the point just before a device hard_start_xmit where
   * dev_queue_xmit_nit is called to dispatch the packet to the PF_PACKET
   * ETH_P_ALL handlers.
   *
   * On the receive side, this trace hook will fire when a packet is received,
   * just before the receive callback is executed.  In Linux, for example,
   * this would correspond to the point at which the packet is dispatched to
   * packet sniffers in netif_receive_skb.
   *
   * \see class CallBackTraceSource
   */
  TracedCallback<Ptr<const Packet> > m_promiscSnifferTrace;

  /**
   * A trace source that fires when the LrWpanMac changes states.
   * Parameters are the old mac state and the new mac state.
   *
   * \deprecated This TracedCallback is deprecated and will be
   * removed in a future release,  Instead, use the \c MacStateValue
   * TracedValue.
   */
  TracedCallback<LrWpanMacState, LrWpanMacState> m_macStateLogger;

  /**
   * The PHY associated with this MAC.
   */
  Ptr<LrWpanPhy> m_phy;

  /**
   * The CSMA/CA implementation used by this MAC.
   */
  Ptr<LrWpanCsmaCa> m_csmaCa;

  /**
   * This callback is used to notify incoming packets to the upper layers.
   * See IEEE 802.15.4-2006, section 7.1.1.3.
   */
  McpsDataIndicationCallback m_mcpsDataIndicationCallback;
  //AM: modified on 30/12
  L2rReceiveUpdateCallback m_l2rReceiveUpdateCallback;
  meshRootRxMsgCallback m_meshRxMsgCallback;
  /**
   * This callback is used to report data transmission request status to the
   * upper layers.
   * See IEEE 802.15.4-2006, section 7.1.1.2.
   */
  McpsDataConfirmCallback m_mcpsDataConfirmCallback;

  /**
   * The current state of the MAC layer.
   */
  TracedValue<LrWpanMacState> m_lrWpanMacState;

  /**
   * The current association status of the MAC layer.
   */
  LrWpanAssociationStatus m_associationStatus;

  /**
   * The packet which is currently being sent by the MAC layer.
   */
  Ptr<Packet> m_txPkt;  // XXX need packet buffer instead of single packet

  /**
   * The short address used by this MAC. Currently we do not have complete
   * extended address support in the MAC, nor do we have the association
   * primitives, so this address has to be configured manually.
   */
  Mac16Address m_shortAddress;

  /**
   * The extended address used by this MAC. Extended addresses are currently not
   * really supported.
   */
  Mac64Address m_selfExt;

  /**
   * The transmit queue used by the MAC.
   */
  std::deque<TxQueueElement*> m_txQueue;

  /**
   * The number of already used retransmission for the currently transmitted
   * packet.
   */
  uint8_t m_retransmission;

  /**
   * The number of CSMA/CA retries used for sending the current packet.
   */
  uint8_t m_numCsmacaRetry;

  /**
   * Scheduler event for the ACK timeout of the currently transmitted data
   * packet.
   */
  EventId m_ackWaitTimeout;

  /**
   * Scheduler event for a deferred MAC state change.
   */
  EventId m_setMacState;
  //AM: modified at 7/11
};

//AM: modified at 8/11
} // namespace ns3

#endif /* LR_WPAN_MAC_H */
