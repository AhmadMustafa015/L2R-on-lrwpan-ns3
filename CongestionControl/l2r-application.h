#ifndef L2R_APPLICATION_H
#define L2R_APPLICATION_H

#include "ns3/address.h"
#include "ns3/application.h"
#include "ns3/event-id.h"
#include "ns3/ptr.h"
#include "ns3/data-rate.h"
#include "ns3/traced-callback.h"
namespace ns3 {
class NetDevice;
class l2rapplication
{
//OnOffApplication
public:
  l2rapplication(Ptr<NetDevice> dev);
  ~l2rapplication();  
  int64_t AssignStreams (int64_t stream); //ToDo
  void Start(Time start);
void Stop (Time start);
private:
  /**
   * \brief Cancel all pending events.
   */
  void CancelEvents ();
  virtual void StartApplication (void);    // Called at time specified by Start
  virtual void StopApplication (void);     // Called at time specified by Stop
    /**
   * \brief Schedule the next On period start
   */
  void ScheduleStartEvent ();
  // Event handlers
  /**
   * \brief Start an On period
   */
  void StartSending ();
  /**
   * \brief Start an Off period
   */
  void StopSending ();
  /**
   * \brief Send a packet
   */
  void SendPacket ();
  /**
   * \brief Schedule the next packet transmission
   */
  void ScheduleNextTx ();
  /**
   * \brief Schedule the next Off period start
   */
  void ScheduleStopEvent ();
  /**
   * \brief Handle a Connection Succeed event
   * \param socket the connected socket
   */
  void SetMaxBytes (uint64_t maxBytes); //ToDo

  bool            m_connected;    //!< True if connected
  Ptr<RandomVariableStream>  m_onTime;       //!< rng for On Time
  Ptr<RandomVariableStream>  m_offTime;      //!< rng for Off Time
  DataRate        m_cbrRate;      //!< Rate that data is generated
  DataRate        m_cbrRateFailSafe;      //!< Rate that data is generated (check copy)
  uint32_t        m_pktSize;      //!< Size of packets
  uint32_t        m_residualBits; //!< Number of generated, but not sent, bits
  Time            m_lastStartTime; //!< Time last packet sent
  uint64_t        m_maxBytes;     //!< Limit total number of bytes sent
  uint64_t        m_totBytes;     //!< Total bytes sent so far
  EventId         m_startStopEvent;     //!< Event id for next start or stop event
  EventId         m_sendEvent;    //!< Event id of pending "send packet" event
  Ptr<NetDevice>  m_netDevice;
};

} // namespace ns3

#endif /* ONOFF_APPLICATION_H */
