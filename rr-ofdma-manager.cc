/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2019 Universita' degli Studi di Napoli Federico II
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
 * Author: Stefano Avallone <stavallo@unina.it>
 */

#include "ns3/log.h"
#include "rr-ofdma-manager.h"
#include "wifi-ack-policy-selector.h"
#include "wifi-phy.h"
#include <utility>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <cmath>
#include <ctime>
#include <cstdlib>


namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("RrOfdmaManager");

NS_OBJECT_ENSURE_REGISTERED (RrOfdmaManager);

TypeId
RrOfdmaManager::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::RrOfdmaManager")
    .SetParent<OfdmaManager> ()
    .SetGroupName ("Wifi")
    .AddConstructor<RrOfdmaManager> ()
    .AddAttribute ("NStations",
                   "The maximum number of stations that can be granted an RU in the MU DL OFDMA transmission",
                   UintegerValue (4),
                   MakeUintegerAccessor (&RrOfdmaManager::m_nStations),
                   MakeUintegerChecker<uint8_t> (1, 74))
    .AddAttribute ("ForceDlOfdma",
                   "If enabled, return DL_OFDMA even if no DL MU PPDU could be built.",
                   BooleanValue (false),
                   MakeBooleanAccessor (&RrOfdmaManager::m_forceDlOfdma),
                   MakeBooleanChecker ())
    .AddAttribute ("EnableUlOfdma",
                   "If enabled, return UL_OFDMA if DL_OFDMA was returned the previous time.",
                   BooleanValue (true),
                   MakeBooleanAccessor (&RrOfdmaManager::m_enableUlOfdma),
                   MakeBooleanChecker ())
    .AddAttribute ("UlPsduSize",
                   "The size in bytes of the solicited PSDU (to be sent in an HE TB PPDU)",
                   UintegerValue (500),
                   MakeUintegerAccessor (&RrOfdmaManager::m_ulPsduSize),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("ChannelBw",
                   "For TESTING only",
                   UintegerValue (20),
                   MakeUintegerAccessor (&RrOfdmaManager::m_bw),
                   MakeUintegerChecker<uint16_t> (5, 160))
  ;
  return tid;
}

RrOfdmaManager::RrOfdmaManager ()
  : m_startStation (0)
{
  NS_LOG_FUNCTION (this);
}

RrOfdmaManager::~RrOfdmaManager ()
{
  NS_LOG_FUNCTION_NOARGS ();
}


 /**
   * Compute the TX vector and the TX params for a DL MU transmission assuming
   * the given list of receiver stations, the given RU type and the given type
   * of acknowledgment sequence.
   *
   * \param staList the list of receiver stations for the DL MU transmission
   * \param ruType the RU type
   * \param dlMuAckSequence the ack sequence type
   */

void
RrOfdmaManager::InitTxVectorAndParams (std::map<Mac48Address, DlPerStaInfo> staList,
                                         HeRu::RuType ruType, DlMuAckSequenceType dlMuAckSequence)
{

	auto mp=staList.begin();
          while(mp!=staList.end()){
          	std::cout<<"\n  Dlofdma in InitTxVectorAndParams"<<mp->first;
          	mp++;
          }

  std::cout<<"\n start: InitTxVectorAndParams";
  NS_LOG_FUNCTION (this);
//NS_LOG_FUNCTION("ru:::::::::"<<ruType);
  m_txVector = WifiTxVector ();
  m_txVector.SetPreambleType (WIFI_PREAMBLE_HE_MU);
  m_txVector.SetChannelWidth (m_low->GetPhy ()->GetChannelWidth ());
  m_txVector.SetGuardInterval (m_low->GetPhy ()->GetGuardInterval ().GetNanoSeconds ());
  m_txVector.SetTxPowerLevel (GetWifiRemoteStationManager ()->GetDefaultTxPowerLevel ());
  m_txParams = MacLowTransmissionParameters ();
  m_txParams.SetDlMuAckSequenceType (dlMuAckSequence);


  Ptr<WifiMacQueueItem> mpdu = Copy (m_mpdu);

std::cout<<"\n inside InitTxVectorAndParams: before for loop\n";

/////////////////////////////////////////Reshan
unsigned int i=0; 
  for (auto& sta : staList)
    {
      mpdu->GetHeader ().SetAddr1 (sta.first);
      // Get the TX vector used to transmit single user frames to the receiver
      // station (the RU index will be assigned by ComputeDlOfdmaInfo)
      WifiTxVector suTxVector = m_low->GetDataTxVector (mpdu);
      NS_LOG_DEBUG ("Adding STA with AID=" << sta.second.aid << " and TX mode="
                    << suTxVector.GetMode () << " to the TX vector");


///////ReshanFaraz///////////////////
std::cout<<"\n inside InitTxVectorAndParams: before your code\n";

if(finalRUAlloc.size()!=0){
  std::cout<<"mappedRuAllocated-----"<<mappedRuAllocated[i];

  m_txVector.SetHeMuUserInfo (sta.second.aid, {{false,mappedRuAllocated[i], 1}, suTxVector.GetMode (), suTxVector.GetNss ()});
  i++;
}
else      
  m_txVector.SetHeMuUserInfo (sta.second.aid, {{false, ruType, 1}, suTxVector.GetMode (), suTxVector.GetNss ()});
//////////////////////////////

      // Add the receiver station to the appropriate list of the TX params
      Ptr<QosTxop> txop = m_qosTxop[QosUtilsMapTidToAc (sta.second.tid)];
      BlockAckReqType barType = txop->GetBaAgreementEstablished (sta.first, sta.second.tid)
                                ? txop->GetBlockAckReqType (sta.first, sta.second.tid)
                                : BlockAckReqType::COMPRESSED;
      BlockAckType baType = txop->GetBaAgreementEstablished (sta.first, sta.second.tid)
                            ? txop->GetBlockAckType (sta.first, sta.second.tid)
                            : BlockAckType::COMPRESSED;

      if (dlMuAckSequence == DlMuAckSequenceType::DL_SU_FORMAT)
        {
          // Enable BAR/BA exchange for all the receiver stations
          m_txParams.EnableBlockAckRequest (sta.first, barType, baType);
        }
      else if (dlMuAckSequence == DlMuAckSequenceType::DL_MU_BAR)
        {
          // Send a MU-BAR to all the stations
          m_txParams.EnableBlockAckRequest (sta.first, barType, baType);
        }
      else if (dlMuAckSequence == DlMuAckSequenceType::DL_AGGREGATE_TF)
        {
          // Expect to receive a Block Ack from all the stations
          m_txParams.EnableBlockAck (sta.first, baType);
        }
    }
}





/**
   * Select the format of the next transmission, assuming that the AP gained
   * access to the channel to transmit the given MPDU.
   *
   * \param mpdu the MPDU the AP intends to transmit
   * \return the format of the next transmission
   */

OfdmaTxFormat
RrOfdmaManager::SelectTxFormat (Ptr<const WifiMacQueueItem> mpdu)
{
  // --- for TESTING only ---
//   for (uint8_t i = 1; i <= m_nStations; i++)
//     {
//       DlPerStaInfo info {i, 0};
//       m_staInfo.push_back (std::make_pair (Mac48Address::Allocate (), info));
//     }
//   return OfdmaTxFormat::DL_OFDMA;
  // --- --- ---
  NS_LOG_FUNCTION (this << *mpdu);
  NS_ASSERT (mpdu->GetHeader ().IsQosData ());



  if (m_enableUlOfdma && GetTxFormat () == DL_OFDMA)
    {
      // check if an UL OFDMA transmission is possible after a DL OFDMA transmission
      NS_ABORT_MSG_IF (m_ulPsduSize == 0, "The UlPsduSize attribute must be set to a non-null value");

      Ptr<QosTxop> txop = m_qosTxop[QosUtilsMapTidToAc (mpdu->GetHeader ().GetQosTid ())];
      m_ulMuAckSequence = txop->GetAckPolicySelector ()->GetAckSequenceForUlMu ();
      MacLowTransmissionParameters params;
      params.SetUlMuAckSequenceType (m_ulMuAckSequence);
      BlockAckType baType;

      if (m_ulMuAckSequence == UL_MULTI_STA_BLOCK_ACK)
        {
          baType = BlockAckType::MULTI_STA;
          for (auto& userInfo : m_txVector.GetHeMuUserInfoMap ())
            {
              auto addressIt = m_apMac->GetStaList ().find (userInfo.first);
              if (addressIt != m_apMac->GetStaList ().end ())
                {
                  baType.m_bitmapLen.push_back (32);
                  params.EnableBlockAck (addressIt->second, baType);
                }
              else
                {
                  NS_LOG_WARN ("Maybe station with AID=" << userInfo.first << " left the BSS since the last MU DL transmission?");
                }
            }
        }
      else
        {
          NS_FATAL_ERROR ("Sending Block Acks in an MU DL PPDU is not supported yet");
        }

      CtrlTriggerHeader trigger (TriggerFrameType::BASIC_TRIGGER, m_txVector);

      // compute the maximum amount of time that can be granted to stations.
      // This value is limited by the max PPDU duration
      Time maxDuration = GetPpduMaxTime (m_txVector.GetPreambleType ());

      // compute the time required by stations based on the buffer status reports, if any
      uint32_t maxBufferSize = 0;

      for (auto& userInfo : m_txVector.GetHeMuUserInfoMap ())
        {
          auto addressIt = m_apMac->GetStaList ().find (userInfo.first);
          if (addressIt != m_apMac->GetStaList ().end ())
            {
              uint8_t queueSize = m_apMac->GetMaxBufferStatus (addressIt->second);
              if (queueSize == 255)
                {
                  NS_LOG_DEBUG ("Buffer status of station " << addressIt->second << " is unknown");
                  maxBufferSize = std::max (maxBufferSize, m_ulPsduSize);
                }
              else if (queueSize == 254)
                {
                  NS_LOG_DEBUG ("Buffer status of station " << addressIt->second << " is not limited");
                  maxBufferSize = 0xffffffff;
                  break;
                }
              else
                {
                  NS_LOG_DEBUG ("Buffer status of station " << addressIt->second << " is " << +queueSize);
                  maxBufferSize = std::max (maxBufferSize, static_cast<uint32_t> (queueSize * 256));
                }
            }
          else
            {
              NS_LOG_WARN ("Maybe station with AID=" << userInfo.first << " left the BSS since the last MU DL transmission?");
            }
        }

      // if the maximum buffer size is 0, skip UL OFDMA and proceed with trying DL OFDMA
      if (maxBufferSize > 0)
        {
          // if we are within a TXOP, we have to consider the response time and the
          // remaining TXOP duration
          if (txop->GetTxopLimit ().IsStrictlyPositive ())
            {
              // we need to define the HE TB (trigger based) PPDU duration in order to compute the response to
              // the Trigger Frame. Let's use 1 ms for this purpose. We'll subtract it later.
              uint16_t length = WifiPhy::ConvertHeTbPpduDurationToLSigLength (MilliSeconds (1),
                                                                              m_low->GetPhy ()->GetFrequency ());
              trigger.SetUlLength (length);

              Ptr<Packet> packet = Create<Packet> ();
              packet->AddHeader (trigger);
              WifiMacHeader hdr;
              hdr.SetType (WIFI_MAC_CTL_TRIGGER);
              hdr.SetAddr1 (Mac48Address::GetBroadcast ());
              Ptr<WifiMacQueueItem> item = Create<WifiMacQueueItem> (packet, hdr);

              Time response = m_low->GetResponseDuration (params, m_txVector, item);

              // Add the time to transmit the Trigger Frame itself
              WifiTxVector txVector = GetWifiRemoteStationManager ()->GetRtsTxVector (hdr.GetAddr1 (), &hdr, packet);

              response += m_low->GetPhy ()->CalculateTxDuration (item->GetSize (), txVector,
                                                                 m_low->GetPhy ()->GetFrequency ());

              // Subtract the duration of the HE TB PPDU
              response -= WifiPhy::ConvertLSigLengthToHeTbPpduDuration (length, m_txVector,
                                                                        m_low->GetPhy ()->GetFrequency ());

              if (response > txop->GetTxopRemaining ())
                {
                  // an UL OFDMA transmission is not possible. Reset m_staInfo and return DL_OFDMA.
                  // In this way, no transmission will occur now and the next time we will try again
                  // performing an UL OFDMA transmission.
                  NS_LOG_DEBUG ("Remaining TXOP duration is not enough for UL MU exchange");
                  m_staInfo.clear ();
                  return DL_OFDMA;
                }

              maxDuration = Min (maxDuration, txop->GetTxopRemaining () - response);
            }

          Time bufferTxTime = m_low->GetPhy ()->CalculateTxDuration (maxBufferSize, m_txVector,
                                                                     m_low->GetPhy ()->GetFrequency (),
                                                                     trigger.begin ()->GetAid12 ());
          if (bufferTxTime < maxDuration)
            {
              // the maximum buffer size can be transmitted within the allowed time
              maxDuration = bufferTxTime;
            }
          else
            {
              // maxDuration may be a too short time. If it does not allow to transmit
              // at least m_ulPsduSize bytes, give up the UL MU transmission for now
              Time minDuration = m_low->GetPhy ()->CalculateTxDuration (m_ulPsduSize, m_txVector,
                                                                        m_low->GetPhy ()->GetFrequency (),
                                                                        trigger.begin ()->GetAid12 ());
              if (maxDuration < minDuration)
                {
                  // maxDuration is a too short time. Reset m_staInfo and return DL_OFDMA.
                  // In this way, no transmission will occur now and the next time we will try again
                  // performing an UL OFDMA transmission.
                  NS_LOG_DEBUG ("Available time " << maxDuration << " is too short");
                  m_staInfo.clear ();
                  return DL_OFDMA;
                }
            }

          // maxDuration is the time to grant to the stations. Store it in the TX vector
          NS_LOG_DEBUG ("HE TB PPDU duration: " << maxDuration.ToDouble (Time::MS));
          uint16_t length = WifiPhy::ConvertHeTbPpduDurationToLSigLength (maxDuration,
                                                                          m_low->GetPhy ()->GetFrequency ());
          m_txVector.SetLength (length);
          m_txParams = params;
          return UL_OFDMA;
        }
    }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // get the list of associated stations ((AID, MAC address) pairs)
  const std::map<uint16_t, Mac48Address>& staList = m_apMac->GetStaList ();
  auto startIt = staList.find (m_startStation);

  // This may be the first invocation or the starting station left
  if (startIt == staList.end ())
    {
      startIt = staList.begin ();
      m_startStation = startIt->first;
std::cout<<"SelectTxFormat: if (startIt == staList.end ()),  start station"<<m_startStation<<"\n";

    }


  uint8_t currTid = mpdu->GetHeader ().GetQosTid ();
  AcIndex primaryAc = QosUtilsMapTidToAc (currTid);
  m_staInfo.clear ();

  // If the primary AC holds a TXOP, we can select a station as a receiver of
  // the MU PPDU only if the AP has frames to send to such station that fit into
  // the remaining TXOP time. To this end, we need to determine the type of ack
  // sequence and the time it takes. To compute the latter, we can call the
  // MacLow::GetResponseDuration () method, which requires TX vector and TX params.
  // Our best guess at this stage is that the AP has frames to send to all the
  // associated stations and hence we initialize the TX vector and the TX params
  // by considering the starting station and those that immediately follow it in
  // the list of associated stations.


  std::size_t count = m_nStations;


  HeRu::RuType ruType = GetNumberAndTypeOfRus (m_low->GetPhy ()->GetChannelWidth (), count);



  NS_ASSERT (count >= 1);

  std::map<Mac48Address, DlPerStaInfo> guess;
  auto staIt = startIt;
  do
    {
      guess[staIt->second] = {staIt->first, currTid};
      if (++staIt == staList.end ())
        {
          staIt = staList.begin ();
        }
    } while (guess.size () < count && staIt != startIt);


  Ptr<WifiAckPolicySelector> ackSelector = m_qosTxop[primaryAc]->GetAckPolicySelector ();
  NS_ASSERT (ackSelector != 0);
  m_dlMuAckSequence = ackSelector->GetAckSequenceForDlMu ();
  
//This call doesn't matter for our use case because this function is called again from compute dl ofdma which again finds the tx vector and params
InitTxVectorAndParams (guess, ruType, m_dlMuAckSequence);

  // if the AC owns a TXOP, compute the time available for the transmission of data frames
  Time txopLimit = Seconds (0);
  if (m_qosTxop[primaryAc]->GetTxopLimit ().IsStrictlyPositive ())
    {
      // TODO Account for MU-RTS/CTS when implemented
      CtrlTriggerHeader trigger;

      if (m_dlMuAckSequence == DlMuAckSequenceType::DL_MU_BAR
          || m_dlMuAckSequence == DlMuAckSequenceType::DL_AGGREGATE_TF)
        {
          // Need to prepare the MU-BAR to correctly get the response time
          trigger = GetTriggerFrameHeader (m_txVector, 5);
          trigger.SetUlLength (m_low->CalculateUlLengthForBlockAcks (trigger, m_txParams));
        }
      txopLimit = m_qosTxop[primaryAc]->GetTxopRemaining () - GetResponseDuration (m_txParams, m_txVector, trigger);

      if (txopLimit.IsNegative ())
        {
          if (m_forceDlOfdma)
            {
              NS_LOG_DEBUG ("Not enough TXOP remaining time: return DL_OFDMA with empty set of receiver stations");
              return OfdmaTxFormat::DL_OFDMA;
            }
          NS_LOG_DEBUG ("Not enough TXOP remaining time: return NON_OFDMA");
          return OfdmaTxFormat::NON_OFDMA;
        }
    }


/////////////////////////////////////////////////////////////////
dataStaPair1.clear(); 
v_QosType.clear();
v_powerLevel.clear();
v_dataStaPair.clear();
finalStaPairIndex.clear();


  do
    {
      NS_LOG_DEBUG ("Next candidate STA (MAC=" << startIt->second << ", AID=" << startIt->first << ")");
      // check if the AP has at least one frame to be sent to the current station
      for (uint8_t tid : std::initializer_list<uint8_t> {currTid, 1, 2, 0, 3, 4, 5, 6, 7})
        {
          AcIndex ac = QosUtilsMapTidToAc (tid);
          // check that a BA agreement is established with the receiver for the
          // considered TID, since ack sequences for DL MU PPDUs require block ack
          if (ac >= primaryAc && m_qosTxop[ac]->GetBaAgreementEstablished (startIt->second, tid))
            {
              mpdu = m_qosTxop[ac]->PeekNextFrame (tid, startIt->second);

              // we only check if the first frame of the current TID meets the size
              // and duration constraints. We do not explore the queues further.
              if (mpdu != 0)
                {
                  // Use a temporary TX vector including only the STA-ID of the
                  // candidate station to check if the MPDU meets the size and time limits.
                  // An RU of the computed size is tentatively assigned to the candidate
                  // station, so that the TX duration can be correctly computed.
                  WifiTxVector suTxVector = m_low->GetDataTxVector (mpdu),
                               muTxVector;

                  muTxVector.SetPreambleType (WIFI_PREAMBLE_HE_MU);
                  muTxVector.SetChannelWidth (m_low->GetPhy ()->GetChannelWidth ());
                  muTxVector.SetGuardInterval (m_low->GetPhy ()->GetGuardInterval ().GetNanoSeconds ());
                  muTxVector.SetHeMuUserInfo (startIt->first,
                                              {{false, ruType, 1}, suTxVector.GetMode (), suTxVector.GetNss ()});

                  if (m_low->IsWithinSizeAndTimeLimits (mpdu, muTxVector, 0, txopLimit))
                    {

                      int type_Qos;
                      if((startIt->second=="00:00:00:00:00:01") || (startIt->second=="00:00:00:00:00:05") || (startIt->second=="00:00:00:00:00:09") || (startIt->second=="00:00:00:00:00:0d") || (startIt->second=="00:00:00:00:00:11") || (startIt->second=="00:00:00:00:00:15") || (startIt->second=="00:00:00:00:00:19") || (startIt->second=="00:00:00:00:00:1d")){
                        std::cout<<"\n Inside Type Qos 1";
                        type_Qos=1;
                      }
                      else if((startIt->second=="00:00:00:00:00:03") || (startIt->second=="00:00:00:00:00:07") || (startIt->second=="00:00:00:00:00:0b") || (startIt->second=="00:00:00:00:00:0e") || (startIt->second=="00:00:00:00:00:13") || (startIt->second=="00:00:00:00:00:17") || (startIt->second=="00:00:00:00:00:1b") || (startIt->second=="00:00:00:00:00:1f"))
                      {
                        std::cout<<"\n Inside Type Qos 3";
                        type_Qos=3;
                      }
                      else if((startIt->second=="00:00:00:00:00:04") || (startIt->second=="00:00:00:00:00:08") ||(startIt->second=="00:00:00:00:00:0c") || (startIt->second=="00:00:00:00:00:0f") || (startIt->second=="00:00:00:00:00:14") || (startIt->second=="00:00:00:00:00:18") || (startIt->second=="00:00:00:00:00:1c") || (startIt->second=="00:00:00:00:00:20"))
                      {
                        std::cout<<"\n Inside Type Qos 2";
                        type_Qos=2;
                      } 
                      else{
                        std::cout<<"\n Inside Type Qos Else";
                        type_Qos=4;
                      }
                  

                      v_QosType.push_back(type_Qos);
                       v_dataStaPair.push_back(mpdu->GetPacket()->GetSize());

  //        
                      // the frame meets the constraints, add the station to the list
                      NS_LOG_DEBUG ("Adding candidate STA (MAC=" << startIt->second << ", AID="
                                    << startIt->first << ") TID=" << +tid);
                      DlPerStaInfo info {startIt->first, tid};
                      m_staInfo.push_back (std::make_pair (startIt->second, info));
                      break;    // terminate the for loop
                    }
                }
              else
                {
                  NS_LOG_DEBUG ("No frames to send to " << startIt->second << " with TID=" << +tid);
                }
            }
        }

      // move to the next station in the map
      startIt++;
      if (startIt == staList.end ())
        {
          startIt = staList.begin ();
        }
    } while (m_staInfo.size () < m_nStations && startIt->first != m_startStation);

  if (m_staInfo.empty ())
    {
      if (m_forceDlOfdma)
        {
          NS_LOG_DEBUG ("The AP does not have suitable frames to transmit: return DL_OFDMA with empty set of receiver stations");
          return OfdmaTxFormat::DL_OFDMA;
        }
      NS_LOG_DEBUG ("The AP does not have suitable frames to transmit: return NON_OFDMA");
      return OfdmaTxFormat::NON_OFDMA;
    }






  m_startStation = startIt->first;
  return OfdmaTxFormat::DL_OFDMA;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

 /**
   * Given the channel bandwidth and the number of stations candidate for being
   * assigned an RU, maximize the number of candidate stations that can be assigned
   * an RU subject to the constraint that all the stations must be assigned an RU
   * of the same size (in terms of number of tones).
   *
   * \param bandwidth the channel bandwidth in MHz
   * \param nStations the number of candidate stations. On return, it is set to
   *                  the number of stations that are assigned an RU
   * \return the RU type
   */

HeRu::RuType
RrOfdmaManager::GetNumberAndTypeOfRus (uint16_t bandwidth, std::size_t& nStations) const
{
  NS_LOG_FUNCTION (this);

NS_LOG_FUNCTION("no.of sta, count value:::::::::::::::::::::::::::::::::::::;; "<<nStations);
  HeRu::RuType ruType;
  uint8_t nRusAssigned = 0;

  // iterate over all the available RU types
  for (auto& ru : HeRu::m_heRuSubcarrierGroups)
    {
      if (ru.first.first == bandwidth && ru.second.size () <= nStations)
        {
          ruType = ru.first.second;
          nRusAssigned = ru.second.size ();
          break;
        }
      else if (bandwidth == 160 && ru.first.first == 80 && (2 * ru.second.size () <= nStations))
        {
          ruType = ru.first.second;
          nRusAssigned = 2 * ru.second.size ();
          break;
        }
    }
  if (nRusAssigned == 0)
    {
      NS_ASSERT (bandwidth == 160 && nStations == 1);
      nRusAssigned = 1;
      ruType = HeRu::RU_2x996_TONE;
    }

NS_LOG_FUNCTION("--: "<<ruType);
  nStations = nRusAssigned;
  return ruType;
}



double RrOfdmaManager::type_of_App(int type_of_Application)
{
  std::cout<<"\ncalculating Type of Application";
  double a_i=0;
  switch(type_of_Application){
    case 1:
    a_i=10;
    
    break;

    case 2:
    a_i=5.4927;
    
    break;

    case 3:
    a_i=2.3299;
    
    break;

    case 4:
    a_i=1.50515;
     break;
    default:
    break;
  }
  return a_i;
    

}



static double returnRate(int rand_mcs,uint16_t bw) {
  if(bw==20){
  switch(rand_mcs) {
    case 1: return 8.1;     //BPSK 1/2
    case 2: return 16.3;    //QPSK 1/2
    case 3: return 24.4;    //QPSK 3/4
    case 4: return 32.5;    //16-QAM 1/2
    case 5: return 48.8;    //16-QAM 3/4    
    case 6: return 65.0;    //64-QAM 2/3
    case 7: return 73.1;    //64-QAM 3/4
    case 8: return 81.3;    //64-QAM 5/6
    case 9: return 97.5;    //256-QAM 3/4
    case 10: return 108.3;    //256-QAM 5/6
    case 11: return 129.0;    //1024-QAM 3/4
    case 12: return 135.4;    //1024-QAM 5/6  
    default: return 0.0;
  }
}
else{
  switch(rand_mcs) {
    case 1: return 16.3;    //BPSK 1/2
    case 2: return 32.5;    //QPSK 1/2
    case 3: return 48.8;    //QPSK 3/4
    case 4: return 65.0;    //16-QAM 1/2
    case 5: return 97.5;    //16-QAM 3/4    
    case 6: return 130.0;   //64-QAM 2/3
    case 7: return 146.3;   //64-QAM 3/4
    case 8: return 162.5;   //64-QAM 5/6
    case 9: return 195.0;   //256-QAM 3/4
    case 10: return 216.7;    //256-QAM 5/6 
    case 11: return 243.8;    //1024-QAM 3/4
    case 12: return 270.8;    //1024-QAM 5/6
    default: return 0.0;
  }
}
}

double RrOfdmaManager::timeReq(uint32_t dataSize,int mcs)
{
    if(dataSize==0)
        return 0;
    double symbols=0.0;  
    uint32_t bits=dataSize*8;
 std::cout<<"\n Inside Time Req";
   double encodeingRate=5.0/6.0;
int mcs_QAM=2;
switch(mcs){

case 0: 
encodeingRate=1.0/2.0;
mcs_QAM=2;
break;

case 1: 
encodeingRate=1.0/2.0;
mcs_QAM=4;
break;

case 2: 
encodeingRate=3.0/4.0;
mcs_QAM=4;
break;

case 3: 
encodeingRate=1.0/2.0;
mcs_QAM=16;
break;

case 4: 
encodeingRate=3.0/4.0;
mcs_QAM=16;
break;

case 5: 
encodeingRate=2.0/3.0;
mcs_QAM=64;
break;

case 6: 
encodeingRate=3.0/4.0;
mcs_QAM=64;
break;

case 7: 
encodeingRate=5.0/6.0;
mcs_QAM=64;
break;

case 8: 
encodeingRate=3.0/4.0;
mcs_QAM=256;
break;

case 9: 
encodeingRate=5.0/6.0;
mcs_QAM=256;
break;

case 10: 
encodeingRate=3.0/4.0;
mcs_QAM=1024;
break;

case 11: 
encodeingRate=5.0/6.0;
mcs_QAM=1024;
break;

default: 
break;

}
    double bitsPerSec=log2f(mcs_QAM)*encodeingRate*(242);

    symbols=bits/bitsPerSec;


    return symbols*0.0000136; //guard interval + symbol duration = 0.0000136sec
}



double RrOfdmaManager::averageChannelcapacity(int mcs1)
{
  double avgThrohput=0;
  
    for(unsigned int i=0;i<v_dataStaPair.size();i++)
    {
        avgThrohput+= (v_dataStaPair[i]/timeReq(v_dataStaPair[i],mcs1));
    }
    return log2((avgThrohput/v_dataStaPair.size()));



}

WifiSpectrumBand
RrOfdmaManager::GetBand (uint16_t bandWidth, uint8_t /*bandIndex*/)
{
  WifiSpectrumBand band;
  band.first = 0;
  band.second = 0;
  return band;
}



void RrOfdmaManager::RuAlloc(int number_of_clients,uint16_t m_bw)
{

  std::cout<<"\n Inside RU Alloca\n "<<number_of_clients;
  //std::vector<int> RuSet;
if (m_bw==20){
  if(number_of_clients>0){
switch(number_of_clients){
  case 1:
  finalRUAlloc.push_back(242);
  break;

  case 2:
  finalRUAlloc.push_back(106);
  finalRUAlloc.push_back(106);
 // RuSet.push_back(26);
  break;

  case 3:
  finalRUAlloc.push_back(106);
  finalRUAlloc.push_back(106);
  finalRUAlloc.push_back(26);
  break;

  case 4:
  finalRUAlloc.push_back(106);
  finalRUAlloc.push_back(52);
  finalRUAlloc.push_back(52);
  finalRUAlloc.push_back(26);
  break;

  case 5:
  finalRUAlloc.push_back(106);
  finalRUAlloc.push_back(52);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
 break;

case 6:
  finalRUAlloc.push_back(52);
  finalRUAlloc.push_back(52);
  finalRUAlloc.push_back(52);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26); // other combination are also there
  break;

  case 7:
  finalRUAlloc.push_back(52);
  finalRUAlloc.push_back(52);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  break;

  case 8:
  finalRUAlloc.push_back(52);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  break;


  default:
 for(int i=0;i<9;i++){
  finalRUAlloc.push_back(26);
 }
break;

}

}
else{
  std::cout<<"No client to allocate Resources";
  

}
}
else
{if(number_of_clients>0){
switch(number_of_clients){
  case 1:
  finalRUAlloc.push_back(484);
  break;

  case 2:
  finalRUAlloc.push_back(242);
  finalRUAlloc.push_back(242);
  break;

  case 3:
  finalRUAlloc.push_back(242);
  finalRUAlloc.push_back(106);
  finalRUAlloc.push_back(106);
//finalRUAlloc.push_back(26);
  break;

  case 4:
finalRUAlloc.push_back(242);
  finalRUAlloc.push_back(106);
  finalRUAlloc.push_back(106);
  finalRUAlloc.push_back(26);
    break;

  case 5:
finalRUAlloc.push_back(242);
  finalRUAlloc.push_back(106);
  finalRUAlloc.push_back(52);
  finalRUAlloc.push_back(52);
  finalRUAlloc.push_back(26);
  
 break;

case 6:
finalRUAlloc.push_back(242);
  finalRUAlloc.push_back(106);
  finalRUAlloc.push_back(52);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26); 


  break;

  case 7:
 finalRUAlloc.push_back(242);
  finalRUAlloc.push_back(52);
finalRUAlloc.push_back(52);
  finalRUAlloc.push_back(52);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  
  break;


 case 8:
   finalRUAlloc.push_back(242);
  finalRUAlloc.push_back(52);
finalRUAlloc.push_back(52);
finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
 
  
  break;
 case 9:
   finalRUAlloc.push_back(106);
 finalRUAlloc.push_back(106);
  finalRUAlloc.push_back(106);
finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  break;
 case 10:
   finalRUAlloc.push_back(106);
 finalRUAlloc.push_back(106);
  finalRUAlloc.push_back(52);
finalRUAlloc.push_back(52);
finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  break;
 case 11:
    finalRUAlloc.push_back(106);
 finalRUAlloc.push_back(106);
  finalRUAlloc.push_back(52);
finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  break;
 case 12:
    finalRUAlloc.push_back(106);
 finalRUAlloc.push_back(106);
  finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  break;
 case 13:
   finalRUAlloc.push_back(106);
 finalRUAlloc.push_back(52);
finalRUAlloc.push_back(52);
  finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  break;
 case 14:
    finalRUAlloc.push_back(106);
 finalRUAlloc.push_back(52);
  finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  break;
 case 15:
finalRUAlloc.push_back(106);
  finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  break;
 case 16:
finalRUAlloc.push_back(52);
finalRUAlloc.push_back(52);
  finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  break;

case 17:
finalRUAlloc.push_back(52);
  finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  break;


  default:
 for(int i=0;i<18;i++){
  finalRUAlloc.push_back(26);
 }
break;

}

}
else{
  std::cout<<"No client to allocate Resources";
  

}

}}














void RrOfdmaManager::mlwdf(){


int lower=1,upper=11;
int count=v_dataStaPair.size();

  random_MCS.clear();
for(int i=0;i<count;i++){
  int num= (rand()%(upper-lower+1))+lower;
  random_MCS.push_back(num);

}
uint16_t bw = m_low->GetPhy ()->GetChannelWidth ();


  std::cout<<"inside Largest_Weighted_First before clearing vectors";



 std::cout<<"inside Largest_Weighted_First before calculating (loop)";

for(unsigned int i = 0; i<v_dataStaPair.size();i++){
  //get type of application and calucate what is the  Pr {W i > T i } £ d i
  
  double a_i_i=type_of_App(v_QosType[i]) ;   ////=log(delta)/Time
 

  double ratee=returnRate(10,bw);
  std::cout<<"\n rateeeeee-------"<<ratee;
  double avgThrohput=averageChannelcapacity(10);

  final_cost=((v_dataStaPair[i]*a_i_i*ratee)/avgThrohput);

  std::cout<<"\n Before dataStaPair1 inserting";
 dataStaPair1.push_back(std::make_pair(final_cost,2));
 
 
}














std::cout<<"\n After While Loop in LWDF";
//auto x=dataStaPair1.begin();
std::cout<<"\n Before dataStaPair1 Loop fofr finalStaPairIndex";
int zz = dataStaPair1.size();
std::cout<<"\n"<<zz;

int lol[zz];
for(int i=0;i<zz;i++){
	lol[i]=i;
}

int countt=0;
finalStaPairIndex.clear();
for(int x=0;x<zz;x++)
{
  finalStaPairIndex.push_back(std::make_pair(dataStaPair1[x].first,countt));
  //zz--;

  countt++;
}


std::cout<<"\n \n Before Sorting";
int n=finalStaPairIndex.size();


for(unsigned int i=0;i<finalStaPairIndex.size();i++){
	std::cout<<"\n\n Data Before Sorting==================== "<<finalStaPairIndex[i].first<<"Second==========="<<finalStaPairIndex[i].second;
	std::cout<<"\n\n LOL===="<<lol[i];
}

std::cout<<"\n finalStaPairIndex Size"<<n;


    int counter, counter1,key1;
    double key;  //i=counter, j=counter1
    for (counter = 1; counter < n; counter++) 
    {  
        key = finalStaPairIndex[counter].first;  
		key1= lol[counter];

        counter1 = counter - 1;  
        std::cout<<"\n \n inside insertion sort";
        /* Move elements of arr[0..i-s1], that are  
        greater than key, to one position ahead  
        of their current position */
        while (counter1 >= 0 && finalStaPairIndex[counter1].first < key) 
        {  
            finalStaPairIndex[counter1+1].first = finalStaPairIndex[counter1].first;
            lol[counter1+1]=lol[counter1] ; 
            counter1 = counter1 - 1;  


        }  
        finalStaPairIndex[counter1+1].first = key;  
        lol[counter1+1]=key1;
    }  


std::cout<<"\n \n After  Sorting";
for(unsigned int i=0;i<finalStaPairIndex.size();i++){
	std::cout<<"\n\n Data After Sorting==================== "<<finalStaPairIndex[i].first<<"Second==========="<<finalStaPairIndex[i].second;
	std::cout<<"\n\n LOL After Sorting=========="<<lol[i];
}

int noOfSTA=finalStaPairIndex.size();
int temp2=noOfSTA;
int temp4=noOfSTA;

std::cout<<"\n \n Before RuAlloc Call ";
RuAlloc(noOfSTA,bw); 
for(unsigned int j=0;j<finalRUAlloc.size();j++){
  std::cout<<"\n--- finalRUAlloc   "<<finalRUAlloc[j];


}
int cc=0;

for(unsigned int j=0;j<finalRUAlloc.size();j++){
  
  finalRUAlloc1.push_back(finalRUAlloc[j]);

}

while(temp4){

int ss=lol[cc];
//finalRUAlloc1.insert(finalRUAlloc1.begin()+ss,finalRUAlloc[cc]);i
finalRUAlloc1[ss]=finalRUAlloc[cc];
//std::cout<<"---finalRUAlloc1111--  "<<
cc++;
temp4--;


}
for(unsigned int j=0;j<finalRUAlloc1.size();j++){
  std::cout<<"\n--- finalRUAlloc111   "<<finalRUAlloc1[j];


}



mappedRuAllocated.clear();
std::cout<<"\n \n Before Mapped RU LOOP Call ";
for (int i=0; i<noOfSTA; i++){
 //int ff=lol[i];

switch(finalRUAlloc1[i]){
  case 26: mappedRuAllocated.push_back(HeRu::RU_26_TONE);
           break;
  case 52: mappedRuAllocated.push_back(HeRu::RU_52_TONE);
           break;
  case 106: mappedRuAllocated.push_back(HeRu::RU_106_TONE);
           break;
  case 242: mappedRuAllocated.push_back(HeRu::RU_242_TONE);
           break;
  case 484: mappedRuAllocated.push_back(HeRu::RU_484_TONE);
           break;
  }
  std::cout<<"\nMapping============================="<<mappedRuAllocated[i]<<"\n";
}

std::cout<<"\nmapping done\n";


std::cout<<"\n\n before staAllocated1 check how many m_staInfo there"<<m_staInfo.size();
int temp3=0;
staAllocated1.clear();

auto checking1=m_staInfo.begin();

  while(checking1!=m_staInfo.end()){
          std::cout<<"\n staAllocated1-- "<< checking1->first;
    checking1++;
  }

while(temp2){
  auto startItt = m_staInfo.begin();
 int dd = lol[temp3];
 //std::cout<<"\n \nstartITT==="<<startItt->first;
 std::cout<<"\n\n dd=============="<<dd;
      for(int i=0;i<dd;i++){
        // if(finalStaPairIndex[i].second==count1){
        //   staAllocated1.push_back(*startItt);
        //   count1++;

        //}
        startItt++;
        //break;
      }
      std::cout<<"\n \nstartITT==="<<startItt->first;
      staAllocated1.push_back(*startItt);
      temp3++;

      temp2--;

}







auto checking=staAllocated1.begin();

  while(checking!=staAllocated1.end()){
          std::cout<<"\n staAllocated1-- "<< checking->first;
    checking++;
  }

std::cout<<"\n \n After Sorting based Mac48Address and store in staAllocated1"<<staAllocated1.size();


  
}











////////////////////////////////////////////////COMPUTE DL OFDMA/////////////////////////////////////////////////////////
OfdmaManager::DlOfdmaInfo
RrOfdmaManager::ComputeDlOfdmaInfo (void)
{
  NS_LOG_FUNCTION (this);

  if (m_staInfo.empty ())
    {
      return DlOfdmaInfo ();
    }

  uint16_t bw = m_low->GetPhy ()->GetChannelWidth ();
//   uint16_t bw = m_bw;   // for TESTING only

  // compute how many stations can be granted an RU and the RU size
  std::size_t nRusAssigned = m_staInfo.size ();
std::cout<<"m_sta_info size: ComputeDlOfdmaInfo "<<m_staInfo.size ()<<"\n";
  





HeRu::RuType ruType ;

if(v_dataStaPair.size()>1){
  //call your function
  
mlwdf();

  nRusAssigned=finalRUAlloc.size();
  std::cout<<"\nmin ru alloc size assigned to nru assigned"<<nRusAssigned<<"\n";

  if(nRusAssigned==0) //in case no feasible ru allocation for current input exists.
  {
    std::cout<<"if(nRusAssigned==0) \n";
    //std::size_t nRusAs = m_staInfo.size ();
    nRusAssigned = m_staInfo.size ();
    ruType = GetNumberAndTypeOfRus (bw, nRusAssigned);
  }
}else{
  std::cout<<"inside else";
  std::cout<<"if(dataStaPair.size()>1): false, in else case\n";
  //std::size_t nRusAs = m_staInfo.size ();
  nRusAssigned = m_staInfo.size ();
  ruType = GetNumberAndTypeOfRus (bw, nRusAssigned);
}

//HeRu::RuType ruType = GetNumberAndTypeOfRus (bw, nRusAssigned);

///////////////////////////////////////////////////////

//std::cout<<nRusAssigned << " stations are being assigned a " << ruType << " RU\n";
 // NS_LOG_DEBUG (nRusAssigned << " stations are being assigned a " << ruType << " RU");

std::cout<<"\n before Intilaizing DlOfdmaInfo dlOfdmaInfo\n";
  DlOfdmaInfo dlOfdmaInfo;
  auto staInfoIt = m_staInfo.begin (); // iterator over the list of candidate receivers

  if(staAllocated1.empty()){
    for (std::size_t i = 0; i < nRusAssigned; i++){
      
      std::cout<<"\n staAllocated1 is empty"<<i;
      NS_ASSERT (staInfoIt != m_staInfo.end ());
      dlOfdmaInfo.staInfo.insert (*staInfoIt);
      staInfoIt++;
    }
  }else if(staAllocated1.size()>0){
        auto sti = staAllocated1.begin (); // iterator over the list of candidate receivers
        for (std::size_t i = 0; i < finalRUAlloc.size(); i++){
           std::cout<<"\n staAllocated1 is > 0"<<i;
        	NS_ASSERT (sti != staAllocated1.end ());
          dlOfdmaInfo.staInfo.insert (*sti);

          auto mp=dlOfdmaInfo.staInfo.begin();
          while(mp!=dlOfdmaInfo.staInfo.end()){
          	std::cout<<"\n  Dlofdma after insert "<<mp->first;
          	mp++;
          }

          std::cout<<"\n sti--"<<sti->first;
          sti++;
        }
    }
  
    std::cout<<"\n After DlOfdmaInfo initialization\n";

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////
  // if not all the stations are assigned an RU, the first station to serve next
  // time is the first one that was not served this time
  if (nRusAssigned < m_staInfo.size ())
    {
      NS_ASSERT (staInfoIt != m_staInfo.end ());
      m_startStation = staInfoIt->second.aid;
      std::cout<<"Next station to serve has AID=" << m_startStation<<"\n";
    }

  auto firstSTA = m_staInfo.begin (); 
  m_startStation=firstSTA->second.aid; ////// Thinkkkkkk
////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  NS_LOG_DEBUG ("Next station to serve has AID=" << m_startStation);


    std::cout<<"\n before InitTxVectorAndParams func call\n";



  InitTxVectorAndParams (dlOfdmaInfo.staInfo, ruType, m_dlMuAckSequence);
  dlOfdmaInfo.params = m_txParams;

    std::cout<<"\n after InitTxVectorAndParams func call\n";


 

  if (ruType == HeRu::RU_2x996_TONE)
    {
      HeRu::RuSpec ru = {true, ruType, 1};
      NS_LOG_DEBUG ("STA " << m_staInfo.front ().first << " assigned " << ru);
      m_txVector.SetRu (ru, m_staInfo.front ().second.aid);
    }
  else
    {
      std::vector<bool> primary80MHzSet {true};

      if (bw == 160)
        {
          primary80MHzSet.push_back (false);
          bw = 80;
        }

      auto mapIt = dlOfdmaInfo.staInfo.begin ();
        if(mappedRuAllocated.size()!=0){
        std::size_t ru26 = 0;
        std::size_t ru52 = 0;
        std::size_t ru106 = 0;
        std::size_t ru242 = 0;
        std::size_t ru484 = 0;
          std::vector<int>::size_type len = mappedRuAllocated.size();
          std::cout<<"\n mappedRuAllocated size="<<len;
          for (unsigned i=0; i<len; i++){
           HeRu::RuSpec ru;
        switch(mappedRuAllocated[i]){
          case HeRu::RU_26_TONE: ru26++;
                     ru = {true, mappedRuAllocated[i],ru26};
                   break;
          case HeRu::RU_52_TONE: ru52++;
                     ru = {true, mappedRuAllocated[i],ru52};
                   break;
          case HeRu::RU_106_TONE: ru106++;
                     ru = {true, mappedRuAllocated[i],ru106};
                   break;
          case HeRu::RU_242_TONE: ru242++;
                    ru = {true, mappedRuAllocated[i],ru242};
                   break;
          case HeRu::RU_484_TONE: ru484++;
                    ru = {true, mappedRuAllocated[i],ru484};
                   break;
        default: break;
          }
            std::cout<<"\n STA ----" << mapIt->first << " assigned---- " << ru;
           NS_LOG_DEBUG ("STA " << mapIt->first << " assigned " << ru);
           m_txVector.SetRu (ru, mapIt->second.aid);
           mapIt++;
          }
        }
        else{

              for (auto primary80MHz : primary80MHzSet)
                {
                  for (std::size_t ruIndex = 1; ruIndex <= HeRu::m_heRuSubcarrierGroups.at ({bw, ruType}).size (); ruIndex++)
                    {
                      NS_ASSERT (mapIt != dlOfdmaInfo.staInfo.end ());
                      HeRu::RuSpec ru = {primary80MHz, ruType, ruIndex};
                      NS_LOG_DEBUG ("STA " << mapIt->first << " assigned " << ru);
                      m_txVector.SetRu (ru, mapIt->second.aid);
                      mapIt++;
                    }
                }
             }
}
  dlOfdmaInfo.txVector = m_txVector;

  if (m_txParams.GetDlMuAckSequenceType () == DlMuAckSequenceType::DL_MU_BAR
      || m_txParams.GetDlMuAckSequenceType () == DlMuAckSequenceType::DL_AGGREGATE_TF)
    {
      Ptr<WifiRemoteStationManager> stationManager = GetWifiRemoteStationManager ();
      // The Trigger Frame to be returned is built from the TX vector used for the DL MU PPDU
      // (i.e., responses will use the same set of RUs) and modified to ensure that responses
      // are sent at a rate not higher than MCS 5.
      dlOfdmaInfo.trigger = GetTriggerFrameHeader (dlOfdmaInfo.txVector, 5);
      dlOfdmaInfo.trigger.SetUlLength (m_low->CalculateUlLengthForBlockAcks (dlOfdmaInfo.trigger, m_txParams));
      SetTargetRssi (dlOfdmaInfo.trigger);
    }

finalRUAlloc.clear();
finalRUAlloc1.clear();
mappedRuAllocated.clear();


  std::cout<<"\nsetting mcs before\n";

// if(random_MCS.size()>1){
//   std::cout<<"\nsetting mcs inside loop\n";
// unsigned int x=0;
// auto userInfoMap = dlOfdmaInfo.txVector.GetHeMuUserInfoMap ();
// for (auto& userInfo : userInfoMap)
//     {
//       uint8_t mcs = random_MCS.at(x);
//       NS_LOG_FUNCTION("MCS"<<mcs);
//  dlOfdmaInfo.txVector.SetHeMuUserInfo (userInfo.first, {userInfo.second.ru,
//                                                      WifiPhy::GetHeMcs (mcs),
//                                                      userInfo.second.nss});
// x++;
//     }
//   random_MCS.clear();
//  }
  

  std::cout<<"\nsetting mcs after\n";
staAllocated1.clear();





// auto userInfoMap = dlMuTxVector.GetHeMuUserInfoMap ();

//   for (auto& userInfo : userInfoMap)
//     {
//       uint8_t mcs = std::min (userInfo.second.mcs.GetMcsValue (), maxMcs);
//       dlMuTxVector.SetHeMuUserInfo (userInfo.first, {userInfo.second.ru,
//                                                      WifiPhy::GetHeMcs (mcs),
//                                                      userInfo.second.nss});
//     }


  return dlOfdmaInfo;
}

CtrlTriggerHeader
RrOfdmaManager::GetTriggerFrameHeader (WifiTxVector dlMuTxVector, uint8_t maxMcs)
{
  auto userInfoMap = dlMuTxVector.GetHeMuUserInfoMap ();

  for (auto& userInfo : userInfoMap)
    {
      uint8_t mcs = std::min (userInfo.second.mcs.GetMcsValue (), maxMcs);
      dlMuTxVector.SetHeMuUserInfo (userInfo.first, {userInfo.second.ru,
                                                     WifiPhy::GetHeMcs (mcs),
                                                     userInfo.second.nss});
    }

  return CtrlTriggerHeader (TriggerFrameType::MU_BAR_TRIGGER, dlMuTxVector);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




OfdmaManager::UlOfdmaInfo
RrOfdmaManager::ComputeUlOfdmaInfo (void)
{
  CtrlTriggerHeader trigger (TriggerFrameType::BASIC_TRIGGER, m_txVector);
  trigger.SetUlLength (m_txVector.GetLength ());
  SetTargetRssi (trigger);

  UlOfdmaInfo ulOfdmaInfo;
  ulOfdmaInfo.params = m_txParams;
  ulOfdmaInfo.trigger = trigger;

  return ulOfdmaInfo;
}

} //namespace ns3
