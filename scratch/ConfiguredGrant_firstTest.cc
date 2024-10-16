/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 *   Copyright (c) 2020 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License version 2 as
 *   published by the Free Software Foundation;
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/*
 * Description: This code transmits 100 packets from UE to the gNB. It works with
 * dynamic or configured grant (CG) schedulers (both schedulers cannot work simultaneously).
 *
 * In case of CG, a configuration time is selected. In this time, the UEs transmit
 * their requirements to the gNB. The gNB creates a CG for each UE.
 *
 * You can use OFDMA or TDMA access mode.
 * However, we include two new scheduling policies to use with OFDMA access mode.
 *
 * This code is based on "cttc-3gpp-channel-simple-ran.cc" (5G-LENA) code.
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-module.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/internet-module.h"
#include "ns3/eps-bearer-tag.h"
#include "ns3/grid-scenario-helper.h"
#include "ns3/log.h"
#include "ns3/antenna-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/aoi.h" // 0jkim : AoI 클래스 포함
#include "ns3/aoi-tag.h" // 0jkim : AoITag 클래스 포함

#include <cstdlib>
#include <ctime>

using namespace ns3;

/*
 * Enable the logs of the file by enabling the component "ConfiguredGrant",
 * in this way:
 * $ export NS_LOG="ConfiguredGrant=level_info|prefix_func|prefix_time"
 * 0jkim : ConfiguredGrant 컴포넌트를 활성화하여 파일의 로그를 활성화합니다.
 */
NS_LOG_COMPONENT_DEFINE ("ConfiguredGrant");

static bool g_rxPdcpCallbackCalled = false;
static bool g_rxRxRlcPDUCallbackCalled = false;

/*
 * Global variables
 */
Time g_txPeriod = Seconds (0.1); // 0jkim : 패킷의 전송 주기 0.1초
Time delay; // 0jkim : 패킷 전송시 지연 시간 저장
std::fstream m_ScenarioFile; // 0jkim : 시나리오 파일 스트림
std::vector<uint64_t> packetCreationTimes; // 패킷 생성시간을 저장하는 전역 벡터변수

/*
 * MyModel class. It contains the function that generates the event to send a packet from the UE to the gNB
 * 0jkim : MyModel 클래스는 UE에서 gNB로 패킷을 전송하는 이벤트를 생성하는 함수를 포함함.
*/

class MyModel : public Application // 0jkim : Application 클래스를 상속받음
{
public:
  MyModel ();
  virtual ~MyModel ();

  // 0jkim : AoI 관련 메서드 시작
  void
  SetAoI (Ptr<AoI> aoi) // 0jkim : AoI 포인터의 aoi 설정
  {
    m_aoi = aoi;
  }
  Ptr<AoI>
  GetAoI () const // 0jkim : AoI 포인터의 aoi 반환
  {
    return m_aoi;
  }
  // 0jkim : AoI 관련 메서드 끝

  // 0jkim : 디바이스, 주소, 패킷 크기, 패킷 개수, 데이터 속도, 주기, 마감 시간을 설정하는 함수
  void Setup (Ptr<NetDevice> device, Address address, uint32_t packetSize, uint32_t nPackets,
              DataRate dataRate, uint8_t period, uint32_t deadline);

  void SendPacketDl (); // 0jkim : DL 트래픽 패킷 전송 함수
  void ScheduleTxDl (); // 0jkim : DL 패킷 전송 스케줄링 함수

  void SendPacketUl (); // 0jkim : UL 트래픽 패킷 전송 함수
  void ScheduleTxUl (uint8_t period); // 0jkim : UL 패킷 전송 스케줄링 함수
  void ScheduleTxUl_Configuration (); // 0jkim : UL 패킷 전송 스케줄링 함수

private:
  Ptr<AoI> m_aoi; // 0jkim : AoI 포인터
  Ptr<NetDevice> m_device; // 0jkim : 디바이스 포인터
  Address m_addr; // 0jkim : 주소
  uint32_t m_packetSize; // 0jkim : 패킷 크기
  uint32_t m_nPackets; // 0jkim : 패킷 개수(하나의 UE당 하나의 패킷을 생성)
  DataRate m_dataRate; // 0jkim : 데이터 속도
  EventId m_sendEvent; // 0jkim : 이벤트 아이디
  bool m_running; // 0jkim : 실행 여부
  uint32_t m_packetsSent; // 0jkim : 전송된 패킷 개수
  uint8_t m_periodicity; // 0jkim : 주기
  uint32_t m_deadline; // 0jkim : 마감 시간
  Ptr<UniformRandomVariable> m_random; // 0jkim : 랜덤 변수
};

MyModel::MyModel () // 0jkim : 생성자
    : m_device (),
      m_addr (),
      m_packetSize (0),
      m_nPackets (0),
      m_dataRate (0),
      m_sendEvent (),
      m_running (false),
      m_packetsSent (0),
      m_periodicity (0),
      m_deadline (0)
{
  m_random = CreateObject<UniformRandomVariable> ();
}

MyModel::~MyModel () // 0jkim : 소멸자
{
}

void
MyModel::Setup (Ptr<NetDevice> device, Address address, uint32_t packetSize, uint32_t nPackets,
                DataRate dataRate, uint8_t period, uint32_t deadline) // 0jkim : 설정 메서드
{
  m_device = device;
  m_packetSize = packetSize;
  m_nPackets = nPackets;
  m_dataRate = dataRate;
  m_running = true;
  m_packetsSent = 0;
  m_periodicity = period;
  m_deadline = deadline;
}

/*
 * This is the first event that is executed  for DL traffic.
 */
void
StartApplicationDl (Ptr<MyModel> model)
{
  model->SendPacketDl ();
}
/*
 * Function creates a single packet and directly calls the function send
 * of a device to send the packet to the destination address.
 * (DL TRAFFIC)
 */
void
MyModel::SendPacketDl ()
{
  Ptr<Packet> pkt = Create<Packet> (m_packetSize, m_periodicity, m_deadline);
  Ipv4Header ipv4Header;
  ipv4Header.SetProtocol (Ipv4L3Protocol::PROT_NUMBER);
  pkt->AddHeader (ipv4Header);

  EpsBearerTag tag (1, 1);
  pkt->AddPacketTag (tag);

  m_device->Send (pkt, m_addr, Ipv4L3Protocol::PROT_NUMBER);
  NS_LOG_INFO ("Sending DL");

  if (++m_packetsSent < m_nPackets)
    {
      ScheduleTxDl ();
    }
}
/*
 * SendPacket creates the packet at tNext time instant.
 */

void
MyModel::ScheduleTxDl ()
{
  if (m_running)
    {
      Time tNext = MilliSeconds (2);
      m_sendEvent = Simulator::Schedule (tNext, &MyModel::SendPacketDl, this);
    }
}

/*
 * This is the first event that is executed  for UL traffic.
 * 0jkim : UL 트래픽에 대한 첫 번째 이벤트
 */
void
StartApplicationUl (Ptr<MyModel> model) // 0jkim : UL 트래픽에 대한 시작 이벤트
{
  model->SendPacketUl ();
}
/*
 * Function creates a single packet and directly calls the function send
 * of a device to send the packet to the destination address.
 * (UL TRAFFIC)
 * 0jkim : 함수는 단일 패킷을 생성하고 디바이스의 send 함수를 직접 호출하여 대상 주소로 패킷을 전송합니다.
 */
void
MyModel::SendPacketUl () // 0jkim : UL 트래픽 패킷 전송 함수
{
  Ptr<Packet> pkt = Create<Packet> (m_packetSize, m_periodicity, m_deadline);

  uint64_t creationTimeNs = Simulator::Now ().GetNanoSeconds ();
  packetCreationTimes.push_back (creationTimeNs); // 패킷 생성시간을 백터에 추가

  // 패킷에 생성 시간 태그 추가
  PacketCreationTimeTag creationTimeTag (
      creationTimeNs); // PacketCreationTimeTag 클래스의 멤버 변수인 m_creationTime을 패킷의 생성시간으로 초기화
  pkt->AddPacketTag (creationTimeTag); // 패킷태그에 패킷생성시간을 추가함

  // UE의 ID를 기록하기 위한 태그 추가
  uint32_t ueId = m_device->GetNode ()->GetId ();
  PacketUeIdTag ueIdTag (
      ueId); // PacketUeIdTag 클래스의 멤버 변수인 m_ueid을 패킷을 생성한 UE의 ID로 초기화
  pkt->AddPacketTag (ueIdTag); // 패킷태그에 패킷UEID를 추가함

  std::cout << "\n Packet created by UE " << ueId << " at: " << creationTimeNs << " ns"
            << std::endl; // UE가 생성한 패킷의 생성시간을 출력

  // 0jkim : IPv4 헤더 설정
  Ipv4Header ipv4Header;
  ipv4Header.SetProtocol (Ipv4L3Protocol::PROT_NUMBER);
  pkt->AddHeader (ipv4Header);

  // 0jkim : 데이터 패킷을 MAC 계층으로 전송
  m_device->Send (pkt, m_addr, Ipv4L3Protocol::PROT_NUMBER);
  NS_LOG_INFO ("Sending UL");

  if (m_packetsSent == 0) // 0jkim : 첫 번째 패킷인 경우
    {
      ScheduleTxUl_Configuration ();
      m_packetsSent = 1;
    }
  else if (++m_packetsSent < m_nPackets) // 패킷개수만큼 ScheduleTxUL 진행
    {
      ScheduleTxUl (m_periodicity);
    }
}
/*
 * SendPacket creates the packet at tNext time instant.
 */

void
MyModel::ScheduleTxUl (uint8_t period)
{
  if (m_running)
    {
      Time tNext = MilliSeconds (period);
      m_sendEvent = Simulator::Schedule (
          tNext, &MyModel::SendPacketUl,
          this); // 현재 sendpacketul메서드와 함께 상향링크 스케줄링 이벤트 시작
    }
}

void
MyModel::ScheduleTxUl_Configuration (void) // 0jkim : UL 패킷 전송 스케줄링 함수
{
  uint8_t configurationTime = 60;
  Time tNext = MilliSeconds (configurationTime);
  m_sendEvent = Simulator::Schedule (tNext, &MyModel::SendPacketUl, this);
}

/*
 * TraceSink, RxRlcPDU connects the trace sink with the trace source (RxPDU). It connects the UE with gNB and vice versa.
 */
void
RxRlcPDU (std::string path, uint16_t rnti, uint8_t lcid, uint32_t bytes, uint64_t rlcDelay)
{
  g_rxRxRlcPDUCallbackCalled = true;
  delay = Time::FromInteger (rlcDelay, Time::NS);
  std::cout << "\n rlcDelay in NS (Time):" << delay << std::endl;

  std::cout << "\n\n Data received at RLC layer at:" << Simulator::Now () << std::endl;

  m_ScenarioFile << "\n\n Data received at RLC layer at:" << Simulator::Now () << std::endl;
  m_ScenarioFile << "\n rnti:" << rnti << std::endl;
  m_ScenarioFile << "\n delay :" << rlcDelay << std::endl;
}

void
RxPdcpPDU (std::string path, uint16_t rnti, uint8_t lcid, uint32_t bytes, uint64_t pdcpDelay)
{
  std::cout << "\n Packet PDCP delay:" << pdcpDelay << "\n";
  g_rxPdcpCallbackCalled = true;
}

void
ConnectUlPdcpRlcTraces ()
{
  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/UeMap/*/DataRadioBearerMap/*/LtePdcp/RxPDU",
                   MakeCallback (&RxPdcpPDU));

  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/UeMap/*/DataRadioBearerMap/*/LteRlc/RxPDU",
                   MakeCallback (&RxRlcPDU));
  NS_LOG_INFO ("Received PDCP RLC UL");
}

int
main (int argc, char *argv[])
{
  uint16_t numerologyBwp1 = 1; // 0jkim : Bwp1의 numerology 설정
  uint32_t packetSize = 10; // 0jkim : 패킷 사이즈 설정
  double centralFrequencyBand1 = 3550e6; // 0jkim : 중심 주파수 설정 35.5 GHz
  double bandwidthBand1 = 20e6; // 0jkim : 대역폭 설정 20 MHz
  uint8_t period = uint8_t (10); // 0jkim : 주기 설정 10 ms

  uint16_t gNbNum = 1; // 0jkim : gNB 개수 설정
  uint16_t ueNumPergNb = 20; // 0jkim : UE 개수 설정

  bool enableUl = true; // 0jkim : UL 트래픽 활성화 여부 설정(true)
  uint32_t nPackets = 1000; // 0jkim : 패킷 개수 설정
  Time sendPacketTime = Seconds (0.2); // 0jkim : 패킷 전송 시간 설정
  uint8_t sch = 1; // 5G-OFDMA 방식

  delay = MicroSeconds (10); // 0jkim : 전송 시간 설정

  CommandLine cmd;
  cmd.AddValue ("numerologyBwp1", "The numerology to be used in bandwidth part 1", numerologyBwp1);
  cmd.AddValue ("centralFrequencyBand1", "The system frequency to be used in band 1",
                centralFrequencyBand1);
  cmd.AddValue ("bandwidthBand1", "The system bandwidth to be used in band 1", bandwidthBand1);
  cmd.AddValue ("packetSize", "packet size in bytes", packetSize);
  cmd.AddValue ("enableUl", "Enable Uplink", enableUl);
  cmd.AddValue ("scheduler", "Scheduler", sch);
  cmd.Parse (argc, argv);

  std::vector<uint32_t> v_init (ueNumPergNb); // 0jkim :gNB에 연결된 UE의 초기 지연 시간 벡터 선언
  std::vector<uint32_t> v_period (ueNumPergNb); // 0jkim : gNB에 연결된 UE의 주기 벡터 선언
  std::vector<uint32_t> v_deadline (ueNumPergNb); // 0jkim : gNB에 연결된 UE의 마감 시간 벡터 선언
  std::vector<uint32_t> v_packet (ueNumPergNb); // 0jkim : gNB에 연결된 UE의 패킷 크기 벡터 선언

  std::cout << "\n Init values: " << '\n';
  v_init = std::vector<uint32_t> (ueNumPergNb,
                                  {100000}); // 0jkim : gNB에 연결된 UE의 초기 지연 시간 벡터 설정
  for (int val : v_init) // 0jkim : 벡터 출력
    std::cout << val << std::endl;

  std::cout << "Deadline values: " << '\n';
  v_deadline = std::vector<uint32_t> (ueNumPergNb,
                                      {10000000}); // 0jkim : gNB에 연결된 UE의 마감 시간 벡터 설정
  for (int val : v_deadline) // 0jkim : 벡터 출력
    std::cout << val << std::endl;

  std::cout << "Packet values: " << '\n';
  v_packet = std::vector<uint32_t> (ueNumPergNb,
                                    {packetSize}); // 0jkim : gNB에 연결된 UE의 패킷 크기 벡터 설정
  for (int val : v_packet) // 0jkim : 벡터 출력
    std::cout << val << std::endl;

  std::cout << "Period values: " << '\n';
  v_period = std::vector<uint32_t> (ueNumPergNb, {10}); // 0jkim : gNB에 연결된 UE의 주기 벡터 설정
  for (int val : v_period) // 0jkim : 벡터 출력
    std::cout << val << "\t";

  // 0jkim : 파일 생성해서 시나리오를 저장하는 부분
  m_ScenarioFile.open ("Scenario.txt", std::ofstream::out | std::ofstream::trunc);

  std::ostream_iterator<std::uint32_t> output_iterator (m_ScenarioFile, "\n");
  m_ScenarioFile << "Nº UE" << "\t" << "Init" << "\t" << "Latency" << "\t" << "Periodicity"
                 << std::endl;

  m_ScenarioFile << ueNumPergNb << std::endl;
  std::copy (v_init.begin (), v_init.end (), output_iterator);
  m_ScenarioFile << std::endl;
  std::copy (v_deadline.begin (), v_deadline.end (), output_iterator);
  m_ScenarioFile << std::endl;
  std::copy (v_period.begin (), v_period.end (), output_iterator);
  m_ScenarioFile << std::endl;
  // 0jkim : 파일 생성 끝

  int64_t randomStream = 1; // 0jkim : 랜덤 스트림 설정(추후 변경)

  // 0jkim : 네트워크 토폴로지를 구성하는 부분
  // Create the scenario
  GridScenarioHelper gridScenario;
  gridScenario.SetRows (1);
  gridScenario.SetColumns (gNbNum);
  gridScenario.SetHorizontalBsDistance (5.0);
  gridScenario.SetBsHeight (10.0);
  gridScenario.SetUtHeight (1.5);

  // must be set before BS number
  gridScenario.SetSectorization (GridScenarioHelper::SINGLE);
  gridScenario.SetBsNumber (gNbNum);
  gridScenario.SetUtNumber (ueNumPergNb * gNbNum);
  gridScenario.SetScenarioHeight (10);
  gridScenario.SetScenarioLength (10);
  randomStream += gridScenario.AssignStreams (randomStream); // 0jkim : 랜덤 스트림 할당
  gridScenario.CreateScenario (); // 0jkim : 시나리오 생성

  Ptr<NrPointToPointEpcHelper> epcHelper =
      CreateObject<NrPointToPointEpcHelper> (); // 0jkim : epchelper 객체 생성
  Ptr<IdealBeamformingHelper> idealBeamformingHelper =
      CreateObject<IdealBeamformingHelper> (); // 0jkim : idealbeamforminghelper 객체 생성
  Ptr<NrHelper> nrHelper = CreateObject<NrHelper> (); // 0jkim : nrhelper 객체 생성
  nrHelper->SetBeamformingHelper (idealBeamformingHelper); // 0jkim : beamforminghelper 설정

  // Scheduler type: configured grant or grant based
  /* false -> grant based : true -> configured grant */
  bool scheduler_CG = false; // 0jkim : grant based 방식을 선택하므로 false
  uint8_t configurationTime = 60;

  nrHelper->SetUeMacAttribute ("CG", BooleanValue (scheduler_CG));
  nrHelper->SetUePhyAttribute ("CG", BooleanValue (scheduler_CG));
  nrHelper->SetGnbMacAttribute ("CG", BooleanValue (scheduler_CG));
  nrHelper->SetGnbPhyAttribute ("CG", BooleanValue (scheduler_CG));

  if (scheduler_CG) // grant free 방식
    {
      //Configuration time
      // UE
      nrHelper->SetUeMacAttribute ("ConfigurationTime", UintegerValue (configurationTime));
      nrHelper->SetUePhyAttribute ("ConfigurationTime", UintegerValue (configurationTime));
      // gNB
      nrHelper->SetGnbMacAttribute ("ConfigurationTime", UintegerValue (configurationTime));
      nrHelper->SetGnbPhyAttribute ("ConfigurationTime", UintegerValue (configurationTime));
    }
  else // grant based 방식
    {
      nrHelper->SetSchedulerAttribute ("CG", BooleanValue (scheduler_CG));
    }

  nrHelper->SetEpcHelper (epcHelper);

  // Disable the SRS
  nrHelper->SetSchedulerAttribute ("SrsSymbols", UintegerValue (0));

  // Add the desired flexible pattern (the needed DL DATA symbols (default 0))
  nrHelper->SetSchedulerAttribute ("DlDataSymbolsFpattern", UintegerValue (0)); //symStart - 1

  // enable or disable HARQ retransmissions
  // 0jkim : HARQ 재전송 활성화 여부 설정 (현재 False)
  nrHelper->SetSchedulerAttribute ("EnableHarqReTx", BooleanValue (false));
  Config::SetDefault ("ns3::NrHelper::HarqEnabled", BooleanValue (false));

  // Select scheduler
  if (sch != 0)
    {
      nrHelper->SetSchedulerTypeId (
          NrMacSchedulerOfdmaRR::GetTypeId ()); // 이곳에서 스케줄러 타입 설정
      nrHelper->SetSchedulerAttribute ("schOFDMA", UintegerValue (sch)); // sch = 0 for TDMA
          // 1 for 5GL-OFDMA
          // 2 for Sym-OFDMA
          // 3 for RB-OFDMA
    }

  // Create one operational band containing one CC with one bandwidth part
  // 0jkim : 주파수 구조를 설정하는 부분 (하나의 주파수 대역에 하나의 CC와 하나의 대역폭 파트가 있음)
  BandwidthPartInfoPtrVector allBwps; // 0jkim : 모든 대역폭 파트 정보를 저장하는 벡터 선언
  CcBwpCreator
      ccBwpCreator; // 0jkim : ccBwpCreator 객체를 선언해서 CC 및 대역폭 파트를 생성하는 데 사용
  const uint8_t numCcPerBand = 1; // 0jkim : 각 주파수 대역(Band)당 사용할 CC의 수 설정

  // 0jkim : 간단한 주파수 운영 대역 생성(대역의 중심 주파수, 대역의 전체 대역폭, CC의 수, 시나리오(스마트 시티 도시환경))
  CcBwpCreator::SimpleOperationBandConf bandConf1 (centralFrequencyBand1, bandwidthBand1,
                                                   numCcPerBand,
                                                   BandwidthPartInfo::UMi_StreetCanyon_nLoS);

  // 0jkim : 생성한 구성을 사용하여 운영 대역 생성
  OperationBandInfo band1 = ccBwpCreator.CreateOperationBandContiguousCc (bandConf1);

  // 0jkim : 채널 모델 업데이트 주기 설정(동적인 채널 모델을 위해 추후 변경)
  Config::SetDefault ("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue (MilliSeconds (0)));

  // 0jkim : 하향링크 스케줄러와 하향링크 채널모델 추가 설정
  nrHelper->SetSchedulerAttribute ("FixedMcsDl",
                                   BooleanValue (true)); // MCS 타입을 설정(고정 or 가변)
  nrHelper->SetSchedulerAttribute ("StartingMcsDl", UintegerValue (4)); // 초기 MCS 값 설정

  // 0jkim : 상향링크 스케줄러와 상향링크 채널모델 추가 설정
  nrHelper->SetSchedulerAttribute ("FixedMcsUl",
                                   BooleanValue (true)); // MCS 타입을 설정(고정 or 가변)
  nrHelper->SetSchedulerAttribute ("StartingMcsUl", UintegerValue (12)); // 초기 MCS 값 설정

  // 0jkim : 채널 상태 모델의 업데이트 주기를 설정함(0으로 설정하면 채널 상태 모델이 업데이트되지 않음)
  nrHelper->SetChannelConditionModelAttribute ("UpdatePeriod", TimeValue (MilliSeconds (0))); //
  nrHelper->SetPathlossAttribute ("ShadowingEnabled", BooleanValue (true)); //false

  // Error Model: UE and GNB with same spectrum error model.
  // ns3::NrEesmIrT2 (256QAM), ns3::NrEesmIrT1 (64QAM) more robust but with less througput
  // 0jkim : 오류 모델 설정(ns3::NrEesmIrT1은 64QAM 모델, ns3::NrEesmIrT2는 256QAM 모델)
  std::string errorModel = "ns3::NrEesmIrT1";
  nrHelper->SetUlErrorModel (errorModel);
  nrHelper->SetDlErrorModel (errorModel);

  // Both DL and UL AMC will have the same model behind.
  // 0jkim : AMC = Adaptive Modulation and Coding으로써 채널 조건에 따라 변조 방식과 코딩 방식을 동적으로 조절하는 기술
  // 0jkim : 근데 위에서 MCS를 고정했기 때문에 추후에 변경 필요
  nrHelper->SetGnbDlAmcAttribute (
      "AmcModel", EnumValue (NrAmc::ErrorModel)); // NrAmc::ShannonModel or NrAmc::ErrorModel
  nrHelper->SetGnbUlAmcAttribute (
      "AmcModel", EnumValue (NrAmc::ErrorModel)); // NrAmc::ShannonModel or NrAmc::ErrorModel

  bool fadingEnabled =
      true; // 0jkim : 페이딩 모델 활성화 여부 설정(페이딩 시 무선 신호의 강도가 시간에 따라 변화하는 현상을 모델링 하는데, 다중경로 전파나 장애물 및 이동성 등에 의해 발생)
  auto bandMask =
      NrHelper::INIT_PROPAGATION |
      NrHelper::
          INIT_CHANNEL; // 0jkim : bandmask는 비트마스크로써 초기화 시 초기화할 비트를 지정하는 데 사용
  if (fadingEnabled) // 0jkim : 페이딩 모델 활성화 여부 확인
    {
      bandMask |= NrHelper::INIT_FADING; // 0jkim : 페이딩 모델 활성화
    }

  nrHelper->InitializeOperationBand (
      &band1, bandMask); // 0jkim : 위에서 설정한 비트마스크를 사용하여 운영 대역 초기화
  allBwps = CcBwpCreator::GetAllBwps (
      {band1}); // 0jkim : band1에서 정의한 모든 CC와 그에 속한 BWP 정보를 반환해서 allBwps 벡터에 저장

  // 0jkim : 빔포밍 방법 설정(빔포밍은 무선 신호를 특정 방향으로 조절하는 기술로, 신호의 강도와 방향을 조절하여 수신 신호의 품질을 향상시키는 기술)
  idealBeamformingHelper->SetAttribute ("BeamformingMethod",
                                        TypeIdValue (QuasiOmniDirectPathBeamforming::GetTypeId ()));

  // 0jkim : UE의 안테나 설정
  nrHelper->SetUeAntennaAttribute ("NumRows", UintegerValue (2));
  nrHelper->SetUeAntennaAttribute ("NumColumns", UintegerValue (4));
  nrHelper->SetUeAntennaAttribute ("AntennaElement",
                                   PointerValue (CreateObject<IsotropicAntennaModel> ()));
  // 0jkim : UE의 안테나 배열을 2*4로 설정함 (총 8개의 안테나)
  // 0jkim : gNB의 안테나 설정
  nrHelper->SetGnbAntennaAttribute ("NumRows", UintegerValue (4));
  nrHelper->SetGnbAntennaAttribute ("NumColumns", UintegerValue (4));
  nrHelper->SetGnbAntennaAttribute ("AntennaElement",
                                    PointerValue (CreateObject<IsotropicAntennaModel> ()));
  // 0jkim : gNB의 안테나 배열을 4*4로 설정함 (총 16개의 안테나)
  // UE와 gNB 모두 다중 안테나 기술(MIMO)을 사용함

  //Install and get the pointers to the NetDevices
  // 0jkim : NetDevice를 설치하고 구성하는 부분
  NetDeviceContainer enbNetDev = nrHelper->InstallGnbDevice (
      gridScenario.GetBaseStations (),
      allBwps); // 0jkim : gridscenario에서 정의된 모든 기지국 노드를 가져와서 기지국 디바이스를 설치하고 그 결과를 enbNetDev에 저장
  NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice (
      gridScenario.GetUserTerminals (),
      allBwps); // 0jkim : gridscenario에서 정의된 모든 사용자 단말 노드를 가져와서 사용자 디바이스를 설치하고 그 결과를 ueNetDev에 저장

  randomStream += nrHelper->AssignStreams (
      enbNetDev,
      randomStream); // 0jkim : gNB Netdevice에 랜덤 스트림을 할당하여 스트림 인덱스 업데이트
  randomStream += nrHelper->AssignStreams (
      ueNetDev,
      randomStream); // 0jkim : UE Netdevice에 랜덤 스트림을 할당하여 스트림 인덱스 업데이트

  // Set the attribute of the netdevice (enbNetDev.Get (0)) and bandwidth part (0)
  // 0jkim : 네트워크 디바이스의 속성을 설정하는 부분
  nrHelper->GetGnbPhy (enbNetDev.Get (0), 0)
      ->SetAttribute ("Numerology",
                      UintegerValue (numerologyBwp1)); // 0jkim : gNB의 BWP1에 대한 numerology 설정

  for (auto it = enbNetDev.Begin (); it != enbNetDev.End (); ++it)
    {
      DynamicCast<NrGnbNetDevice> (*it)
          ->UpdateConfig (); // 0jkim : 모든 gNB 네트워크 디바이스의 구성을 업데이트(위에서 설정한 numerology, 안테나 등)
    }

  for (auto it = ueNetDev.Begin (); it != ueNetDev.End (); ++it)
    {
      DynamicCast<NrUeNetDevice> (*it)
          ->UpdateConfig (); // 0jkim : 모든 UE 네트워크 디바이스의 구성을 업데이트(위에서 설정한 numerology, 안테나 등)
    }

  // 0jkim : UE에게 인터넷 프로토콜 스택을 설치하고 IP 주소를 할당하는 부분
  InternetStackHelper internet; // 0jkim : 인터넷 스택 생성(TCP/IP 프로토콜 스택 설치)
  internet.Install (
      gridScenario
          .GetUserTerminals ()); // 0jkim : gridscenario에서 모든 ue노드를 가져와서 각 UE노드에게 인터넷 프로토콜을 설치함
  Ipv4InterfaceContainer ueIpIface; // 0jkim : UE의 IP 인터페이스 정보를 저장할 컨테이너 선언
  ueIpIface =
      epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueNetDev)); // 0jkim : UE에게 IP 주소 할당

  // 0jkim : 상향 링크 트래픽을 다루는 부분
  std::vector<Ptr<MyModel>> v_modelUl; // 0jkim : UL 트래픽에 대한 모델 벡터 선언
  v_modelUl = std::vector<Ptr<MyModel>> (ueNumPergNb, {0}); // 0jkim : 벡터 초기화
  for (uint8_t ii = 0; ii < ueNumPergNb; ++ii) // 0jkim : 각 UE에 대해 반복
    {
      Ptr<MyModel> modelUl = CreateObject<MyModel> (); // 0jkim : 각 UE에 대한 MyModel 객체 생성
      modelUl->Setup (ueNetDev.Get (ii), enbNetDev.Get (0)->GetAddress (), v_packet[ii], nPackets,
                      DataRate ("1Mbps"), v_period[ii],
                      v_deadline[ii]); // 0jkim : 각 UE에 대한 setup 함수 호출
      v_modelUl[ii] = modelUl; // 0jkim : 설정한 모델을 벡터에 추가
      Simulator::Schedule (
          MicroSeconds (v_init[ii]), &StartApplicationUl,
          v_modelUl
              [ii]); // 0jkim : 각 UE에 대한 시작 이벤트 스케줄링, v_init[ii]는 UE의 초기 지연 시간 따라서 각 UE는 초기 지연 시간 후에 시작되고\
               StartApplicationUl 메서드에 있는 SendPacketUl 함수가 호출됨
    }

  // DL traffic
  //Ptr<MyModel> modelDl = CreateObject<MyModel> ();
  //modelDl -> Setup(enbNetDev.Get(0), ueNetDev.Get(0)->GetAddress(), 10, nPackets, DataRate("1Mbps"),20, uint32_t(100000));
  //Simulator::Schedule(MicroSeconds(0.099625), &StartApplicationDl, modelDl);

  nrHelper->AttachToClosestEnb (ueNetDev,
                                enbNetDev); // 0jkim : 각 UE를 가장 가까운 gNB에 연결하는 부분

  nrHelper->EnableTraces (); // 0jkim : 트레이스 기능 활성화
  Simulator::Schedule (
      Seconds (0.16),
      &ConnectUlPdcpRlcTraces); // 0jkim : 시뮬레이션 시작 0.16초 후에 ConnectUlPdcpRlcTraces 함수 호출해서 상향링크 PDCP와 RLC 트레이스 연결

  Simulator::Stop (Seconds (1)); // 0jkim : 시뮬레이션 종료 시간 설정
  Simulator::Run (); // 0jkim : 시뮬레이션 실행

  std::cout << "\n FIN. " << std::endl; // 0jkim : 시뮬레이션 종료 메시지 출력

  if (g_rxPdcpCallbackCalled &&
      g_rxRxRlcPDUCallbackCalled) // 0jkim : 상향링크 PDCP와 RLC 트레이스 확인해서 모두 호출되었는지 확인
    {
      return EXIT_SUCCESS; // 0jkim : 성공 코드 반환
    }
  else
    {
      return EXIT_FAILURE; // 0jkim : 실패 코드 반환
    }

  Simulator::Destroy (); // 0jkim : 시뮬레이션 자원 해제
}