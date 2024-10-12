// aoi.cc
#include "aoi.h"

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (AoI);

TypeId
AoI::GetTypeId (void)
{
  static TypeId tid =
      TypeId ("ns3::AoI").SetParent<Object> ().SetGroupName ("Nr").AddConstructor<AoI> ();
  return tid;
}

AoI::
    AoI () // 0jkim : 생성자 초기화 (aoi = 1.0, wma = 1, totalTransmissions = 0, metricWeight = 0.5)
    : m_currentAoI (1.0), m_wma (1), m_totalTransmissions (0), m_metricWeight (0.5)
{
}

AoI::~AoI ()
{
}

void
AoI::SetPacketCreationTime (Time time) // 0jkim : 패킷 생성 시간 설정
{
  m_packetCreationTime = time;
  m_lastUpdateTime = time;
}

Time
AoI::GetPacketCreationTime () const // 0jkim : 패킷 생성 시간 반환
{
  return m_packetCreationTime;
}

void
AoI::UpdateAoI (Time currentTime) // 0jkim : AoI 업데이트
{
  m_currentAoI = (currentTime - m_lastUpdateTime).GetSeconds (); // 0jkim : AoI 계산
  m_lastUpdateTime = currentTime; // 0jkim : 마지막 업데이트 시간 업데이트
}

double
AoI::GetCurrentAoI () const // 0jkim : 현재 AoI 값 반환
{
  return m_currentAoI;
}

void
AoI::IncrementAoI (uint32_t slots) // 0jkim : AoI가 slots만큼 증가
{
  m_currentAoI += slots; // 0jkim : AoI 값 증가
}

void
AoI::ResetAoI (Time currentTime) // 0jkim : AoI 초기화
{
  m_currentAoI = 1.0; // 0jkim : AoI 값 1.0으로 초기화
  m_lastUpdateTime = currentTime; // 0jkim : 마지막 업데이트 시간 초기화
  m_packetCreationTime = currentTime; // 0jkim : 패킷 생성 시간 초기화
}

void
AoI::UpdateWMA (bool success) // 0jkim : WMA 업데이트
{
  if (success)
    {
      m_wma++; // 성공 시 WMA를 1 증가
    }
  // 실패 시 아무 것도 하지 않음
}

uint32_t
AoI::GetWMA () const
{
  return m_wma;
}

void
AoI::SetMetricWeight (double x) // 0jkim : 메트릭 가중치 설정
{
  NS_ASSERT (x >= 0.0 && x <= 1.0); // x가 0과 1 사이의 값인지 확인
  m_metricWeight = x;
}

double
AoI::GetMetricWeight () const // 0jkim : 메트릭 가중치 반환
{
  return m_metricWeight;
}

double
AoI::CalculateMetric (double x) const // 0jkim : 메트릭 계산
{
  // WMA가 0일 때 분모가 0이 되는 것을 방지하기 위해 1을 더함
  return x * m_currentAoI + (1 - x) * (1.0 / (m_wma + 1));
}

} // namespace ns3