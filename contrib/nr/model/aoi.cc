#include "aoi.h"
#include "ns3/log.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("AoI");

NS_OBJECT_ENSURE_REGISTERED (AoI);

TypeId
AoI::GetTypeId (void)
{
  static TypeId tid =
      TypeId ("ns3::AoI").SetParent<Object> ().SetGroupName ("Nr").AddConstructor<AoI> ();
  return tid;
}

AoI::AoI () : m_currentAoI (0)
{
}

AoI::~AoI ()
{
}

void
AoI::SetPacketCreationTime (Time time)
{
  m_packetCreationTime = time;
  m_lastUpdateTime = time;
}

Time
AoI::GetPacketCreationTime () const
{
  return m_packetCreationTime;
}

void
AoI::UpdateAoI (Time currentTime)
{
  Time timeDiff = currentTime - m_lastUpdateTime;
  m_currentAoI += timeDiff.GetMicroSeconds () / 125; // 1 슬롯 = 125 마이크로초로 가정
  m_lastUpdateTime = currentTime;
}

void
AoI::IncrementAoI (uint32_t slots)
{
  m_currentAoI += slots;
}

uint32_t
AoI::GetCurrentAoI () const
{
  return m_currentAoI;
}

void
AoI::ResetAoI (Time currentTime)
{
  m_currentAoI = 0;
  m_lastUpdateTime = currentTime;
  m_packetCreationTime = currentTime;
}

} // namespace ns3