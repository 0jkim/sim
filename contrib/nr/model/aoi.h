#ifndef AOI_H
#define AOI_H

#include "ns3/nstime.h"
#include "ns3/object.h"

namespace ns3 {

class AoI : public Object
{
public:
  static TypeId GetTypeId (void);

  AoI ();
  virtual ~AoI ();

  void SetPacketCreationTime (Time time);
  Time GetPacketCreationTime () const;

  void UpdateAoI (Time currentTime);
  void IncrementAoI (uint32_t slots);
  uint32_t GetCurrentAoI () const;

  void ResetAoI (Time currentTime);

private:
  Time m_packetCreationTime;
  Time m_lastUpdateTime;
  uint32_t m_currentAoI; // 슬롯 단위로 저장
};

} // namespace ns3

#endif /* AOI_H */