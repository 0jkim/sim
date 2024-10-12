// aoi.h
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
  double GetCurrentAoI () const;

  void IncrementAoI (uint32_t slots);

  void ResetAoI (Time currentTime);

  void UpdateWMA (bool success);
  uint32_t GetWMA () const;

  void SetMetricWeight (double x); // 0jkim : 메트릭 가중치 설정 메서드
  double GetMetricWeight () const; // 0jkim : 메트릭 가중치 반환 메서드
  double CalculateMetric (double x) const; // 0jkim : 메트릭 계산 메서드

private:
  Time m_packetCreationTime; // 패킷 생성 시간
  Time m_lastUpdateTime; // 마지막 AoI 업데이트 시간
  double m_currentAoI; // 현재 AoI 값
  uint32_t m_wma; // WMA 값
  uint32_t m_totalTransmissions; // 총 전송 횟수
  double m_metricWeight; // 메트릭 가중치
};

} // namespace ns3

#endif /* AOI_H */