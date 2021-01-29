/*
 * Copyright 2019-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_udp/data/RobotControl.h>
#include <mc_udp/data/utils.h>

#include <cstring>

namespace mc_udp
{

size_t RobotControl::size() const
{
#ifdef APPLY_LINK_EXTFORCES // for use with RTCSimExtForce
  return sizeof(uint64_t) + sizeof(uint64_t) + encoders.size() * sizeof(double) + sizeof(uint64_t)
         + encoderVelocities.size() * sizeof(double) + sizeof(uint64_t) + simExtForceFlag.size() * sizeof(int)
         + sizeof(uint64_t) + simExtForceVal.size() * sizeof(sva::ForceVecd);
#else
  return sizeof(uint64_t) + sizeof(uint64_t) + encoders.size() * sizeof(double) + sizeof(uint64_t)
         + encoderVelocities.size() * sizeof(double);
#endif
}

size_t RobotControl::toBuffer(uint8_t * buffer) const
{
  size_t offset = 0;
  utils::memcpy_advance(buffer, &id, sizeof(uint64_t), offset);
  utils::toBuffer(buffer, encoders, offset);
  utils::toBuffer(buffer, encoderVelocities, offset);
#ifdef APPLY_LINK_EXTFORCES // for use with RTCSimExtForce
  utils::toBuffer(buffer, simExtForceFlag, offset);
  utils::toBuffer(buffer, simExtForceVal, offset);
#endif
  return offset;
}

size_t RobotControl::fromBuffer(uint8_t * buffer)
{
  size_t offset = 0;
  utils::memcpy_advance(&id, buffer, sizeof(uint64_t), offset);
  utils::fromBuffer(encoders, buffer, offset);
  utils::fromBuffer(encoderVelocities, buffer, offset);
#ifdef APPLY_LINK_EXTFORCES // for use with RTCSimExtForce
  utils::fromBuffer(simExtForceFlag, buffer, offset);
  utils::fromBuffer(simExtForceVal, buffer, offset);
#endif
  return offset;
}

} // namespace mc_udp
