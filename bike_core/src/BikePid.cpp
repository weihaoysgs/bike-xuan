#include "bike_core/BikePid.hpp"

BikePid::BikePid()
{
    LOG(INFO) << "Bike PID";
}

float BikePid::operator()(const float target, float current) const {
  LOG(INFO) << "calculate PID";
}
