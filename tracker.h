#ifndef TRACKER_H
#define TRACKER_H

#include <QPoint>

#include "kalmanfilter.hpp"

class Tracker
{
public:
  Tracker();
  QPoint initializeStartState(QPoint pos);
  QPoint getTrackPosition(QPoint pos);

private:
  KalmanFilter<double> kf;
};

#endif // TRACKER_H
