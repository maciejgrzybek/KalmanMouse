#include "tracker.h"

Tracker::Tracker()
  : kf(4,2)
{
  KalmanFilter<double>::matrix R(2,2); // measurement noise
  KalmanFilter<double>::matrix Q(4,4); // process noise
  KalmanFilter<double>::matrix trans(4,4);
  KalmanFilter<double>::matrix H(2,4);

  /*
   * 0.00001 0.00000
   * 0.00000 0.00001
   */
  R.clear();
  R(0,0) = 1000;
  R(1,1) = 1000;

  /*
   * 0.01 0.00 0.00 0.00
   * 0.00 0.01 0.00 0.00
   * 0.00 0.00 0.01 0.00
   * 0.00 0.00 0.00 0.01
   */
  Q.clear();
  Q(0,0) = 0.01;
  Q(1,1) = 0.01;
  Q(2,2) = 0.01;
  Q(3,3) = 0.01;

  /*
   * 1 0 1 0
   * 0 1 0 1
   * 0 0 1 0
   * 0 0 0 1
   */
  trans.clear();
  trans(0,0) = 1;
  trans(0,1) = 0;
  trans(0,2) = 1;
  trans(0,3) = 0;

  trans(1,0) = 0;
  trans(1,1) = 1;
  trans(1,2) = 0;
  trans(1,3) = 1;

  trans(2,0) = 0;
  trans(2,1) = 0;
  trans(2,2) = 1;
  trans(2,3) = 0;

  trans(3,0) = 0;
  trans(3,1) = 0;
  trans(3,2) = 0;
  trans(3,3) = 1;

  /*
   * 1 0 0 0
   * 0 1 0 0
   */
  H.clear();
  H(0,0) = 1;
  H(0,1) = 0;
  H(0,2) = 0;
  H(0,3) = 0;
  H(1,0) = 0;
  H(1,1) = 1;
  H(1,2) = 0;
  H(1,3) = 0;


  kf.setTransitionModel(trans);
  kf.setMeasurementModel(H);
  kf.setMeasurementNoise(R);
  kf.setProcessNoise(Q);
}

void Tracker::initializeStartState(QPoint pos)
{
  KalmanFilter<double>::vector state(4);
  KalmanFilter<double>::matrix covErr(4,4);

  /*
   * x
   * y
   * 0
   * 0
   */
  state.clear();
  state(0) = pos.x(); // X coord
  state(1) = pos.y(); // Y coord
  state(2) = 0; // Vx
  state(3) = 0; // Vy

  /*
   * 0.1 0.0 0.0 0.0
   * 0.0 0.1 0.0 0.0
   * 0.0 0.0 0.1 0.0
   * 0.0 0.0 0.0 0.1
   */
  covErr.clear();
  covErr(0,0) = 0.1;
  covErr(1,1) = 0.1;
  covErr(2,2) = 0.1;
  covErr(3,3) = 0.1;

  kf.initializeState(state,covErr);
}

QPoint Tracker::getTrackPosition(QPoint pos)
{
  kf.predict();
  KalmanFilter<double>::vector z(2);
  z(0) = pos.x();
  z(1) = pos.y();
  auto corr = kf.correct(z);
  return QPoint(corr.first(0),corr.first(1));
}
