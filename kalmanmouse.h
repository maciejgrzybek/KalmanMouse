#ifndef KALMANMOUSE_H
#define KALMANMOUSE_H

#include <QMainWindow>

class KalmanMouse : public QMainWindow
{
  Q_OBJECT
  
public:
  KalmanMouse(QWidget* parent = 0);
  virtual ~KalmanMouse();
};

#endif // KALMANMOUSE_H
