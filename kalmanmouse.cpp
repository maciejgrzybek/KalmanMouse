#include "drawablearea.h"
#include "kalmanmouse.h"

KalmanMouse::KalmanMouse(QWidget* parent)
  : QMainWindow(parent)
{
  setCentralWidget(new DrawableArea());

  setWindowTitle(tr("KalmanMouse"));
  resize(500, 500);
}

KalmanMouse::~KalmanMouse()
{
  
}
