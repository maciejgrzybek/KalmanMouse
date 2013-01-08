#include <QApplication>
#include "kalmanmouse.h"

int main(int argc, char* argv[])
{
  QApplication a(argc, argv);
  KalmanMouse w;
  w.show();
  
  return a.exec();
}
