#ifndef DRAWABLEAREA_H
#define DRAWABLEAREA_H

#include <QWidget>
#include <QImage>
#include <QColor>
#include <QPoint>
#include <QTimer>

#include "tracker.h"

class DrawableArea : public QWidget
{
  Q_OBJECT
public:
  explicit DrawableArea(QWidget* parent = 0);
  
signals:
  
protected slots:
  void refreshTrack();

protected:
  void paintEvent(class QPaintEvent* event);
  void resizeEvent(class QResizeEvent* event);
  bool event(class QEvent* event);

private:
  void resizeImage(const QSize& newSize);

  QImage image;
  QColor backgroundColor;
  QColor userPenColor;
  QColor trackerPenColor;
  QPoint lastPoint;
  QPoint lastTrack;

  QTimer timer;

  Tracker tracker;
};

#endif // DRAWABLEAREA_H
