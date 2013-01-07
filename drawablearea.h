#ifndef DRAWABLEAREA_H
#define DRAWABLEAREA_H

#include <QWidget>
#include <QImage>
#include <QColor>
#include <QPoint>

class DrawableArea : public QWidget
{
  Q_OBJECT
public:
  explicit DrawableArea(QWidget* parent = 0);
  
signals:
  
public slots:

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
  
};

#endif // DRAWABLEAREA_H
