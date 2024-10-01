#ifndef CAMERAS_VIEW_H
#define CAMERAS_VIEW_H

#include <QApplication>
#include <QWidget>
#include <QTimer>
#include <QLabel>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <vector>
#include <functional>

namespace Ui {
class CamerasView;
}

class CamerasView : public QWidget
{
  Q_OBJECT

public:
  explicit CamerasView(QWidget *parent = nullptr, int n = 1);
  ~CamerasView();

private:
  std::string addFrontZeros(int, int);
  void imgCallBack(const sensor_msgs::ImageConstPtr&, int);
  QImage MatToQImage(const cv::Mat&);

signals:
  void closeWidget();

private slots:
  void on_closeButton_clicked();
  void on_viewButton_clicked();

private:
  Ui::CamerasView *ui;
  std::pair<int,int> widgetSize;

  int cameraNumber;
  int cameraGridSize;
  std::vector<QLabel*> labelVector;

  ros::NodeHandlePtr nh_;
  std::vector<ros::Subscriber> vSub_;

  bool runningSingle;
};

#endif // CAMERAS_VIEW_H
