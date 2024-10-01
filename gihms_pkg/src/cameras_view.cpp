#include "cameras_view.h"
#include "ui_cameras_view.h"

CamerasView::CamerasView(QWidget *parent, int n) :  // Not completed - Damaged section / vector of subscriber
  QWidget(parent),
  ui(new Ui::CamerasView),
  cameraNumber(n),
  cameraGridSize([] (int n) -> int { int s=2; while(n > s*s) s++; return s; } (cameraNumber))
{
  // Ui setup
  ui->setupUi(this);
  ui->cameraComboBox->setVisible(false);
  // ui->actionCloseWindow->setVisible(false);

  // std:vector setup
  labelVector.reserve(cameraNumber);
  vSub_.reserve(cameraNumber);

  // Instance setup
  this->setAttribute(Qt::WA_DeleteOnClose);
  widgetSize = std::pair<int,int>(ui->page_1->geometry().width(), ui->page_1->geometry().height());
  runningSingle = false;

  // Widgets setup
  nh_.reset(new ros::NodeHandle("~"));
  for(int i=0; i<cameraNumber; i++)
  {
    // Load cameraComboBox
    std::string s1 = "Camera " + std::to_string(i+1), s2 = "/opt_" + addFrontZeros(i+1, 10) + "/kinect2/rgb/image_raw";
    ui->cameraComboBox->addItem(QString::fromStdString(s1), QString::fromStdString(s2));

    // Load title labels
    QLabel *textLabel = new QLabel(this);
    textLabel->setAlignment(Qt::AlignCenter);
    textLabel->setText(QString::fromStdString(s1));
    ui->imagesGridLayout->addWidget(textLabel, 2*(i/cameraGridSize), i%cameraGridSize);

    // Load image labels
    QLabel *imageLabel = new QLabel(this);
    imageLabel->setAlignment(Qt::AlignCenter);
    ui->imagesGridLayout->addWidget(imageLabel, 2*(i/cameraGridSize)+1, i%cameraGridSize);
    labelVector.push_back(imageLabel);

    // Load subscribers
    vSub_.emplace_back(nh_->subscribe<sensor_msgs::Image>(s2, 1, std::bind(&CamerasView::imgCallBack, this, std::placeholders::_1, i)));
  }
}

CamerasView::~CamerasView()
{
  for (ros::Subscriber sub : vSub_) { sub.shutdown(); }
  vSub_.erase(vSub_.begin(), vSub_.end());
  for (QLabel* label : labelVector) { delete label; }
  delete ui;
}


// ----------------------------------------------------------------------------
// CLASS - GENERAL
/*
 * Support function to format the length of a number (ex. from 10 in "0010")
*/
std::string CamerasView::addFrontZeros(int number, int lengthExample) // NOT SLOT
{
  std::string temp = "";
  if(number < lengthExample)
    if(number == 0)
      while(lengthExample > 1) { temp += "0"; lengthExample /= 10; }
    else
      while(number > number/lengthExample) { temp += "0"; lengthExample /= 10; }
  return temp += std::to_string(number);
}

/*
 * Updates an image of the view
*/
void CamerasView::imgCallBack(const sensor_msgs::ImageConstPtr& msg, int index) // Not verified
{
  try {
    if(runningSingle) {
      if(ui->cameraComboBox->currentIndex() == index) {
        // Translate ROS msg in cv::Mat with cv_bridge
        cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
        QImage qimage = MatToQImage(image);

        ui->cameraLabel->setPixmap(QPixmap::fromImage(qimage).scaled(widgetSize.first, widgetSize.second, Qt::KeepAspectRatio));
      }
    }
    else {
      // Translate ROS msg in cv::Mat with cv_bridge
      cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
      QImage qimage = MatToQImage(image);

      labelVector[index]->setPixmap(QPixmap::fromImage(qimage).scaled(widgetSize.first/cameraGridSize, widgetSize.second/(cameraNumber/cameraGridSize), Qt::KeepAspectRatio));
    }
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}

/*
 * Support function for converting a cv::Mat into a QImage
*/
QImage CamerasView::MatToQImage(const cv::Mat& mat)
{
  if (mat.type() == CV_8UC3) { // BGR8 or RGB8
    return QImage(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB888).rgbSwapped();
  } else if (mat.type() == CV_8UC1) { // MONO8
    return QImage(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_Grayscale8);
  } else {
    return QImage();
  }
}


// ----------------------------------------------------------------------------
// UI SLOTS
void CamerasView::on_closeButton_clicked() { emit closeWidget(); }

/*
 * Changes window perspective (single - multi)
*/
void CamerasView::on_viewButton_clicked()
{
  if(runningSingle) {
    ui->stackedWidget->setCurrentIndex(0);
    ui->viewButton->setText(tr("Single-view"));
    ui->cameraComboBox->setVisible(false);
    runningSingle = false;
  }
  else {
    ui->stackedWidget->setCurrentIndex(1);
    ui->viewButton->setText(tr("Multi-view"));
    ui->cameraComboBox->setVisible(true);
    runningSingle = true;
  }
}

