#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <QMainWindow>
#include <QApplication>
#include <QTimer>
#include <QProgressBar>
#include <QGraphicsEffect>
#include <QRadioButton>
#include <QTextStream>
#include <QDirIterator>
#include <QThread>

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/RobotState.h>
#include <sensor_msgs/JointState.h>
#include <yaml-cpp/yaml.h>
#include <sensor_msgs/MultiDOFJointState.h>
#include <moveit_msgs/RobotTrajectory.h>


#include <random>
#include <filesystem>

#include <setup_dialog.h>
#include <cameras_view.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = nullptr);
  ~MainWindow();

private:
  void loadData();
  void saveData();
  void openSetup();
  void openCamerasView();
  void buttonsAfterSetup();
  void clearStatusBar();

  void writeYamlPos(sensor_msgs::JointState*, QString);
  void readYamlPos(QString, sensor_msgs::JointState*);
  std::vector<double> readYamlVectorOfPos(QString);
  sensor_msgs::JointState* readYamlPose(QString);

  std::string addFrontZeros(int, int);
  void saveOneImgPerCamera();

  void moveRobot();

  void createNewSession();
  void openExistingSession();

signals:
  void runningQuit();

private slots:
  // CLASS - GENERAL
  void spinOnce();
  void setIntData(int, std::string);
  void setStringData(QString, std::string);

  // ---------------------------------------
  // SETUP
  void on_setupButton_clicked();
  void on_actionOpenSetup_triggered();

  // ---------------------------------------
  // CAMERAS
  void closeCamerasView();
  void on_camerasButton_clicked();
  void on_actionOpenCameraView_triggered();

  // ---------------------------------------
  // SAMPLING
  void on_autoStartButton_clicked();
  void on_autoStopButton_clicked();
  void on_takeSamplesButton_clicked();
  void singleSaveImage(const sensor_msgs::ImageConstPtr&, int);
  void on_addNewPoseButton_clicked();
  void on_deleteLastPoseButton_clicked();
  void on_saveCustomKButton_clicked();

  // ---------------------------------------
  // CALIBRATION
  void on_startCaliButton_clicked();

  // ---------------------------------------
  // ACTIONS
  void on_actionNewSession_triggered();
  void on_actionOpenSession_triggered();
  void on_actionCloseCurrentSession_triggered();
  void on_newSessionPushButton_clicked();
  void on_openSessionPushButton_clicked();
  void on_actionExitApp_triggered();
  void on_openLastSessionPushButton_clicked();

private:
  Ui::MainWindow *ui;

  QTimer *timer;

  QProgressBar *progressBar;
  std::vector<QWidget*> statusBarElements;

  ros::NodeHandlePtr nh_;
  ros::Publisher robotStatePub;
  std::vector<ros::Subscriber> singleSub;

  QString currentSessionPath;

  SetupDialog *sd;
  int cameraNumber;
  int numberOfPoses;
  QString robotTopic; // da cancellare
  QString robotStartingPose; // da cancellare
  QString kineticPath; // da cancellare

  sensor_msgs::JointState* robotJoints;
  std::vector<sensor_msgs::JointState*> vectorOfTrajectory;

  bool runningPhoto;

  CamerasView *cv;
};

#endif // MAIN_WINDOW_H
