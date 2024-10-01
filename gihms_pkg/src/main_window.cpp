#include "main_window.h"
#include "ui_main_window.h"

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow),
  cv(nullptr),
  cameraNumber(0),
  robotJoints(new sensor_msgs::JointState)
{
  ui->setupUi(this);

  this->setGeometry(QRect(200,200, 800, 600));
  loadSessionName();

  // if() // old session
  //   guiAfterSetup();

  nh_.reset(new ros::NodeHandle("~"));

  timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
  timer->start(100);
}

MainWindow::~MainWindow()
{
  saveSessionName();

  delete ui;
  delete timer;
  for (ros::Subscriber sub : singleSub) { sub.shutdown(); }
  singleSub.erase(singleSub.begin(), singleSub.end());
  for (QWidget* widget : statusBarElements) { delete widget; }
  for (sensor_msgs::JointState* robotTrajectory : vectorOfTrajectory) { delete robotTrajectory; }
}


// ----------------------------------------------------------------------------
// CLASS - GENERAL
/*
 * Adjusts the ros::spinOnce to treat this app like a node ros
*/
void MainWindow::spinOnce()
{
  if(ros::ok()) ros::spinOnce();
  else {
    ROS_INFO("Closing application");
    QApplication::quit();
  }
}

/*
 * Slot for communicate with cameras_view
*/
void MainWindow::setIntData(int data, std::string type)
{
  if(type == "cameraNumber")
    this->cameraNumber = data;
}

/*
 * Slot for communicate with cameras_view
*/
void MainWindow::setStringData(QString data, std::string type)
{
  if(type == "trajectoryPath")
    this->trajectoryPath = data;
}

/*
 * To read the Qt stored style data
*/
void MainWindow::loadSessionName() // NOT SLOT
{
  QSettings settings("UniPd", "GIHMs");

  // Load lastSessionNameLabel
  currentSessionPath = settings.value("lastSessionNameLabel").toString();
  ui->lastSessionNameLabel->setText(currentSessionPath.right(currentSessionPath.length() - currentSessionPath.lastIndexOf("/") - 1));
  if(currentSessionPath == "") {
    ui->openLastSessionPushButton->setDisabled(true);
    ui->lastSessionNameLabel->setText("Not found");
  }
}

/*
 * To store data in Qt style
*/
void MainWindow::saveSessionName() // NOT SLOT
{
  QSettings settings("UniPd", "GIHMs");

  // Remove old keys
  settings.remove("lastSessionNameLabel");

  // Save lastSessionNameLabel
  settings.setValue("lastSessionNameLabel", currentSessionPath);
}

/*
 * Update all the buttons after completing sError: package 'franka_description' not foundetup
*/
void MainWindow::guiAfterSetup() // NOT SLOT
{
  ui->camerasButton->setDisabled(false);
  ui->actionOpenCameraView->setDisabled(false);

  ui->setupButton->setDisabled(false);
  ui->actionOpenSetup->setDisabled(false);

  ui->actionExitApp->setDisabled(false);
  ui->actionCloseCurrentSession->setDisabled(false);
  ui->actionOpenSession->setDisabled(false);
  ui->actionNewSession->setDisabled(false);
  ui->actionUserGuide->setDisabled(false);

  ui->takeSamplesButton->setDisabled(false);
  ui->lastPoseButton->setDisabled(false);
  ui->nextPoseButton->setDisabled(false);
  ui->moveToPoseButton->setDisabled(false);
  ui->moveToPoseSpinBox->setDisabled(false);

  ui->autoStartButton->setDisabled(false);
  ui->autoStopButton->setDisabled(false);
  ui->startPoseSpinBox->setDisabled(false);
  ui->stopPoseSpinBox->setDisabled(false);

  //if() { // there are some images
  //  ui->startScriptButton->setDisabled(false);
  //  ui->clearImagesButton->setDisabled(false);
  //}
}

/*
 * Clears the status bar objects
*/
void MainWindow::clearStatusBar() // NOT SLOT
{
    for(QWidget* element : statusBarElements) {
    statusBar()->removeWidget(element);
    element->close();
  }
}

/*
 * Reads a .yaml file of a  sensor_msgs::JointState object and stores it in an initialized pointer
*/
void MainWindow::readYamlPos(QString fileName, sensor_msgs::JointState *jointStateMsg) // NOT SLOT
{
  // Robot state jointStateMsg
  if(!fileName.endsWith(".yaml", Qt::CaseInsensitive)) { return; }
  else
  {
    // Read file YAML with YAML basic
    YAML::Node config = YAML::LoadFile(fileName.toStdString());

    // START PARSING
    YAML::Node joint_state_node = config["state"][0];  // Assumed it's the first element
    if (joint_state_node) {
      YAML::Node joint_state_header = joint_state_node["state"][0];
      YAML::Node joint_names = joint_state_node["state"][1];
      YAML::Node joint_positions = joint_state_node["state"][2];
      YAML::Node joint_velocities = joint_state_node["state"][3];
      YAML::Node joint_efforts = joint_state_node["state"][4];

      // Parsing header
      jointStateMsg->header.seq = joint_state_header["state"][0].as<int>();
      jointStateMsg->header.stamp.sec = joint_state_header["state"][1]["state"][0].as<int>(); // time_1
      jointStateMsg->header.stamp.nsec = joint_state_header["state"][1]["state"][1].as<int>(); // time_2
      jointStateMsg->header.frame_id = joint_state_header["state"][2].as<std::string>();

      // Parsing names
      for (size_t i = 0; i < joint_names.size(); ++i) {
        jointStateMsg->name.push_back(joint_names[i].as<std::string>());
      }
      // Parsing positions
      for (size_t i = 0; i < joint_positions.size(); ++i) {
        jointStateMsg->position.push_back(joint_positions[i].as<double>());
      }
      // Parsing velocities
      for (size_t i = 0; i < joint_velocities.size(); ++i) {
        jointStateMsg->velocity.push_back(joint_velocities[i].as<double>());
      }
      // Parsing efforts
      for (size_t i = 0; i < joint_efforts.size(); ++i) {
        jointStateMsg->effort.push_back(joint_efforts[i].as<double>());
      }
    }
    // END PARSING

    // Update ros::Time
    // jointStateMsg->header.stamp = ros::Time::now();

    return;
  }
}

// Function to parse the joint positions from the YAML file
std::vector<double> MainWindow::readYamlVectorOfPos(QString fileName) {
  std::vector<double> joint_positions;

  if(fileName.endsWith(".yaml", Qt::CaseInsensitive)){
    // Read file YAML with YAML basic
    YAML::Node config = YAML::LoadFile(fileName.toStdString());

    // Navigate through the YAML structure to get joint positions
    try {
        // Find the joint positions within the structure
        const YAML::Node& joint_state = config["state"][0]["state"][2];  // [2] corresponds to the joint positions

        for (YAML::const_iterator it = joint_state.begin(); it != joint_state.end(); ++it) {
            joint_positions.push_back(it->as<double>());
        }
    } catch (const YAML::Exception& e) {
        ROS_ERROR("Error parsing YAML file: %s", e.what());
    }
  }

  return joint_positions;
}

/*
 * Reads a .yaml file of a sensor_msgs::JointState object
*/
sensor_msgs::JointState* MainWindow::readYamlPose(QString fileName) // NOT SLOT
{
  if(!fileName.endsWith(".yaml", Qt::CaseInsensitive)) { return nullptr; }
  else
  {
    // // Read file YAML with Qt Style
    // QFile file(fileName);
    // if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return nullptr;
    // QTextStream in(&file);
    // YAML::Node config = YAML::Load(in.readAll().toStdString());
    // file.close();

    // Robot state
    sensor_msgs::JointState *jointStateMsg = new sensor_msgs::JointState();

    // Read file YAML with YAML basic
    YAML::Node config = YAML::LoadFile(fileName.toStdString());

    // START PARSING
    YAML::Node joint_state_node = config["state"][0];  // Assumed it's the first element
    if (joint_state_node) {
      YAML::Node joint_state_header = joint_state_node["state"][0];
      YAML::Node joint_names = joint_state_node["state"][1];
      YAML::Node joint_positions = joint_state_node["state"][2];
      YAML::Node joint_velocities = joint_state_node["state"][3];
      YAML::Node joint_efforts = joint_state_node["state"][4];

      // Parsing header
      jointStateMsg->header.seq = joint_state_header["state"][0].as<int>();
      jointStateMsg->header.stamp.sec = joint_state_header["state"][1]["state"][0].as<int>(); // time_1
      jointStateMsg->header.stamp.nsec = joint_state_header["state"][1]["state"][1].as<int>(); // time_2
      jointStateMsg->header.frame_id = joint_state_header["state"][2].as<std::string>();

      // Parsing names
      for (size_t i = 0; i < joint_names.size(); ++i) {
        jointStateMsg->name.push_back(joint_names[i].as<std::string>());
      }
      // Parsing positions
      for (size_t i = 0; i < joint_positions.size(); ++i) {
        jointStateMsg->position.push_back(joint_positions[i].as<double>());
      }
      // Parsing velocities
      for (size_t i = 0; i < joint_velocities.size(); ++i) {
        jointStateMsg->velocity.push_back(joint_velocities[i].as<double>());
      }
      // Parsing efforts
      for (size_t i = 0; i < joint_efforts.size(); ++i) {
        jointStateMsg->effort.push_back(joint_efforts[i].as<double>());
      }
    }
    // END PARSING

    return jointStateMsg;
  }
}

/*
 * Support function to format the length of a number (ex. from 10 in "0010")
*/
std::string MainWindow::addFrontZeros(int number, int lengthExample) // NOT SLOT
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
 * Calls singleSaveImage for every camera
*/
void MainWindow::saveOneImgPerCamera() // NOT SLOT - To update topics
{
  for(int i=0; i<cameraNumber; i++) {
    std::string s = "/opt_" + addFrontZeros(i+1, 10) + "/kinect2/rgb/image_raw";
    singleSub[i] = nh_->subscribe<sensor_msgs::Image>(s, 10, std::bind(&MainWindow::singleSaveImage, this, std::placeholders::_1, i));
  }
}

// ----------------------------------------------------------------------------
// SETUP
/*
 * Opens a neww setup_dialog window
*/
void MainWindow::openSetup() // NOT SLOT
{
  ROS_INFO("Setup dialog opened");
  statusBar()->showMessage(tr("Editing Setup"));

  sd = new SetupDialog(this);
  sd->setWindowTitle("Setup");
  sd->setSessionPath(currentSessionPath);
  connect(sd, &SetupDialog::sendIntData, this, &MainWindow::setIntData);
  connect(sd, &SetupDialog::sendStringData, this, &MainWindow::setStringData);
  sd->exec();
  disconnect(sd, &SetupDialog::sendIntData, this, &MainWindow::setIntData);
  disconnect(sd, &SetupDialog::sendStringData, this, &MainWindow::setStringData);

  ROS_INFO("Setup saved");
  statusBar()->showMessage(tr("Setup Saved"), 1000);

  if (cameraNumber != 0) { // setup = ok
    // clear previus
    for (ros::Subscriber sub : singleSub) { sub.shutdown(); }
    singleSub.erase(singleSub.begin(), singleSub.end());
    // attenzione che elimina il progresso non salvato
    for (sensor_msgs::JointState* robotTrajectory : vectorOfTrajectory) { delete robotTrajectory; }

    ui->setupStatusLabel->setText("Completed");

    for(int i=0; i<cameraNumber; i++) singleSub.emplace_back(ros::Subscriber());

    QDirIterator it(trajectoryPath, QStringList() << "*.yaml", QDir::Files, QDirIterator::Subdirectories);
    int i=0;
    while(it.hasNext()) i++;
    vectorOfTrajectory.reserve(i);

    guiAfterSetup();

    QDir dir;
    for(int i=1; i<=cameraNumber; i++)
      dir.mkdir(currentSessionPath + "/Camera " + QString::fromStdString(std::to_string(i)));
    dir.mkdir(currentSessionPath + "/dataset");
    dir.mkdir(currentSessionPath + "/dataset/start_pos");
  }
  else ui->setupStatusLabel->setText("Not completed/valid");
}

void MainWindow::on_setupButton_clicked() { openSetup(); }

void MainWindow::on_actionOpenSetup_triggered() { openSetup(); }


// ----------------------------------------------------------------------------
// CAMERAS
/*
 * Opens a new cameras_view window
*/
void MainWindow::openCamerasView() // NOT SLOT
{
  ROS_INFO("Cameras view opened successfully");
  statusBar()->showMessage(tr("Viewing cameras"));

  if(cv != nullptr) closeCamerasView();
  cv = new CamerasView(this, cameraNumber);
  ui->stackedWidget->insertWidget(2, cv);
  ui->stackedWidget->setCurrentIndex(2);
  connect(cv, &CamerasView::closeWidget, this, &MainWindow::closeCamerasView);
}

/*
 * Closes the current cameras_view window
*/
void MainWindow::closeCamerasView()
{
  ui->stackedWidget->setCurrentIndex(1);
  ui->stackedWidget->removeWidget(cv);
  cv->close();
  cv = nullptr;

  disconnect(cv, &CamerasView::closeWidget, this, &MainWindow::closeCamerasView);

  statusBar()->clearMessage();
  ROS_INFO("Cameras view closed");
}

void MainWindow::on_camerasButton_clicked() { openCamerasView(); }

void MainWindow::on_actionOpenCameraView_triggered() { openCamerasView(); }


// ----------------------------------------------------------------------------
// SAMPLING

void MainWindow::moveRobot() {
  // Define the MoveGroup interface for the robot's arm
  static const std::string PLANNING_GROUP = "panda_arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  int maxP = ui->stopPoseSpinBox->value();
  for(int i=0; i<maxP; i++) {
    QString s = "home/iaslab/parelia_ws/src/first_gui_pkg/rc/dataset_sphere_tilt/start_pos/" + QString::fromStdString(addFrontZeros(i, 1000)) + ".yaml";
    std::vector<double> target_joint_positions = readYamlVectorOfPos(s);

    if (target_joint_positions.empty()) {
      ROS_ERROR("Failed to load joint positions from YAML.");
      return;
    }

    // Set the target joint values
    move_group.setJointValueTarget(target_joint_positions);

    // Plan and move to the target joint positions
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success) {
        move_group.move();
        ROS_INFO("Motion executed successfully.");
    } else {
        ROS_WARN("Motion planning failed.");
    }

    QThread::sleep(200);
  }
}

/*
 * Performs all the sampling process
*/
void MainWindow::on_autoStartButton_clicked() // Not completed - Wait robot - Path/name of _info.yaml to update
{
  if(true) // solo per non vedere il codice
  {
  ROS_INFO("Starting autonomous sampling routine");
  // Disable all ui
  ui->setupButton->setDisabled(true);
  ui->actionOpenSetup->setDisabled(true);

  ui->actionExitApp->setDisabled(true);
  ui->actionCloseCurrentSession->setDisabled(true);
  ui->actionOpenSession->setDisabled(true);
  ui->actionNewSession->setDisabled(true);
  ui->actionUserGuide->setDisabled(true);

  ui->takeSamplesButton->setDisabled(true);
  ui->lastPoseButton->setDisabled(true);
  ui->nextPoseButton->setDisabled(true);
  ui->moveToPoseButton->setDisabled(true);
  ui->moveToPoseSpinBox->setDisabled(true);

  ui->autoStartButton->setDisabled(true);
  ui->startPoseSpinBox->setDisabled(true);
  ui->stopPoseSpinBox->setDisabled(true);

  ui->startScriptButton->setDisabled(true);
  ui->clearImagesButton->setDisabled(true);
  // Except for
  ui->autoStopButton->setDisabled(false);
  ui->camerasButton->setDisabled(false);
  ui->actionOpenCameraView->setDisabled(false);


  statusBarElements.reserve(2);
  runningPhoto = true;

  QLabel *label = new QLabel(this);
  label->setText(tr("Sampling:"));
  statusBar()->insertWidget(0, label);
  statusBarElements.push_back(label);

  progressBar = new QProgressBar(this);
  progressBar->setTextVisible(false);
  statusBar()->insertWidget(1, progressBar, 1);
  statusBarElements.push_back(progressBar);
  }

  moveRobot();


  // moveit_msgs::RobotTrajectory* currentTrajectory;
  // for(int i=1; i<=numberOfPoses && runningPhoto; i++) {
  //   currentTrajectory = readYamlRobotTrajectory(kineticPath + "/" + QString::fromStdString(addFrontZeros(i, 1000)) + ".yaml");
  //   robotStatePub.publish(*currentTrajectory);

  //   //sleep() until robot is still

  //   progressBar->setValue(progressBar->value() + (500/numberOfPoses)/10);

  //   saveOneImgPerCamera();

  //   progressBar->setValue(progressBar->value() + (500/numberOfPoses)/10);

  //   emit runningQuit();

  //   delete currentTrajectory;
  // }

  // emit on_startCaliButton_clicked();
}

/*
 * Stops the current automatic sampling process
*/
void MainWindow::on_autoStopButton_clicked()  // With delete images function
{
  runningPhoto = false;

  clearStatusBar();
  statusBar()->showMessage(tr("Closing process"));

  // wWait the closing
  QEventLoop loop;
  connect(this, &MainWindow::runningQuit, &loop, &QEventLoop::quit);
  loop.exec();

  statusBar()->showMessage(tr("Sampling interrupted"), 2000);

  guiAfterSetup();
  ui->autoStopButton->setDisabled(true);

  // Delete or not the images
  // if(!ui->saveImagesButton->isChecked()) {
  //   QDirIterator it(currentSessionPath, QStringList() << "*.jpeg", QDir::Files, QDirIterator::Subdirectories);
  //   // Itera attraverso i file .jpeg trovati
  //   while (it.hasNext()) {
  //       QString filePath = it.next(); // Percorso completo del file
  //       QFile::remove(filePath);
  //   }
  // }
}

void MainWindow::on_takeSamplesButton_clicked() { saveOneImgPerCamera(); }

/*
 * Stores in the selected path the given message as an .jpeg image
*/
void MainWindow::singleSaveImage(const sensor_msgs::ImageConstPtr& msg, int index) { // Not verified
  try {
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

    QString s = currentSessionPath + "/Camera " + QString::fromStdString(std::to_string(index+1));
    QDir dir(s);
    QStringList fileList = dir.entryList(QStringList() << "*.jpeg", QDir::Files);
    s += "/" + QString::fromStdString(addFrontZeros(fileList.size(), 1000)) + ".jpeg";

    cv::imwrite(s.toStdString(), image);
    ROS_INFO("Camera %i: image saved successfully", index+1);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  singleSub[index].shutdown();
}

/*
 * Generates a new pose for the robot and sends it
*/
void MainWindow::on_addNewPoseButton_clicked()  // Not completed - How much random? (Position)
{
  // // Create msg randomPosition for robot
  // sensor_msgs::JointState* robotTrajectory = new sensor_msgs::JointState();

  // std::random_device rd;  // Per generare un seme casuale
  // std::mt19937 gen(rd()); // Inizializza il generatore con il seme

  // // Crea una distribuzione normale (gaussiana)
  // std::normal_distribution<> dis1(0.0, 0.6);
  // std::normal_distribution<> dis2(0.0, 0.3);

  // robotTrajectory->joint_trajectory.header.seq = 0;
  // robotTrajectory->joint_trajectory.header.stamp = ros::Time::now();
  // robotTrajectory->joint_trajectory.header.frame_id = robotJoints->header.frame_id;

  // for (size_t i = 0; i < robotJoints->name.size(); ++i) {
  //   if(robotJoints->name[i].find("finger") == std::string::npos)
  //     robotTrajectory->joint_trajectory.joint_names.push_back(robotJoints->name[i]);
  // }

  // int cap = robotTrajectory->joint_trajectory.joint_names.size();
  // for (size_t i = 0; i < cap; ++i) {
  //   trajectory_msgs::JointTrajectoryPoint* jointTrajectoryPoint = new trajectory_msgs::JointTrajectoryPoint;

  //   for (size_t i = 0; i < cap; ++i) {
  //     jointTrajectoryPoint->positions.push_back(dis1(gen));
  //   }
  //   for (size_t i = 0; i < cap; ++i) {
  //     jointTrajectoryPoint->velocities.push_back(dis2(gen));
  //   }
  //   for (size_t i = 0; i < cap; ++i) {
  //     jointTrajectoryPoint->accelerations.push_back(dis2(gen));
  //   }
  //   for (size_t i = 0; i < cap; ++i) {
  //     // jointTrajectoryPoint->effort.push_back();
  //   }
  //   // jointTrajectoryPoint->time_from_start = ros::Time::now();

  //   robotTrajectory->joint_trajectory.points.push_back(*jointTrajectoryPoint);
  // }


  // robotStatePub.publish(*robotTrajectory);
  // vectorOfTrajectory.push_back(robotTrajectory);

  // ui->currentPosesNumberLabel->setText(QString::fromStdString(std::to_string(vectorOfTrajectory.size())));
  // ui->saveCustomKButton->setDisabled(false);
  // ui->deleteLastPoseButton->setDisabled(false);
}

/*
 * Delets the current last pose in the vectorOfTrajectory
*/
void MainWindow::on_deleteLastPoseButton_clicked()
{
  vectorOfTrajectory.pop_back();
  // ui->currentPosesNumberLabel->setText(QString::fromStdString(std::to_string(vectorOfTrajectory.size())));
  // if(vectorOfTrajectory.size() == 0) {
  //   ui->saveCustomKButton->setDisabled(true);
  //   ui->deleteLastPoseButton->setDisabled(true);
  // }
}

/*
 * Saves all the poses of vectorOfTrajectory at the poses directory
*/
void MainWindow::on_saveCustomKButton_clicked()
{
  statusBar()->showMessage(tr("Saving kinematic file"));

  // int i=0;
  // for(sensor_msgs::JointState* robotTrajectory : vectorOfTrajectory)
  //   writeYamlPos(robotTrajectory, currentSessionPath + "/dataset/start_pos/" + QString::fromStdString(addFrontZeros(i++, 1000)));
  // vectorOfTrajectory.erase(vectorOfTrajectory.begin(), vectorOfTrajectory.end());

  statusBar()->showMessage(tr("Kinematic files have been saved"), 2000);
}


// ----------------------------------------------------------------------------
// CALIBRATION
/*
 * Performs the calibration process (sets up and call the script?)
*/
void MainWindow::on_startCaliButton_clicked() // Not completed - Calibration stuff
{
  ROS_INFO("Starting calibration script");

  // Calibration stuff / calls the script

  // OUT
  statusBar()->showMessage(tr("Saving calibration file"));

  // hypothetical
  QString dir = QFileDialog::getSaveFileName(nullptr, tr("Save Calibration Result File"), "", tr("Configuration Files (*.yaml)"));
  if(!dir.isEmpty()) {
    if (!dir.endsWith(".yaml", Qt::CaseInsensitive)) {
        dir += ".yaml";
    }

    QFile file(dir);
    if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
      QTextStream out(&file);
      out << "";
      file.close();

      statusBar()->showMessage(tr("Configuration file saved"), 2000);
    }
    else {
      QMessageBox::warning(nullptr, tr("Error"), tr("File has not been saved"));
      statusBar()->showMessage(tr("Configuration file not saved"), 2000);
    }
  }
  else {
    QMessageBox::information(nullptr, tr("Saving aborted"), tr("No file have been selected"));
    statusBar()->showMessage(tr("Configuration file not saved"), 2000);
  }
}


// ----------------------------------------------------------------------------
// ACTIONS

void MainWindow::createNewSession()
{
  QString dirPath = QFileDialog::getExistingDirectory(nullptr, tr("Select New Session Directory"));
  if(!dirPath.isEmpty()) {
    bool ok = true;
    QString name;
    while(ok) {
      name = QInputDialog::getText(nullptr, tr("New Session Name"), tr("Insert name:"), QLineEdit::Normal, "", &ok);
      if(!name.isEmpty()) ok = false;
    }
    currentSessionPath = dirPath + "/" + name;
    QDir dir;
    if(dir.mkdir(currentSessionPath)) {
      // Write session file
      QFile file(currentSessionPath + "/" + name + ".session");
      if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
          QTextStream outStream(&file);
          outStream << name;
          file.close();
      }

      ui->stackedWidget->setCurrentIndex(1);
      ui->sessionNameLabel->setText(name);
      ui->lastSessionNameLabel->setText(name);
      ROS_INFO("\"%s\" session created successfully", name.toStdString().c_str());
    }
    else QMessageBox::information(nullptr, tr("Operation denied"), tr("Not valid path have been selected"));
  }
}

void MainWindow::on_actionNewSession_triggered() { createNewSession(); }

void MainWindow::on_newSessionPushButton_clicked() { createNewSession(); }

void MainWindow::openExistingSession()
{
  QString dirPath = QFileDialog::getExistingDirectory(nullptr, tr("Select Existing Session Directory"));
  if(!dirPath.isEmpty()) {
    QDir dir(dirPath);
    QStringList fileList = dir.entryList(QStringList() << "*.session", QDir::Files);
    if(fileList.size() == 1) {
      currentSessionPath = dirPath;

      QFile file(currentSessionPath + "/" + fileList[0]);
      if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return;
      QTextStream in(&file);
      QString name = in.readAll();
      file.close();

      ui->stackedWidget->setCurrentIndex(1);
      ui->sessionNameLabel->setText(name);
      ui->lastSessionNameLabel->setText(name);
      ROS_INFO("\"%s\" session opened successfully", name.toStdString().c_str());

    }
    else QMessageBox::information(nullptr, tr("Operation denied"), tr("Selected directory is not valid"));
  }
}

void MainWindow::on_actionOpenSession_triggered() { openExistingSession(); }

void MainWindow::on_openSessionPushButton_clicked() { openExistingSession(); }

void MainWindow::on_openLastSessionPushButton_clicked()
{
  QSettings settings("UniPd", "GUI");
  QString s = settings.value("lastSessionNameLabel").toString();
  QDir dir(s);
  QStringList fileList = dir.entryList(QStringList() << "*.session", QDir::Files);
  if(fileList.size() == 1) {
    currentSessionPath = s;

    QFile file(currentSessionPath + "/" + fileList[0]);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return;
    QTextStream in(&file);
    QString name = in.readAll();
    file.close();

    ui->stackedWidget->setCurrentIndex(1);
    ui->sessionNameLabel->setText(name);
    ui->lastSessionNameLabel->setText(name);
    ROS_INFO("\"%s\" session opened successfully", name.toStdString().c_str());
  }
  else QMessageBox::warning(nullptr, tr("Operation denied"), tr("Selected directory is not valid"));
}

/*
 * Closes current session
*/
void MainWindow::on_actionCloseCurrentSession_triggered()
{
  if(ui->stackedWidget->currentIndex() == 2) closeCamerasView();
  if(ui->stackedWidget->currentIndex() != 0) {
    ROS_INFO("Session closed");
    ui->openLastSessionPushButton->setDisabled(false);
    saveSessionName();
  }
  ui->stackedWidget->setCurrentIndex(0);
}

/*
 * Closes app on call
*/
void MainWindow::on_actionExitApp_triggered()
{
  if(ui->stackedWidget->currentIndex() == 2) closeCamerasView();
  ROS_INFO("Closing application");
  QApplication::quit();
}
