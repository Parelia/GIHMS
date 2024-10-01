#include "setup_dialog.h"
#include "ui_setup_dialog.h"

SetupDialog::SetupDialog(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::SetupDialog)
{
  ui->setupUi(this);

  this->setAttribute(Qt::WA_DeleteOnClose);

  loadData();
}

SetupDialog::~SetupDialog()
{
  saveData();

  delete ui;
}


// ----------------------------------------------------------------------------
// CLASS - GENERAL FUNCTION
/*
 * To read the Qt stored style data
*/
void SetupDialog::loadData()
{
  QSettings settings("UniPd", "GUI");

  // Load cameraNumberSpinBox
  ui->cameraNumberSpinBox->setValue(settings.value("cameraNumberSpinBox").toInt());

  // Load knComboBox
  settings.beginGroup("knComboBox");
  int count = settings.value("count").toInt();
  for (int i = 0; i < count; ++i) {
    QString name = settings.value(QString::number(i)).toString();
    QVariant item = settings.value(name);
    ui->trajComboBox->addItem(name, item);
  }
  settings.endGroup();
}

/*
 * To store data in Qt style
*/
void SetupDialog::saveData()
{
  QSettings settings("UniPd", "GUI");

  // Remove old key in cameraNumberSpinBox
  settings.remove("cameraNumberSpinBox");

  // Remove old key in nPosesSpinBox
  settings.remove("nPosesSpinBox");

  // Remove old keys in rtComboBox
  settings.beginGroup("rtComboBox");
  settings.remove(""); // delete all
  settings.endGroup();

  // Remove old keys in rspComboBox
  settings.beginGroup("rspComboBox");
  settings.remove(""); // delete all
  settings.endGroup();

  // Remove old keys in knComboBox
  settings.beginGroup("knComboBox");
  settings.remove(""); // delete all
  settings.endGroup();

  // Save cameraNumberSpinBox
  settings.setValue("cameraNumberSpinBox", ui->cameraNumberSpinBox->value());

  // Save knComboBox
  settings.beginGroup("knComboBox");
  int count = ui->trajComboBox->count();
  settings.setValue("count", count);
  for (int i = 0; i < count; ++i) {
    settings.setValue(QString::number(i), ui->trajComboBox->itemText(i));
    settings.setValue(ui->trajComboBox->itemText(i), ui->trajComboBox->itemData(i));
  }
  settings.endGroup();
}

void SetupDialog::setSessionPath(QString s) { sessionPath = s; }

/*
 * Writes a .yaml file with "info style" specifics
*/
void SetupDialog::writeYamlInfo(QString fileName) // NOT SLOT
{
  if(!fileName.endsWith(".yaml", Qt::CaseInsensitive)) return;

  // Build file YAML structure
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "number_of_cameras" << YAML::Value << ui->cameraNumberSpinBox->value();
  out << YAML::Comment("Number of cameras you want to calibrate");
  out << YAML::Key << "camera_folder_prefix" << YAML::Value << "camera";
  out << YAML::Comment("Camera folder name where pose and image subfolders are located");
  out << YAML::Key << "pattern_type" << YAML::Value << "checkerboard";
  out << YAML::Comment("Pattern type used");
  out << YAML::Key << "number_of_rows" << YAML::Value << ui->patternRowsSpinBox->value();
  out << YAML::Comment("Number of rows");
  out << YAML::Key << "number_of_columns" << YAML::Value << ui->patternColumnsSpinBox->value();
  out << YAML::Comment("Number of columns");
  out << YAML::Key << "size" << YAML::Value << std::to_string(ui->patternSizeSpinBox->value());
  out << YAML::Comment("Pattern size");
  out << YAML::Key << "resize_factor" << YAML::Value << 1;
  out << YAML::Comment("Resize factor");
  out << YAML::Key << "visual_error" << YAML::Value << 1;
  out << YAML::Comment("Store reprojected corners: 0 to remove visualization, 1 if you want to store them");
  out << YAML::Key << "gt" << YAML::Value << 1;
  out << YAML::Comment("Evaluation with ground truth: 0 if the GT is not available, 1 if the ground truth is provided in GT folder");
  if (ui->eyeBaseRadioButton->isChecked())
    out << YAML::Key << "calibration_setup" << YAML::Value << 1;
  else
    out << YAML::Key << "calibration_setup" << YAML::Value << 0;
  out << YAML::Comment("Calibration type: 0 eye-in-hand, 1 eye-on-base");
  out << YAML::Key << "intrinsic_calibration" << YAML::Value << 0;
  out << YAML::Comment("Calibration info: 0 not needed, 1 perform it");
  out << YAML::Key << "metric" << YAML::Value << 1;
  out << YAML::Comment("Metric AX=ZB");
  out << YAML::EndMap;

  // Write file Qt style
  QFile file(fileName);
  if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
      QTextStream outStream(&file);
      outStream << QString::fromStdString(out.c_str());
      file.close();
  }
}

// ----------------------------------------------------------------------------
// SLOTS
/*
 * Sends all updated data to main_window
*/
void SetupDialog::on_buttonBox_accepted() // Adapted for simulation
{
  // Update cameraNumber
  emit sendIntData(ui->cameraNumberSpinBox->value(), "cameraNumber");

  // Update trajectoryPath
  emit sendStringData(ui->trajComboBox->currentData().toString(), "trajectoryPath");

  // Set environment
  writeYamlInfo(sessionPath + "/CalibrationInfo.yaml");
}

/*
 * Adds an instance to the kinetic combo box
*/
void SetupDialog::on_addKnButton_clicked()
{
  QString dir = QFileDialog::getExistingDirectory(nullptr, tr("Select Kinematic Directory"));
  if(!dir.isEmpty()) {
    bool ok = true;
    QString name, s = tr("Directory selected: ") + dir + tr("\nSave name:");
    while(ok) {
      name = QInputDialog::getText(nullptr, tr("Save New Kinematic Preset"), s, QLineEdit::Normal, "", &ok);
      if(name.isEmpty());
      else if(ui->trajComboBox->findText(name, Qt::MatchFixedString) != -1)
        QMessageBox::warning(nullptr, tr("Bad Name"), tr("Selected name already assigned"));
      else {
        ui->trajComboBox->addItem(name, dir);
        ok = false;
      }
    }
  }
  else QMessageBox::information(nullptr, tr("Saving aborted"), tr("No directory have been selected"));
}

/*
 * Removes an instance from the kinetic combo box
*/
void SetupDialog::on_rmKnButton_clicked()
{
  if(ui->trajComboBox->currentIndex() < 1) QMessageBox::warning(nullptr, tr("Bad Index"), tr("Default value can't be removed"));
  else {
    if(QMessageBox::question(nullptr, tr("Removing kinematic"), tr("Are you sure?")) == QMessageBox::Yes) {
      ui->trajComboBox->removeItem(ui->trajComboBox->currentIndex());
      ui->trajComboBox->setCurrentIndex(0);
    }
  }
}

