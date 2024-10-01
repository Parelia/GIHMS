#ifndef SETUP_DIALOG_H
#define SETUP_DIALOG_H

#include <QDialog>
#include <QFileDialog>
#include <QMessageBox>
#include <QInputDialog>
#include <QComboBox>
#include <QSettings>
#include <QTextStream>

#include <tinyxml2.h>

#include <yaml-cpp/yaml.h>

#include <ros/ros.h>

#include <vector>

namespace Ui {
class SetupDialog;
struct Position { std::string x, y, a; }; // Only for simulation
}

class SetupDialog : public QDialog
{
  Q_OBJECT

public:
  explicit SetupDialog(QWidget *parent = nullptr);
  ~SetupDialog();

  void setSessionPath(QString);

private:
  void loadData();
  void saveData();
  void writeYamlInfo(QString);

private slots:
  void on_buttonBox_accepted();
  void on_addKnButton_clicked();
  void on_rmKnButton_clicked();

signals:
  void sendIntData(int, std::string);
  void sendStringData(QString, std::string);

private:
  Ui::SetupDialog *ui;

  QString sessionPath;
};

#endif // SETUP_DIALOG_H
