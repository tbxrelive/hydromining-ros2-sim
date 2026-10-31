#include "HydrominingStatus.hh"
#include <QFile>
#include <QTimer>
#include <gz/plugin/Register.hh>

void HydrominingStatus::LoadConfig(const tinyxml2::XMLElement *)
{
  this->title = "Hydromining status";
  auto timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, [this]() {
    QFile file(qEnvironmentVariable("HYDROMINING_GUI_STATUS"));
    if (file.open(QIODevice::ReadOnly)) {
      const QString value = QString::fromUtf8(file.readAll());
      if (value != status_) {
        status_ = value;
        emit StatusChanged();
      }
    }
  });
  timer->start(500);
}

GZ_ADD_PLUGIN(HydrominingStatus, gz::gui::Plugin)
