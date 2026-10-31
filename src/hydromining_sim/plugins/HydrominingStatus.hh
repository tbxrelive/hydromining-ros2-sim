#pragma once
#include <gz/gui/Plugin.hh>
#include <QString>

class HydrominingStatus : public gz::gui::Plugin
{
  Q_OBJECT
  Q_PROPERTY(QString status READ Status NOTIFY StatusChanged)
public:
  void LoadConfig(const tinyxml2::XMLElement *) override;
  QString Status() const { return status_; }
signals:
  void StatusChanged();
private:
  QString status_ = "Waiting for task data";
};
