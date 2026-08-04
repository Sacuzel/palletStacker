#ifndef PALLET_LOADER_FORKLIFT_TELEOP_HH_
#define PALLET_LOADER_FORKLIFT_TELEOP_HH_

#include <memory>
#include <string>

#include <gz/gui/Plugin.hh>
#include <gz/gui/qt.h>
#include <gz/transport/Node.hh>

namespace gz
{
namespace msgs
{
class WorldStatistics;
}
}

class ForkliftTeleopPrivate;

/// \brief Gazebo GUI keyboard controller for the simplified forklift.
///
/// Unlike Gazebo's stock KeyPublisher, this plugin reads QKeyEvent modifiers
/// and key-release events. That is required to distinguish Up from Shift+Up
/// and to implement true press-and-hold motion with an acceleration ramp.
class ForkliftTeleop : public gz::gui::Plugin
{
  Q_OBJECT

  public: ForkliftTeleop();
  public: ~ForkliftTeleop() override;

  public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

  protected: bool eventFilter(QObject *_obj, QEvent *_event) override;

  private slots: void OnControlTick();

  /// \brief Cache the latest simulation clock and pause state.
  private: void OnWorldStats(const gz::msgs::WorldStatistics &_msg);

  private: std::unique_ptr<ForkliftTeleopPrivate> dataPtr;
};

#endif
