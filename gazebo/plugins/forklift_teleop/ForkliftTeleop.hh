#ifndef PALLET_STACKER_FORKLIFT_TELEOP_HH_
#define PALLET_STACKER_FORKLIFT_TELEOP_HH_

#include <memory>

#include <QString>

#include <gz/gui/Plugin.hh>
#include <gz/gui/qt.h>

class ForkliftTeleopPrivate;

/// \brief Gazebo GUI plugin that controls the simplified forklift directly
/// through Gazebo Transport. It captures physical key press / release events,
/// ramps chassis velocities, and publishes a persistent fork position target.
class ForkliftTeleop final : public gz::gui::Plugin
{
  Q_OBJECT

  Q_PROPERTY(double linearVelocity READ LinearVelocity NOTIFY StateChanged)
  Q_PROPERTY(double angularVelocity READ AngularVelocity NOTIFY StateChanged)
  Q_PROPERTY(double forkTarget READ ForkTarget NOTIFY StateChanged)
  Q_PROPERTY(QString controlsText READ ControlsText NOTIFY ConfigChanged)
  Q_PROPERTY(QString statusText READ StatusText NOTIFY StatusChanged)

  public: ForkliftTeleop();
  public: ~ForkliftTeleop() override;

  public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

  public: double LinearVelocity() const;
  public: double AngularVelocity() const;
  public: double ForkTarget() const;
  public: QString ControlsText() const;
  public: QString StatusText() const;

  /// \brief Immediately command zero chassis velocity. The fork target is not
  /// changed, so the forks continue holding their selected position.
  public: Q_INVOKABLE void Stop();

  signals: void StateChanged();
  signals: void ConfigChanged();
  signals: void StatusChanged();

  protected: bool eventFilter(QObject *_watched, QEvent *_event) override;

  private slots: void OnUpdate();
  private slots: void OnAutoStartAttempt();

  private: std::unique_ptr<ForkliftTeleopPrivate> dataPtr;
};

#endif  // PALLET_STACKER_FORKLIFT_TELEOP_HH_
