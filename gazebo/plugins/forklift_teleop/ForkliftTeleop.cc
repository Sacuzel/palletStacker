#include "ForkliftTeleop.hh"

#include <algorithm>
#include <cmath>
#include <string>

#include <QElapsedTimer>
#include <QEvent>
#include <QKeyEvent>
#include <QPointer>
#include <QSet>
#include <QTimer>

#include <tinyxml2.h>

#include <gz/common/Console.hh>
#include <gz/gui/Application.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

namespace
{
//////////////////////////////////////////////////
double Approach(const double _current, const double _target,
                const double _maximumDelta)
{
  if (_current < _target)
    return std::min(_current + _maximumDelta, _target);
  if (_current > _target)
    return std::max(_current - _maximumDelta, _target);
  return _target;
}

//////////////////////////////////////////////////
std::string ReadString(const tinyxml2::XMLElement *_root,
                       const char *_name,
                       const std::string &_fallback)
{
  if (nullptr == _root)
    return _fallback;

  const auto *element = _root->FirstChildElement(_name);
  if (nullptr == element || nullptr == element->GetText())
    return _fallback;
  return element->GetText();
}

//////////////////////////////////////////////////
double ReadDouble(const tinyxml2::XMLElement *_root,
                  const char *_name,
                  const double _fallback)
{
  if (nullptr == _root)
    return _fallback;

  const auto *element = _root->FirstChildElement(_name);
  if (nullptr == element)
    return _fallback;

  double value = _fallback;
  return tinyxml2::XML_SUCCESS == element->QueryDoubleText(&value)
      ? value
      : _fallback;
}

//////////////////////////////////////////////////
int ReadInt(const tinyxml2::XMLElement *_root,
            const char *_name,
            const int _fallback)
{
  if (nullptr == _root)
    return _fallback;

  const auto *element = _root->FirstChildElement(_name);
  if (nullptr == element)
    return _fallback;

  int value = _fallback;
  return tinyxml2::XML_SUCCESS == element->QueryIntText(&value)
      ? value
      : _fallback;
}

//////////////////////////////////////////////////
bool ReadBool(const tinyxml2::XMLElement *_root,
              const char *_name,
              const bool _fallback)
{
  if (nullptr == _root)
    return _fallback;

  const auto *element = _root->FirstChildElement(_name);
  if (nullptr == element)
    return _fallback;

  bool value = _fallback;
  return tinyxml2::XML_SUCCESS == element->QueryBoolText(&value)
      ? value
      : _fallback;
}
}  // namespace

class ForkliftTeleopPrivate
{
  public: gz::transport::Node node;
  public: gz::transport::Node::Publisher drivePublisher;
  public: gz::transport::Node::Publisher forkPublisher;

  public: std::string driveTopic{"/forklift/cmd_vel"};
  public: std::string forkTopic{"/forklift/fork_position"};
  public: std::string worldControlService{"/world/pallet_stacker_world/control"};

  public: double maxLinearVelocity{2.0};
  public: double maxAngularVelocity{0.75};
  public: double maxLinearAcceleration{1.0};
  public: double maxAngularAcceleration{1.5};
  public: double maxForkVelocity{0.5};
  public: double forkMinimum{0.0};
  public: double forkMaximum{1.8};
  public: double forkTarget{0.0};
  public: double updateRateHz{50.0};

  public: bool autoStart{true};
  public: int autoStartRetryIntervalMs{250};
  public: int autoStartRequestTimeoutMs{100};
  public: int autoStartMaxAttempts{60};
  public: int autoStartAttempts{0};
  public: bool autoStartComplete{false};

  public: int forwardKey{Qt::Key_W};
  public: int reverseKey{Qt::Key_S};
  public: int leftKey{Qt::Key_A};
  public: int rightKey{Qt::Key_D};
  public: int liftKey{Qt::Key_Up};
  public: int lowerKey{Qt::Key_Down};
  public: int stopKey{Qt::Key_Space};

  public: QString forwardLabel{"W"};
  public: QString reverseLabel{"S"};
  public: QString leftLabel{"A"};
  public: QString rightLabel{"D"};
  public: QString liftLabel{"Up"};
  public: QString lowerLabel{"Down"};
  public: QString stopLabel{"Space"};
  public: QString statusText{"Initializing forklift controls..."};

  public: double currentLinearVelocity{0.0};
  public: double currentAngularVelocity{0.0};
  public: QSet<int> pressedKeys;
  public: QTimer *updateTimer{nullptr};
  public: QTimer *autoStartTimer{nullptr};
  public: QElapsedTimer elapsed;
  public: bool publishersReady{false};
  public: QPointer<QObject> eventFilterTarget;

  public: bool IsControlKey(const int _key) const
  {
    return _key == this->forwardKey || _key == this->reverseKey ||
           _key == this->leftKey || _key == this->rightKey ||
           _key == this->liftKey || _key == this->lowerKey ||
           _key == this->stopKey;
  }

  public: void ClearMotionKeys()
  {
    this->pressedKeys.remove(this->forwardKey);
    this->pressedKeys.remove(this->reverseKey);
    this->pressedKeys.remove(this->leftKey);
    this->pressedKeys.remove(this->rightKey);
    this->pressedKeys.remove(this->liftKey);
    this->pressedKeys.remove(this->lowerKey);
  }

  public: void Publish()
  {
    if (!this->publishersReady)
      return;

    gz::msgs::Twist driveMessage;
    driveMessage.mutable_linear()->set_x(this->currentLinearVelocity);
    driveMessage.mutable_angular()->set_z(this->currentAngularVelocity);
    this->drivePublisher.Publish(driveMessage);

    gz::msgs::Double forkMessage;
    forkMessage.set_data(this->forkTarget);
    this->forkPublisher.Publish(forkMessage);
  }
};

//////////////////////////////////////////////////
ForkliftTeleop::ForkliftTeleop()
    : gz::gui::Plugin(), dataPtr(std::make_unique<ForkliftTeleopPrivate>())
{
}

//////////////////////////////////////////////////
ForkliftTeleop::~ForkliftTeleop()
{
  if (!this->dataPtr->eventFilterTarget.isNull())
    this->dataPtr->eventFilterTarget->removeEventFilter(this);

  this->dataPtr->ClearMotionKeys();
  this->dataPtr->currentLinearVelocity = 0.0;
  this->dataPtr->currentAngularVelocity = 0.0;
  this->dataPtr->Publish();
}

//////////////////////////////////////////////////
void ForkliftTeleop::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  if (this->title.empty())
    this->title = "Forklift controls";

  this->dataPtr->driveTopic = ReadString(
      _pluginElem, "drive_topic", this->dataPtr->driveTopic);
  this->dataPtr->forkTopic = ReadString(
      _pluginElem, "fork_topic", this->dataPtr->forkTopic);
  this->dataPtr->worldControlService = ReadString(
      _pluginElem, "world_control_service",
      this->dataPtr->worldControlService);

  this->dataPtr->maxLinearVelocity = ReadDouble(
      _pluginElem, "max_linear_velocity",
      this->dataPtr->maxLinearVelocity);
  this->dataPtr->maxAngularVelocity = ReadDouble(
      _pluginElem, "max_angular_velocity",
      this->dataPtr->maxAngularVelocity);
  this->dataPtr->maxLinearAcceleration = ReadDouble(
      _pluginElem, "max_linear_acceleration",
      this->dataPtr->maxLinearAcceleration);
  this->dataPtr->maxAngularAcceleration = ReadDouble(
      _pluginElem, "max_angular_acceleration",
      this->dataPtr->maxAngularAcceleration);
  this->dataPtr->maxForkVelocity = ReadDouble(
      _pluginElem, "max_fork_velocity",
      this->dataPtr->maxForkVelocity);
  this->dataPtr->forkMinimum = ReadDouble(
      _pluginElem, "fork_minimum", this->dataPtr->forkMinimum);
  this->dataPtr->forkMaximum = ReadDouble(
      _pluginElem, "fork_maximum", this->dataPtr->forkMaximum);
  this->dataPtr->forkTarget = ReadDouble(
      _pluginElem, "fork_initial", this->dataPtr->forkTarget);
  this->dataPtr->updateRateHz = ReadDouble(
      _pluginElem, "update_rate_hz", this->dataPtr->updateRateHz);

  this->dataPtr->autoStart = ReadBool(
      _pluginElem, "auto_start", this->dataPtr->autoStart);
  this->dataPtr->autoStartRetryIntervalMs = ReadInt(
      _pluginElem, "auto_start_retry_interval_ms",
      this->dataPtr->autoStartRetryIntervalMs);
  this->dataPtr->autoStartRequestTimeoutMs = ReadInt(
      _pluginElem, "auto_start_request_timeout_ms",
      this->dataPtr->autoStartRequestTimeoutMs);
  this->dataPtr->autoStartMaxAttempts = ReadInt(
      _pluginElem, "auto_start_max_attempts",
      this->dataPtr->autoStartMaxAttempts);

  this->dataPtr->forwardKey = ReadInt(
      _pluginElem, "forward_key", this->dataPtr->forwardKey);
  this->dataPtr->reverseKey = ReadInt(
      _pluginElem, "reverse_key", this->dataPtr->reverseKey);
  this->dataPtr->leftKey = ReadInt(
      _pluginElem, "left_key", this->dataPtr->leftKey);
  this->dataPtr->rightKey = ReadInt(
      _pluginElem, "right_key", this->dataPtr->rightKey);
  this->dataPtr->liftKey = ReadInt(
      _pluginElem, "lift_key", this->dataPtr->liftKey);
  this->dataPtr->lowerKey = ReadInt(
      _pluginElem, "lower_key", this->dataPtr->lowerKey);
  this->dataPtr->stopKey = ReadInt(
      _pluginElem, "stop_key", this->dataPtr->stopKey);

  this->dataPtr->forwardLabel = QString::fromStdString(ReadString(
      _pluginElem, "forward_label",
      this->dataPtr->forwardLabel.toStdString()));
  this->dataPtr->reverseLabel = QString::fromStdString(ReadString(
      _pluginElem, "reverse_label",
      this->dataPtr->reverseLabel.toStdString()));
  this->dataPtr->leftLabel = QString::fromStdString(ReadString(
      _pluginElem, "left_label", this->dataPtr->leftLabel.toStdString()));
  this->dataPtr->rightLabel = QString::fromStdString(ReadString(
      _pluginElem, "right_label", this->dataPtr->rightLabel.toStdString()));
  this->dataPtr->liftLabel = QString::fromStdString(ReadString(
      _pluginElem, "lift_label", this->dataPtr->liftLabel.toStdString()));
  this->dataPtr->lowerLabel = QString::fromStdString(ReadString(
      _pluginElem, "lower_label", this->dataPtr->lowerLabel.toStdString()));
  this->dataPtr->stopLabel = QString::fromStdString(ReadString(
      _pluginElem, "stop_label", this->dataPtr->stopLabel.toStdString()));

  const bool validLimits =
      !this->dataPtr->driveTopic.empty() &&
      !this->dataPtr->forkTopic.empty() &&
      !this->dataPtr->worldControlService.empty() &&
      std::isfinite(this->dataPtr->maxLinearVelocity) &&
      this->dataPtr->maxLinearVelocity > 0.0 &&
      std::isfinite(this->dataPtr->maxAngularVelocity) &&
      this->dataPtr->maxAngularVelocity > 0.0 &&
      std::isfinite(this->dataPtr->maxLinearAcceleration) &&
      this->dataPtr->maxLinearAcceleration > 0.0 &&
      std::isfinite(this->dataPtr->maxAngularAcceleration) &&
      this->dataPtr->maxAngularAcceleration > 0.0 &&
      std::isfinite(this->dataPtr->maxForkVelocity) &&
      this->dataPtr->maxForkVelocity > 0.0 &&
      std::isfinite(this->dataPtr->updateRateHz) &&
      this->dataPtr->updateRateHz > 0.0 &&
      std::isfinite(this->dataPtr->forkMinimum) &&
      std::isfinite(this->dataPtr->forkMaximum) &&
      this->dataPtr->forkMaximum > this->dataPtr->forkMinimum &&
      this->dataPtr->autoStartRetryIntervalMs > 0 &&
      this->dataPtr->autoStartRequestTimeoutMs > 0 &&
      this->dataPtr->autoStartMaxAttempts > 0;

  if (!validLimits)
  {
    this->dataPtr->statusText = "Invalid forklift-control configuration";
    emit this->StatusChanged();
    gzerr << "ForkliftTeleop received invalid controller limits. "
          << "The plugin will not start." << std::endl;
    return;
  }

  this->dataPtr->forkTarget = std::clamp(
      this->dataPtr->forkTarget,
      this->dataPtr->forkMinimum,
      this->dataPtr->forkMaximum);

  this->dataPtr->drivePublisher =
      this->dataPtr->node.Advertise<gz::msgs::Twist>(
          this->dataPtr->driveTopic);
  this->dataPtr->forkPublisher =
      this->dataPtr->node.Advertise<gz::msgs::Double>(
          this->dataPtr->forkTopic);

  if (!this->dataPtr->drivePublisher || !this->dataPtr->forkPublisher)
  {
    this->dataPtr->statusText = "Could not advertise forklift command topics";
    emit this->StatusChanged();
    gzerr << "ForkliftTeleop could not advertise its Gazebo Transport topics."
          << std::endl;
    return;
  }
  this->dataPtr->publishersReady = true;

  auto *application = gz::gui::App();
  if (nullptr == application)
  {
    this->dataPtr->statusText = "Could not capture Gazebo keyboard events";
    emit this->StatusChanged();
    gzerr << "ForkliftTeleop could not access the Gazebo application."
          << std::endl;
    return;
  }

  // Install at application scope instead of depending on main-window creation
  // order. This captures keys while the 3D view or the floating control panel
  // has focus. QPointer becomes null safely if the application is destroyed
  // before this plugin.
  this->dataPtr->eventFilterTarget = application;
  application->installEventFilter(this);

  this->dataPtr->updateTimer = new QTimer(this);
  const int intervalMs = std::max(
      1, static_cast<int>(std::lround(1000.0 / this->dataPtr->updateRateHz)));
  this->dataPtr->updateTimer->setInterval(intervalMs);
  connect(this->dataPtr->updateTimer, &QTimer::timeout,
          this, &ForkliftTeleop::OnUpdate);
  this->dataPtr->elapsed.start();
  this->dataPtr->updateTimer->start();

  this->dataPtr->autoStartTimer = new QTimer(this);
  this->dataPtr->autoStartTimer->setInterval(
      this->dataPtr->autoStartRetryIntervalMs);
  connect(this->dataPtr->autoStartTimer, &QTimer::timeout,
          this, &ForkliftTeleop::OnAutoStartAttempt);

  if (this->dataPtr->autoStart)
  {
    this->dataPtr->statusText = "Starting simulation...";
    this->dataPtr->autoStartTimer->start();
    QTimer::singleShot(0, this, &ForkliftTeleop::OnAutoStartAttempt);
  }
  else
  {
    this->dataPtr->autoStartComplete = true;
    this->dataPtr->statusText = "Controls ready - press Play to run";
  }

  // Publish repeatedly from OnUpdate so late Transport discovery still receives
  // the initial fork target and a zero chassis command.
  this->dataPtr->Publish();
  emit this->ConfigChanged();
  emit this->StateChanged();
  emit this->StatusChanged();

  gzmsg << "ForkliftTeleop publishing drive commands on ["
        << this->dataPtr->driveTopic << "] and fork targets on ["
        << this->dataPtr->forkTopic << "]." << std::endl;
}

//////////////////////////////////////////////////
double ForkliftTeleop::LinearVelocity() const
{
  return this->dataPtr->currentLinearVelocity;
}

//////////////////////////////////////////////////
double ForkliftTeleop::AngularVelocity() const
{
  return this->dataPtr->currentAngularVelocity;
}

//////////////////////////////////////////////////
double ForkliftTeleop::ForkTarget() const
{
  return this->dataPtr->forkTarget;
}

//////////////////////////////////////////////////
QString ForkliftTeleop::ControlsText() const
{
  return QString("%1/%2 drive  %3/%4 turn  %5/%6 forks  %7 stop")
      .arg(this->dataPtr->forwardLabel)
      .arg(this->dataPtr->reverseLabel)
      .arg(this->dataPtr->leftLabel)
      .arg(this->dataPtr->rightLabel)
      .arg(this->dataPtr->liftLabel)
      .arg(this->dataPtr->lowerLabel)
      .arg(this->dataPtr->stopLabel);
}

//////////////////////////////////////////////////
QString ForkliftTeleop::StatusText() const
{
  return this->dataPtr->statusText;
}

//////////////////////////////////////////////////
void ForkliftTeleop::Stop()
{
  this->dataPtr->ClearMotionKeys();
  this->dataPtr->currentLinearVelocity = 0.0;
  this->dataPtr->currentAngularVelocity = 0.0;
  this->dataPtr->Publish();
  emit this->StateChanged();
}

//////////////////////////////////////////////////
bool ForkliftTeleop::eventFilter(QObject *_watched, QEvent *_event)
{
  if (nullptr == _event)
    return QObject::eventFilter(_watched, _event);

  const auto eventType = _event->type();
  if (eventType == QEvent::WindowDeactivate ||
      eventType == QEvent::ApplicationDeactivate)
  {
    this->Stop();
    return QObject::eventFilter(_watched, _event);
  }

  if (eventType == QEvent::ShortcutOverride)
  {
    auto *keyEvent = static_cast<QKeyEvent *>(_event);
    if (this->dataPtr->IsControlKey(keyEvent->key()))
    {
      keyEvent->accept();
      return true;
    }
    return QObject::eventFilter(_watched, _event);
  }

  if (eventType != QEvent::KeyPress && eventType != QEvent::KeyRelease)
    return QObject::eventFilter(_watched, _event);

  auto *keyEvent = static_cast<QKeyEvent *>(_event);
  const int key = keyEvent->key();
  if (!this->dataPtr->IsControlKey(key))
    return QObject::eventFilter(_watched, _event);

  // Qt emits synthetic press / release pairs while a key auto-repeats. The
  // physical state is already represented by pressedKeys, so ignore them.
  if (keyEvent->isAutoRepeat())
    return true;

  if (key == this->dataPtr->stopKey)
  {
    if (eventType == QEvent::KeyPress)
      this->Stop();
    return true;
  }

  if (eventType == QEvent::KeyPress)
    this->dataPtr->pressedKeys.insert(key);
  else
    this->dataPtr->pressedKeys.remove(key);

  return true;
}

//////////////////////////////////////////////////
void ForkliftTeleop::OnUpdate()
{
  double dt = static_cast<double>(this->dataPtr->elapsed.restart()) / 1000.0;
  dt = std::clamp(dt, 0.0, 0.25);

  const int linearDirection =
      (this->dataPtr->pressedKeys.contains(this->dataPtr->forwardKey) ? 1 : 0) -
      (this->dataPtr->pressedKeys.contains(this->dataPtr->reverseKey) ? 1 : 0);
  const int angularDirection =
      (this->dataPtr->pressedKeys.contains(this->dataPtr->leftKey) ? 1 : 0) -
      (this->dataPtr->pressedKeys.contains(this->dataPtr->rightKey) ? 1 : 0);
  const int forkDirection =
      (this->dataPtr->pressedKeys.contains(this->dataPtr->liftKey) ? 1 : 0) -
      (this->dataPtr->pressedKeys.contains(this->dataPtr->lowerKey) ? 1 : 0);

  const double targetLinear =
      linearDirection * this->dataPtr->maxLinearVelocity;
  const double targetAngular =
      angularDirection * this->dataPtr->maxAngularVelocity;

  const double previousLinear = this->dataPtr->currentLinearVelocity;
  const double previousAngular = this->dataPtr->currentAngularVelocity;
  const double previousFork = this->dataPtr->forkTarget;

  this->dataPtr->currentLinearVelocity = Approach(
      this->dataPtr->currentLinearVelocity,
      targetLinear,
      this->dataPtr->maxLinearAcceleration * dt);
  this->dataPtr->currentAngularVelocity = Approach(
      this->dataPtr->currentAngularVelocity,
      targetAngular,
      this->dataPtr->maxAngularAcceleration * dt);

  if (forkDirection != 0)
  {
    this->dataPtr->forkTarget = std::clamp(
        this->dataPtr->forkTarget +
            forkDirection * this->dataPtr->maxForkVelocity * dt,
        this->dataPtr->forkMinimum,
        this->dataPtr->forkMaximum);
  }

  this->dataPtr->Publish();

  constexpr double epsilon = 1e-6;
  if (std::abs(previousLinear - this->dataPtr->currentLinearVelocity) > epsilon ||
      std::abs(previousAngular - this->dataPtr->currentAngularVelocity) > epsilon ||
      std::abs(previousFork - this->dataPtr->forkTarget) > epsilon)
  {
    emit this->StateChanged();
  }
}

//////////////////////////////////////////////////
void ForkliftTeleop::OnAutoStartAttempt()
{
  if (!this->dataPtr->autoStart || this->dataPtr->autoStartComplete)
    return;

  ++this->dataPtr->autoStartAttempts;

  gz::msgs::WorldControl request;
  request.set_pause(false);
  gz::msgs::Boolean response;
  bool serviceResult = false;

  const bool requestExecuted = this->dataPtr->node.Request(
      this->dataPtr->worldControlService,
      request,
      static_cast<unsigned int>(this->dataPtr->autoStartRequestTimeoutMs),
      response,
      serviceResult);

  if (requestExecuted && serviceResult && response.data())
  {
    this->dataPtr->autoStartComplete = true;
    if (nullptr != this->dataPtr->autoStartTimer)
      this->dataPtr->autoStartTimer->stop();
    this->dataPtr->statusText = "Controls ready - simulation running";
    emit this->StatusChanged();
    gzmsg << "ForkliftTeleop started the simulation through ["
          << this->dataPtr->worldControlService << "]." << std::endl;
    return;
  }

  if (this->dataPtr->autoStartAttempts >=
      this->dataPtr->autoStartMaxAttempts)
  {
    this->dataPtr->autoStartComplete = true;
    if (nullptr != this->dataPtr->autoStartTimer)
      this->dataPtr->autoStartTimer->stop();
    this->dataPtr->statusText =
        "Controls ready - autostart failed; press Play";
    emit this->StatusChanged();
    gzwarn << "ForkliftTeleop could not unpause the world through ["
           << this->dataPtr->worldControlService << "] after "
           << this->dataPtr->autoStartAttempts << " attempts." << std::endl;
  }
}

GZ_ADD_PLUGIN(ForkliftTeleop, gz::gui::Plugin)
