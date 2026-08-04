#include "ForkliftTeleop.hh"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <mutex>

#include <QKeyEvent>
#include <QTimer>

#include <gz/gui/Application.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/world_stats.pb.h>
#include <gz/plugin/Register.hh>

namespace
{
double ReadDouble(const tinyxml2::XMLElement *_root, const char *_name,
                  const double _fallback)
{
  if (!_root)
    return _fallback;
  const auto *elem = _root->FirstChildElement(_name);
  if (!elem || !elem->GetText())
    return _fallback;
  return std::stod(elem->GetText());
}

int ReadInt(const tinyxml2::XMLElement *_root, const char *_name,
            const int _fallback)
{
  if (!_root)
    return _fallback;
  const auto *elem = _root->FirstChildElement(_name);
  if (!elem || !elem->GetText())
    return _fallback;
  return std::stoi(elem->GetText());
}

std::string ReadString(const tinyxml2::XMLElement *_root, const char *_name,
                       const std::string &_fallback)
{
  if (!_root)
    return _fallback;
  const auto *elem = _root->FirstChildElement(_name);
  if (!elem || !elem->GetText())
    return _fallback;
  return elem->GetText();
}

double MoveTowards(const double _current, const double _target,
                   const double _maxDelta)
{
  if (_current < _target)
    return std::min(_current + _maxDelta, _target);
  if (_current > _target)
    return std::max(_current - _maxDelta, _target);
  return _current;
}
}

class ForkliftTeleopPrivate
{
  public: gz::transport::Node node;
  public: gz::transport::Node::Publisher drivePublisher;
  public: gz::transport::Node::Publisher leftForkPublisher;
  public: gz::transport::Node::Publisher rightForkPublisher;

  public: std::string driveTopic{ "/forklift/cmd_vel" };
  public: std::string leftForkTopic{ "/forklift/left_fork/cmd_vel" };
  public: std::string rightForkTopic{ "/forklift/right_fork/cmd_vel" };
  public: std::string worldStatsTopic{ "/world/pallet_towers/stats" };

  public: double maxLinearSpeed{2.0};
  public: double maxLinearAcceleration{0.5};
  public: double maxAngularSpeed{0.8};
  public: double maxAngularAcceleration{1.0};
  public: double forkSpeed{0.15};
  public: double updateRateHz{50.0};

  public: int forwardKey{Qt::Key_Up};
  public: int reverseKey{Qt::Key_Down};
  public: int leftKey{Qt::Key_Left};
  public: int rightKey{Qt::Key_Right};
  public: int liftKey{Qt::Key_Up};
  public: int lowerKey{Qt::Key_Down};

  public: bool forwardPressed{false};
  public: bool reversePressed{false};
  public: bool leftPressed{false};
  public: bool rightPressed{false};
  public: bool liftPressed{false};
  public: bool lowerPressed{false};

  public: double currentLinear{0.0};
  public: double currentAngular{0.0};

  // Keyboard events arrive on the GUI thread, while world statistics arrive
  // on a Gazebo Transport callback thread. Protect shared state explicitly.
  public: std::mutex stateMutex;
  public: bool haveSimTime{false};
  public: bool simulationPaused{true};
  public: double latestSimTimeSec{0.0};
  public: double lastControlSimTimeSec{0.0};

  public: QTimer timer;

  public: void PublishStop()
  {
    gz::msgs::Twist twist;
    twist.mutable_linear()->set_x(0.0);
    twist.mutable_angular()->set_z(0.0);
    this->drivePublisher.Publish(twist);

    gz::msgs::Double fork;
    fork.set_data(0.0);
    this->leftForkPublisher.Publish(fork);
    this->rightForkPublisher.Publish(fork);
  }
};

ForkliftTeleop::ForkliftTeleop()
    : gz::gui::Plugin(), dataPtr(std::make_unique<ForkliftTeleopPrivate>())
{
  connect(&this->dataPtr->timer, &QTimer::timeout,
          this, &ForkliftTeleop::OnControlTick);
}

ForkliftTeleop::~ForkliftTeleop()
{
  this->dataPtr->timer.stop();
  this->dataPtr->PublishStop();
}

void ForkliftTeleop::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  if (this->title.empty())
    this->title = "Forklift teleoperation";

  this->dataPtr->driveTopic = ReadString(
      _pluginElem, "drive_topic", this->dataPtr->driveTopic);
  this->dataPtr->leftForkTopic = ReadString(
      _pluginElem, "left_fork_topic", this->dataPtr->leftForkTopic);
  this->dataPtr->rightForkTopic = ReadString(
      _pluginElem, "right_fork_topic", this->dataPtr->rightForkTopic);
  this->dataPtr->worldStatsTopic = ReadString(
      _pluginElem, "world_stats_topic", this->dataPtr->worldStatsTopic);

  this->dataPtr->maxLinearSpeed = std::max(
      0.0, ReadDouble(_pluginElem, "max_linear_speed", 2.0));
  this->dataPtr->maxLinearAcceleration = std::max(
      0.001, ReadDouble(_pluginElem, "max_linear_acceleration", 0.5));
  this->dataPtr->maxAngularSpeed = std::max(
      0.0, ReadDouble(_pluginElem, "max_angular_speed", 0.8));
  this->dataPtr->maxAngularAcceleration = std::max(
      0.001, ReadDouble(_pluginElem, "max_angular_acceleration", 1.0));
  this->dataPtr->forkSpeed = std::max(
      0.0, ReadDouble(_pluginElem, "fork_speed", 0.15));
  this->dataPtr->updateRateHz = std::max(
      10.0, ReadDouble(_pluginElem, "update_rate_hz", 50.0));

  this->dataPtr->forwardKey = ReadInt(
      _pluginElem, "forward_key", Qt::Key_Up);
  this->dataPtr->reverseKey = ReadInt(
      _pluginElem, "reverse_key", Qt::Key_Down);
  this->dataPtr->leftKey = ReadInt(
      _pluginElem, "left_key", Qt::Key_Left);
  this->dataPtr->rightKey = ReadInt(
      _pluginElem, "right_key", Qt::Key_Right);
  this->dataPtr->liftKey = ReadInt(
      _pluginElem, "lift_key", Qt::Key_Up);
  this->dataPtr->lowerKey = ReadInt(
      _pluginElem, "lower_key", Qt::Key_Down);

  this->dataPtr->drivePublisher =
      this->dataPtr->node.Advertise<gz::msgs::Twist>(this->dataPtr->driveTopic);
  this->dataPtr->leftForkPublisher =
      this->dataPtr->node.Advertise<gz::msgs::Double>(this->dataPtr->leftForkTopic);
  this->dataPtr->rightForkPublisher =
      this->dataPtr->node.Advertise<gz::msgs::Double>(this->dataPtr->rightForkTopic);

  if (!this->dataPtr->node.Subscribe(
          this->dataPtr->worldStatsTopic,
          &ForkliftTeleop::OnWorldStats,
          this))
  {
    std::cerr << "ForkliftTeleop: failed to subscribe to world statistics ["
              << this->dataPtr->worldStatsTopic << "].\n";
  }

  auto *window = gz::gui::App()->findChild<gz::gui::MainWindow *>();
  if (window && window->QuickWindow())
    window->QuickWindow()->installEventFilter(this);
  else
    std::cerr << "ForkliftTeleop: Gazebo main window was not found.\n";

  const int periodMs = std::max(
      1, static_cast<int>(std::lround(1000.0 / this->dataPtr->updateRateHz)));
  this->dataPtr->timer.start(periodMs);

  std::cout << "ForkliftTeleop active: arrows drive, Shift+Up/Down lift.\n";
}

bool ForkliftTeleop::eventFilter(QObject *_obj, QEvent *_event)
{
  (void)_obj;
  if (_event->type() != QEvent::KeyPress &&
      _event->type() != QEvent::KeyRelease)
  {
    return QObject::eventFilter(_obj, _event);
  }

  auto *keyEvent = static_cast<QKeyEvent *>(_event);
  const int key = keyEvent->key();

  // Qt generates repeated KeyPress events while an arrow is held. The first
  // press already changed our state, so consume repeats to stop Gazebo's
  // camera shortcuts from seeing them.
  if (keyEvent->isAutoRepeat())
  {
    if (key == this->dataPtr->forwardKey ||
        key == this->dataPtr->reverseKey ||
        key == this->dataPtr->leftKey ||
        key == this->dataPtr->rightKey ||
        key == Qt::Key_Space)
    {
      keyEvent->accept();
      return true;
    }
    return QObject::eventFilter(_obj, _event);
  }

  const bool pressed = (_event->type() == QEvent::KeyPress);
  const bool shifted = keyEvent->modifiers().testFlag(Qt::ShiftModifier);
  bool handled = false;

  std::lock_guard<std::mutex> lock(this->dataPtr->stateMutex);

  if (key == this->dataPtr->forwardKey)
  {
    if (shifted)
    {
      this->dataPtr->liftPressed = pressed;
      if (pressed)
        this->dataPtr->forwardPressed = false;
    }
    else
    {
      this->dataPtr->forwardPressed = pressed;
      if (pressed)
        this->dataPtr->liftPressed = false;
    }
    handled = true;
  }
  else if (key == this->dataPtr->reverseKey)
  {
    if (shifted)
    {
      this->dataPtr->lowerPressed = pressed;
      if (pressed)
        this->dataPtr->reversePressed = false;
    }
    else
    {
      this->dataPtr->reversePressed = pressed;
      if (pressed)
        this->dataPtr->lowerPressed = false;
    }
    handled = true;
  }
  else if (key == this->dataPtr->leftKey)
  {
    this->dataPtr->leftPressed = pressed;
    handled = true;
  }
  else if (key == this->dataPtr->rightKey)
  {
    this->dataPtr->rightPressed = pressed;
    handled = true;
  }
  else if (key == Qt::Key_Space && pressed)
  {
    this->dataPtr->forwardPressed = false;
    this->dataPtr->reversePressed = false;
    this->dataPtr->leftPressed = false;
    this->dataPtr->rightPressed = false;
    this->dataPtr->liftPressed = false;
    this->dataPtr->lowerPressed = false;
    this->dataPtr->currentLinear = 0.0;
    this->dataPtr->currentAngular = 0.0;
    this->dataPtr->PublishStop();
    handled = true;
  }

  // On release, Qt may report the modifier state after Shift was released.
  // Clear both actions associated with the same arrow to prevent sticking.
  if (!pressed && key == this->dataPtr->liftKey)
  {
    this->dataPtr->forwardPressed = false;
    this->dataPtr->liftPressed = false;
    handled = true;
  }
  if (!pressed && key == this->dataPtr->lowerKey)
  {
    this->dataPtr->reversePressed = false;
    this->dataPtr->lowerPressed = false;
    handled = true;
  }

  if (handled)
  {
    keyEvent->accept();
    return true;
  }
  return QObject::eventFilter(_obj, _event);
}

void ForkliftTeleop::OnWorldStats(const gz::msgs::WorldStatistics &_msg)
{
  const double simTimeSec =
      static_cast<double>(_msg.sim_time().sec()) +
      static_cast<double>(_msg.sim_time().nsec()) * 1e-9;

  std::lock_guard<std::mutex> lock(this->dataPtr->stateMutex);

  // A lower timestamp means the world was reset or rewound. Reset the command
  // state as well so the next run starts from rest.
  if (this->dataPtr->haveSimTime &&
      simTimeSec < this->dataPtr->latestSimTimeSec)
  {
    this->dataPtr->forwardPressed = false;
    this->dataPtr->reversePressed = false;
    this->dataPtr->leftPressed = false;
    this->dataPtr->rightPressed = false;
    this->dataPtr->liftPressed = false;
    this->dataPtr->lowerPressed = false;
    this->dataPtr->currentLinear = 0.0;
    this->dataPtr->currentAngular = 0.0;
    this->dataPtr->lastControlSimTimeSec = simTimeSec;
    this->dataPtr->PublishStop();
  }

  this->dataPtr->latestSimTimeSec = simTimeSec;
  this->dataPtr->simulationPaused = _msg.paused();
  if (!this->dataPtr->haveSimTime)
  {
    this->dataPtr->lastControlSimTimeSec = simTimeSec;
    this->dataPtr->haveSimTime = true;
  }
}

void ForkliftTeleop::OnControlTick()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->stateMutex);

  if (!this->dataPtr->haveSimTime)
    return;

  // Do not ramp commands while paused. Reset the integration origin so time
  // spent paused can never create a velocity jump when play resumes.
  if (this->dataPtr->simulationPaused)
  {
    this->dataPtr->lastControlSimTimeSec = this->dataPtr->latestSimTimeSec;
    return;
  }

  double dt = this->dataPtr->latestSimTimeSec -
              this->dataPtr->lastControlSimTimeSec;
  this->dataPtr->lastControlSimTimeSec = this->dataPtr->latestSimTimeSec;
  if (dt <= 0.0)
    return;

  // A transport hiccup may skip several statistics messages. Limiting the
  // integration interval can only make acceleration slower, never faster than
  // the configured maximum.
  dt = std::min(dt, 0.1);

  const double linearDirection =
      (this->dataPtr->forwardPressed ? 1.0 : 0.0) -
      (this->dataPtr->reversePressed ? 1.0 : 0.0);
  const double angularDirection =
      (this->dataPtr->leftPressed ? 1.0 : 0.0) -
      (this->dataPtr->rightPressed ? 1.0 : 0.0);

  const double targetLinear =
      linearDirection * this->dataPtr->maxLinearSpeed;
  const double targetAngular =
      angularDirection * this->dataPtr->maxAngularSpeed;

  this->dataPtr->currentLinear = MoveTowards(
      this->dataPtr->currentLinear, targetLinear,
      this->dataPtr->maxLinearAcceleration * dt);
  this->dataPtr->currentAngular = MoveTowards(
      this->dataPtr->currentAngular, targetAngular,
      this->dataPtr->maxAngularAcceleration * dt);

  gz::msgs::Twist twist;
  twist.mutable_linear()->set_x(this->dataPtr->currentLinear);
  twist.mutable_angular()->set_z(this->dataPtr->currentAngular);
  this->dataPtr->drivePublisher.Publish(twist);

  const double forkDirection =
      (this->dataPtr->liftPressed ? 1.0 : 0.0) -
      (this->dataPtr->lowerPressed ? 1.0 : 0.0);
  gz::msgs::Double forkCommand;
  forkCommand.set_data(forkDirection * this->dataPtr->forkSpeed);
  this->dataPtr->leftForkPublisher.Publish(forkCommand);
  this->dataPtr->rightForkPublisher.Publish(forkCommand);
}

GZ_ADD_PLUGIN(ForkliftTeleop, gz::gui::Plugin)
