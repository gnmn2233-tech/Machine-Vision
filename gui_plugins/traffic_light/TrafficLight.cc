#include <iostream>
#include <sstream>

#include <gz/plugin/Register.hh>
#include <gz/msgs/pose.pb.h>

#include "TrafficLight.hh"

// 灯球沉到地下的高度（和 light.sh / traffic_daemon.py 一致）
static constexpr double kParkZ = -50.0;

/////////////////////////////////////////////////
TrafficLight::TrafficLight()
  : gz::gui::Plugin()
{
}

/////////////////////////////////////////////////
TrafficLight::~TrafficLight() = default;

/////////////////////////////////////////////////
void TrafficLight::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  if (this->title.empty())
    this->title = "Traffic light";

  if (!_pluginElem)
    return;

  if (auto *e = _pluginElem->FirstChildElement("world"); e && e->GetText())
    this->world = e->GetText();

  if (auto *e = _pluginElem->FirstChildElement("lamp_xy"); e && e->GetText())
  {
    std::istringstream iss(e->GetText());
    iss >> this->x >> this->y;
  }

  if (auto *e = _pluginElem->FirstChildElement("red_z"); e && e->GetText())
    this->redZ = std::stod(e->GetText());

  if (auto *e = _pluginElem->FirstChildElement("green_z"); e && e->GetText())
    this->greenZ = std::stod(e->GetText());

  std::cerr << "[TrafficLight] 已加载: world=" << this->world
            << " lamp_xy=" << this->x << "," << this->y
            << " red_z=" << this->redZ << " green_z=" << this->greenZ
            << std::endl;
}

/////////////////////////////////////////////////
void TrafficLight::SetRed()
{
  this->Apply("red");
}

/////////////////////////////////////////////////
void TrafficLight::SetGreen()
{
  this->Apply("green");
}

/////////////////////////////////////////////////
void TrafficLight::Apply(const std::string &_color)
{
  if (_color == "red")
  {
    this->MoveLamp("lamp_red", this->redZ);
    this->MoveLamp("lamp_green", kParkZ);
  }
  else
  {
    this->MoveLamp("lamp_green", this->greenZ);
    this->MoveLamp("lamp_red", kParkZ);
  }

  this->current = _color;
  emit this->LightColorChanged();
}

/////////////////////////////////////////////////
void TrafficLight::MoveLamp(const std::string &_model, double _z)
{
  gz::msgs::Pose req;
  req.set_name(_model);
  req.mutable_position()->set_x(this->x);
  req.mutable_position()->set_y(this->y);
  req.mutable_position()->set_z(_z);
  req.mutable_orientation()->set_w(1.0);

  const std::string service = "/world/" + this->world + "/set_pose";
  if (!this->node.Request(service, req, &TrafficLight::OnPoseResult, this))
    qWarning() << "[TrafficLight] 服务不可达:" << QString::fromStdString(service);
}

/////////////////////////////////////////////////
void TrafficLight::OnPoseResult(const gz::msgs::Boolean &_reply, bool _result)
{
  if (!_result || !_reply.data())
    qWarning() << "[TrafficLight] set_pose 失败";
}

// Register this plugin
GZ_ADD_PLUGIN(TrafficLight, gz::gui::Plugin);
