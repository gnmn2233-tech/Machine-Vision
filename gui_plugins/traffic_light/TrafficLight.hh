/*
 * TrafficLight.hh -- Gazebo (gz-sim 8 / Harmonic) GUI 插件：红绿灯切换按钮
 *
 * 场景里的灯是"两颗亮球 + 两颗固定黑球"：
 *   红灯亮 = lamp_red 升到 z=1.17，lamp_green 沉到地下（露出下面的黑球）
 *   绿灯亮 = 反之。
 * 本插件只做一件事：点按钮 -> 调 /world/<world>/set_pose 把两颗球挪位置，
 * 不依赖任何额外的守护进程，界面里点一下就生效。
 */
#ifndef GZ_ROS_TRAFFICLIGHT_HH_
#define GZ_ROS_TRAFFICLIGHT_HH_

#include <string>

#include <gz/gui/qt.h>
#include <gz/gui/Plugin.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/boolean.pb.h>

class TrafficLight : public gz::gui::Plugin
{
  Q_OBJECT
  Q_PROPERTY(QString lightColor READ LightColor NOTIFY LightColorChanged)

  public: TrafficLight();
  public: ~TrafficLight() override;

  /// \brief 读取 <world> / <lamp_xy> / <red_z> / <green_z> 等参数
  public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

  /// \brief 当前颜色 "red" / "green"（给 QML 绑定高亮用）
  public: Q_INVOKABLE QString LightColor() const
  { return QString::fromStdString(this->current); }

  public: Q_INVOKABLE void SetRed();
  public: Q_INVOKABLE void SetGreen();

  signals: void LightColorChanged();

  private: void Apply(const std::string &_color);
  private: void MoveLamp(const std::string &_model, double _z);
  private: void OnPoseResult(const gz::msgs::Boolean &_reply, bool _result);

  private: gz::transport::Node node;
  private: std::string current{"red"};
  private: std::string world{"default"};
  private: double x{0.7};
  private: double y{6.0};
  private: double redZ{1.17};
  private: double greenZ{0.73};
};

#endif  // GZ_ROS_TRAFFICLIGHT_HH_
