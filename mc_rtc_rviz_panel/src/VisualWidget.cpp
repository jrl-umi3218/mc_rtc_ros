#include "VisualWidget.h"

namespace mc_rtc_rviz
{

VisualWidget::VisualWidget(const ClientWidgetParam & params, visualization_msgs::MarkerArray & markers)
: ClientWidget(params), markers_(markers), visible_(visible()), was_visible_(visible_)
{
  auto layout = new QHBoxLayout(this);
  if(!secret()) { layout->addWidget(new QLabel(id().name.c_str())); }
  button_ = new QPushButton(this);
  button_->setCheckable(true);
  button_->setChecked(!visible());
  toggled(!visible_);
  layout->addWidget(button_);
  connect(button_, SIGNAL(toggled(bool)), this, SLOT(toggled(bool)));
  marker_.ns = id2name(id());
  marker_.header.frame_id = "robot_map";
  marker_.header.seq = 0;
}

VisualWidget::~VisualWidget()
{
  marker_.action = visualization_msgs::Marker::DELETE;
  markers_.markers.push_back(marker_);
}

void VisualWidget::update(const rbd::parsers::Visual & visual, const sva::PTransformd & pose)
{
  {
    // Marker general settings
    marker_.header.seq++;
    marker_.header.stamp = ros::Time::now();
    marker_.action = visualization_msgs::Marker::ADD;
  }
  {
    // Convert the position
    auto X_0_marker = visual.origin * pose;
    geometry_msgs::Pose p;
    Eigen::Quaterniond q{X_0_marker.rotation().transpose()};
    p.orientation.w = q.w();
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    const auto & t = X_0_marker.translation();
    p.position.x = t.x();
    p.position.y = t.y();
    p.position.z = t.z();
    marker_.pose = p;
  }
  {
    // Color from material, defaults to white
    if(visual.material.type == rbd::parsers::Material::Type::COLOR)
    {
      auto & c = boost::get<rbd::parsers::Material::Color>(visual.material.data);
      marker_.color.r = static_cast<float>(c.r);
      marker_.color.g = static_cast<float>(c.g);
      marker_.color.b = static_cast<float>(c.b);
      marker_.color.a = static_cast<float>(c.a);
    }
    else
    {
      marker_.color.r = 1.0;
      marker_.color.g = 1.0;
      marker_.color.b = 1.0;
      marker_.color.a = 1.0;
    }
  }
  {
    // Convert geometry
    using Geometry = rbd::parsers::Geometry;
    using Type = Geometry::Type;
    auto & type = visual.geometry.type;
    auto & geom_data = visual.geometry.data;
    if(type == Type::BOX)
    {
      auto & b = boost::get<Geometry::Box>(geom_data);
      marker_.type = visualization_msgs::Marker::CUBE;
      marker_.scale.x = b.size.x();
      marker_.scale.y = b.size.y();
      marker_.scale.z = b.size.z();
    }
    else if(type == Type::CYLINDER)
    {
      auto & c = boost::get<Geometry::Cylinder>(geom_data);
      marker_.type = visualization_msgs::Marker::CYLINDER;
      marker_.scale.x = 2 * c.radius;
      marker_.scale.y = 2 * c.radius;
      marker_.scale.z = c.length;
    }
    else if(type == Type::SPHERE)
    {
      auto & s = boost::get<Geometry::Sphere>(geom_data);
      marker_.type = visualization_msgs::Marker::SPHERE;
      marker_.scale.x = 2 * s.radius;
      marker_.scale.y = 2 * s.radius;
      marker_.scale.z = 2 * s.radius;
    }
    else if(type == Type::MESH)
    {
      auto & m = boost::get<Geometry::Mesh>(geom_data);
      marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
      marker_.scale.x = m.scale;
      marker_.scale.y = m.scale;
      marker_.scale.z = m.scale;
      marker_.mesh_resource = m.filename;
      marker_.mesh_use_embedded_materials = true;
    }
    else if(type == Type::SUPERELLIPSOID)
    {
      auto & se = boost::get<Geometry::Superellipsoid>(geom_data);
      if(se.epsilon1 != 1.0 || se.epsilon2 != 1.0)
      {
        static bool show_error_once = false;
        if(!show_error_once)
        {
          mc_rtc::log::error(
              "mc_rtc RViZ panel cannot handle Superellispoid visual if epsilon1 != 1.0 or epsilon2 != 1.0");
          show_error_once = true;
        }
      }
      else
      {
        marker_.type = visualization_msgs::Marker::SPHERE;
        marker_.scale.x = se.size.x();
        marker_.scale.y = se.size.y();
        marker_.scale.z = se.size.z();
      }
    }
  }
  if(visible_ || was_visible_)
  {
    if(!visible_) { marker_.action = visualization_msgs::Marker::DELETE; }
    markers_.markers.push_back(marker_);
  }
  was_visible_ = visible_;
}

void VisualWidget::toggled(bool hide)
{
  visible_ = !hide;
  button_->setText(hide ? "Show" : "Hide");
  visible(!hide);
}

} // namespace mc_rtc_rviz
