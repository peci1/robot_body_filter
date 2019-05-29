#define protected public
#include <geometric_shapes/bodies.h>
#undef protected

#include <geometric_shapes/body_operations.h>
#include <robot_body_filter/utils/bodies.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Core>

void initMarker(visualization_msgs::Marker& obj)
{
  obj.header.frame_id = "odom";
  obj.header.stamp = ros::Time::now();
  obj.lifetime = ros::Duration(100.0);
  obj.frame_locked = true;
  obj.action = visualization_msgs::Marker::ADD;
  obj.color.a = 0.5;
  obj.id = 0;
}

#define _BOX 1
#define _SPHERE 0
#define _CYLINDER 0

int main(int argc, char** argv)
{
  srand((unsigned int) time(0));

  ros::init(argc, argv, "test");

  ros::NodeHandle nh;
  visualization_msgs::Marker m, m2;
  visualization_msgs::Marker obj, obju, o1, o2, c1, c2;
  initMarker(m);
  initMarker(m2);
  initMarker(obj);
  initMarker(obju);
  initMarker(o1);
  initMarker(o2);
  initMarker(c1);
  initMarker(c2);

  ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("/test", 10);

#if _SPHERE
  shapes::ShapePtr s(new shapes::Sphere(fabs(Eigen::internal::random<double>()) * 1.5));
#endif
#if _CYLINDER
  shapes::ShapePtr s(new shapes::Cylinder(fabs(Eigen::internal::random<double>()) * 1.5, fabs(Eigen::internal::random<double>()) * 3));
#endif
#if _BOX
  shapes::ShapePtr s(new shapes::Box(fabs(Eigen::internal::random<double>()) * 3, fabs(Eigen::internal::random<double>()) * 3, fabs(Eigen::internal::random<double>()) * 3));
#endif

  bodies::BodyPtr b(bodies::createBodyFromShape(s.get()));

#if _BOX
  bodies::Box* bb = ((bodies::Box*)b.get());
  Eigen::Vector3d sizes;
  sizes.x() = bb->length2_;
  sizes.y() = bb->width2_;
  sizes.z() = bb->height2_;

  std::cout << sizes << std::endl;
#endif

  Eigen::Isometry3d bPos = Eigen::Isometry3d::Identity();
  Eigen::Vector3d bTrans = Eigen::Vector3d::Random();
  bPos.translation() = bTrans;
//  bPos.linear() = Eigen::AngleAxisd( EIGEN_PI, Eigen::Vector3d::Ones().normalized()).toRotationMatrix();
  bPos.rotate(Eigen::Quaterniond::UnitRandom().slerp(0.0, Eigen::Quaterniond::Identity()));
  b->setPose(bPos);
#if _BOX
  sizes.x() = bb->length2_;
  sizes.y() = bb->width2_;
  sizes.z() = bb->height2_;

  std::cout << sizes << std::endl;
#endif

  bodies::constructMarkerFromBody(b.get(), obju);
  obju.ns = "obju";
  obju.color.b = 1;

  b->setScale(1.0 + Eigen::internal::random<double>() * 0.9);
#if _BOX
  sizes.x() = bb->length2_;
  sizes.y() = bb->width2_;
  sizes.z() = bb->height2_;

  std::cout << sizes << std::endl;
#endif

  b->setPadding(Eigen::internal::random<double>() * 0.3);
#if _BOX
  sizes.x() = bb->length2_;
  sizes.y() = bb->width2_;
  sizes.z() = bb->height2_;

  std::cout << sizes << std::endl;
#endif

  bodies::constructMarkerFromBody(b.get(), obj);
  obj.color.r = 1;
  obj.ns = "obj";

#if _BOX
  o1.pose.orientation.w = 1;
  o1.pose.position = tf2::toMsg(bPos * (-sizes));
  o1.type = visualization_msgs::Marker::SPHERE;
  o1.scale.x = o1.scale.y = o1.scale.z = 0.2;
  o1.color.g = 1;
  o1.ns = "o1";

  o2.pose.orientation.w = 1;
  o2.pose.position = tf2::toMsg(bPos * sizes);
  o2.type = visualization_msgs::Marker::SPHERE;
  o2.scale.x = o2.scale.y = o2.scale.z = 0.2;
  o2.color.g = 1;
  o2.color.r = 1;
  o2.ns = "o2";

  c1.pose.orientation.w = 1;
  c1.pose.position = tf2::toMsg(bb->corner1_);
  c1.type = visualization_msgs::Marker::SPHERE;
  c1.scale.x = c1.scale.y = c1.scale.z = 0.15;
  c1.color.g = 0.5;
  c1.ns = "c1";

  c2.pose.orientation.w = 1;
  c2.pose.position = tf2::toMsg(bb->corner2_);
  c2.type = visualization_msgs::Marker::SPHERE;
  c2.scale.x = c2.scale.y = c2.scale.z = 0.15;
  c2.color.g = 0.5;
  c2.color.r = 0.5;
  c2.ns = "c2";
#endif

  m.type = visualization_msgs::Marker::LINE_LIST;
  m.ns = "lines";
  m.color.a = 1;
  m.scale.x = 0.01;

  m2.type = visualization_msgs::Marker::POINTS;
  m2.ns = "points";
  m2.color.a = 1;
  m2.scale.x = m2.scale.y = 0.02;
  m2.pose.orientation.w = 1;

  std_msgs::ColorRGBA red;
  std_msgs::ColorRGBA green;
  std_msgs::ColorRGBA blue;
  std_msgs::ColorRGBA white;
  std_msgs::ColorRGBA yellow;
  std_msgs::ColorRGBA black;

  red.r = red.a = 1.0;
  green.g = green.a = 1.0;
  blue.b = blue.a = 1.0;
  white.b = white.a = white.r = white.g = 1.0;
  yellow.a = yellow.r = yellow.g = 1.0;
  black.a = 1.0;

  for (size_t i = 0; i < 35; ++i) {
    Eigen::Vector3d p1 = Eigen::Vector3d::Random() * 3;
    Eigen::Vector3d dir = (Eigen::Vector3d::Random() * 1 - p1);

    if (b->containsPoint(p1))
      continue;

    EigenSTL::vector_Vector3d ints;
    bool intersects = bodies::intersectsRay(b.get(), p1, dir.normalized(), &ints, 0);
    if (intersects) {
      m.points.push_back(tf2::toMsg(p1));
      m.points.push_back(tf2::toMsg(ints[0]));
      m.colors.push_back(green);
      m.colors.push_back(green);
      if (ints.size() == 1) {
        m.points.push_back(tf2::toMsg(ints[0]));
        m.points.push_back(tf2::toMsg(Eigen::Vector3d(p1 + 10 * dir)));
        m.colors.push_back(white);
        m.colors.push_back(white);
      } else {
        m.points.push_back(tf2::toMsg(ints[0]));
        m.points.push_back(tf2::toMsg(ints[1]));
        m.points.push_back(tf2::toMsg(ints[1]));
        m.points.push_back(tf2::toMsg(Eigen::Vector3d(p1 + 10 * dir)));
        m.colors.push_back(white);
        m.colors.push_back(white);
        m.colors.push_back(blue);
        m.colors.push_back(blue);
      }
    } else {
      m.points.push_back(tf2::toMsg(p1));
      m.points.push_back(tf2::toMsg(Eigen::Vector3d(p1 + 2*dir)));
      m.colors.push_back(red);
      m.colors.push_back(yellow);
    }
  }

  for (double x = -1; x < 1; x += 0.01) {
    for (double y = -1; y < 1; y += 0.01) {
      for (double z = -1; z < 1; z += 0.01) {
        Eigen::Vector3d p1 = Eigen::Vector3d(x, y, z) * 3;
        if (b->containsPoint(p1)) {
          m2.points.push_back(tf2::toMsg(p1));
          m2.colors.push_back(white);
        } else {
//          m2.colors.push_back(black);
        }
      }
    }
  }

  while (ros::ok()) {
    pub.publish(obj);
    pub.publish(obju);
    pub.publish(o1);
    pub.publish(o2);
    pub.publish(c1);
    pub.publish(c2);
    pub.publish(m);
    pub.publish(m2);
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }
}