#include "omplrscurve.h"
#define SEGMENT_L 0.1

geometry_msgs::PoseStamped pack_state(State* s);
visualization_msgs::Marker pack_state_marker(State* s, int id);

void make_rs_curve();
void start_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
void end_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
ros::Publisher pubPath;
ros::Publisher pubMarker;
ros::Publisher pubPose_s;
ros::Publisher pubPose_e;
geometry_msgs::PoseStamped start;
geometry_msgs::PoseStamped end;
visualization_msgs::MarkerArray markerarray;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  start.pose.orientation.z=1;
  end.pose.orientation.z=1;

  //publisher
  pubPath = n.advertise<nav_msgs::Path>("RS_curve", 1, true);
  pubMarker = n.advertise<visualization_msgs::MarkerArray>("RS_curve_m", 1, true);
  pubPose_s = n.advertise<geometry_msgs::PoseStamped>("start", 1, true);
  pubPose_e = n.advertise<geometry_msgs::PoseStamped>("end", 1, true);
  //subscriber

  ros::Subscriber sub1 =  n.subscribe("/move_base_simple/goal",1,end_cb);
  ros::Subscriber sub2 =  n.subscribe("/initialpose",1,start_cb);
  
  ros::spin();
  return 0;
}

geometry_msgs::PoseStamped pack_state(State* s)
{
  geometry_msgs::PoseStamped pose;
  tf::Quaternion myQuaternion;
  pose.header.frame_id="map";
  myQuaternion.setRPY(0,0,s->getYaw());
  pose.pose.position.x = s->getX();
  pose.pose.position.y = s->getY();
  pose.pose.orientation.x = myQuaternion.getX();
  pose.pose.orientation.y = myQuaternion.getY();
  pose.pose.orientation.z = myQuaternion.getZ();
  pose.pose.orientation.w = myQuaternion.getW();
  return pose;

}

visualization_msgs::Marker pack_state_marker(State* s, int id=1)
{
  visualization_msgs::Marker pose;
  tf::Quaternion myQuaternion;
  pose.header.frame_id="map";
  pose.type = 0; //arrow
  pose.action = 0;//add and modify
  pose.id = id;
  myQuaternion.setRPY(0,0,s->getYaw());
  pose.pose.position.x = s->getX();
  pose.pose.position.y = s->getY();
  pose.pose.orientation.x = myQuaternion.getX();
  pose.pose.orientation.y = myQuaternion.getY();
  pose.pose.orientation.z = myQuaternion.getZ();
  pose.pose.orientation.w = myQuaternion.getW();
  pose.scale.x = 0.07;
  pose.scale.y = 0.02;
  pose.scale.z = 0.01;
  pose.color.a = 1.0;
  pose.color.r = 1.0;
  pose.color.g = 1.0;
  pose.color.b = 1.0;
  pose.lifetime = ros::Duration(0.0);
  return pose;

}

void start_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  start.pose = msg->pose.pose;
  make_rs_curve();
}

void end_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
  end = *msg;
  make_rs_curve();
}

void make_rs_curve(){
  nav_msgs::Path rspath;
  geometry_msgs::PoseStamped tmp;
  tf::Quaternion myQuaternion;
  double roll=0, pitch=0, yaw=0;
  double prop = 0.0;

  ompl::base::ReedsSheppStateSpace reedsSheppPath(2.0);
  State* rsStart = (State*)reedsSheppPath.allocState();
  State* rsEnd = (State*)reedsSheppPath.allocState();

  // SET start and end pose
  rsStart->setXY(start.pose.position.x, start.pose.position.y);
  myQuaternion.setZ(start.pose.orientation.z);
  myQuaternion.setW(start.pose.orientation.w);
  tf::Matrix3x3 m(myQuaternion);
  m.getEulerYPR(yaw, pitch, roll);
  rsStart->setYaw(yaw);

  rsEnd->setXY(end.pose.position.x, end.pose.position.y);
  myQuaternion.setZ(end.pose.orientation.z);
  myQuaternion.setW(end.pose.orientation.w);
  tf::Matrix3x3 m2(myQuaternion);
  m2.getEulerYPR(yaw, pitch, roll);
  rsEnd->setYaw(yaw);
  
  std::cout<<"start x:"<<rsStart->getX()<<" y :"<<rsStart->getX()<<" yaw :"<<rsStart->getYaw()<<std::endl;
  std::cout<<"end x:"<<rsEnd->getX()<<" y :"<<rsEnd->getX()<<" yaw :"<<rsEnd->getYaw()<<std::endl;

  float reedsSheppCost = 0;
  reedsSheppCost = reedsSheppPath.distance(rsStart, rsEnd);
  std::cout<<"distance of reed shepp curve :"<<reedsSheppCost<<std::endl;
  prop = 1.0 / (reedsSheppCost/SEGMENT_L);
  // State* middle = (State*)reedsSheppPath.allocState();
  // reedsSheppPath.interpolate(rsStart,rsEnd, 0.5, middle);
  // reedsSheppPath.freeState(middle);


  rspath.header.frame_id="map";
  tmp = pack_state(rsStart);
  rspath.poses.push_back(tmp);
  pubPose_s.publish(tmp);

  // marker array 지우기
  
  std::vector<visualization_msgs::Marker>::iterator iter;
  for (iter=markerarray.markers.begin(); iter !=markerarray.markers.end(); iter++){
    iter->action = 2;
  }
  pubMarker.publish(markerarray);
  markerarray.markers.clear();

  // std::cout<<"expected nodes :"<<int(reedsSheppCost/SEGMENT_L)+2<<std::endl;
  int count = 0;
  for(float i=prop; i<1.0; i+=prop){
    State* middle = (State*)reedsSheppPath.allocState();
    reedsSheppPath.interpolate(rsStart,rsEnd, i, middle);
    markerarray.markers.push_back(pack_state_marker(middle,count));
    rspath.poses.push_back(pack_state(middle));
    reedsSheppPath.freeState(middle);
    count ++;
  }
  std::cout<<"total nodes :"<<count+2<<std::endl;

  tmp = pack_state(rsEnd);
  rspath.poses.push_back(tmp);
  pubPose_e.publish(tmp);

  pubPath.publish(rspath);
  pubMarker.publish(markerarray);
  reedsSheppPath.freeState(rsStart);
  reedsSheppPath.freeState(rsEnd);
}

