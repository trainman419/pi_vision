#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>
#include <GL/glut.h>
#include <string>
#include "KinectController.h"
#include "KinectDisplay.h"

using std::string;

#ifndef PI
#define PI 3.14159265359
#endif
#ifndef HALFPI
#define HALFPI 1.57079632679
#endif 
#ifndef QUARTPI
#define QUARTPI 0.785398163397
#endif

namespace veltrobot_teleop
{

	class TeleopKinect
  {
    public:
      TeleopKinect()
      {    
      }
      
      void init()
      {
        ros::NodeHandle n;
        motion_pub_ = n.advertise <std_msgs::String> ("motion_name", 1);
        joint_states_pub_ = n.advertise<sensor_msgs::JointState>("/joint_states", 10);
        cmd_joints_pub_ = n.advertise<sensor_msgs::JointState>("/cmd_joints", 1);
        cmd_vel_pub_ = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
      }
      
      void publishTransform(KinectController& kinect_controller, XnUserID const& user, XnSkeletonJoint const& joint, string const& frame_id, string const& child_frame_id)
      {

      	xn::UserGenerator& UserGenerator = kinect_controller.getUserGenerator();
        static tf::TransformBroadcaster br;

        XnSkeletonJointPosition joint_position;
        UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, joint, joint_position);
        double x = joint_position.position.X / 1000.0;
        double y = joint_position.position.Y / 1000.0;
        double z = joint_position.position.Z / 1000.0;

        XnSkeletonJointOrientation joint_orientation;
        UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, joint, joint_orientation);

        XnFloat* m = joint_orientation.orientation.elements;
        KDL::Rotation rotation(m[0], m[1], m[2],
			       m[3], m[4], m[5],
			       m[6], m[7], m[8]);
        double qx, qy, qz, qw;
        rotation.GetQuaternion(qx, qy, qz, qw);

        tf::Transform transform;
        transform.setOrigin(tf::Vector3(x, y, z));
        transform.setRotation(tf::Quaternion(qx, qy, qz, qw));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_id));
			}
        
      void processKinect(KinectController& kinect_controller)
      {
        XnUserID users[15];
        XnUInt16 users_count = 15;
        xn::UserGenerator& UserGenerator = kinect_controller.getUserGenerator();
        UserGenerator.GetUsers(users, users_count);

        for (int i = 0; i < users_count; ++i)
        {
          XnUserID user = users[i];
          if (!UserGenerator.GetSkeletonCap().IsTracking(user))
            continue;
            
          string frame_id("openni_depth");

          publishTransform(kinect_controller, user, XN_SKEL_HEAD,           frame_id, "head");
          publishTransform(kinect_controller, user, XN_SKEL_NECK,           frame_id, "neck");
          publishTransform(kinect_controller, user, XN_SKEL_TORSO,          frame_id, "torso");

          publishTransform(kinect_controller, user, XN_SKEL_LEFT_SHOULDER,  frame_id, "left_shoulder");
          publishTransform(kinect_controller, user, XN_SKEL_LEFT_ELBOW,     frame_id, "left_elbow");
          publishTransform(kinect_controller, user, XN_SKEL_LEFT_HAND,      frame_id, "left_hand");

          publishTransform(kinect_controller, user, XN_SKEL_RIGHT_SHOULDER, frame_id, "right_shoulder");
          publishTransform(kinect_controller, user, XN_SKEL_RIGHT_ELBOW,    frame_id, "right_elbow");
          publishTransform(kinect_controller, user, XN_SKEL_RIGHT_HAND,     frame_id, "right_hand");

          publishTransform(kinect_controller, user, XN_SKEL_LEFT_HIP,       frame_id, "left_hip");
          publishTransform(kinect_controller, user, XN_SKEL_LEFT_KNEE,      frame_id, "left_knee");
          publishTransform(kinect_controller, user, XN_SKEL_LEFT_FOOT,      frame_id, "left_foot");

          publishTransform(kinect_controller, user, XN_SKEL_RIGHT_HIP,      frame_id, "right_hip");
          publishTransform(kinect_controller, user, XN_SKEL_RIGHT_KNEE,     frame_id, "right_knee");
          publishTransform(kinect_controller, user, XN_SKEL_RIGHT_FOOT,     frame_id, "right_foot");  
      		
          // two ways to go about this...
          // 1 calculated approach
          // 1.1? publish transform of two joints relative to openni_depth frame
          // 1.2? request transform between two joints own frames 
      		// 2 [nearly] direct aproach
          // 2.1 get points of three joints
          // 2.2 do many specific calculations relating that geometry to my own
          //     robots geometry

	  // get joint positions
	  XnSkeletonJointPosition joint_position_head;
	  UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_HEAD, joint_position_head);
          KDL::Vector head(joint_position_head.position.X, joint_position_head.position.Y, joint_position_head.position.Z);
	  XnSkeletonJointPosition joint_position_neck;
	  UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_NECK, joint_position_neck);
          KDL::Vector neck(joint_position_neck.position.X, joint_position_neck.position.Y, joint_position_neck.position.Z);
	  XnSkeletonJointPosition joint_position_torso;
	  UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_TORSO, joint_position_torso);
          KDL::Vector torso(joint_position_torso.position.X, joint_position_torso.position.Y, joint_position_torso.position.Z);

	  // Left Arm
	  XnSkeletonJointPosition joint_position_left_hand;
          UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_HAND, joint_position_left_hand);
	  KDL::Vector left_hand(joint_position_left_hand.position.X, joint_position_left_hand.position.Y, joint_position_left_hand.position.Z);

	  XnSkeletonJointPosition joint_position_left_elbow;
          UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_ELBOW, joint_position_left_elbow);
	  KDL::Vector left_elbow(joint_position_left_elbow.position.X, joint_position_left_elbow.position.Y, joint_position_left_elbow.position.Z);        

	  XnSkeletonJointPosition joint_position_left_shoulder;
          UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_SHOULDER, joint_position_left_shoulder);
	  KDL::Vector left_shoulder(joint_position_left_shoulder.position.X, joint_position_left_shoulder.position.Y, joint_position_left_shoulder.position.Z);

	  // Right Arm
          XnSkeletonJointPosition joint_position_right_hand;
          UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_HAND, joint_position_right_hand);
	  KDL::Vector right_hand(joint_position_right_hand.position.X, joint_position_right_hand.position.Y, joint_position_right_hand.position.Z);

          XnSkeletonJointPosition joint_position_right_elbow;
          UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_ELBOW, joint_position_right_elbow);
	  KDL::Vector right_elbow(joint_position_right_elbow.position.X, joint_position_right_elbow.position.Y, joint_position_right_elbow.position.Z);        

	  XnSkeletonJointPosition joint_position_right_shoulder;
          UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_SHOULDER, joint_position_right_shoulder);
	  KDL::Vector right_shoulder(joint_position_right_shoulder.position.X, joint_position_right_shoulder.position.Y, joint_position_right_shoulder.position.Z);

	  // Right Leg
          XnSkeletonJointPosition joint_position_right_hip;
          UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_HIP, joint_position_right_hip);
	  KDL::Vector right_hip(joint_position_right_hip.position.X, joint_position_right_hip.position.Y, joint_position_right_hip.position.Z);

          XnSkeletonJointPosition joint_position_right_knee;
          UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_KNEE, joint_position_right_knee);
	  KDL::Vector right_knee(joint_position_right_knee.position.X, joint_position_right_knee.position.Y, joint_position_right_knee.position.Z);

          XnSkeletonJointPosition joint_position_right_foot;
          UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_RIGHT_FOOT, joint_position_right_foot);
	  KDL::Vector right_foot(joint_position_right_foot.position.X, joint_position_right_foot.position.Y, joint_position_right_foot.position.Z);

	  // Left Leg
          XnSkeletonJointPosition joint_position_left_hip;
          UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_HIP, joint_position_left_hip);
	  KDL::Vector left_hip(joint_position_left_hip.position.X, joint_position_left_hip.position.Y, joint_position_left_hip.position.Z);

          XnSkeletonJointPosition joint_position_left_knee;
          UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_KNEE, joint_position_left_knee);
	  KDL::Vector left_knee(joint_position_left_knee.position.X, joint_position_left_knee.position.Y, joint_position_left_knee.position.Z);

          XnSkeletonJointPosition joint_position_left_foot;
          UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_LEFT_FOOT, joint_position_left_foot);
	  KDL::Vector left_foot(joint_position_left_foot.position.X, joint_position_left_foot.position.Y, joint_position_left_foot.position.Z);

	  // Torso
	  XnSkeletonJointOrientation joint_orientation_torso;
          UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_TORSO, joint_orientation_torso);
	  XnFloat* m = joint_orientation_torso.orientation.elements;
	  KDL::Rotation torso_rotation(m[0], m[1], m[2],
				       m[3], m[4], m[5],
				       m[6], m[7], m[8]);

	  double t_roll, t_pitch, t_yaw;
	  torso_rotation.GetRPY(t_roll, t_pitch, t_yaw);

	  //double qx, qy, qz, qw;
	  //rotation.GetQuaternion(qx, qy, qz, qw);

          // torso yaw
	  static double torso_angle_yaw = 0;
          if (joint_orientation_torso.fConfidence >= 0.5)
          {           
          	torso_angle_yaw = t_pitch;
          }

	  XnSkeletonJointOrientation joint_orientation_head;
          UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_HEAD, joint_orientation_head);
	  XnFloat* h = joint_orientation_head.orientation.elements;
	  KDL::Rotation head_rotation(h[0], h[1], h[2],
				      h[3], h[4], h[5],
				      h[6], h[7], h[8]);

	  double h_roll, h_pitch, h_yaw;
	  head_rotation.GetRPY(h_roll, h_pitch, h_yaw);

          // head yaw
	  static double head_angle_yaw = 0;
          if (joint_orientation_head.fConfidence >= 0.5)
          {     
	    head_angle_yaw = h_pitch;
          }

          // head pitch
	  static double head_angle_pitch = 0;
          if (joint_orientation_head.fConfidence >= 0.5)
          {           
	    head_angle_pitch = h_roll;
          }

	  /*
          // head pan/tilt
          KDL::Vector head_torso(head - torso);
	  KDL::Vector head_neck(head - neck);
          head_torso.Normalize();
          head_neck.Normalize();
          static double head_angle_pitch = 0;
          if (joint_position_neck.fConfidence >= 0.5 && 
          		joint_position_torso.fConfidence >= 0.5 && 
              joint_position_head.fConfidence >= 0.5)
          {
          	head_angle_pitch = asin(KDL::dot(head_torso, head_neck));
		//          	head_angle_pitch = head_angle_pitch - HALFPI;
          }
	  */

	  // Right hip roll
          KDL::Vector right_hip_knee(right_knee - right_hip);
	  KDL::Vector right_hip_torso(right_hip - torso);
          right_hip_knee.Normalize();
          right_hip_torso.Normalize();
          static double right_hip_angle_roll = 0;
          if (joint_position_right_hip.fConfidence >= 0.5 && 
          		joint_position_right_knee.fConfidence >= 0.5 && 
              joint_position_torso.fConfidence >= 0.5)
          {     
          	right_hip_angle_roll = asin(KDL::dot(right_hip_knee, right_hip_torso));
		//          	right_hip_angle_roll = -(right_hip_angle_roll - HALFPI);
	  } 

	  // Left hip roll
          KDL::Vector left_hip_knee(left_knee - left_hip);
	  KDL::Vector left_hip_torso(left_hip - torso);
          left_hip_knee.Normalize();
          left_hip_torso.Normalize();
          static double left_hip_angle_roll = 0;
          if (joint_position_left_hip.fConfidence >= 0.5 && 
          		joint_position_left_knee.fConfidence >= 0.5 && 
              joint_position_torso.fConfidence >= 0.5)
          {     
          	left_hip_angle_roll = asin(KDL::dot(left_hip_knee, left_hip_torso));
		//          	left_hip_angle_roll = -(left_hip_angle_roll - HALFPI);
	  } 


          // left elbow roll
	  KDL::Vector left_elbow_hand(left_hand - left_elbow);
	  KDL::Vector left_elbow_shoulder(left_shoulder - left_elbow);
          left_elbow_hand.Normalize();
          left_elbow_shoulder.Normalize();
          static double left_elbow_angle_roll = 0;
          if (joint_position_left_hand.fConfidence >= 0.5 && 
          		joint_position_left_elbow.fConfidence >= 0.5 && 
              joint_position_left_shoulder.fConfidence >= 0.5)
          {          
          	left_elbow_angle_roll = acos(KDL::dot(left_elbow_hand, left_elbow_shoulder));
		left_elbow_angle_roll = (left_elbow_angle_roll - PI);
          }  


          // right elbow roll
	  KDL::Vector right_elbow_hand(right_hand - right_elbow);
	  KDL::Vector right_elbow_shoulder(right_shoulder - right_elbow);
          right_elbow_hand.Normalize();
          right_elbow_shoulder.Normalize();
          static double right_elbow_angle_roll = 0;
          if (joint_position_right_hand.fConfidence >= 0.5 && 
          		joint_position_right_elbow.fConfidence >= 0.5 && 
              joint_position_right_shoulder.fConfidence >= 0.5)
          {          
          	right_elbow_angle_roll = acos(KDL::dot(right_elbow_hand, right_elbow_shoulder));
        		right_elbow_angle_roll = -(right_elbow_angle_roll - PI);
          }  

          // left shoulder roll
          KDL::Vector left_shoulder_elbow(left_elbow - left_shoulder);
        	KDL::Vector left_shoulder_neck(neck - left_shoulder);
          left_shoulder_elbow.Normalize();
          left_shoulder_neck.Normalize();
          static double left_shoulder_angle_roll = 0;
          if (joint_position_neck.fConfidence >= 0.5 && 
          		joint_position_left_elbow.fConfidence >= 0.5 && 
              joint_position_left_shoulder.fConfidence >= 0.5)
          {
          	left_shoulder_angle_roll = acos(KDL::dot(left_shoulder_elbow, left_shoulder_neck));
          	left_shoulder_angle_roll = left_shoulder_angle_roll - HALFPI;
          }
                   
          // right shoulder roll
          KDL::Vector right_shoulder_elbow(right_elbow - right_shoulder);
        	KDL::Vector right_shoulder_neck(neck - right_shoulder);
          right_shoulder_elbow.Normalize();
          right_shoulder_neck.Normalize();
          static double right_shoulder_angle_roll = 0;
          if (joint_position_neck.fConfidence >= 0.5 && 
          		joint_position_right_elbow.fConfidence >= 0.5 && 
              joint_position_right_shoulder.fConfidence >= 0.5)
          {     
          	right_shoulder_angle_roll = acos(KDL::dot(right_shoulder_elbow, right_shoulder_neck));
          	right_shoulder_angle_roll = -(right_shoulder_angle_roll - HALFPI);                                      
          } 
                          
          // left shoulder pitch
          static double left_shoulder_angle_pitch = 0;
          if (joint_position_left_shoulder.fConfidence >= 0.5)
          { 
          	left_shoulder_angle_pitch = asin(left_shoulder_elbow.y());
          	left_shoulder_angle_pitch = left_shoulder_angle_pitch + HALFPI;
          }
          
	  // right shoulder pitch
          static double right_shoulder_angle_pitch = 0;
        	if (joint_position_right_shoulder.fConfidence >= 0.5)
          { 
          	right_shoulder_angle_pitch = asin(right_shoulder_elbow.y());
          	right_shoulder_angle_pitch = -(right_shoulder_angle_pitch + HALFPI);
          }
                                                        
          // left shoulder yaw
	static double left_shoulder_angle_yaw = 0;
          if (joint_position_left_shoulder.fConfidence >= 0.5)
          {           
          	left_shoulder_angle_yaw = asin(left_elbow_hand.x());  // left_shoulder_elbow.x()
          }
          
          // right shoulder yaw
	  static double right_shoulder_angle_yaw = 0;
          if (joint_position_right_shoulder.fConfidence >= 0.5)
          {  
          	right_shoulder_angle_yaw = asin(right_elbow_hand.x());  // left_shoulder_elbow.x()
		right_shoulder_angle_yaw = -right_shoulder_angle_yaw;
          }

	  KDL::Vector right_left_foot(right_foot - left_foot);
	  static double foot_mouse_angle = 0;
	  static double foot_mouse_length = 0;
	  if (joint_position_left_foot.fConfidence >= 0.5 && 
	      joint_position_right_foot.fConfidence >= 0.5)
	    {
	      foot_mouse_length = sqrt(right_left_foot.x()*right_left_foot.x() + 
				       right_left_foot.z()*right_left_foot.z());
	      foot_mouse_angle = atan2(right_left_foot.x(), right_left_foot.z());
	      foot_mouse_angle = foot_mouse_angle - HALFPI;
	    }

	  geometry_msgs::Twist cmd_vel;
	  cmd_vel.linear.x = foot_mouse_length / 2400;
	  cmd_vel.angular.z = 0;

	  if (cmd_vel.linear.x > 0.2)
	    {
	      cmd_vel.linear.x = 0.2;
	    }

	  if (foot_mouse_angle > -0.5 && foot_mouse_angle < 0.5)
	    {
	      cmd_vel.linear.x = 0.0;
	    }
	  else if (foot_mouse_angle < 0.0)
	    {
	      cmd_vel.linear.x = -cmd_vel.linear.x;
	    }

	  ROS_INFO("ANGLE: %f", foot_mouse_angle);

          cmd_vel_pub_.publish(cmd_vel);

          // hand open/close or palm recognition to control grippers
          
          // PROBLEM:
          // doing each of the 3 degrees of freedom in the shoulder separately is having problems.
          // 1. the acos has no sign, it only reads less than one quater pi rotation
          // 2. there is a paradox/redundancy with joint movement to be considered: ie, to lift the
          //    arm vertically up it could either rotate the pitch or roll joint by one pi
          // SOLUTION?:
          // if we take the raw tf rotation from the input shoulder, can we apply that to our
          // final dof of our shoulder chain, and then back solve for the two joints leading to it?
        
	  // send to robot
          sensor_msgs::JointState js; 
          js.name.push_back("left_elbow_joint");
          js.position.push_back(left_elbow_angle_roll);
          js.velocity.push_back(1);
          js.name.push_back("right_elbow_joint");
          js.position.push_back(right_elbow_angle_roll);
          js.velocity.push_back(1);
          js.name.push_back("left_shoulder_lift_joint");
          js.position.push_back(left_shoulder_angle_pitch);
          js.velocity.push_back(1);
          js.name.push_back("right_shoulder_lift_joint");
	  js.position.push_back(right_shoulder_angle_pitch);
          js.velocity.push_back(1);          
          js.name.push_back("left_arm_roll_joint");
          js.position.push_back(0);
          js.velocity.push_back(1);
          js.name.push_back("right_arm_roll_joint");
          js.position.push_back(0);
          js.velocity.push_back(1);          
          js.name.push_back("left_shoulder_pan_joint");
          js.position.push_back(left_shoulder_angle_yaw);
          js.velocity.push_back(1);
          js.name.push_back("right_shoulder_pan_joint");
          js.position.push_back(right_shoulder_angle_yaw);
          js.velocity.push_back(1);
          js.name.push_back("torso_joint");
	  //          js.position.push_back(torso_angle_yaw);
	  js.position.push_back(foot_mouse_angle);
          js.velocity.push_back(1);
          js.name.push_back("head_pan_joint");
          js.position.push_back(head_angle_yaw);
          js.velocity.push_back(1);
          js.name.push_back("head_tilt_joint");
          js.position.push_back(head_angle_pitch);
          js.velocity.push_back(1);

          js.name.push_back("left_wrist_joint");
          js.position.push_back(0);
          js.velocity.push_back(1);

          js.name.push_back("right_wrist_joint");
          js.position.push_back(0);
          js.velocity.push_back(1);
          
          /*
          js.name.push_back("neck_pitch");
          js.position.push_back(0);
          js.velocity.push_back(1);
          js.name.push_back("neck_yaw");
          js.position.push_back(0);
          js.velocity.push_back(1);

          js.name.push_back("hip_left_yaw");
          js.position.push_back(0);
          js.velocity.push_back(1);
          js.name.push_back("hip_left_roll");
          js.position.push_back(0);
          js.velocity.push_back(1);
          js.name.push_back("hip_left_pitch");
          js.position.push_back(0);
          js.velocity.push_back(1);
          js.name.push_back("knee_left_pitch");
          js.position.push_back(0);
          js.velocity.push_back(1);
          js.name.push_back("ankle_left_pitch");
          js.position.push_back(0);
          js.velocity.push_back(1);
          js.name.push_back("ankle_left_roll");
          js.position.push_back(0);
          js.velocity.push_back(1);
          js.name.push_back("hip_right_yaw");
          js.position.push_back(0);
          js.velocity.push_back(1);
          js.name.push_back("hip_right_roll");
          js.position.push_back(0);
          js.velocity.push_back(1);
          js.name.push_back("hip_right_pitch");
          js.position.push_back(0);
          js.velocity.push_back(1);
          js.name.push_back("knee_right_pitch");
          js.position.push_back(0);
          js.velocity.push_back(1);
          js.name.push_back("ankle_right_pitch");
          js.position.push_back(0);
          js.velocity.push_back(1);
          js.name.push_back("ankle_right_roll");
          js.position.push_back(0);
          js.velocity.push_back(1);                
          */          

//    <joint name="base_yaw"  position="0.0" />
//    <joint name="base_pitch"  position="0.0" />
//    <joint name="base_roll"  position="0.0" />
//    <joint name="base"  position="0.0" />
//    <joint name="stereo_camera_fixed"  position="0.0" />                                    

          joint_states_pub_.publish(js);
          //cmd_joints_pub_.publish(js);
	  break;	// only read first user
        }
      }
         
    private:
      ros::Publisher   motion_pub_;
      ros::Publisher   joint_states_pub_;
      ros::Publisher   cmd_joints_pub_;
      ros::Publisher   cmd_vel_pub_;
  };

} // namespace veltrobot_teleop


#define GL_WIN_SIZE_X 720
#define GL_WIN_SIZE_Y 480
KinectController g_kinect_controller;
veltrobot_teleop::TeleopKinect g_teleop_kinect;

void glutIdle (void)
{
	glutPostRedisplay();
}

void glutDisplay (void)
{
	xn::SceneMetaData sceneMD;
	xn::DepthMetaData depthMD;
  
  g_kinect_controller.getContext().WaitAndUpdateAll();
  g_teleop_kinect.processKinect(g_kinect_controller);
  g_kinect_controller.getDepthGenerator().GetMetaData(depthMD);
  g_kinect_controller.getUserGenerator().GetUserPixels(0, sceneMD);  
	
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Setup the OpenGL viewpoint
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	glOrtho(0, depthMD.XRes(), depthMD.YRes(), 0, -1.0, 1.0);
  
  glDisable(GL_TEXTURE_2D);
  
  kinect_display_drawDepthMapGL(depthMD, sceneMD);
	kinect_display_drawSkeletonGL(g_kinect_controller.getUserGenerator(),
                                g_kinect_controller.getDepthGenerator());  
  
	glutSwapBuffers();
}

void glutKeyboard (unsigned char key, int x, int y)
{
	switch (key)
	{
	case 27:
    exit(1);
    break;
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "teleop_kinect");
  ros::NodeHandle n;
  g_teleop_kinect.init();
  char* onifile = NULL;
  if (argc > 1)
  	onifile = argv[1];
  g_kinect_controller.init(onifile);
  
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
	glutCreateWindow ("Veltrobot Kinect Controller");
	//glutFullScreen();
	//glutSetCursor(GLUT_CURSOR_NONE);

	glutKeyboardFunc(glutKeyboard);
	glutDisplayFunc(glutDisplay);
	glutIdleFunc(glutIdle);

	glDisable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);

	glEnableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);  
  
  glutMainLoop();
  
  g_kinect_controller.shutdown();
  
	return 0;
}


