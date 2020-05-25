#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

namespace gazebo {
  class ModelControl : public ModelPlugin {

    
  public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
    // Store the pointer to the model
    this->model = _parent;
    
    // Store the pointers to the joints
    this->jointR1_ = this->model->GetJoint("r1");
    
    // Listen to the update event. This event is broadcast every simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ModelControl::OnUpdate, this, _1));
  }
    
    
    // Called by the world update start event
  public: void OnUpdate(const common::UpdateInfo & /*_info*/) {
    
    common::Time current_time = this->model->GetWorld()->GetSimTime();  // Get the world simulation time
    double timeNow = current_time.Double();


    const double pi = boost::math::constants::pi<double>();
    double theta1Desired = 1.5 * pi;
    this->theta1_ = this->jointR1_->GetAngle(0);
    printf("Current angle is \t%f\tdegrees\n", this->theta1_.Degree());
    double eTheta1 = theta1Desired - this->theta1_.Radian();


    double kP = 25;
    double torque = kP * eTheta1;
    
    this->jointR1_->SetForce(0, torque);
    //printf("Current time is \t %f\n", timeNow);
  }
 
   
    // Maybe I want to keep track of time?
    common::Time last_update_time_;
    
    // Pointer to the model
  private: physics::ModelPtr model;
    
    // Pointer to the update event connection
  private: event::ConnectionPtr updateConnection;
    
    // Pointers to joints
    physics::JointPtr jointR1_;

    // Pointers to joint angles
    gazebo::math::Angle theta1_;
  };
  
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelControl)
}
