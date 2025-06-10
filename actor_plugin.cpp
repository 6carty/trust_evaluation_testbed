#include <rclcpp/rclcpp.hpp>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_msgs/msg/model_states.hpp>
#include <geometry_msgs/msg/twist.hpp>

 namespace gazebo

{

  class SimpleActorPlugin : public ModelPlugin

  {

  private:

    physics::ModelPtr model;

    event::ConnectionPtr updateConnection;

    ignition::math::Vector3d velocity;

 

  public:

    SimpleActorPlugin() {}

    void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
    {
      // Store the model pointer
      this->model = _model;

      // Print message to confirm plugin is running
      std::cout << "SimpleActorPlugin Loaded: " << this->model->GetName() << std::endl;


      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&SimpleActorPlugin::OnUpdate, this));
      // Set initial velocity (random direction)
      this->velocity = ignition::math::Vector3d(
          (rand() % 3 - 1) * 0.2,  // Random X direction (-0.2, 0, 0.2)
          (rand() % 3 - 1) * 0.2,  // Random Y direction (-0.2, 0, 0.2)
          0);
    }

    void OnUpdate()
    {
      // Move the actor by setting its linear velocity
      this->model->SetLinearVel(this->velocity);

      // Change direction randomly every 5 seconds
      if (common::Time::GetWallTime().sec % 5 == 0)

      {
        this->velocity = ignition::math::Vector3d(
            (rand() % 3 - 1) * 0.2,  // New random X direction
            (rand() % 3 - 1) * 0.2,  // New random Y direction
            0);
      }
    }
  };

  GZ_REGISTER_MODEL_PLUGIN(SimpleActorPlugin)
}

