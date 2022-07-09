#include <gazebo/common/Plugin.hh>
//#include <gazebo/common/common.hh>
#include "rclcpp/rclcpp.hpp"
#include <gazebo/util/system.hh>
#include <gazebo/physics/physics.hh>
#include "gazebo/common/Animation.hh"
#include "gazebo/common/KeyEvent.hh"
#include "gazebo/common/KeyFrame.hh"
#include <ignition/math/Vector3.hh>
#include <ignition/math.hh>
#include <random>
#include <stdio.h>
#include <iostream>
//#include <windows.h>

namespace gazebo
{
    class Obstacle2 : public ModelPlugin
    {
    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr)
        {
            // Store the pointer to the model
            this->model = _parent;

            //std::random_device rd;

            // random_device \B8\A6 \C5\EB\C7\D8 \B3\AD\BC\F6 \BB\FD\BC\BA \BF\A3\C1\F8\C0\BB \C3ʱ\E2ȭ \C7Ѵ\D9.
            //std::mt19937 gen(rd());

            // 0 \BA\CE\C5\CD 99 \B1\EE\C1\F6 \B1յ\EE\C7ϰ\D4 \B3\AAŸ\B3\AA\B4\C2 \B3\AD\BC\F6\BF\AD\C0\BB \BB\FD\BC\BA\C7ϱ\E2 \C0\A7\C7\D8 \B1յ\EE \BA\D0\C6\F7 \C1\A4\C0\C7.
            //std::uniform_int_distribution<int> dis(-40,40);
            // create the animation
            gazebo::common::PoseAnimationPtr anim(
                // name the animation "move_1",
                // make it last 260 seconds,
                // and set it on a repeat loop
                new gazebo::common::PoseAnimation("move2", 105.0, true));

            gazebo::common::PoseKeyFrame* key;

            // set starting location of the box

            key = anim->CreateKeyFrame(0);
            key->Translation(ignition::math::Vector3d(0.0, 0.0, 0.0));
            key->Rotation(ignition::math::Quaterniond(0, 0, 0));

            key = anim->CreateKeyFrame(5);
            key->Translation(ignition::math::Vector3d(2.5, 0.0, 0.0));
            key->Rotation(ignition::math::Quaterniond(0, 0, 0));
            key = anim->CreateKeyFrame(10);
            key->Translation(ignition::math::Vector3d(5.0, 0.0, 0.0));
            key->Rotation(ignition::math::Quaterniond(0, 0, 0));
            key = anim->CreateKeyFrame(15);
            key->Translation(ignition::math::Vector3d(7.5, 0.0, 0.0));
            key->Rotation(ignition::math::Quaterniond(0, 0, 0));
            key = anim->CreateKeyFrame(20);
            key->Translation(ignition::math::Vector3d(10.0, 0.0, 0.0));
            key->Rotation(ignition::math::Quaterniond(0, 0, 0));
            key = anim->CreateKeyFrame(25);
            key->Translation(ignition::math::Vector3d(12.5, 0.0, 0.0));
            key->Rotation(ignition::math::Quaterniond(0, 0, 0));
            key = anim->CreateKeyFrame(30);
            key->Translation(ignition::math::Vector3d(15.0, 0.0, 0.0));
            key->Rotation(ignition::math::Quaterniond(0, 0, 0));
            key = anim->CreateKeyFrame(35);
            key->Translation(ignition::math::Vector3d(18.0, 0.0, 0.0));
            key->Rotation(ignition::math::Quaterniond(0, 0, 0));
            key = anim->CreateKeyFrame(40);
            key->Translation(ignition::math::Vector3d(13.0, 0.0, 0.0));
            key->Rotation(ignition::math::Quaterniond(0, 0, 0));
            key = anim->CreateKeyFrame(45);
            key->Translation(ignition::math::Vector3d(8.0, 0.0, 0.0));
            key->Rotation(ignition::math::Quaterniond(0, 0, 0));
            key = anim->CreateKeyFrame(50);
            key->Translation(ignition::math::Vector3d(3.0, 0.0, 0.0));
            key->Rotation(ignition::math::Quaterniond(0, 0, 0));
            key = anim->CreateKeyFrame(55);
            key->Translation(ignition::math::Vector3d(0.0, 0.0, 0.0));
            key->Rotation(ignition::math::Quaterniond(0, 0, 0));
            key = anim->CreateKeyFrame(60);
            key->Translation(ignition::math::Vector3d(3.0, 0.0, 0.0));
            key->Rotation(ignition::math::Quaterniond(0, 0, 0));
            key = anim->CreateKeyFrame(65);
            key->Translation(ignition::math::Vector3d(8.0, 0.0, 0.0));
            key->Rotation(ignition::math::Quaterniond(0, 0, 0));
            key = anim->CreateKeyFrame(70);
            key->Translation(ignition::math::Vector3d(13.0, 0.0, 0.0));
            key->Rotation(ignition::math::Quaterniond(0, 0, 0));
            key = anim->CreateKeyFrame(75);
            key->Translation(ignition::math::Vector3d(18.0, 0.0, 0.0));
            key->Rotation(ignition::math::Quaterniond(0, 0, 0));
            key = anim->CreateKeyFrame(80);
            key->Translation(ignition::math::Vector3d(15.5, 0.0, 0.0));
            key->Rotation(ignition::math::Quaterniond(0, 0, 0));
            key = anim->CreateKeyFrame(85);
            key->Translation(ignition::math::Vector3d(13.0, 0.0, 0.0));
            key->Rotation(ignition::math::Quaterniond(0, 0, 0));
            key = anim->CreateKeyFrame(90);
            key->Translation(ignition::math::Vector3d(10.5, 0.0, 0.0));
            key->Rotation(ignition::math::Quaterniond(0, 0, 0));
            key = anim->CreateKeyFrame(95);
            key->Translation(ignition::math::Vector3d(8.0, 0.0, 0.0));
            key->Rotation(ignition::math::Quaterniond(0, 0, 0));
            key = anim->CreateKeyFrame(100);
            key->Translation(ignition::math::Vector3d(5.5, 0.0, 0.0));
            key->Rotation(ignition::math::Quaterniond(0, 0, 0));
            key = anim->CreateKeyFrame(105);
            key->Translation(ignition::math::Vector3d(3.0, 0.0, 0.0));
            key->Rotation(ignition::math::Quaterniond(0, 0, 0));
            key = anim->CreateKeyFrame(110);
            key->Translation(ignition::math::Vector3d(0.0, 0.0, 0.0));
            key->Rotation(ignition::math::Quaterniond(0, 0, 0));
            /*
            for (int i = 1; i <= 200; i++)
            {
                key = anim->CreateKeyFrame(i*5);
                key->Translation(ignition::math::Vector3d((dis(gen)) / 10, 0, 0.0));
                key->Rotation(ignition::math::Quaterniond(0, 0, 0));
            }*/
            // set the animation
            _parent->SetAnimation(anim);
        }

        // Pointer to the model

    private:
        physics::ModelPtr model;

        // Pointer to the update event connection

    private:
        event::ConnectionPtr updateConnection;
    };
    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(Obstacle2)
}  // namespace gazebo
