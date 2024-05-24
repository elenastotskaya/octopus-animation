// Modified by Elena Stotskaya
// Added controls, loading obstacles, and additional learning parameters

#include "Environment.h"
#include <chrono>
#include <random>
#include <fstream>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

#define LOCAL   0
#define GLOBAL  1
unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
std::default_random_engine generator(seed);
std::uniform_real_distribution<double> l_distribution(0.75,0.75);
std::uniform_real_distribution<double> angle_distribution(0.0,0.0);
std::uniform_real_distribution<double> vx_distribution(-1.0,1.0);
std::uniform_real_distribution<double> vy_distribution(-1.0,1.0);
std::uniform_real_distribution<double> vz_distribution(-1.0,1.0);
Environment::
Environment()
    :mPhase(0)
{
	mSimulationHz = 240;
    mControlHz = 30;

    mSoftWorld = new FEM::World(
        1.0/mSimulationHz,
        50,
        0.9999,
        false
    );

    mOctopus = new Octopus(5E5,2E5,0.4);
    mOctopus->SetOctopus(std::string(SOFTCON_DIR)+"/data/octopus.meta");
    mOctopus->Initialize(mSoftWorld);
    mSoftWorld->Initialize();

    mOctopus->SetInitReferenceRotation(mSoftWorld->GetPositions());

    LoadObstacles(std::string(SOFTCON_DIR)+"/data/meshes/mesh_list.txt");

    InitializeActions();

    mTargetVelocity = { 0.0, 1.0, 0.0 };

    mAverageVelocity.setZero();
    mAverageVelocityDeque.clear();

    mPushbackCoeff.setZero();

    mColliderBounds.resize(6);
    const Eigen::VectorXd& x = mSoftWorld->GetPositions();
	mColliderBounds[0] = x[0];
    mColliderBounds[1] = x[0];
    mColliderBounds[2] = x[1];
    mColliderBounds[3] = x[1];
    mColliderBounds[4] = x[2];
    mColliderBounds[5] = x[2];

    for (int i = 3; i < x.size(); ++i)
    {
        if (!(i % 3))
        {
            if (x[i] < mColliderBounds[0])
            {
                mColliderBounds[0] = x[i];
            }
            else if (x[i] > mColliderBounds[1])
            {
                mColliderBounds[1] = x[i];
            }
        }
        else if (i % 3 == 1)
        {
            if (x[i] < mColliderBounds[2])
            {
                mColliderBounds[2] = x[i];
            }
            else if (x[i] > mColliderBounds[3])
            {
                mColliderBounds[3] = x[i];
            }
        }
        else
        {
            if (x[i] < mColliderBounds[4])
            {
                mColliderBounds[4] = x[i];
            }
            else if (x[i] > mColliderBounds[5])
            {
                mColliderBounds[5] = x[i];
            }
        }
    }

    mCollisionFlag = -1;
    mCollisionPrev[0] = -1;
    mCollisionPrev[1] = -1;
}
void
Environment::
Step()
{
    Eigen::VectorXd external_force = mOctopus->ComputeDragForces(mSoftWorld);
    mSoftWorld->SetExternalForce(external_force);
    //mSoftWorld->AddPushbackForce(mPushbackCoeff);

    mSoftWorld->TimeStepping();

    Eigen::Vector3d v_front = Eigen::Vector3d(0,0,0);
    if(mAverageVelocityDeque.size() > mSimulationHz) {
        v_front = mAverageVelocityDeque.front();
        mAverageVelocityDeque.pop_front();
    }
    Eigen::Vector3d v_center = mSoftWorld->GetVelocities().block<3,1>(3*mOctopus->GetCenterIndex(),0);
    mAverageVelocityDeque.push_back(v_center);
    mAverageVelocity = mAverageVelocity - (v_front)/mSimulationHz + v_center/mSimulationHz;
    
    mCurrentDirection = mOctopus->GetForwardVector(mSoftWorld->GetPositions()).normalized();

    const Eigen::VectorXd& x = mSoftWorld->GetPositions();
	mColliderBounds[0] = x[0];
    mColliderBounds[1] = x[0];
    mColliderBounds[2] = x[1];
    mColliderBounds[3] = x[1];
    mColliderBounds[4] = x[2];
    mColliderBounds[5] = x[2];

    for (int i = 3; i < x.size(); ++i)
    {
        if (!(i % 3))
        {
            if (x[i] < mColliderBounds[0])
            {
                mColliderBounds[0] = x[i];
            }
            else if (x[i] > mColliderBounds[1])
            {
                mColliderBounds[1] = x[i];
            }
        }
        else if (i % 3 == 1)
        {
            if (x[i] < mColliderBounds[2])
            {
                mColliderBounds[2] = x[i];
            }
            else if (x[i] > mColliderBounds[3])
            {
                mColliderBounds[3] = x[i];
            }
        }
        else
        {
            if (x[i] < mColliderBounds[4])
            {
                mColliderBounds[4] = x[i];
            }
            else if (x[i] > mColliderBounds[5])
            {
                mColliderBounds[5] = x[i];
            }
        }
    }

    mLocalUp = (x.block<3,1>(3*mUpVectorIndex1,0) - x.block<3,1>(3*mUpVectorIndex2,0)).normalized();
    mLocalRight = (x.block<3,1>(3*mEyeIndex1,0) - x.block<3,1>(3*mEyeIndex2,0)).normalized();
    static unsigned count = 0;

    const std::vector<Eigen::Vector3i>& triangles = mOctopus->GetContours();
    Eigen::VectorXd x_temp = x;
    for (int i = 0; i < mObstacles.size(); ++i)
    {
        const std::vector<double>& obstacle_bounds = mObstacles[i].GetColliderBounds();
        if (((mColliderBounds[0] - obstacle_bounds[1]) * (obstacle_bounds[0] - mColliderBounds[1]) > 0) && ((mColliderBounds[2] - obstacle_bounds[3]) * (obstacle_bounds[2] - mColliderBounds[3]) > 0) && ((mColliderBounds[4] - obstacle_bounds[5]) * (obstacle_bounds[4] - mColliderBounds[5]) > 0))
        {
            Eigen::Vector3d pushback_dist;
            pushback_dist.setZero();
            Eigen::Vector3d ob_normal;
            const Eigen::VectorXd& v = mSoftWorld->GetVelocities();
            Eigen::VectorXd v_temp = v;
            double diff_x = std::min(mColliderBounds[1] - obstacle_bounds[0], obstacle_bounds[1] - mColliderBounds[0]);
            double diff_y = std::min(mColliderBounds[3] - obstacle_bounds[2], obstacle_bounds[3] - mColliderBounds[2]);
            double diff_z = std::min(mColliderBounds[5] - obstacle_bounds[4], obstacle_bounds[5] - mColliderBounds[4]);
            if (diff_x < diff_y && diff_x < diff_z)
            {
                    if (mColliderBounds[1] - obstacle_bounds[0] < obstacle_bounds[1] - mColliderBounds[0])
                    {
                        diff_x = -diff_x;
                        ob_normal = Eigen::Vector3d(-1.0, 0.0, 0.0);
                        mPushbackCoeff(0) -= REFLECTION_ALPHA;
                    }
                    else
                    {
                        ob_normal = Eigen::Vector3d(1.0, 0.0, 0.0);
                        mPushbackCoeff(0) += REFLECTION_ALPHA;
                   
                    }
                    mCollisionFlag = 0;
                    mCollisionPrev[0] = i;
                    mCollisionPrev[1] = mCollisionFlag;
                pushback_dist(0) = diff_x;
            }
            else if (diff_y < diff_x && diff_y < diff_z)
            {
                    if (mColliderBounds[3] - obstacle_bounds[2] < obstacle_bounds[3] - mColliderBounds[2])
                    {
                        diff_y = -diff_y;
                        ob_normal = Eigen::Vector3d(0.0, -1.0, 0.0);
                        mPushbackCoeff(1) -= REFLECTION_ALPHA;
                    }
                    else
                    {
                        ob_normal = Eigen::Vector3d(0.0, 1.0, 0.0);
                        mPushbackCoeff(1) += REFLECTION_ALPHA;
                    }
                    mCollisionFlag = 1;
                    mCollisionPrev[0] = i;
                    mCollisionPrev[1] = mCollisionFlag;
                pushback_dist(1) = diff_y;
            }
            else
            {
                    if (mColliderBounds[5] - obstacle_bounds[4] < obstacle_bounds[5])
                    {
                        diff_z = -diff_z;
                        ob_normal = Eigen::Vector3d(0.0, 0.0, -1.0);
                        mPushbackCoeff(2) -= REFLECTION_ALPHA;
                    }
                    else
                    {
                        ob_normal = Eigen::Vector3d(0.0, 0.0, 1.0);
                        mPushbackCoeff(2) += REFLECTION_ALPHA;
                    }
                    mCollisionFlag = 2;
                    mCollisionPrev[0] = i;
                    mCollisionPrev[1] = mCollisionFlag;
                pushback_dist(2) = diff_z;
            }

            for (int j = 3; j < x.size(); ++j)
            {
                x_temp(j) += pushback_dist(j % 3);
            }
            mSoftWorld->SetPositions(x_temp);
            mColliderBounds[0] = x_temp[0];
            mColliderBounds[1] = x_temp[0];
            mColliderBounds[2] = x_temp[1];
            mColliderBounds[3] = x_temp[1];
            mColliderBounds[4] = x_temp[2];
            mColliderBounds[5] = x_temp[2];

            for (int j = 3; j < x_temp.size(); ++j)
            {
                if (!(j % 3))
                {
                    if (x_temp[j] < mColliderBounds[0])
                    {
                        mColliderBounds[0] = x_temp[j];
                    }
                    else if (x_temp[j] > mColliderBounds[1])
                    {
                        mColliderBounds[1] = x_temp[j];
                    }
                }
                else if (j % 3 == 1)
                {
                    if (x_temp[j] < mColliderBounds[2])
                    {
                        mColliderBounds[2] = x_temp[j];
                    }
                    else if (x_temp[j] > mColliderBounds[3])
                    {
                        mColliderBounds[3] = x_temp[j];
                    }
                }
                else
                {
                    if (x_temp[j] < mColliderBounds[4])
                    {
                        mColliderBounds[4] = x_temp[j];
                    }
                    else if (x_temp[j] > mColliderBounds[5])
                    {
                        mColliderBounds[5] = x_temp[j];
                    }
                }
            }
        }
    }
}
void 
Environment::
SetPhase(const int& phase)
{
    mPhase = phase;
}
void 
Environment::
Reset()
{
    mAverageVelocityDeque.clear();
    mAverageVelocity.setZero();
    for(int i=0;i<mOctopus->GetMuscles().size();i++)
    {
        mOctopus->GetMuscles()[i]->Reset();
    }
    mSoftWorld->Reset();
    mPhase = 0;
}




// DeepRL
const Eigen::VectorXd&
Environment::
GetStates()
{
    const Eigen::VectorXd& x = mSoftWorld->GetPositions();
    const Eigen::VectorXd& v = mSoftWorld->GetVelocities();
    Eigen::Vector3d center_position = x.block<3,1>(3*mOctopus->GetCenterIndex(),0);
    
    Eigen::Matrix3d R = mOctopus->GetReferenceRotation(LOCAL,x);

    Eigen::Vector3d local_target_velocity = R*mTargetVelocity;
    Eigen::Vector3d local_average_velocity = R*mAverageVelocity;

    const std::vector<int>& sampling_index = mOctopus->GetSamplingIndex();
    Eigen::VectorXd local_position(3*sampling_index.size());
    Eigen::VectorXd local_velocity(3*sampling_index.size());

    for(int i=0; i<sampling_index.size(); i++) {
        local_position.block<3,1>(3*i,0) = R*x.block<3,1>(3*sampling_index[i],0);
        local_velocity.block<3,1>(3*i,0) = R*v.block<3,1>(3*sampling_index[i],0);
    }

    int num_states = local_target_velocity.size() + local_average_velocity.size()
        + local_position.size() + local_velocity.size();

    mStates.resize(num_states);
    mStates.setZero();

    mStates<<local_target_velocity,local_average_velocity,
        local_position,local_velocity;

    return mStates;
}
void
Environment::
InitializeActions()
{
    const auto& muscles = mOctopus->GetMuscles();

    int num_action =4*muscles.size();
    mActions.resize(num_action);

    Eigen::VectorXd real_lower_bound(num_action);
    Eigen::VectorXd real_upper_bound(num_action);

    int cnt =0;
    for(const auto& m : muscles) 
    {
        real_lower_bound.segment(cnt,4) = m->GetActionLowerBound();
        real_upper_bound.segment(cnt,4) = m->GetActionUpperBound();
        cnt+=4;
    }

    mNormLowerBound.resize(real_lower_bound.size());
    mNormUpperBound.resize(real_upper_bound.size());

    mNormLowerBound.setOnes();
    mNormLowerBound *= -5.0;
    mNormUpperBound.setOnes();
    mNormUpperBound *= 5.0;

    mNormalizer = new Normalizer(real_upper_bound,real_lower_bound,
        mNormUpperBound,mNormLowerBound);
}
void 
Environment::
SetActions(const Eigen::VectorXd& actions)
{
    Eigen::VectorXd real_actions = mNormalizer->NormToReal(actions);
    mOctopus->SetActivationLevels(real_actions,mPhase);
    mPhase+=1;
}
std::map<std::string,double>
Environment::
GetRewards()
{
    std::map<std::string,double> reward_map;

    double d = (mAverageVelocity-mTargetVelocity).norm();
    double reward_target = exp(-20.0*d*d);

    Eigen::Vector3d v_face = mOctopus->GetForwardVector(mSoftWorld->GetPositions());
    Eigen::Vector3d v_tar_dir = mTargetVelocity.normalized();
    auto d_direction = (v_face - v_tar_dir).norm();
    double reward_direction = exp(-fabs(d_direction)*10.0);

    const Eigen::VectorXd& x = mSoftWorld->GetPositions();
    Eigen::Vector3d eye_position_1;
    Eigen::Vector3d eye_position_2;
    double eye_axis = 0.0;
    for (int i = 0; i < NUM_AXIS_SAMPLES; ++i)
    {
        eye_position_1 = x.block<3,1>(3*mAxisBlock1[i],0);
        eye_position_2 = x.block<3,1>(3*mAxisBlock2[i],0);
        eye_axis += (eye_position_1 - eye_position_2)[1];
    }
    double reward_vertical = exp(-5000.0*eye_axis*eye_axis);

    double reward_twisting = 0.7;
    const Eigen::VectorXd& v = mSoftWorld->GetVelocities();
    Eigen::Vector3d eye_velocity_1 = v.block<3,1>(3*mEyeIndex1,0);
    Eigen::Vector3d eye_velocity_2 = v.block<3,1>(3*mEyeIndex2,0);
    for (int i = 0; i < 3; ++i)
    {
        if (std::signbit(eye_velocity_1[i]) != std::signbit(eye_velocity_2[i]))
        {
            reward_twisting = -3.0;
        }
    }

    double w_target = 1.0;
    double w_direction = 2.0;
    double w_vertical = 2.0;
    double w_twisting = 1.0;

    double reward = w_target*reward_target + w_direction*reward_direction + w_vertical*reward_vertical + w_twisting*reward_twisting;

    reward_map["vertical"] = w_vertical*reward_vertical;
    reward_map["target"] = w_target*reward_target;
    reward_map["direction"] = w_direction*reward_direction;
    reward_map["twisting"] = w_twisting*reward_twisting;
    reward_map["total"] = reward;

    return reward_map;
}
bool
Environment::
isEndOfEpisode()
{
    bool eoe =false; 

    return eoe;
}
void
Environment::
UpdateRandomTargetVelocity()
{
    Eigen::Vector3d v,axis;

    double length = l_distribution(generator);

    v = length*mOctopus->GetForwardVector(mSoftWorld->GetPositions()).normalized();

    double angle = angle_distribution(generator);
    axis[0] = vx_distribution(generator);
    axis[1] = vy_distribution(generator);
    axis[2] = vz_distribution(generator);

    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(angle,axis.normalized());

    mTargetVelocity = R*v;
}

void Environment::MoveForward()
{
    mTargetVelocity = mSpeed * mCurrentDirection;
}

void Environment::StopMovement()
{
    mTargetVelocity.setZero();
}

void Environment::Turn(TurnDirection direction)
{
    Eigen::Matrix3d rotation;
    switch (direction)
    {
        case TurnDirection::Left:
        {
            rotation = Eigen::AngleAxisd(mTurnAngle, mLocalUp);
            //rotation = Eigen::AngleAxisd(-mTurnAngle, Eigen::Vector3d{0.0, 1.0, 0.0});
            break;
        }
        case TurnDirection::Right:
        {
            rotation = Eigen::AngleAxisd(-mTurnAngle, mLocalUp);
            //rotation = Eigen::AngleAxisd(mTurnAngle, Eigen::Vector3d{0.0, 1.0, 0.0});
            break;
        }
         case TurnDirection::Up:
        {
            //rotation = Eigen::AngleAxisd(-mTurnAngle, mLocalUp.cross(mOctopus->GetForwardVector(mSoftWorld->GetPositions()).normalized()));
            //rotation = Eigen::AngleAxisd(-mTurnAngle, Eigen::Vector3d{1.0, 0.0, 0.0});
            rotation = Eigen::AngleAxisd(-mTurnAngle, mLocalRight);
            break;
        }
        case TurnDirection::Down:
        {
            //rotation = Eigen::AngleAxisd(mTurnAngle, mLocalUp.cross(mOctopus->GetForwardVector(mSoftWorld->GetPositions()).normalized()));
            //rotation = Eigen::AngleAxisd(mTurnAngle, Eigen::Vector3d{1.0, 0.0, 0.0});
            rotation = Eigen::AngleAxisd(mTurnAngle, mLocalRight);
            break;
        }
    }
    mTargetVelocity = rotation * mTargetVelocity;
}

void Environment::LoadObstacles(const std::string& list_file)
{
    std::ifstream ifs(list_file);
	if(!(ifs.is_open()))
	{
		std::cout<<"Can't read file "<<list_file<<std::endl;
		return;
	}
	std::string file_name;
    std::string transform;

    while(!ifs.eof())
	{
		std::getline(ifs,file_name);
        std::getline(ifs,transform);

        if (file_name.empty())
        {
            break;
        }

        std::stringstream ss(transform);
        char marker;
        double element;
        Eigen::Affine3d transform_matrix;

        Eigen::Affine3d in_translation = Eigen::Affine3d::Identity();
        Eigen::Affine3d in_rotation = Eigen::Affine3d::Identity();
        Eigen::Affine3d in_scale = Eigen::Affine3d::Identity();
        
        while (ss >> marker)
        {
            if (marker == 't')
            {
                Eigen::Vector3d translation_vector;
                for (int i = 0; i < 3; ++i)
                {
                    if (ss >> element)
                    {
                        translation_vector(i) = element;
                    }
                }
                in_translation = Eigen::Translation3d(translation_vector);
            }
            else if (marker == 'r')
            {
                double rotation_angle;
                if (!(ss >> rotation_angle))
                {
                    break;
                }
                Eigen::Vector3d rotation_axis;
                for (int i = 0; i < 3; ++i)
                {
                    if (ss >> element)
                    {
                        rotation_axis(i) = element;
                    }
                }
                if (rotation_axis == Eigen::Vector3d(0.0, 0.0, 0.0))
                {
                    break;
                }
                in_rotation = Eigen::AngleAxisd(rotation_angle, rotation_axis);
            }
            else if (marker == 's')
            {
                Eigen::Vector3d scale_vector;
                for (int i = 0; i < 3; ++i)
                {
                    if (ss >> element)
                    {
                        scale_vector(i) = element;
                    }
                }
                if (scale_vector(0) <= 0.0 || scale_vector(1) <= 0.0 || scale_vector(2) <= 0.0)
                {
                    break;
                }
                in_scale = Eigen::Scaling(scale_vector);
            }
        }
        
        transform_matrix = in_translation * in_rotation * in_scale;

        mObstacles.push_back(Obstacle(std::string(SOFTCON_DIR)+"/data/meshes/"+file_name, transform_matrix));
        file_name.clear();
        transform.clear();
    }
}
