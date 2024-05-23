// Modified by Elena Stotskaya
// Added controls, loading obstacles, and additional learning parameters

#ifndef __ENVIRONMENT_H__
#define __ENVIRONMENT_H__
#include <deque>
#include "fem/World.h"
#include "Octopus.h"
#include "Normalizer.h"
#include "Obstacle.h"

enum TurnDirection
{
	Left,
	Right,
	Up,
	Down
};

class Octopus;
class Normalizer;
class Environment
{
public:
	
	Environment();
	
	FEM::World* GetSoftWorld(){return mSoftWorld;};
	Octopus* GetOctopus(){return mOctopus;};

	double GetSimulationHz(){return mSimulationHz;};
	double GetControlHz(){return mControlHz;};

	void Step();
	void Reset();

	const Eigen::VectorXd& GetStates();

	void InitializeActions();
	const Eigen::VectorXd& GetActions() {return mActions;};
	void SetActions(const Eigen::VectorXd& actions);

	std::map<std::string,double> GetRewards();

	bool isEndOfEpisode();

	Eigen::VectorXd GetNormLowerBound() {return mNormLowerBound;};
	Eigen::VectorXd GetNormUpperBound() {return mNormUpperBound;};
	Normalizer*	GetNormalizer() {return mNormalizer;};

	void SetPhase(const int& phase); 
	const int& GetPhase() {return mPhase;};

	Eigen::Vector3d GetAverageVelocity() {return mAverageVelocity;};
	Eigen::Vector3d GetTargetVelocity() {return mTargetVelocity;};
	void UpdateRandomTargetVelocity();
	void MoveForward();
	void StopMovement();
	void Turn(TurnDirection direction);
	void SetTargetVelocity(Eigen::Vector3d velocity) {mTargetVelocity = velocity;}

	void LoadObstacles(const std::string& list_file);
	const std::vector<Obstacle>& GetObstacles() {return mObstacles;}
	const std::vector<double>& GetColliderBounds() {return mColliderBounds;}
	int GetEyeIndex1() {return mEyeIndex1;}
	int GetEyeIndex2() {return mEyeIndex2;}

private:
	FEM::World*						mSoftWorld;
	Octopus*						mOctopus;

	int 							mSimulationHz;
	int 							mControlHz;

	Normalizer*						mNormalizer;
	Eigen::VectorXd 				mNormLowerBound;
	Eigen::VectorXd 				mNormUpperBound;

	Eigen::VectorXd					mStates;
	Eigen::VectorXd					mActions;

	int 							mPhase;

	Eigen::Vector3d					mTargetVelocity;
	Eigen::Vector3d					mAverageVelocity;
	std::deque<Eigen::Vector3d>		mAverageVelocityDeque;
	Eigen::Vector3d					mCurrentDirection;
	Eigen::Vector3d					mLocalUp;
	Eigen::Vector3d					mLocalRight;
	double mSpeed = 1.0;
	double mTurnAngle = 0.1;
	int mEyeIndex1 = 29;
	int mEyeIndex2 = 96;
	//int mEyeIndex1 = 28;
	//int mEyeIndex2 = 95;
	int mUpVectorIndex1 = 62;
	int mUpVectorIndex2 = 76;

	std::vector<Obstacle> mObstacles;
	std::vector<double> mColliderBounds;

	Eigen::Vector3d mPushbackCoeff;
	int mCollisionFlag;
	int mCollisionPrev[2];

	static constexpr int NUM_AXIS_SAMPLES = 4;
	static constexpr int mAxisBlock1[NUM_AXIS_SAMPLES] = {29, 31, 39, 56};
	static constexpr int mAxisBlock2[NUM_AXIS_SAMPLES] = {96, 94, 87, 86};
	static constexpr double REFLECTION_ALPHA = 0.5;
};
#endif
