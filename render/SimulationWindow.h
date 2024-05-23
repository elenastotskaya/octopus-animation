// Modified by Elena Stotskaya
// Added obstacle processing and eye axis calculation

#ifndef __SIMULATION_WINDOW_H__
#define __SIMULATION_WINDOW_H__
#include "GLWindow.h"
#include "DrawFunctions.h"
#include "DrawPrimitives.h"
#include "Environment.h"
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
namespace p = boost::python;
namespace np = boost::python::numpy;
class SimulationWindow : public GLWindow
{
public:
	SimulationWindow();
	SimulationWindow(std::string network_name);

protected:
	void Display() override;
	void Keyboard(unsigned char key,int x,int y) override;
	void Mouse(int button, int state, int x, int y) override;
	void Motion(int x, int y) override;
	void Reshape(int w, int h) override;
	void Timer(int value) override;
	void initLights();

	Eigen::VectorXd GetActionFromNN(const Eigen::VectorXd& state);

protected:
	bool 				mPlay;
	int 				mCurFrame;
	int 				mTotalFrame;
	double 				mElapsedTime;

	Environment* 		mEnvironment;

	Eigen::VectorXd		mActions;

	p::object mm,mns,sys_module,env_module;
	bool mIsNNLoaded;

private:
	static constexpr int NUM_AXIS_SAMPLES = 4;
	int mAxisBlock1[NUM_AXIS_SAMPLES] = {29, 31, 39, 56};
	int mAxisBlock2[NUM_AXIS_SAMPLES] = {96, 94, 87, 86};
};
#endif