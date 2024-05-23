#include "SimulationWindow.h"
#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#endif
namespace p = boost::python;
namespace np = boost::python::numpy;
int main(int argc,char** argv)
{
	/////////////////////////////////
	omp_set_num_threads(20);
    std::cout << "Max threads: " << omp_get_max_threads() << "\n";

    int id;
#pragma omp parallel private(id)
    {
        std::cout << "Number of threads running in parallel: " << omp_get_num_threads() << "\n";
        std::cout << "Thread ID in parallel region: " << omp_get_thread_num() << "\n";
    }
	/////////////////////

	Py_Initialize();
	np::initialize();
	if( argc < 2 ) {
		SimulationWindow* simwindow = new SimulationWindow();
		glutInit(&argc, argv);
		simwindow->InitWindow(1920,1080,"OctoCon");
		glutMainLoop();
	} else {
		SimulationWindow* simwindow = new SimulationWindow(std::string(argv[1]));
		glutInit(&argc, argv);

		simwindow->InitWindow(1920,1080,"OctoCon");
		glutMainLoop();
	}
}
