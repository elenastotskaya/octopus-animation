// Added by Elena Stotskaya

#ifndef __OBSTACLE_H__
#define __OBSTACLE_H__
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "fem/Mesh/MeshHeaders.h"

class Obstacle
{
public:
    Obstacle(const std::string& obj_file,const Eigen::Affine3d& T = Eigen::Affine3d::Identity());
    //~Obstacle() {delete mMesh;}
    const std::vector<double>& GetColliderBounds() {return mColliderBounds;}
    FEM::OBJMesh* GetMesh() {return mMesh;}

private:
    std::vector<double> mColliderBounds;
    FEM::OBJMesh* mMesh;
};

#endif