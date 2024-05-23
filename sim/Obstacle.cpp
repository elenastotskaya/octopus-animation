// Added by Elena Stotskaya

#include "Obstacle.h"
#include <iostream>

Obstacle::Obstacle(const std::string& obj_file,const Eigen::Affine3d& T)
{
    mMesh = new FEM::OBJMesh(obj_file, T);
    const auto& vertices = mMesh->GetVertices();
    mColliderBounds.push_back(vertices[0](0));
    mColliderBounds.push_back(vertices[0](0));
    mColliderBounds.push_back(vertices[0](1));
    mColliderBounds.push_back(vertices[0](1));
    mColliderBounds.push_back(vertices[0](2));
    mColliderBounds.push_back(vertices[0](2));

    for (auto vertex : vertices)
    {
        if (vertex(0) < mColliderBounds[0])
        {
            mColliderBounds[0] = vertex(0);
        }
        else if (vertex(0) > mColliderBounds[1])
        {
            mColliderBounds[1] = vertex(0);
        }
        if (vertex(1) < mColliderBounds[2])
        {
            mColliderBounds[2] = vertex(1);
        }
        else if (vertex(1) > mColliderBounds[3])
        {
            mColliderBounds[3] = vertex(1);
        }
        if (vertex(2) < mColliderBounds[4])
        {
            mColliderBounds[4] = vertex(2);
        }
        else if (vertex(2) > mColliderBounds[5])
        {
            mColliderBounds[5] = vertex(2);
        }
    }
}