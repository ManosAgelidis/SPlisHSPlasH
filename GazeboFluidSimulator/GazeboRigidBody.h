#ifndef __GazeboRigidBody_h__
#define __GazeboRigidBody_h__

#include "SPlisHSPlasH/Common.h"
#include "SPlisHSPlasH/RigidBodyObject.h"
#include "SPlisHSPlasH/TriangleMesh.h"
#include "gazebo/gazebo.hh"
#include "gazebo/util/system.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/physics/PhysicsTypes.hh"

namespace SPH
{
    /** \brief This class stores the information of a static rigid body which 
	* is not part of a rigid body simulation. 
	*/
    class GazeboRigidBody : public RigidBodyObject
    {
    private:
        gazebo::physics::ModelPtr gazeboModel;

    protected:
        Vector3r m_x;
        Vector3r m_x_world;
        Vector3r m_zero;
        Matrix3r m_R;
        Matrix3r m_R_world;
        TriangleMesh m_geometry;
        bool m_isDynamic;

    public:
        GazeboRigidBody() { m_zero = Vector3r::Zero(); }

        virtual bool isDynamic() const { return m_isDynamic; }
        virtual void setDynamic(bool dynamic) { m_isDynamic = dynamic; }

        virtual Real const getMass() const { return 0.0; }
        virtual Vector3r const &getPosition() const { return m_x; }
        virtual void setPosition(const Vector3r &x) { m_x = x; }
        virtual Vector3r getWorldSpacePosition() const { return m_x_world; }
        virtual Vector3r const &getVelocity() const { return m_zero; }
        virtual void setVelocity(const Vector3r &v) {}
        virtual Matrix3r const &getRotation() const { return m_R; }
        virtual void setRotation(const Matrix3r &r) { m_R = r; }
        virtual Matrix3r getWorldSpaceRotation() const { return m_R_world; }
        virtual Vector3r const &getAngularVelocity() const { return m_zero; }
        virtual void setAngularVelocity(const Vector3r &v) {}
        virtual void addForce(const Vector3r &f) {}
        virtual void addTorque(const Vector3r &t) {}

        void setGazeboCollision(const gazebo::physics::ModelPtr &model) { gazeboModel = model; }
        void setWorldSpacePosition(const Vector3r &x)
        {
            /*  if (model->GetLinks().size())
                model->SetLinkWorldPose(gazebo::math::Pose(gazebo::math::Vector3(x[0], x[1], x[2]),model->GetLinks()[0]->G, model->GetLinks()[0]);
            */
        }
        void setWorldSpaceRotation(const Matrix3r &r) { m_R_world = r; }
        TriangleMesh &getGeometry() { return m_geometry; }

        virtual const std::vector<Vector3r> &getVertices() const {  return m_geometry.getVertices(); };
        virtual const std::vector<Vector3r> &getVertexNormals() const { return m_geometry.getVertexNormals(); };
        virtual const std::vector<unsigned int> &getFaces() const { return m_geometry.getFaces(); };
    };
}; // namespace SPH

#endif