//
// Copyright (c) 2008-2018 the Urho3D project.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
#pragma once

#include "../Scene/LogicComponent.h"

class btSoftBody;
class btTransform;

namespace Urho3D
{
class PhysicsWorld;
class BoundingBox;
class Model;

/// SoftBody component.
class URHO3D_API SoftBody : public LogicComponent
{
    URHO3D_OBJECT(SoftBody, LogicComponent);

public:
    /// Construct.
    SoftBody(Context* context);
    /// Destruct. Free the rigid body and geometries.
    virtual ~SoftBody();
    /// Register object factory.
    static void RegisterObject(Context* context);

    /// Called before the first update. At this point all other components of the node should exist. Will also be called if update events are not wanted; in that case the event is immediately unsubscribed afterward.
    virtual void DelayedStart();
    /// Called on physics post-update, fixed timestep.
    virtual void FixedPostUpdate(float timeStep);

    bool CreateFromStaticModel();

    /// Set collision layer.
    void SetCollisionLayer(unsigned layer);
    /// Set collision mask.
    void SetCollisionMask(unsigned mask);
    /// Set collision group and mask.
    void SetCollisionLayerAndMask(unsigned layer, unsigned mask);
    /// Set mass. Zero mass makes the body static.
    void SetMass(float mass);
    /// Set rigid body position in world space.
    void SetPosition(const Vector3& position);
    /// Set rigid body rotation in world space.
    void SetRotation(const Quaternion& rotation);
    /// Set rigid body position and rotation in world space as an atomic operation.
    void SetTransform(const Vector3& position, const Quaternion& rotation);
    void SetScale(const Vector3& scale);
    void SetVelocity(const Vector3& velocity);

    /// Return Bullet soft body.
    btSoftBody* GetBody() const { return body_.Get(); }
    /// Return mass.
    float GetMass() const { return mass_; }
    /// Return rigid body position in world space.
    Vector3 GetPosition() const;
    /// Return rigid body rotation in world space.
    Quaternion GetRotation() const;

    /// Activate.
    void Activate();
    bool IsActive() const;
    /// Remove the rigid body.
    void ReleaseBody();
    void UpdateMass();

    void SetConfigLST(float lst);
    void SetConfigMT(float mt);
    void SetConfigVC(float vc);
    void SetConfigPR(float pr);

    float GetConfigLST() const   { return configLST_; }
    float GetConfigMT() const    { return configMT_; }
    float GetConfigVC() const    { return configVC_; }
    float GetConfigPR() const    { return configPR_; }

    void SetDeactivationVelocity(float deactiveVel) { deactivationVelocity_ = deactiveVel; }
    float GetDeactivationVelocity() const           { return deactivationVelocity_; }

    void SetFaceNormals(bool setFaceNormals)        { setToFaceNormals_ = setFaceNormals; }
    bool GetFaceNormals() const                     { return setToFaceNormals_; }

    static SharedPtr<Model> CreateModelFromBulletMesh(Context *context, float *varray, int numVertices, int *iarray, int numTriangles);

protected:
    bool CreateFromModel(Model *model);
    void SetToFaceNormals(Model *model);
    void UpdateVertexBuffer(Model *model);
    /// Handle node transform being dirtied.
    virtual void OnMarkedDirty(Node* node);
    /// Handle node being assigned.
    virtual void OnNodeSet(Node* node);
    /// Handle scene being assigned.
    virtual void OnSceneSet(Scene* scene);
    /// Create the rigid body, or re-add to the physics world with changed flags. Calls UpdateMass().
    void AddBodyToWorld();
    /// Remove the rigid body from the physics world.
    void RemoveBodyFromWorld();

    void SetDefaultConfiguration();
    SharedPtr<Model> PruneModel(Model *model);
    void CheckRestCondition();

protected:
    /// Bullet soft body.
    UniquePtr<btSoftBody> body_;
    WeakPtr<PhysicsWorld> physicsWorld_;

    /// Gravity override vector.
    Vector3 gravityOverride_;
    /// Center of mass offset.
    Vector3 centerOfMass_;
    /// Mass.
    float mass_;
    /// Collision layer.
    unsigned collisionLayer_;
    /// Collision mask.
    unsigned collisionMask_;
    /// Last interpolated position from the simulation.
    mutable Vector3 lastPosition_;
    /// Last interpolated rotation from the simulation.
    mutable Quaternion lastRotation_;
    /// Use gravity flag.
    bool useGravity_;
    /// Readd body to world flag.
    bool readdBody_;
    /// Body exists in world flag.
    bool inWorld_;
    /// Mass update enable flag.
    bool enableMassUpdate_;
    /// Deactivation velocity.
    float deactivationVelocity_;
    /// Deactivation delay.
    int deactivationDelay_;
    /// Boundingbox
    BoundingBox boundingBox_;
    /// Vertex duplicate pairs.
    Vector<Pair<unsigned, unsigned> > duplicatePairs_;
    /// Vertex remap list.
    PODVector<unsigned> remapList_;
    /// Normals based on MDL model
    bool setToFaceNormals_;
    /// Linear stiffness coefficient [0,1]
    float configLST_;
    /// Pose matching coefficient [0,1]
    float configMT_;
    /// Volume conservation coefficient [0,+inf]
    float configVC_;
    /// Pressure coefficient [-inf,+inf]
    float configPR_;
};

}
