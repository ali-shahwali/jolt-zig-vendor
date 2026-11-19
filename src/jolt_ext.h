/* Extensions to joltc (JPHX_*): Jolt features that the joltc C API does not expose. */
#ifndef JOLT_EXT_H
#define JOLT_EXT_H

#include "joltc.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Receives bytes that a save writes. Returns false to stop the save. */
typedef bool (*JPHX_WriteFn)(void* context, const void* data, uint32_t size);

/* A physics material with a surface index, friction and restitution. The index is the material's id in
 * saved shapes, so a saved shape restores with the same table of materials. */
JPH_CAPI JPH_PhysicsMaterial* JPHX_PhysicsMaterial_Create(uint32_t index, float friction, float restitution);
/* Changes the friction and restitution of a JPHX material. Call it only between steps. */
JPH_CAPI void JPHX_PhysicsMaterial_Set(JPH_PhysicsMaterial* material, float friction, float restitution);
/* 0 for no material or a material that JPHX_PhysicsMaterial_Create did not create. */
JPH_CAPI uint32_t JPHX_PhysicsMaterial_GetIndex(const JPH_PhysicsMaterial* material);
/* The surface index at a sub shape of a body's shape. 0 for the default material. */
JPH_CAPI uint32_t JPHX_Body_GetSurface(const JPH_Body* body, JPH_SubShapeID subShapeID);
/* The friction or restitution of a JPHX material. 0 for a material that JPHX_PhysicsMaterial_Create did not create. */
JPH_CAPI float JPHX_PhysicsMaterial_GetFriction(const JPH_PhysicsMaterial* material);
JPH_CAPI float JPHX_PhysicsMaterial_GetRestitution(const JPH_PhysicsMaterial* material);
/* The material at a sub shape of a body's shape. Jolt's default material when the sub shape has none. */
JPH_CAPI const JPH_PhysicsMaterial* JPHX_Body_GetMaterial(const JPH_Body* body, JPH_SubShapeID subShapeID);

/* Combines the friction or the restitution of two touching sub shapes. Jolt calls it during a step, on
 * several threads at once. */
typedef float (*JPHX_CombineFn)(const JPH_Body* body1, const JPH_SubShapeID* subShapeID1, const JPH_Body* body2, const JPH_SubShapeID* subShapeID2);
/* Replaces Jolt's combine function (geometric mean for friction, maximum for restitution). Call it only between steps. */
JPH_CAPI void JPHX_PhysicsSystem_SetCombineFriction(JPH_PhysicsSystem* system, JPHX_CombineFn combine);
JPH_CAPI void JPHX_PhysicsSystem_SetCombineRestitution(JPH_PhysicsSystem* system, JPHX_CombineFn combine);

/* Writes the shape, its children and references to `materials` (by index). Returns false on a write error
 * or when the shape uses a material outside the table. */
JPH_CAPI bool JPHX_Shape_Save(const JPH_Shape* shape, const JPH_PhysicsMaterial* const* materials, uint32_t materialCount, JPHX_WriteFn write, void* context);
/* The restored shape with one reference (release with JPH_Shape_Destroy), or null. */
JPH_CAPI JPH_Shape* JPHX_Shape_Restore(const void* data, uint32_t size, const JPH_PhysicsMaterial* const* materials, uint32_t materialCount);

/* Every body, constraint and contact cache of the system. */
JPH_CAPI bool JPHX_PhysicsSystem_SaveState(JPH_PhysicsSystem* system, JPHX_WriteFn write, void* context);
JPH_CAPI bool JPHX_PhysicsSystem_RestoreState(JPH_PhysicsSystem* system, const void* data, uint32_t size);

/* Adds created bodies in one broad phase subtree. The bodies must not be in the system. */
JPH_CAPI void JPHX_BodyInterface_AddBodies(JPH_BodyInterface* bodyInterface, JPH_BodyID* ids, uint32_t count, JPH_Activation activation);
/* Creates a body without adding it. Returns JPH_INVALID_BODY_ID-equivalent 0xffffffff on failure. */
JPH_CAPI JPH_BodyID JPHX_BodyInterface_CreateBody(JPH_BodyInterface* bodyInterface, const JPH_BodyCreationSettings* settings);

/* The world-space triangles of a shape, for debug lines: 9 floats per triangle. Returns the count written,
 * at most `triangleMax`. */
JPH_CAPI uint32_t JPHX_Shape_GetTriangles(const JPH_Shape* shape, const JPH_RVec3* position, const JPH_Quat* rotation, const JPH_Vec3* scale, float* outVertices, uint32_t triangleMax);

/* The world-space triangles of the leaf shapes of the bodies whose bounds touch `box`. Takes only bodies on
 * an object layer below 32 whose bit is set in `layerMask`, and with a motion type whose bit (1 << JPH_MotionType)
 * is set in `motionMask`. Compounds, scaled and offset shapes give their leaves. Writes 9 floats per triangle to
 * `outVertices` and the triangle's surface index (JPHX_PhysicsMaterial_GetIndex, 0 for none) to `outSurfaces`.
 * Returns the count written, at most `triangleMax`. Sets `*outTruncated` when more triangles remain.
 * Triangles of a body that only touches `box` can lie outside it. */
JPH_CAPI uint32_t JPHX_PhysicsSystem_GetTriangles(JPH_PhysicsSystem* system, const JPH_AABox* box, uint32_t layerMask, uint32_t motionMask, float* outVertices, uint32_t* outSurfaces, uint32_t triangleMax, bool* outTruncated);

/* Sets the convex radius-independent motion quality. 0 discrete, 1 linear cast (continuous). */
JPH_CAPI void JPHX_BodyCreationSettings_SetCcd(JPH_BodyCreationSettings* settings, bool ccd);
JPH_CAPI void JPHX_BodyCreationSettings_SetMass(JPH_BodyCreationSettings* settings, float mass);
JPH_CAPI void JPHX_BodyCreationSettings_SetEnhancedInternalEdgeRemoval(JPH_BodyCreationSettings* settings, bool enabled);
JPH_CAPI void JPHX_BodyCreationSettings_SetMaxLinearVelocity(JPH_BodyCreationSettings* settings, float velocity);

/* Solver and sleep settings of the system. */
JPH_CAPI void JPHX_PhysicsSystem_SetSolver(JPH_PhysicsSystem* system, uint32_t velocitySteps, uint32_t positionSteps, float sleepVelocity, float sleepSeconds);

/* Adds a reference to a shape. Release it with JPH_Shape_Destroy. */
JPH_CAPI void JPHX_Shape_AddRef(const JPH_Shape* shape);

/* Sets the material of a convex shape (box, sphere, capsule, tapered capsule, cylinder, convex hull). */
JPH_CAPI void JPHX_ConvexShape_SetMaterial(JPH_Shape* shape, const JPH_PhysicsMaterial* material);

/* A triangle mesh. `materialIndices` (one per triangle, may be null, values below 256) index `materials`.
 * Null on a degenerate mesh or when the triangles use more than 32 materials. */
JPH_CAPI JPH_Shape* JPHX_MeshShape_Create(const float* positions, uint32_t vertexCount, const uint32_t* indices, uint32_t triangleCount, const uint8_t* materialIndices, const JPH_PhysicsMaterial* const* materials, uint32_t materialCount);

/* A square height field of `sampleCount` x `sampleCount` samples. A sample of FLT_MAX is a hole.
 * `materialIndices` has (sampleCount - 1)^2 entries or is null. */
JPH_CAPI JPH_Shape* JPHX_HeightFieldShape_Create(const float* samples, uint32_t sampleCount, const JPH_Vec3* offset, const JPH_Vec3* scale, const uint8_t* materialIndices, const JPH_PhysicsMaterial* const* materials, uint32_t materialCount);

/* A convex hull of the points. Null when the points span no volume. */
JPH_CAPI JPH_Shape* JPHX_ConvexHullShape_Create(const float* points, uint32_t pointCount, const JPH_PhysicsMaterial* material);

/* Replaces the shape of the kinematic body that follows the character, so queries see its new size. */
JPH_CAPI void JPHX_CharacterVirtual_SetInnerBodyShape(JPH_CharacterVirtual* character, const JPH_Shape* shape);

/* The sub shape user data at `subShapeID` of a compound shape (the part index), else 0. */
JPH_CAPI uint32_t JPHX_Body_GetSubShapeUserData(const JPH_Body* body, JPH_SubShapeID subShapeID);

#ifdef __cplusplus
}
#endif

#endif
