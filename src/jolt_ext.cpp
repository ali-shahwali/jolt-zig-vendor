// Extensions to joltc. See jolt_ext.h.

#include "jolt_ext.h"

#include <Jolt/Jolt.h>
#include <Jolt/Core/UnorderedMap.h>
#include <Jolt/Core/StreamOut.h>
#include <Jolt/Core/StreamIn.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/StateRecorderImpl.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/Collision/PhysicsMaterial.h>
#include <Jolt/Physics/Collision/Shape/ConvexShape.h>
#include <Jolt/Physics/Collision/Shape/CompoundShape.h>
#include <Jolt/Physics/Collision/Shape/DecoratedShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>
#include <Jolt/Physics/Character/CharacterVirtual.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Collision/TransformedShape.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>

#include <cstring>

namespace {

// joltc's handle of a physics system (joltc.cpp, `struct JPH_PhysicsSystem`). The layout is pinned by the
// package hash of joltc in build.zig.zon.
struct JoltcPhysicsSystem {
	JPH::BroadPhaseLayerInterface* broadPhaseLayerInterface;
	JPH::ObjectLayerPairFilter* objectLayerPairFilter;
	JPH::ObjectVsBroadPhaseLayerFilter* objectVsBroadPhaseLayerFilter;
	JPH::PhysicsSystem* physicsSystem;
};

JPH::PhysicsSystem* system(JPH_PhysicsSystem* s) {
	return reinterpret_cast<JoltcPhysicsSystem*>(s)->physicsSystem;
}

// The marker tells JPHX materials from others: Jolt's RTTI is the base class's for a subclass that does
// not declare its own.
constexpr JPH::uint32 material_marker = 0x6a707831; // "jpx1"

class ExtMaterial final : public JPH::PhysicsMaterial {
public:
	ExtMaterial(JPH::uint32 index, float friction, float restitution)
		: marker(material_marker), index(index), friction(friction), restitution(restitution) {}

	JPH::uint32 marker;
	JPH::uint32 index;
	float friction;
	float restitution;
};

const ExtMaterial* extMaterial(const JPH::PhysicsMaterial* m) {
	if (m == nullptr) return nullptr;
	if (m == JPH::PhysicsMaterial::sDefault.GetPtr()) return nullptr;
	// The caller creates materials only with JPHX_PhysicsMaterial_Create (not joltc's JPH_PhysicsMaterial_Create).
	const ExtMaterial* a = static_cast<const ExtMaterial*>(m);
	if (a->marker != material_marker) return nullptr;
	return a;
}

class WriteStream final : public JPH::StreamOut {
public:
	WriteStream(JPHX_WriteFn write, void* context) : write(write), context(context) {}

	void WriteBytes(const void* data, size_t size) override {
		if (failed) return;
		if (size > 0xffffffffu) { failed = true; return; }
		if (!write(context, data, static_cast<uint32_t>(size))) failed = true;
	}

	bool IsFailed() const override { return failed; }

private:
	JPHX_WriteFn write;
	void* context;
	bool failed = false;
};

class ReadStream final : public JPH::StreamIn {
public:
	ReadStream(const void* data, uint32_t size) : data(static_cast<const uint8_t*>(data)), size(size) {}

	void ReadBytes(void* out, size_t count) override {
		if (failed || count > size - offset) {
			failed = true;
			std::memset(out, 0, count);
			return;
		}
		std::memcpy(out, data + offset, count);
		offset += static_cast<uint32_t>(count);
	}

	bool IsEOF() const override { return offset >= size; }
	bool IsFailed() const override { return failed; }

private:
	const uint8_t* data;
	uint32_t size;
	uint32_t offset = 0;
	bool failed = false;
};

// A C++ reference passes as a pointer on every supported ABI, and JPH_Body and JPH_SubShapeID are Jolt's
// Body and SubShapeID. So a JPHX_CombineFn is a Jolt CombineFunction, and Jolt calls it with no wrapper.
static_assert(sizeof(JPH::SubShapeID) == sizeof(JPH_SubShapeID), "SubShapeID is one 32-bit value");

JPH::ContactConstraintManager::CombineFunction combineFunction(JPHX_CombineFn combine) {
	return reinterpret_cast<JPH::ContactConstraintManager::CombineFunction>(combine);
}

// Passes object layers below 32 whose bit is set in the mask.
class LayerMaskFilter final : public JPH::ObjectLayerFilter {
public:
	explicit LayerMaskFilter(uint32_t mask) : mask(mask) {}

	bool ShouldCollide(JPH::ObjectLayer layer) const override {
		if (layer >= 32) return false;
		return (mask >> layer) & 1u;
	}

private:
	uint32_t mask;
};

// Passes bodies whose motion type has its bit (1 << EMotionType) set in the mask.
class MotionMaskFilter final : public JPH::BodyFilter {
public:
	explicit MotionMaskFilter(uint32_t mask) : mask(mask) {}

	bool ShouldCollideLocked(const JPH::Body& body) const override {
		const uint32_t motion = static_cast<uint32_t>(body.GetMotionType());
		return (mask >> motion) & 1u;
	}

private:
	uint32_t mask;
};

static_assert(static_cast<int>(JPH::EMotionType::Static) == JPH_MotionType_Static, "motion bits match joltc");
static_assert(static_cast<int>(JPH::EMotionType::Kinematic) == JPH_MotionType_Kinematic, "motion bits match joltc");
static_assert(static_cast<int>(JPH::EMotionType::Dynamic) == JPH_MotionType_Dynamic, "motion bits match joltc");

} // namespace

extern "C" {

JPH_PhysicsMaterial* JPHX_PhysicsMaterial_Create(uint32_t index, float friction, float restitution) {
	ExtMaterial* m = new ExtMaterial(index, friction, restitution);
	m->AddRef();
	return reinterpret_cast<JPH_PhysicsMaterial*>(static_cast<JPH::PhysicsMaterial*>(m));
}

void JPHX_PhysicsMaterial_Set(JPH_PhysicsMaterial* material, float friction, float restitution) {
	ExtMaterial* m = const_cast<ExtMaterial*>(extMaterial(reinterpret_cast<const JPH::PhysicsMaterial*>(material)));
	if (m == nullptr) return;
	m->friction = friction;
	m->restitution = restitution;
}

uint32_t JPHX_PhysicsMaterial_GetIndex(const JPH_PhysicsMaterial* material) {
	const ExtMaterial* m = extMaterial(reinterpret_cast<const JPH::PhysicsMaterial*>(material));
	return m != nullptr ? m->index : 0;
}

uint32_t JPHX_Body_GetSurface(const JPH_Body* body, JPH_SubShapeID subShapeID) {
	const JPH::Body* b = reinterpret_cast<const JPH::Body*>(body);
	JPH::SubShapeID id;
	id.SetValue(subShapeID);
	const ExtMaterial* m = extMaterial(b->GetShape()->GetMaterial(id));
	return m != nullptr ? m->index : 0;
}

float JPHX_PhysicsMaterial_GetFriction(const JPH_PhysicsMaterial* material) {
	const ExtMaterial* m = extMaterial(reinterpret_cast<const JPH::PhysicsMaterial*>(material));
	return m != nullptr ? m->friction : 0.0f;
}

float JPHX_PhysicsMaterial_GetRestitution(const JPH_PhysicsMaterial* material) {
	const ExtMaterial* m = extMaterial(reinterpret_cast<const JPH::PhysicsMaterial*>(material));
	return m != nullptr ? m->restitution : 0.0f;
}

const JPH_PhysicsMaterial* JPHX_Body_GetMaterial(const JPH_Body* body, JPH_SubShapeID subShapeID) {
	const JPH::Body* b = reinterpret_cast<const JPH::Body*>(body);
	JPH::SubShapeID id;
	id.SetValue(subShapeID);
	return reinterpret_cast<const JPH_PhysicsMaterial*>(b->GetShape()->GetMaterial(id));
}

void JPHX_PhysicsSystem_SetCombineFriction(JPH_PhysicsSystem* s, JPHX_CombineFn combine) {
	system(s)->SetCombineFriction(combineFunction(combine));
}

void JPHX_PhysicsSystem_SetCombineRestitution(JPH_PhysicsSystem* s, JPHX_CombineFn combine) {
	system(s)->SetCombineRestitution(combineFunction(combine));
}

bool JPHX_Shape_Save(const JPH_Shape* shape, const JPH_PhysicsMaterial* const* materials, uint32_t materialCount, JPHX_WriteFn write, void* context) {
	WriteStream stream(write, context);
	JPH::Shape::ShapeToIDMap shapes;
	JPH::Shape::MaterialToIDMap ids;
	for (uint32_t i = 0; i < materialCount; ++i) {
		ids[reinterpret_cast<const JPH::PhysicsMaterial*>(materials[i])] = i;
	}
	reinterpret_cast<const JPH::Shape*>(shape)->SaveWithChildren(stream, shapes, ids);
	// A material outside the table got a new id: the restore cannot resolve it.
	if (ids.size() != materialCount) return false;
	return !stream.IsFailed();
}

JPH_Shape* JPHX_Shape_Restore(const void* data, uint32_t size, const JPH_PhysicsMaterial* const* materials, uint32_t materialCount) {
	ReadStream stream(data, size);
	JPH::Shape::IDToShapeMap shapes;
	JPH::Shape::IDToMaterialMap ids;
	ids.reserve(materialCount);
	for (uint32_t i = 0; i < materialCount; ++i) {
		ids.push_back(JPH::Ref<JPH::PhysicsMaterial>(const_cast<JPH::PhysicsMaterial*>(reinterpret_cast<const JPH::PhysicsMaterial*>(materials[i]))));
	}
	JPH::Shape::ShapeResult result = JPH::Shape::sRestoreWithChildren(stream, shapes, ids);
	if (stream.IsFailed() || !result.IsValid()) return nullptr;
	// A material id past the table restored a new material from the stream: refuse the shape.
	if (ids.size() != materialCount) return nullptr;
	JPH::Shape* restored = const_cast<JPH::Shape*>(result.Get().GetPtr());
	restored->AddRef();
	return reinterpret_cast<JPH_Shape*>(restored);
}

bool JPHX_PhysicsSystem_SaveState(JPH_PhysicsSystem* s, JPHX_WriteFn write, void* context) {
	JPH::StateRecorderImpl recorder;
	system(s)->SaveState(recorder, JPH::EStateRecorderState::All);
	const std::string data = recorder.GetData();
	if (data.size() > 0xffffffffu) return false;
	return write(context, data.data(), static_cast<uint32_t>(data.size()));
}

bool JPHX_PhysicsSystem_RestoreState(JPH_PhysicsSystem* s, const void* data, uint32_t size) {
	JPH::StateRecorderImpl recorder;
	recorder.WriteBytes(data, size);
	return system(s)->RestoreState(recorder);
}

JPH_BodyID JPHX_BodyInterface_CreateBody(JPH_BodyInterface* bodyInterface, const JPH_BodyCreationSettings* settings) {
	JPH::BodyInterface* bi = reinterpret_cast<JPH::BodyInterface*>(bodyInterface);
	JPH::Body* body = bi->CreateBody(*reinterpret_cast<const JPH::BodyCreationSettings*>(settings));
	if (body == nullptr) return 0xffffffffu;
	return body->GetID().GetIndexAndSequenceNumber();
}

void JPHX_BodyInterface_AddBodies(JPH_BodyInterface* bodyInterface, JPH_BodyID* ids, uint32_t count, JPH_Activation activation) {
	static_assert(sizeof(JPH::BodyID) == sizeof(JPH_BodyID), "BodyID is one 32-bit value");
	if (count == 0) return;
	JPH::BodyInterface* bi = reinterpret_cast<JPH::BodyInterface*>(bodyInterface);
	JPH::BodyID* jolt_ids = reinterpret_cast<JPH::BodyID*>(ids);
	JPH::BodyInterface::AddState state = bi->AddBodiesPrepare(jolt_ids, static_cast<int>(count));
	bi->AddBodiesFinalize(jolt_ids, static_cast<int>(count), state, static_cast<JPH::EActivation>(activation));
}

uint32_t JPHX_Shape_GetTriangles(const JPH_Shape* shape, const JPH_RVec3* position, const JPH_Quat* rotation, const JPH_Vec3* scale, float* outVertices, uint32_t triangleMax) {
	const JPH::Shape* s = reinterpret_cast<const JPH::Shape*>(shape);
	const JPH::RVec3 p(position->x, position->y, position->z);
	const JPH::Quat q(rotation->x, rotation->y, rotation->z, rotation->w);
	const JPH::Vec3 sc(scale->x, scale->y, scale->z);
	JPH::Shape::GetTrianglesContext context;
	s->GetTrianglesStart(context, JPH::AABox::sBiggest(), p, q, sc);
	uint32_t count = 0;
	constexpr int batch = JPH::Shape::cGetTrianglesMinTrianglesRequested;
	JPH::Float3 vertices[batch * 3];
	// Each call returns at least one triangle until the shape has none left.
	for (uint32_t round = 0; round < triangleMax && count < triangleMax; ++round) {
		const int n = s->GetTrianglesNext(context, batch, vertices, nullptr);
		if (n <= 0) break;
		for (int t = 0; t < n && count < triangleMax; ++t) {
			for (int v = 0; v < 3; ++v) {
				outVertices[count * 9 + v * 3 + 0] = vertices[t * 3 + v].x;
				outVertices[count * 9 + v * 3 + 1] = vertices[t * 3 + v].y;
				outVertices[count * 9 + v * 3 + 2] = vertices[t * 3 + v].z;
			}
			count += 1;
		}
	}
	return count;
}

uint32_t JPHX_PhysicsSystem_GetTriangles(JPH_PhysicsSystem* physicsSystem, const JPH_AABox* box, uint32_t layerMask, uint32_t motionMask, float* outVertices, uint32_t* outSurfaces, uint32_t triangleMax, bool* outTruncated) {
	*outTruncated = false;
	const JPH::AABox bounds(JPH::Vec3(box->min.x, box->min.y, box->min.z), JPH::Vec3(box->max.x, box->max.y, box->max.z));
	const LayerMaskFilter layerFilter(layerMask);
	const MotionMaskFilter motionFilter(motionMask);
	// Jolt walks compounds and decorated shapes, and gives each leaf with its world transform.
	JPH::AllHitCollisionCollector<JPH::TransformedShapeCollector> leaves;
	system(physicsSystem)->GetNarrowPhaseQuery().CollectTransformedShapes(bounds, leaves, {}, layerFilter, motionFilter);

	uint32_t count = 0;
	constexpr int batch = JPH::Shape::cGetTrianglesMinTrianglesRequested;
	JPH::Float3 vertices[batch * 3];
	const JPH::PhysicsMaterial* materials[batch];
	for (const JPH::TransformedShape& leaf : leaves.mHits) {
		JPH::Shape::GetTrianglesContext context;
		leaf.GetTrianglesStart(context, bounds, JPH::RVec3::sZero());
		// Each call returns at least one triangle until the leaf has none left.
		for (uint32_t round = 0; round <= triangleMax; ++round) {
			const int n = leaf.GetTrianglesNext(context, batch, vertices, materials);
			if (n <= 0) break;
			for (int t = 0; t < n; ++t) {
				if (count == triangleMax) {
					*outTruncated = true;
					return count;
				}
				for (int v = 0; v < 3; ++v) {
					outVertices[count * 9 + v * 3 + 0] = vertices[t * 3 + v].x;
					outVertices[count * 9 + v * 3 + 1] = vertices[t * 3 + v].y;
					outVertices[count * 9 + v * 3 + 2] = vertices[t * 3 + v].z;
				}
				const ExtMaterial* m = extMaterial(materials[t]);
				outSurfaces[count] = m != nullptr ? m->index : 0;
				count += 1;
			}
		}
	}
	return count;
}

void JPHX_BodyCreationSettings_SetCcd(JPH_BodyCreationSettings* settings, bool ccd) {
	reinterpret_cast<JPH::BodyCreationSettings*>(settings)->mMotionQuality = ccd ? JPH::EMotionQuality::LinearCast : JPH::EMotionQuality::Discrete;
}

void JPHX_BodyCreationSettings_SetMass(JPH_BodyCreationSettings* settings, float mass) {
	JPH::BodyCreationSettings* s = reinterpret_cast<JPH::BodyCreationSettings*>(settings);
	s->mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
	s->mMassPropertiesOverride.mMass = mass;
}

void JPHX_BodyCreationSettings_SetEnhancedInternalEdgeRemoval(JPH_BodyCreationSettings* settings, bool enabled) {
	reinterpret_cast<JPH::BodyCreationSettings*>(settings)->mEnhancedInternalEdgeRemoval = enabled;
}

void JPHX_BodyCreationSettings_SetMaxLinearVelocity(JPH_BodyCreationSettings* settings, float velocity) {
	reinterpret_cast<JPH::BodyCreationSettings*>(settings)->mMaxLinearVelocity = velocity;
}

void JPHX_PhysicsSystem_SetSolver(JPH_PhysicsSystem* s, uint32_t velocitySteps, uint32_t positionSteps, float sleepVelocity, float sleepSeconds) {
	JPH::PhysicsSettings settings = system(s)->GetPhysicsSettings();
	settings.mNumVelocitySteps = velocitySteps;
	settings.mNumPositionSteps = positionSteps;
	settings.mPointVelocitySleepThreshold = sleepVelocity;
	settings.mTimeBeforeSleep = sleepSeconds;
	system(s)->SetPhysicsSettings(settings);
}

void JPHX_Shape_AddRef(const JPH_Shape* shape) {
	reinterpret_cast<const JPH::Shape*>(shape)->AddRef();
}

void JPHX_ConvexShape_SetMaterial(JPH_Shape* shape, const JPH_PhysicsMaterial* material) {
	JPH::Shape* s = reinterpret_cast<JPH::Shape*>(shape);
	if (s->GetType() != JPH::EShapeType::Convex) return;
	static_cast<JPH::ConvexShape*>(s)->SetMaterial(reinterpret_cast<const JPH::PhysicsMaterial*>(material));
}

static JPH::PhysicsMaterialList materialList(const JPH_PhysicsMaterial* const* materials, uint32_t count) {
	JPH::PhysicsMaterialList list;
	list.reserve(count);
	for (uint32_t i = 0; i < count; ++i) list.push_back(reinterpret_cast<const JPH::PhysicsMaterial*>(materials[i]));
	return list;
}

JPH_Shape* JPHX_MeshShape_Create(const float* positions, uint32_t vertexCount, const uint32_t* indices, uint32_t triangleCount, const uint8_t* materialIndices, const JPH_PhysicsMaterial* const* materials, uint32_t materialCount) {
	JPH::VertexList vertices;
	vertices.reserve(vertexCount);
	for (uint32_t i = 0; i < vertexCount; ++i) vertices.push_back(JPH::Float3(positions[i * 3], positions[i * 3 + 1], positions[i * 3 + 2]));
	// A mesh shape holds at most 32 materials: the triangles index a compact list of the ones they use.
	constexpr uint32_t mesh_materials_max = 32;
	int32_t local[256];
	for (int32_t& l : local) l = -1;
	JPH::PhysicsMaterialList used;
	JPH::IndexedTriangleList triangles;
	triangles.reserve(triangleCount);
	for (uint32_t t = 0; t < triangleCount; ++t) {
		uint32_t m = materialIndices != nullptr ? materialIndices[t] : 0;
		if (m >= materialCount) m = 0;
		if (local[m] < 0) {
			if (used.size() >= mesh_materials_max) return nullptr;
			local[m] = static_cast<int32_t>(used.size());
			used.push_back(reinterpret_cast<const JPH::PhysicsMaterial*>(materials[m]));
		}
		triangles.push_back(JPH::IndexedTriangle(indices[t * 3], indices[t * 3 + 1], indices[t * 3 + 2], static_cast<uint32_t>(local[m])));
	}
	JPH::MeshShapeSettings settings(std::move(vertices), std::move(triangles), std::move(used));
	JPH::Shape::ShapeResult result = settings.Create();
	if (!result.IsValid()) return nullptr;
	JPH::Shape* s = const_cast<JPH::Shape*>(result.Get().GetPtr());
	s->AddRef();
	return reinterpret_cast<JPH_Shape*>(s);
}

JPH_Shape* JPHX_HeightFieldShape_Create(const float* samples, uint32_t sampleCount, const JPH_Vec3* offset, const JPH_Vec3* scale, const uint8_t* materialIndices, const JPH_PhysicsMaterial* const* materials, uint32_t materialCount) {
	JPH::HeightFieldShapeSettings settings(samples, JPH::Vec3(offset->x, offset->y, offset->z), JPH::Vec3(scale->x, scale->y, scale->z), sampleCount, materialIndices, materialList(materials, materialCount));
	JPH::Shape::ShapeResult result = settings.Create();
	if (!result.IsValid()) return nullptr;
	JPH::Shape* s = const_cast<JPH::Shape*>(result.Get().GetPtr());
	s->AddRef();
	return reinterpret_cast<JPH_Shape*>(s);
}

JPH_Shape* JPHX_ConvexHullShape_Create(const float* points, uint32_t pointCount, const JPH_PhysicsMaterial* material) {
	JPH::Array<JPH::Vec3> list;
	list.reserve(pointCount);
	for (uint32_t i = 0; i < pointCount; ++i) list.push_back(JPH::Vec3(points[i * 3], points[i * 3 + 1], points[i * 3 + 2]));
	JPH::ConvexHullShapeSettings settings(list, JPH::cDefaultConvexRadius, reinterpret_cast<const JPH::PhysicsMaterial*>(material));
	JPH::Shape::ShapeResult result = settings.Create();
	if (!result.IsValid()) return nullptr;
	JPH::Shape* s = const_cast<JPH::Shape*>(result.Get().GetPtr());
	s->AddRef();
	return reinterpret_cast<JPH_Shape*>(s);
}

void JPHX_CharacterVirtual_SetInnerBodyShape(JPH_CharacterVirtual* character, const JPH_Shape* shape) {
	reinterpret_cast<JPH::CharacterVirtual*>(character)->SetInnerBodyShape(reinterpret_cast<const JPH::Shape*>(shape));
}

uint32_t JPHX_Body_GetSubShapeUserData(const JPH_Body* body, JPH_SubShapeID subShapeID) {
	const JPH::Body* b = reinterpret_cast<const JPH::Body*>(body);
	JPH::SubShapeID id;
	id.SetValue(subShapeID);
	// The part id is the compound's user data of the sub shape. Decorators (scale, offset) use no ID bits.
	const JPH::Shape* s = b->GetShape();
	for (int depth = 0; depth < 8; ++depth) {
		if (s->GetType() == JPH::EShapeType::Compound) {
			const JPH::CompoundShape* compound = static_cast<const JPH::CompoundShape*>(s);
			JPH::SubShapeID remainder;
			const JPH::uint index = compound->GetSubShapeIndexFromID(id, remainder);
			return compound->GetCompoundUserData(index);
		}
		if (s->GetType() != JPH::EShapeType::Decorated) return 0;
		s = static_cast<const JPH::DecoratedShape*>(s)->GetInnerShape();
	}
	return 0;
}

} // extern "C"
