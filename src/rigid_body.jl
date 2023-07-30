export RigidBody,
    RigidBodyState,
    RigidBodyLatents

"""
A rigid body in `BulletSim`

$(TYPEDEF)

Consists of a single base collision object.

---

$(TYPEFIELDS)
"""
struct RigidBody <: BulletElement
    "The `bodyUniqueId` of the base object"
    object_id::Int64
end

"""
State for `RigidBody`

$(TYPEDEF)

---

$(TYPEFIELDS)
"""
struct RigidBodyState <: BulletElemState{RigidBody}
    "XYZ position"
    position::SVector{3, Float64}
    "Quaternion xyzw"
    orientation::SVector{4, Float64}
    "Linear velocity XYZ"
    linear_vel::SVector{3, Float64}
    "Angular velocity wX wY wZ"
    angular_vel::SVector{3, Float64}
end

function RigidBodyState(e::RigidBody, sim::BulletSim)
    get_state(e, sim)
end

function get_state(e::RigidBody, sim::BulletSim)
    @pycall pos, orn = pb.getBasePositionAndOrientation(
        e.object_id,
        physicsClientId = sim.client
    )::Tuple{PyArray, PyArray}
    @pycall lin_vel, ang_vel = pb.getBaseVelocity(
        e.object_id,
        physicsClientId = sim.client
    )::Tuple{PyArray, PyArray}

    RigidBodyState(pos, orn, lin_vel, ang_vel)
end

function set_state!(e::RigidBody, sim::BulletSim, st::RigidBodyState)
    @pycall pb.resetBasePositionAndOrientation(
        e.object_id,
        posObj =  st.position,
        ornObj = st.orientation,
        physicsClientId = sim.client
    )::PyObject
    @pycall pb.resetBaseVelocity(
        e.object_id,
        linearVelocity = st.linear_vel,
        angularVelocity = st.ang_vel,
        physicsClientId = sim.client
    )::PyObject

    return nothing
end

"""
Latents for `RigidBody`

$(TYPEDEF)

Any collection of property values can be declared in `data`.
Most commonly these will properties such as "mass" or "lateralFriction"
and any undeclared properties will use default values (see pybullet)

A non-exhaustive list of properties
- mass
- lateralFriction
- localInertiaDiagonal
- localInertialPos
- localInertialOrn
- restitution
- rollingFriction
- spinningFriction
- contactDamping
- contactStiffness

---

$(TYPEFIELDS)
"""
struct RigidBodyLatents <: BulletElemLatents{RigidBody}
    data::NamedTuple
end

function get_latents(e::RigidBody, sim::BulletSim)
    # REVIEW: pybullet docs says `getDynamicsInfo` is incomplete / weird
    ls = @pycall pb.getDynamicsInfo(;
        bodyUniqueId = e.object_id,
        linkIndex = -1, # base (assumption for `RigidBody`)
        ls.data..., # REVIEW: is there a more direct way to passing in latents?
        physicsClientId = sim.client
    )::Dict
    NamedTuple(ls)
end

function set_latents!(e::RigidBody, sim::BulletSim, ls::RigidBodyLatents)

    @pycall pb.changeDynamics(;
        bodyUniqueId = e.object_id,
        linkIndex = -1, # base (assumption for `RigidBody`)
        ls.data..., # REVIEW: more direct?
        physicsClientId = sim.client
    )::PyObject
    return nothing
end
