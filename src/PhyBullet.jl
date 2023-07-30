module PhyBullet

using PyCall
using PhySMC
using DocStringExtensions

export pb,
    BulletSim,
    BulletState,
    BulletElement,
    BulletElemState,
    BulletElemLatents,


const pb = PyNull()

function __init__()
    copy!(pb, pyimport("pybullet"))
end


"""
Parameters for using the Bullet physics engine

$(TYPEDEF)

---

$(TYPEDFIELDS)
"""
@with_kw struct BulletSim <: PhySim
    "Client id for pybullet"
    client::Int64
    "Amount of time between `forward_steps` (default=16.7ms)"
    step_dur::Float64 = 1 / 60
    "Timestep duration of bullet engine (default: 4.2ms)"
    pb_timestep::Float64 = 1 / 240
end


""" An element for `BulletSim` """
abstract type BulletElement <: Element{BulletSim}
abstract type BulletElemState{T<:BulletElement} <: ElemState{T}
abstract type BulletElemLatents{T<:BulletElement} <: ElemLatents{T}

function get_state end

function set_state! end

function get_latents end

function set_latents! end

"""
State for `BulletSim`

$(TYPEDEF)

---

$(TYPEFIELDS)
"""
struct BulletState <: PhyState{BulletSim}
    elements::AbstractVector{BulletElement}
    latents::AbstractVector{BulletElemLatents}
    kinematics::AbstractVector{BulletElemState}
end

function BulletState(sim::BulletSim, elements::AbstractVector{BulletElement})
    latents = map(x -> get_latents(x, sim), elements)
    kinematics = map(x -> get_state(x, sim), elements)
    BulletState(elements, latents, kinematics)
end

function sync!(sim::BulletSim, world_state::BulletState)
    for elem, est, ls in zip(world_state.elements,
                             world_state.latents,
                             world_state.kinematics)
        set_state!(elem, sim, est)
        set_latents!(elem, sim, ls)
    end
    return nothing
end

function forward_step(sim::BulletSim, st::BulletState)
    # progress by `st.step_dur`
    dt::Float64 = 0.0
    while dt <= sim.step_dur
        @pycall pb.stepSimulation(;
                                  physicsClientId = sim.client
                                  )::PyObject
        dt += sim.pb_timestep
    end
    # extract resulting state
    ne = length(st.elements)
    elements = Vector{BulletElement}(undef, ne)
    latents = Vector{BulletLatents}(undef, ne)
    kinematics = Vector{BulletElemState}(undef, ne)
    @inbounds for i = 1:ne
        elements[i] = st.elements[i]
        latents[i] = st.latents[i] # REVIEW: this vs `get_latents`
        kinematics[i] = get_state(elements[i], sim)
    end
    BulletState(elements, latents, kinematics)
end

end # module PhyBullet
