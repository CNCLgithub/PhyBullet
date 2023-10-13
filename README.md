# PhyBullet - Bullet backend for PhySMC

> NOTE!: This is a work in progress and the API is NOT stable

Implements the core interface of [PhySMC](https://github.com/CNCLgithub/PhySMC) for `BulletSim` and `BulletState`.

Currently the only `Element{BulletSim}` implemented is `RigidBody` but others (such as those with complex joints) are possible. 

See https://github.com/CNCLgithub/PhyBullet-examples and `src/` for docstrings. 


## Installation

Assumes that you have installed `pybullet` (e.g., `pip install pybullet`) and that the python path is exposed to `PyCall`. For more info see https://github.com/JuliaPy/PyCall.jl#python-virtual-environments


With the proper python env activated simply run 

``` julia-repl
pkg> add https://github.com/CNCLgithub/PhyBullet.git
```


### Example setup

> NOTE: this is just an example, there are many roads to nirvana

First step, create python virtualenv to install pybullet 

``` sh
# run this at the root of this repo 
virtualenv .venv
source .venv/bin/activate

```


Install pybullet
``` sh
python -m pip install --upgrade pip
pip install pybullet
```


After `pybullet` is installed, active the virtualenv whenever you need to, you should not need to re-install pybullet
``` sh
source .venv/bin/activate
```

Need some exports for Julia's `PyCall` to use the right python environment
``` sh
export PYCALL_JL_RUNTIME_PYTHON="${PWD}/.venv/bin/python3"
export PYCALL_JL_RUNTIME_PYTHONHOME="${PWD}/.venv"
```

``` sh
julia --project=.
```

## Example usage
> Further examples of how to use this Julia package can be found in the repository [PhyBullet-examples](https://github.com/CNCLgithub/PhyBullet-examples).

Initialize a simple scene in pybullet

```julia
using Gen
using PyCall
using PhySMC
using PhyBullet
using Accessors

function simple_scene(mass::Float64=1.0,
                      restitution::Float64=0.9)
    client = @pycall pb.connect(pb.DIRECT)::Int64
    pb.setGravity(0,0,-10; physicsClientId = client)

    # add a table
    dims = [1.0, 1.0, 0.1] # in meters
    col_id = pb.createCollisionShape(pb.GEOM_BOX,
                                     halfExtents = dims,
                                     physicsClientId = client)
    obj_id = pb.createMultiBody(baseCollisionShapeIndex = col_id,
                                basePosition = [0., 0., -0.1],
                                physicsClientId = client)
    pb.changeDynamics(obj_id,
                      -1;
                      mass = 0., # 0 mass are stationary
                      restitution = 0.9, # some is necessary
                      physicsClientId=client)


    # add a ball
    bcol_id = pb.createCollisionShape(pb.GEOM_SPHERE,
                                      radius = 0.1,
                                      physicsClientId = client)
    bobj_id = pb.createMultiBody(baseCollisionShapeIndex = bcol_id,
                                 basePosition = [0., 0., 1.0],
                                 physicsClientId = client)
    pb.changeDynamics(bobj_id,
                      -1;
                      mass = mass,
                      restitution = restitution,
                      physicsClientId=client)

    (client, bobj_id)
end
```

Initialize the `PhyBullet` simulation context.
```julia
# start with a ball above a table
client, ball_id = simple_scene()
# configure simulator with the provided
# client id
sim = BulletSim(;client=client)
# This is the object of interest in the scene
# (the table is static)
ball = RigidBody(ball_id)
# Retrieve the default latents for the ball
# as well as its initial positions
# Note: alternative latents will be suggested by the `prior`
init_state = BulletState(sim, [ball])
```

Define a simple generative model that samples new latents (mass and restitution) for the ball and simulates `t` steps.

```julia
@gen function prior(ls::RigidBodyLatents)
    mass = @trace(gamma(1.2, 10.), :mass)
    res = @trace(uniform(0, 1), :restitution)
    new_ls = setproperties(ls.data;
                           mass = mass,
                           restitution = res)
    new_latents = RigidBodyLatents(new_ls)
    return new_latents
end

@gen function observe(k::RigidBodyState)
    pos = k.position # XYZ position
    # add noise to position
    obs = @trace(broadcasted_normal(pos, 0.01), :position)
    return obs
end

@gen function kernel(t::Int, prev_state::BulletState, sim::BulletSim)
    # use of PhySMC.step
    next_state::BulletState = PhySMC.step(sim, prev_state)
    # elem state could be a different type
    # but here we only have one `RigidBody` element
    # so  `next_state.kinematics = [RigidBodyState]`
    obs = @trace(Gen.Map(observe)(next_state.kinematics), :observe)
    return next_state
end

@gen function model(t::Int, sim::BulletSim, template::BulletState)
    # sample new mass and restitution for objects
    latents = @trace(Gen.Map(prior)(template.latents), :prior)
    init_state = setproperties(template; latents = latents)
    # simulate `t` timesteps
    states = @trace(Gen.Unfold(kernel)(t, init_state, sim), :kernel)
    return states
end
```

You can now sample dynamic scenes from `model` and also use it for inference (see [PhyBullet-examples](https://github.com/CNCLgithub/PhyBullet-examples)).
