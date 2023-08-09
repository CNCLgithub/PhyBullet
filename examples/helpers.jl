
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

@gen function prior(ls::RigidBodyLatents)
    mass = @trace(uniform(0., 1.), :mass)
    res = @trace(uniform(0.2, 0.90), :restitution)
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

@load_generated_functions
