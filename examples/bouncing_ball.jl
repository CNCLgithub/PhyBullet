using Gen
using PyCall
using PhySMC
using PhyBullet
using Accessors
using UnicodePlots


function simple_scene()
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
                      mass = 1.0,
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

function draw_trace(tr::Gen.Trace)
    (t, _, _) = get_args(tr)
    # get the prior choice for restitution
    choices = get_choices(tr)
    restitution = choices[:prior => 1 => :restitution]
    # get the z positions
    states = get_retval(tr)
    zs = map(st -> st.kinematics[1].position[3], states)
    plt = lineplot(1:t, zs,
                   title="Height of ball", name="res: $(restitution)",
                   xlabel="t", ylabel="z", canvas=DotCanvas,
                   border=:ascii)
end

function update_plot(plt, tr::Gen.Trace, n::Int)
    (t, _, _) = get_args(tr)
    choices = get_choices(tr)
    restitution = choices[:prior => 1 => :restitution]
    states = get_retval(tr)
    zs = map(st -> st.kinematics[1].position[3], states)
    lineplot!(plt, 1:t, zs,
              name="res: $(restitution)")
end

function main()

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
    # arguments for `model`
    gargs = (60, # number of steps (1s)
             sim,
             init_state)

    # execute `model`
    trace, _ = generate(model, gargs)
    # visualize the height for the ball across time
    plt = draw_trace(trace)
    # visualize unique tracetories for different
    # resitution values
    for i = 2:10
        trace, _ = generate(model, gargs)
        plt = update_plot(plt, trace, i)
    end
    display(plt);
    return nothing
end


main();
