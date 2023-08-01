using Gen
using PyCall
using PhySMC
using PhyBullet


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

@gen function prior()
    mass = @trace(uniform(0., 1.), :mass)
    res = @trace(uniform(0.5, 0.9), :restitution)
    ls = RigidBodyLatents((mass = mass, restitution = res))
    return ls
end

@load_generated_functions

function main()

    # start with a ball above a table
    client, ball_id = simple_scene()

    sim = BulletSim(;client=client)
    ball = RigidBody(ball_id)
    init_state = BulletState(sim, [ball])

    gargs = (100, # number of steps
             init_state, # initial state
             sim)
    trace, _ = generate(Gen.Unfold(kernel), gargs)

    display(get_choices(trace))
end


main();
