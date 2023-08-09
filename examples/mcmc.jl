using Gen
using UnicodePlots

include("__@DIR__/helpers.jl")

@gen function proposal(tr::Gen.Trace)
    # TODO: fill in TODO's
    # HINT: https://www.gen.dev/tutorials/iterative-inference/tutorial#mcmc-2
    #
    # get previous values from `tr`
    prev_mass = TODO
    prev_res = TODO
    # sample new values conditioned on the old ones
    mass ~ TODO
    restitution ~ TODO
    # the return of this function is not
    # neccessary but could be useful
    # for debugging.
    return (mass, restitution)
end

"""
    inference_procedure

Performs Metropolis-Hastings MCMC.
"""
function inference_procedure(gm_args::Tuple,
                             obs::Gen.Choicemap,
                             steps::Int = 100)

    # start with an initial guess of physical latents
    # `ls` is the log score or how well this
    # initial guess explains the observations
    tr, ls = Gen.generate(model, gm_args, obs)

    println("Initial logscore: $(ls)")

    # TODO: use this to count the number of accepted moves
    acceptance_count = 0
    for _ = 1:steps
        # apply the proposal funciton to generate a
        # new guess over the ball's latents
        # that is related to the previous trace
        # see `?mh` in the REPL for more info
        tr, accepted = mh(tr, proposal, ())

    end

    acceptance_ratio = 0 # TODO: implement

    println("Final logscore: $(get_score(tr))")
    println("Acceptance ratio: $(acceptance_ratio)")

    return (tr, acceptance_ratio)
end

"""
    data_generating_procedure(t::Int64)

Create a trial (ground truth and observations) with `t` timepoints
"""
function data_generating_procedure(t::Int64)

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
    gargs = (t, # number of steps
             sim,
             init_state)

    # execute `model`
    trace, _ = Gen.generate(model, gargs)
    choices = get_choices(trace)
    # extract noisy positions
    obs = Gen.choicemap()
    for i = 1:t
        addr = :kernel => i => :observe
        _choices = Gen.get_submap(choices, addr)
        Gen.set_submap!(obs, addr, _choices)
    end

    return (gargs, obs)

end

function main()

    t = 60 # 1 second of observations
    (gargs, obs) = data_generating_procedure(t)

    (tr, aratio) = inference_procedure(gargs, obs)

    return nothing
end


main();
