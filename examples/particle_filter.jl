using Gen
using UnicodePlots
using Distributions
using Plots

include(joinpath(@__DIR__, "helpers.jl"))

# this proposal function implements a truncated random walk for mass and restitution
@gen function proposal(tr::Gen.Trace)
    # HINT: https://www.gen.dev/tutorials/iterative-inference/tutorial#mcmc-2
    #
    # get previous values from `tr`
    choices = get_choices(tr)
    prev_mass = choices[:prior => 1 => :mass]
    prev_res  = choices[:prior => 1 => :restitution]
    
    # sample new values conditioned on the old ones
    mass = {:prior => 1 => :mass} ~ trunc_norm(prev_mass, .1, 0., Inf)
    restitution = {:prior => 1 => :restitution} ~ trunc_norm(prev_res, .1, 0., 1.)
    
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
                             obs::Vector{Gen.ChoiceMap},
                             particles::Int = 100)
    get_args(t) = (t, gm_args[2:3]...)

    # initialize particle filter
    state = Gen.initialize_particle_filter(model, get_args(0), EmptyChoiceMap(), particles)

    # Then increment through each observation step
    for (t, o) = enumerate(obs)
        # apply a rejuvenation move to each particle
        # for i=1:particles
        #     initial_choices = select(:mass, :restitution)
        #     state.traces[i], _  = mh(state.traces[i], initial_choices)
        # end

        Gen.maybe_resample!(state, ess_threshold=particles/2) 
        Gen.particle_filter_step!(state, get_args(t), (UnknownChange(), NoChange(), NoChange()), o, proposal, ())
    end

    # return the "unweighted" set of traces after t steps
    return Gen.sample_unweighted_traces(state, particles)
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
    obs = Vector{Gen.ChoiceMap}(undef, t)
    for i = 1:t
        addr = :kernel => i => :observe
        obs[i] = Gen.get_submap(choices, addr)
    end
    
    return (gargs, obs, trace)

end


function main()

    t = 60 # 1 second of observations
    (gargs, obs, truth) = data_generating_procedure(t)

    # display(plot_zs(truth))

    (traces, aratio) = inference_procedure(gargs, obs, 20)

    display(plot_traces(truth, traces))

    println("press enter to exit the program")
    readline()
    return nothing
end


main();
