using Gen
using UnicodePlots
using Distributions
using Plots

include(joinpath(@__DIR__, "helpers.jl"))

# Truncated Distributions
struct TruncNorm <: Gen.Distribution{Float64} end
const trunc_norm = TruncNorm()
function Gen.random(::TruncNorm, mu::U, noise::T, low::T, high::T) where {U<:Real,T<:Real}
    d = Distributions.Truncated(Distributions.Normal(mu, noise),
                                low, high)
    return Distributions.rand(d)
end;
function Gen.logpdf(::TruncNorm, x::Float64, mu::U, noise::T, low::T, high::T) where {U<:Real,T<:Real}
    d = Distributions.Truncated(Distributions.Normal(mu, noise),
                                low, high)
    return Distributions.logpdf(d, x)
end;

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

function get_zs(tr::Gen.Trace)
    # get the z positions
    states = get_retval(tr)
    map(st -> st.kinematics[1].position[3], states)
end

"""
    inference_procedure

Performs Metropolis-Hastings MCMC.
"""
function inference_procedure(gm_args::Tuple,
                             obs::Gen.ChoiceMap, 
                             update_vis::Function = (tr)=>(),
                             steps::Int = 100)

    # start with an initial guess of physical latents
    # `ls` is the log score or how well this
    # initial guess explains the observations
    tr, ls = Gen.generate(model, gm_args, obs)
    # visualize predictions of initial trace
    update_vis(tr)

    println("Initial logscore: $(ls)")

    # count the number of accepted moves and track accepted proposals
    acceptance_count = 0
    mass_log = Vector{Float64}(undef, steps) 
    res_log = Vector{Float64}(undef, steps)
    scores = Vector{Float64}(undef, steps)

    for i = 1:steps
        # apply the proposal funciton to generate a
        # new guess over the ball's latents
        # that is related to the previous trace
        # see `?mh` in the REPL for more info
        tr, accepted = mh(tr, proposal, ())
        mass_log[i] = tr[:prior => 1 => :mass]
        res_log[i] = tr[:prior => 1 => :restitution]
        scores[i] = get_score(tr)
        acceptance_count += Int(accepted)
        if accepted
            update_vis(tr)
        end
    end

    acceptance_ratio = acceptance_count / steps

    println("Final logscore: $(get_score(tr))")
    println("Acceptance ratio: $(acceptance_ratio)")

    return (tr, acceptance_ratio, mass_log, res_log, scores)
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

    zs = [get_submap(obs, :kernel => i => :observe => 1)[:position][3] for i in 1:t]
    plt = plot(1:t, zs, title="Height of ball", xlabel="t", ylabel="z", label="Observation")
    display(plt)
    
    update_vis(trace) = display(plot!(plt, 1:t, get_zs(trace), label="res $(round(trace[:prior => 1 => :restitution]; digits=3))"))
    (tr, aratio, mass_log, res_log, scores) = inference_procedure(gargs, obs, update_vis)

    scores_plt = Plots.plot(1:length(scores), scores, title="Log of scores", xlabel="step", ylabel="log score")
    mass_plt = Plots.histogram(1:length(mass_log), mass_log, title="Histogram of mass", legend=false)
    res_plt = Plots.histogram(1:length(res_log), res_log, title="Histogram of res", legend=false)
    display(Plots.plot(scores_plt, mass_plt, res_plt))

    println("press enter to exit the program")
    readline()
    return nothing
end


main();
