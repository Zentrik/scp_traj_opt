#= This file stores the data structures and methods which define the Reentry
guidance numerical example for SCP. =#

include("../utils/types.jl")

# ..:: Data structures ::..

#= Reentry vehicle parameters. =#
struct ReentryVehicleParameters
    id_r::T_IntRange # Position individes of the state vector
    # TODO
end

#= Environment parameters. =#
struct ReentryEnvironmentParameters
    ρ::T_Real # Atmospheric density
    # TODO
end

#= Trajectory parameters. =#
struct ReentryTrajectoryParameters
    tf::T_Real # Flight time
    # TODO
end

#= Quadrotor trajectory optimization problem parameters all in one. =#
struct ReentryProblem
    vehicle::ReentryVehicleParameters # The ego-vehicle
    env::ReentryEnvironmentParameters # The environment
    traj::ReentryTrajectoryParameters # The trajectory
end

# ..:: Methods ::..

# TODO
function ρ(r::T_RealVector, mdl::ReentryProblem)::T_Real
    # TODO
    ρ = exp(-r)
    return ρ
end
