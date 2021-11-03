#= SCvx algorithm data structures and methods.

Sequential convex programming algorithms for trajectory optimization.
Copyright (C) 2021 Autonomous Controls Laboratory (University of Washington),
                   and Autonomous Systems Laboratory (Stanford University)

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program.  If not, see <https://www.gnu.org/licenses/>. =#

LangServer = isdefined(@__MODULE__, :LanguageServer)

if LangServer
    include("../../utils/src/Utils.jl")
    include("../../parser/src/Parser.jl")

    include("discretization.jl")
    include("scp.jl")

    using .Utils
    using .Utils.Types: improvement_percent
    using .Parser.ConicLinearProgram

    import .Parser.ConicLinearProgram: ConicProgram, ConvexCone, SupportedCone
    import .Parser.ConicLinearProgram: VariableArgumentBlock
    import .Parser.ConicLinearProgram: ConstantArgumentBlock
    import .Parser.ConicLinearProgram: ZERO, NONPOS, L1, SOC, LINF, GEOM, EXP
end

using LinearAlgebra
using JuMP
using Printf

using Utils
using Parser

import ..ST, ..RealTypes, ..IntRange, ..RealVector, ..RealMatrix, ..Trajectory,
    ..Objective, ..VarArgBlk, ..CstArgBlk, ..DLTV

import ..SCPParameters, ..SCPSubproblem, ..SCPSubproblemSolution, ..SCPProblem,
    ..SCPSolution, ..SCPHistory

import ..discretize!
import ..add_dynamics!, ..add_convex_state_constraints!,
    ..add_convex_input_constraints!, ..add_nonconvex_constraints!, ..add_bcs!
import ..solve_subproblem!, ..solution_deviation, ..unsafe_solution,
    ..overhead!, ..save!, ..get_time

const CLP = ConicLinearProgram #noerr
const Variable = ST.Variable
const Optional = ST.Optional
const OptVarArgBlk = Optional{VarArgBlk}

export Parameters, create, solve

# PUT SOMEWHERE ELSE / REPLACE
macro k(traj)
    :( $(esc(traj))[fill(:, ndims($(esc(traj)))-1)..., $(esc(:(k)))] )
end

macro k(traj, k)
    :( $(esc(traj))[fill(:, ndims($(esc(traj)))-1)..., $(esc(k))] )
end

macro k(traj, k, l)
    :( $(esc(traj))[fill(:, ndims($(esc(traj)))-1)..., $(esc(k)):$(esc(l))] )
end

macro first(traj)
    :( @k($(esc(traj)), 1) )
end

macro last(traj)
    :( @k($(esc(traj)), size($(esc(traj)))[end]) )
end

""" Structure holding the SCvx algorithm parameters."""
struct Parameters <: SCPParameters
    N::Int          # Number of temporal grid nodes
    Nsub::Int       # Number of subinterval integration time nodes
    iter_max::Int   # Maximum number of iterations
    disc_method::DiscretizationType # The discretization method
    λ::Real         # Virtual control weight
    ρ_0::Real       # Trust region update threshold (lower, bad solution)
    ρ_1::Real       # Trust region update threshold (middle, OK solution)
    ρ_2::Real       # Trust region update threshold (upper, good solution)
    β_sh::Real      # Trust region shrinkage factor
    β_gr::Real      # Trust region growth factor
    η_init::Real    # Initial trust region radius
    η_lb::Real      # Minimum trust region radius
    η_ub::Real      # Maximum trust region radius
    ε_abs::Real     # Absolute convergence tolerance
    ε_rel::Real     # Relative convergence tolerance
    feas_tol::Real  # Dynamic feasibility tolerance
    q_tr::Real      # Trust region norm (possible: 1, 2, 4 (2^2), Inf)
    q_exit::Real    # Stopping criterion norm
    solver::Module    # The numerical solver to use for the subproblems
    solver_opts::Dict{String, Any} # Numerical solver options
end

""" SCvx subproblem solution."""
mutable struct SubproblemSolution <: SCPSubproblemSolution
    iter::Int          # SCvx iteration number
    # >> Discrete-time rajectory <<
    xd::RealMatrix     # States
    ud::RealMatrix     # Inputs
    p::RealVector      # Parameter vector
    # >> Virtual control terms <<
    vd::RealMatrix     # Dynamics virtual control
    vs::RealMatrix     # Nonconvex constraints virtual control
    vic::RealVector    # Initial conditions virtual control
    vtc::RealVector    # Terminal conditions virtual control
    P::RealVector      # Virtual control penalty integrand terms
    Pf::RealVector     # Boundary condition virtual control penalty
    # >> Cost values <<
    L::Real            # The original cost
    L_pen::Real        # The virtual control penalty
    L_aug::Real        # Overall cost
    J_aug::Real        # Overall cost for nonlinear problem
    act_improv::Real   # Actual cost improvement
    pre_improv::Real   # Predicted cost improvement
    # >> Trajectory properties <<
    status::ST.ExitStatus # Numerical optimizer exit status
    feas::Bool         # Dynamic feasibility flag
    defect::RealMatrix # "Defect" linearization accuracy metric
    deviation::Real    # Deviation from reference trajectory
    unsafe::Bool       # Indicator that the solution is unsafe to use
    ρ::Real            # Convexification performance metric
    tr_update::String  # Indicator of growth direction for trust region
    reject::Bool       # Indicator whether SCvx rejected this solution
    dyn::DLTV          # The dynamics
end

""" Subproblem definition in JuMP format for the convex numerical optimizer."""
mutable struct Subproblem <: SCPSubproblem
    iter::Int          # SCvx iteration number
    prg::ConicProgram    # The optimization problem object
    algo::String       # SCP and convex algorithms used
    # >> Algorithm parameters <<
    def::SCPProblem      # The SCvx problem definition
    η::Real            # Trust region radius
    # >> Reference and solution trajectories <<
    sol::Union{SubproblemSolution, Missing} # Solution trajectory
    ref::Union{SubproblemSolution, Missing} # Reference trajectory
    # >> Cost function <<
    L::Objective       # The original convex cost function
    L_pen::Objective   # The virtual control penalty
    L_aug::Objective   # Overall cost function
    # >> Physical variables <<
    x::VarArgBlk        # Discrete-time states
    u::VarArgBlk        # Discrete-time inputs
    p::VarArgBlk        # Parameters
    # >> Virtual control (never scaled) <<
    vd::VarArgBlk       # Dynamics virtual control
    vs::OptVarArgBlk    # Nonconvex constraints virtual control
    vic::OptVarArgBlk   # Initial conditions virtual control
    vtc::OptVarArgBlk   # Terminal conditions virtual control
    # >> Other variables <<
    P::VarArgBlk   # Virtual control penalty
    Pf::VarArgBlk  # Boundary condition virtual control penalty
    # >> Statistics <<
    nvar::Int                    # Total number of decision variables
    ncons::Dict{Symbol, Any}     # Number of constraints
    timing::Dict{Symbol, Real} # Runtime profiling
end

"""
    SCvxProblem(pars, traj)

Construct the SCvx problem definition.

# Arguments
- `pars`: SCvx algorithm parameters.
- `traj`: the underlying trajectory optimization problem.

# Returns
- `pbm`: the problem structure ready for being solved by SCvx.
"""
function create(pars::Parameters,
                     traj::TrajectoryProblem)::SCPProblem

    default_columns = [
        # Iteration count
        (:iter, "k", "%d", 2),
        # Solver status
        (:status, "status", "%s", 8),
        # Maximum dynamics virtual control element
        (:maxvd, "vd", "%.0e", 5),
        # Maximum constraints virtual control element
        (:maxvs, "vs", "%.0e", 5),
        # Maximum boundary conditions virtual control element
        (:maxvbc, "vbc", "%.0e", 5),
        # Overall cost (including penalties)
        (:cost, "J", "%.2e", 9),
        # Maximum deviation in state
        (:dx, "Δx", "%.0e", 5),
        # Maximum deviation in input
        (:du, "Δu", "%.0e", 5),
        # Maximum deviation in input
        (:dp, "Δp", "%.0e", 5),
        # Stopping criterion deviation measurement
        (:δ, "δ", "%.0e", 5),
        # Dynamic feasibility flag (true or false)
        (:dynfeas, "dyn", "%s", 3),
        # Trust region size
        (:tr, "η", "%.2f", 5),
        # Convexification performance metric
        (:ρ, "ρ", "%s", 9),
        # Predicted cost improvement (percent)
        (:pre_improv, "J-L %", "%.2f", 9),
        # Update direction for trust region radius (grow? shrink?)
        (:dtr, "Δη", "%s", 3),
        # Reject solution indicator
        (:rej, "rej", "%s", 5)]

    # User-defined extra columns
    user_columns = [tuple(col[1:4]...) for col in traj.table_cols]

    all_columns = [default_columns; user_columns]

    table = ST.Table(all_columns)

    pbm = SCPProblem(pars, traj, table)

    return pbm
end

"""
    Subproblem(pbm[, iter, η, ref])

Constructor for an empty convex optimization subproblem. No cost or
constraints. Just the decision variables and empty associated parameters.

# Arguments
- `pbm`: the SCvx problem being solved.
- `iter`: (optional) SCvx iteration number.
- `η`: (optional) the trust region radius.
- `ref`: (optional) the reference trajectory.

# Returns
- `spbm`: the subproblem structure.
"""
function Subproblem(pbm::SCPProblem,
                        iter::Int=0,
                        η::Real=1.0,
                        ref::Union{SubproblemSolution,
                                   Missing}=missing)::Subproblem

    # Statistics
    timing = Dict(:formulate => get_time(), :total => get_time())
    nvar = 0
    ncons = Dict()

    # Convenience values
    pars = pbm.pars
    scale = pbm.common.scale
    nx = pbm.traj.nx
    nu = pbm.traj.nu
    np = pbm.traj.np
    N = pbm.pars.N
    _E = pbm.common.E

    # Optimization problem handle
    solver = pars.solver
    solver_opts = pars.solver_opts
    prg = ConicProgram(pbm.traj;
                       solver=solver.Optimizer,
                       solver_options=solver_opts)
    cvx_algo = string(pars.solver)
    algo = @sprintf("SCvx (backend: %s)", cvx_algo)

    sol = missing # No solution associated yet with the subproblem

    # Cost
    L = missing
    L_pen = missing
    L_aug = missing

    # Decision variables (scaled)
    x = @new_variable(prg, (nx, N), "x")
    u = @new_variable(prg, (nu, N), "u")
    p = @new_variable(prg, np, "p")
    Sx = diag(scale.Sx)
    Su = diag(scale.Su)
    Sp = diag(scale.Sp)
    @scale(x, Sx, scale.cx)
    @scale(u, Su, scale.cu)
    @scale(p, Sp, scale.cp)

    # Virtual controls
    vd = @new_variable(prg, (size(_E, 2), N-1), "vd")
    vs = nothing
    vic = nothing
    vtc = nothing

    # Other variables
    P = @new_variable(prg, N, "P")
    Pf = @new_variable(prg, 2, "Pf")

    spbm = Subproblem(iter, prg, algo, pbm, η, sol, ref, L, L_pen, L_aug,
                        x, u, p, vd, vs, vic, vtc, P, Pf, nvar,
                          ncons, timing)

    return spbm
end

"""
    SubproblemSolution(x, u, p, iter, pbm)

Construct a subproblem solution from a discrete-time trajectory. This leaves
parameters of the solution other than the passed discrete-time trajectory
unset.

# Arguments
- `x`: discrete-time state trajectory.
- `u`: discrete-time input trajectory.
- `p`: parameter vector.
- `iter`: SCvx iteration number.
- `pbm`: the SCvx problem definition.

# Returns
- `subsol`: subproblem solution structure.
"""
function SubproblemSolution(
    x::RealMatrix,
    u::RealMatrix,
    p::RealVector,
    iter::Int,
    pbm::SCPProblem)::SubproblemSolution

    # Parameters
    N = pbm.pars.N
    nx = pbm.traj.nx
    nu = pbm.traj.nu
    np = pbm.traj.np
    nv = size(pbm.common.E, 2)
    disc = pbm.pars.disc_method

    # Uninitialized parts
    status = MOI.OPTIMIZE_NOT_CALLED
    feas = false
    defect = fill(NaN, nx, N-1)
    deviation = NaN
    unsafe = false
    ρ = NaN
    tr_update = ""
    reject = false
    dyn = DLTV(nx, nu, np, nv, N, disc)

    vd = RealMatrix(undef, 0, N)
    vs = RealMatrix(undef, 0, N)
    vic = RealVector(undef, 0)
    vtc = RealVector(undef, 0)
    P = zeros(N)
    Pf = zeros(2)

    L = NaN
    L_pen = NaN
    L_aug = NaN
    J_aug = NaN
    act_improv = NaN
    pre_improv = NaN

    subsol = SubproblemSolution(iter, x, u, p, vd, vs, vic, vtc, P, Pf, L,
                                    L_pen, L_aug, J_aug, act_improv,
                                    pre_improv, status, feas, defect,
                                    deviation, unsafe, ρ, tr_update, reject,
                                    dyn)

    # Compute the DLTV dynamics around this solution
    discretize!(subsol, pbm)

    # Nonlinear cost along this trajectory
    _scvx__solution_cost!(subsol, :nonlinear, pbm)

    return subsol
end

"""
    Signature

Construct subproblem solution from a subproblem object. Expects that the
subproblem argument is a solved subproblem (i.e. one to which numerical
optimization has been applied).

# Arguments
- `spbm`: the subproblem structure.

# Returns
- `sol`: subproblem solution.
"""
function SubproblemSolution(spbm::Subproblem)::SubproblemSolution
    # Extract the discrete-time trajectory
    x = value(spbm.x)
    u = value(spbm.u)
    p = value(spbm.p)

    # Form the partly uninitialized subproblem
    sol = SubproblemSolution(x, u, p, spbm.iter, spbm.def)

    # Save the virtual control values and penalty terms
    sol.vd = value(spbm.vd)
    if !isnothing(spbm.vs)
        sol.vs = value(spbm.vs)
    end
    if !isnothing(spbm.vic)
        sol.vic = value(spbm.vic)
    end
    if !isnothing(spbm.vtc)
        sol.vtc = value(spbm.vtc)
    end
    
    sol.P = value(spbm.P)
    sol.Pf = value(spbm.Pf)

    # Save the optimal cost values
    sol.L = value(spbm.L)
    sol.L_pen = value(spbm.L_pen)
    sol.L_aug = value(spbm.L_aug)

    return sol
end

"""
    scvx_solve(pbm)

Apply the SCvx algorithm to solve the trajectory generation problem.

# Arguments
- `pbm`: the trajectory problem to be solved.

# Returns
- `sol`: the SCvx solution structure.
- `history`: SCvx iteration data history.
"""
function solve(pbm::SCPProblem,
                warm::Union{Nothing, SCPSolution}=nothing)::Tuple{
                    Union{SCPSolution, Nothing},
                    SCPHistory}
    # ..:: Initialize ::..

    η = pbm.pars.η_init
    if isnothing(warm)
        ref = generate_initial_guess(pbm)
    else
        ref = warm_start(pbm, warm)
    end

    history = SCPHistory()

    callback_fun! = pbm.traj.callback!
    user_callback = !isnothing(callback_fun!)

    # ..:: Iterate ::..

    k = 1 # Iteration counter
    while true
        # >> Construct the subproblem <<
        spbm = Subproblem(pbm, k, η, ref)

        add_dynamics!(spbm)
        add_convex_state_constraints!(spbm)
        add_convex_input_constraints!(spbm)
        add_nonconvex_constraints!(spbm)
        add_bcs!(spbm)
        add_trust_region!(spbm)
        add_cost!(spbm)

        save!(history, spbm)

        try
            # >> Solve the subproblem <<
            solve_subproblem!(spbm)

            # "Emergency exit" the SCvx loop if something bad happened
            # (e.g. numerical problems)
            if unsafe_solution(spbm)
                print_info(spbm)
                break
            end

             # Check stopping criterion
             stop = check_stopping_criterion!(spbm)

             # Run a user-defined callback
             if user_callback
                 user_acted = callback_fun!(spbm)
             end
 
             # Stop iterating if stopping criterion triggered **and** user did
             # not modify anything in the callback
             if stop && !(user_callback && user_acted)
                 print_info(spbm)
                 break
             end

            # >> Update trust region << and reference trajectory?
            ref, η = update_trust_region(spbm)
        catch e
            isa(e, SCPError) || rethrow(e)
            print_info(spbm, e)
            break
        end

        # >> Print iteration info <<
        print_info(spbm)

        # Stop at maximum iterations
        k += 1
        if k>pbm.pars.iter_max
            break
        end
    end

    reset(pbm.common.table)

    # ..:: Save solution ::..
    sol = SCPSolution(history)

    return sol, history
end

"""
    _scvx__generate_initial_guess(pbm)

Construct the initial trajectory guess. Calls problem-specific initial guess
generator, which is converted to an SCvxSubproblemSolution structure.

# Arguments
- `pbm`: the SCvx problem structure.

# Returns
- `guess`: the initial guess.
"""
function generate_initial_guess(
    pbm::SCPProblem)::SubproblemSolution

    # Construct the raw trajectory
    x, u, p = pbm.traj.guess(pbm.pars.N)
    guess = SubproblemSolution(x, u, p, 0, pbm)

    return guess
end

"""
    warm_start(pbm, warm)

Create initial guess from a warm start solution.

# Arguments
- `pbm`: the PTR problem structure.
- `warm`: warm start solution.

# Returns
- `guess`: the initial guess for PTR.
"""
function warm_start(pbm::SCPProblem,
                          warm::SCPSolution)::SubproblemSolution

    # Extract the warm-start trajectory
    x, u, p = warm.xd, warm.ud, warm.p
    guess = SubproblemSolution(x, u, p, 0, pbm)

    return guess
end # function

"""
    _scvx__add_trust_region!(spbm)

Add trust region constraint to the subproblem.

# Arguments
- `spbm`: the subproblem definition.
"""
function add_trust_region!(spbm::Subproblem)::Nothing

    # Variables and parameters
    N = spbm.def.pars.N
    q = spbm.def.pars.q_tr
    scale = spbm.def.common.scale
    prg = spbm.prg
    x = spbm.x
    u = spbm.u
    p = spbm.p
    xh_ref = scale.iSx*(spbm.ref.xd.-scale.cx)
    uh_ref = scale.iSu*(spbm.ref.ud.-scale.cu)
    ph_ref = scale.iSp*(spbm.ref.p-scale.cp)
    # nx = spbm.def.traj.nx # this is no. of states doofus
    # nu = spbm.def.traj.nu
    # np = spbm.def.traj.np
    η = spbm.η

    q2cone = Dict(1 => L1, 2 => SOC, 4 => SOC, Inf => LINF)
    cone = q2cone[q]

    # >> Parameter trust region <<
    dp_lq = @new_variable(prg, "dp_lq")

    @add_constraint(prg, cone, "parameter_trust_region",
                    (p, dp_lq), begin
                        local p, dp_lq = arg #noerr
                        local ph = scale.iSp*(p-scale.cp)
                        local dp = ph-ph_ref
                        vcat(dp_lq, dp)
                    end)

    # State trust regions
    dx_lq = @new_variable(prg, N, "dx_lq")

    for k = 1:N
        @add_constraint(prg, cone, "state_trust_region",
                        (x[:, k], dx_lq[k]), begin
                            local xk, dxk_lq = arg #noerr
                            local xhk = scale.iSx*(xk-scale.cx)
                            local dxk = xhk-xh_ref[:, k]
                            vcat(dxk_lq, dxk)
                        end)
    end

    # Input trust regions
    du_lq = @new_variable(prg, N, "du_lq")

    for k = 1:N
        @add_constraint(prg, cone, "input_trust_region",
                        (u[:, k], du_lq[k]), begin
                            local uk, duk_lq = arg #noerr
                            local uhk = scale.iSu*(uk-scale.cu)
                            local duk = uhk-uh_ref[:, k]
                            vcat(duk_lq, duk)
                        end)
    end

    for k = 1:N
        if q==4
            w = @new_variable(prg, "w")
            @add_constraint(prg, SOC, "trust_region",
                            (w, dp_lq), begin
                                local wp, dx_lq, du_lq, dp_lq = arg #noerr
                                vcat(wp, dx_lq, du_lq, dp_lq)
                            end)
            @add_constraint(prg, GEOM, "trust_region",
                            (w,), begin
                                local wp = arg[1] #noerr
                                vcat(wp, η, 1)
                            end)
        else
            @add_constraint(prg, NONPOS, "trust_region",
                            (dx_lq[k], du_lq[k], dp_lq), begin
                                local dx_lq, du_lq, dp_lq = arg #noerr
                                dx_lq[1] + du_lq[1] + dp_lq[1] - η #idk why its so hacky
                            end)
        end
    end

    return nothing
end

"""
    _scvx__add_cost!(spbm)

Define the subproblem cost function.

# Arguments
- `spbm`: the subproblem definition.
"""
function add_cost!(spbm::Subproblem)::Nothing

    # Compute the cost components
    compute_original_cost!(spbm)
    compute_linear_cost_penalty!(spbm)

    # Overall cost
    spbm.L_aug = cost(spbm.prg)

    return nothing
end

"""
    original_cost(x, u, p, pbm)

Compute the original problem cost function.

# Arguments
- `spbm`: the subproblem definition.

# Returns
- `cost`: the original cost.
"""
function compute_original_cost(
    x::RealMatrix,
    u::RealMatrix,
    p::RealVector,
    pbm::SCPProblem)::Objective

    # Variables and parameters
    N = pbm.pars.N
    t = pbm.common.t_grid
    traj_pbm = pbm.traj


    x_stages = [x[:, k] for k=1:N]
    u_stages = [u[:, k] for k=1:N]

    # Terminal cost
    xf = x_stages[end]
    J_term = isnothing(traj_pbm.φ) ? 0.0 :
        traj_pbm.φ(xf, p)

    # Integrated running cost
    local J_run = Vector{Objective}(undef, N)
    local ∇J_run = Vector{Dict}(undef, N)
    if !isnothing(traj_pbm.Γ)
        for k = 1:N
            local out = traj_pbm.Γ(t[k], k, x_stages[k], u_stages[k], p)
            if out isa Tuple
                J_run[k], ∇J_run[k] = out
            else
                J_run[k] = out
            end
        end
    else
        J_run[:] .= 0.0
    end
    integ_J_run = trapz(J_run, t)

    return J_term+integ_J_run
end # function

function compute_original_cost!(spbm::Subproblem)::Nothing

    # Variables and parameters
    pbm = spbm.def
    N = pbm.pars.N
    t = pbm.common.t_grid
    traj_pbm = pbm.traj
    prg = spbm.prg
    x = spbm.x
    u = spbm.u
    p = spbm.p

    x_stages = [x[:, k] for k=1:N]
    u_stages = [u[:, k] for k=1:N]

    spbm.L = @add_cost(
        prg, (x_stages..., u_stages..., p), begin
            local x = arg[1:N] #noerr
            local u = arg[(1:N).+N] #noerr
            local p = arg[end] #noerr

            # Terminal cost
            local xf = x[end]
            local J_term = isnothing(traj_pbm.φ) ? 0.0 :
                traj_pbm.φ(xf, p)

            # Integrated running cost
            local J_run = Vector{Objective}(undef, N)
            local ∇J_run = Vector{Dict}(undef, N)
            if !isnothing(traj_pbm.Γ)
                for k = 1:N
                    local out = traj_pbm.Γ(t[k], k, x[k], u[k], p)
                    if out isa Tuple
                        J_run[k], ∇J_run[k] = out
                    else
                        J_run[k] = out
                    end
                end
            else
                J_run[:] .= 0.0
            end
            local integ_J_run = trapz(J_run, t)

            J_term+integ_J_run
        end)

    return nothing
end # function

"""
    _scvx__P(vd, vs)

Compute cost penalty at a particular instant.

This is the integrand of the overall cost penalty term for dynamics and
nonconvex constraint violation.

Note: **this function must match the penalty implemented in
_scvx__add_cost!()**.

# Arguments
- `vd`: inconsistency in the dynamics ("defect").
- `vs`: inconsistency in the nonconvex inequality constraints.

# Returns
- `P`: the penalty value.
"""
function _scvx__P(vd::RealVector, vs::RealVector)::Real
    P = norm(vd, 1)+norm(vs, 1)
    return P
end

"""
    _scvx__compute_linear_cost_penalty!(spbm)

Compute the subproblem cost virtual control penalty term.

# Arguments
- `spbm`: the subproblem definition.
"""
function compute_linear_cost_penalty!(spbm::Subproblem)::Nothing
    # Variables and parameters
    N = spbm.def.pars.N
    λ = spbm.def.pars.λ
    t = spbm.def.common.t_grid
    E = spbm.ref.dyn.E
    prg = spbm.prg
    P = spbm.P
    Pf = spbm.Pf
    vd = spbm.vd
    vs = spbm.vs
    vic = spbm.vic
    vtc = spbm.vtc

    # Compute virtual control penalty
    if !isnothing(vs)
        for k = 1:N
            if k<N
                @add_constraint(prg, L1, "vd_vs_penalty",
                                (P[k], vd[:, k], vs[:, k]), begin
                                    local Pk, vdk, vsk = arg #noerr
                                    vcat(Pk, E[:, :, k]*vdk, vsk)
                                end)
            else
                @add_constraint(prg, L1, "vd_vs_penalty",
                                (P[k], vs[:, k]), begin
                                    local Pk, vsk = arg #noerr
                                    vcat(Pk, vsk)
                                end)
            end
        end
    else
        for k = 1:N
            if k<N
                @add_constraint(prg, L1, "vd_vs_penalty",
                                (P[k], vd[:, k]), begin
                                    local Pk, vdk = arg #noerr
                                    vcat(Pk, E[:, :, k]*vdk)
                                end)
            else
                @add_constraint(
                    prg, ZERO, "vd_vs_penalty",
                    (P[k],), begin
                        local Pk, = arg #noerr
                        Pk
                    end)
            end
        end
    end

    if !isnothing(vic)
        @add_constraint(prg, L1, "vic_penalty",
                        (Pf[1], vic), begin
                            local Pf1, vic = arg #noerr
                            vcat(Pf1, vic)
                        end)
    else
        @add_constraint(prg, ZERO, "vic_penalty",
                        (Pf[1]), begin
                            local Pf1, = arg #noerr
                            Pf1
                        end)
    end

    if !isnothing(vtc)
        @add_constraint(prg, L1, "vtc_penalty",
                        (Pf[2], vtc), begin
                            local Pf2, vtc = arg #noerr
                            vcat(Pf2, vtc)
                        end)
    else
        @add_constraint(prg, ZERO, "vtc_penalty",
                        (Pf[2]), begin
                            local Pf2, = arg #noerr
                            Pf2
                        end)
    end

    spbm.L_pen = @add_cost(
        prg, (P, Pf), begin
            local P, Pf = arg #noerr
            trapz(λ*P, t)+sum(λ*Pf)
        end)

    return nothing
end

"""
    _scvx__actual_cost_penalty!(sol, pbm)

Compute the subproblem cost penalty based on actual constraint violation.

This computes the same cost penalty form as in the subproblem. However, instead
of using virtual control terms, it uses defects from nonlinear propagation of
the dynamics and the actual values of the nonconvex inequality constraints.

If the subproblem solution has already had this function called for it,
re-computation is skipped and the already computed value is returned.

# Arguments
- `sol`: the subproblem solution.
- `pbm`: the SCvx problem definition.
- `safe`: (optional) whether to check that the coded penalty function matches
  the optimization.

# Returns
- `pen`: the nonlinear cost penalty term.
"""
function _scvx__actual_cost_penalty!(
    sol::SubproblemSolution,
    pbm::SCPProblem)::Real

    # Values and parameters from the solution
    N = pbm.pars.N
    λ = pbm.pars.λ
    nx = pbm.traj.nx
    t = pbm.common.t_grid
    x = sol.xd
    u = sol.ud
    p = sol.p

    # Values from the solution
    δ = sol.defect
    gic = pbm.traj.gic(@first(sol.xd), sol.p)
    gtc = pbm.traj.gtc(@last(sol.xd), sol.p)

    # Integrate the nonlinear penalty term
    P = RealVector(undef, N)
    for k = 1:N
        δk = (k<N) ? @k(δ) : zeros(nx)
        sk = isnothing(pbm.traj.s) ? [0.0] : pbm.traj.s(@k(t), k, @k(x), @k(u), sol.p)
        sk = isempty(sk) ? [0.0] : sk
        @k(P) = λ*_scvx__P(δk, max.(sk, 0.0))
    end
    pen = trapz(P, t)+λ*(_scvx__P([0.0], gic)+_scvx__P([0.0], gtc))

    return pen
end

"""
    _scvx__solution_cost!(sol, kind, pbm)

Compute the linear or nonlinear overall associated with a solution.

# Arguments
- `sol`: the subproblem solution structure.
- `kind`: whether to compute the linear or nonlinear problem cost.
- `pbm`: the SCvx problem definition.

# Returns
- `cost`: the optimal cost associated with this solution.
"""
function _scvx__solution_cost!(
    sol::SubproblemSolution,
    kind::Symbol,
    pbm::SCPProblem)::Real

    if isnan(sol.L)
        sol.L = compute_original_cost(sol.xd, sol.ud, sol.p, pbm)
    end

    if kind==:linear
        cost = sol.L
    else
        if isnan(sol.J_aug)
            J_orig = sol.L
            J_pen = _scvx__actual_cost_penalty!(sol, pbm)
            sol.J_aug = J_orig+J_pen
        end
        cost = sol.J_aug
    end

    return cost
end

"""
    _scvx__add_cost!(spbm)

Check if stopping criterion is triggered.

# Arguments
- `spbm`: the subproblem definition.

# Returns
- `stop`: true if stopping criterion holds.
"""
function check_stopping_criterion!(spbm::Subproblem)::Bool

    # Extract values
    pbm = spbm.def
    ref = spbm.ref
    sol = spbm.sol
    ε_abs = pbm.pars.ε_abs
    ε_rel = pbm.pars.ε_rel

    # Compute solution deviation from reference
    sol.deviation = solution_deviation(spbm)

    # Check predicted cost improvement
    J_ref = _scvx__solution_cost!(ref, :nonlinear, pbm)
    L_sol = _scvx__solution_cost!(sol, :linear, pbm)
    sol.pre_improv = J_ref-L_sol
    pre_improv_rel = sol.pre_improv/abs(J_ref)

    # Compute stopping criterion
    stop = (spbm.iter>1 &&
            (sol.feas && (pre_improv_rel<=ε_rel || sol.deviation<=ε_abs)))

    return stop
end

"""
    update_trust_region(spbm)

Compute the new reference and trust region. Apply the trust region update rule
based on the most recent subproblem solution. This updates the trust region
radius, and selects either the current or the reference solution to act as the
next iteration's reference trajectory.

# Arguments
- `spbm`: the subproblem definition.

# Returns
- `next_ref`: reference trajectory for the next iteration.
- `next_η`: trust region radius for the next iteration.
"""
function update_trust_region(
    spbm::Subproblem)::Tuple{SubproblemSolution,
                                 Real}

    # Parameters
    pbm = spbm.def
    sol = spbm.sol
    ref = spbm.ref

    # Compute the actual cost improvement
    J_ref = _scvx__solution_cost!(ref, :nonlinear, pbm)
    J_sol = _scvx__solution_cost!(sol, :nonlinear, pbm)
    sol.act_improv = J_ref-J_sol

    # Convexification performance metric
    sol.ρ = sol.act_improv/sol.pre_improv

    # Apply update rule
    next_ref, next_η = update_rule(spbm)

    return next_ref, next_η
end

"""
    update_rule(spbm)

Apply the low-level SCvx trust region update rule. This computes the new trust
region value and reference trajectory, based on the obtained subproblem
solution.

# Arguments
- `spbm`: the subproblem definition.

# Returns
- `next_ref`: reference trajectory for the next iteration.
- `next_η`: trust region radius for the next iteration.
"""
function update_rule(
    spbm::Subproblem)::Tuple{SubproblemSolution,
                                 Real}
    # Extract relevant data
    pars = spbm.def.pars
    sol = spbm.sol
    ref = spbm.ref
    ρ = sol.ρ
    ρ0 = pars.ρ_0
    ρ1 = pars.ρ_1
    ρ2 = pars.ρ_2
    β_sh = pars.β_sh
    β_gr = pars.β_gr
    η_lb = pars.η_lb
    η_ub = pars.η_ub
    η = spbm.η

    # Apply update logic
    # Prediction below means "prediction of cost improvement by the linearized
    # model"
    if ρ<ρ0
        # Very poor prediction
        next_η = max(η_lb, η/β_sh)
        next_ref = ref
        sol.tr_update = "S"
        sol.reject = true
    elseif ρ0<=ρ && ρ<ρ1
        # Mediocre prediction
        next_η = max(η_lb, η/β_sh)
        next_ref = sol
        sol.tr_update = "S"
        sol.reject = false
    elseif ρ1<=ρ && ρ<ρ2
        # Good prediction
        next_η = η
        next_ref = sol
        sol.tr_update = ""
        sol.reject = false
    else
        # Excellent prediction
        next_η = min(η_ub, β_gr*η)
        next_ref = sol
        sol.tr_update = "G"
        sol.reject = false
    end

    return next_ref, next_η
end

"""
    mark_unsafe!(sol, err)

Mark a solution as unsafe to use.

# Arguments
- `sol`: subproblem solution.
- `err`: the SCvx error that occurred.
"""
function mark_unsafe!(sol::SubproblemSolution,
                             err::SCPError)::Nothing
    sol.status = err.status
    sol.unsafe = true
    return nothing
end

"""
    print_info(spbm[, err])

Print command line info message.

# Arguments
- `spbm`: the subproblem that was solved.
- `err`: an SCvx-specific error message.
"""
function print_info(spbm::Subproblem,
                           err::Union{Nothing, SCPError}=nothing)::Nothing

    # Convenience variables
    sol = spbm.sol
    traj = spbm.def.traj
    ref = spbm.ref
    table = spbm.def.common.table

    if !isnothing(err)
        @printf "%s, exiting\n" err.msg
    elseif unsafe_solution(sol)
        @printf "unsafe solution (%s), exiting\n" sol.status
    else
        # Preprocess values
        scale = spbm.def.common.scale
        xh = scale.iSx*(sol.xd.-scale.cx)
        uh = scale.iSu*(sol.ud.-scale.cu)
        ph = scale.iSp*(sol.p-scale.cp)
        xh_ref = scale.iSx*(spbm.ref.xd.-scale.cx)
        uh_ref = scale.iSu*(spbm.ref.ud.-scale.cu)
        ph_ref = scale.iSp*(spbm.ref.p-scale.cp)
        max_dxh = norm(xh-xh_ref, Inf)
        max_duh = norm(uh-uh_ref, Inf)
        max_dph = norm(ph-ph_ref, Inf)
        E = spbm.def.common.E
        status = @sprintf "%s" sol.status
        status = status[1:min(8, length(status))]
        ρ = !isnan(sol.ρ) ? @sprintf("%.2f", sol.ρ) : ""
        ρ = (length(ρ)>8) ? @sprintf("%.1e", sol.ρ) : ρ

        # Associate values with columns
        assoc = Dict(:iter => spbm.iter,
                     :status => status,
                     :maxvd => norm(sol.vd, Inf),
                     :maxvs => norm(sol.vs, Inf),
                     :maxvbc => norm([sol.vic; sol.vtc], Inf),
                     :cost => sol.J_aug,
                     :dx => max_dxh,
                     :du => max_duh,
                     :dp => max_dph,
                     :dynfeas => sol.feas ? "T" : "F",
                     :δ => sol.deviation,
                     :ρ => ρ,
                     :pre_improv => sol.pre_improv/abs(ref.J_aug)*100,
                     :dtr => sol.tr_update,
                     :rej => sol.reject ? "x" : "",
                     :tr => spbm.η)
        
        # Set user-defined columns
        for col in traj.table_cols
            id, col_value = col[1], col[end]
            assoc[id] = col_value(sol.bay)
        end

        print(assoc, table)
    end

    overhead!(spbm)

    return nothing
end
