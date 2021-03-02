#= Re-entry Guidance Problem.

Solution via Sequential Convex Programming using the SCvx algorithm. =#

using Plots

include("core/problem.jl")
include("models/reentry.jl")

###############################################################################
# ..:: Define trajectory problem data ::..

# >> Reentry vehicle <<
id_r = 1:2
veh = ReentryVehicleParameters(id_r)

# >> Reentry environment <<
ρ = 1.0
env = ReentryEnvironmentParameters(ρ)

# >> Trajectory <<
tf = 360.0
traj = ReentryTrajectoryParameters(tf)

###############################################################################

###############################################################################
# ..:: Define the trajectory optimization problem ::..

mdl = ReentryProblem(veh, env, traj)

pbm = TrajectoryProblem(mdl)

# Variable dimensions
problem_set_dims!(pbm, STATE_DIM, INPUT_DIM, PAR_DIM)

# Initial trajectory guess
problem_set_guess!(pbm,
                   (N, pbm) -> begin
                   x = missing # TODO [n x N] matrix
                   u = missing # TODO [m x N] matrix
                   p = missing # TODO [p] vector
                   return x, u, p
                   end)

# Cost to be minimized
problem_set_cost!(pbm,
                  # Terminal cost
                  (x, p, pbm) -> begin
                  φ = missing # TODO
                  return φ
                  end,
                  # Running cost
                  (x, u, p, pbm) -> begin
                  Γ = missing # TODO
                  return Γ
                  end)

# Dynamics constaint
problem_set_dynamics!(pbm,
                      # Dynamics f
                      (τ, x, u, p, pbm) -> begin
                      mdl = pbm.mdl
                      tf = mdl.traj.tf
                      f = missing # TODO
                      f *= tf
                      return f
                      end,
                      # Jacobian df/dx
                      (τ, x, u, p, pbm) -> begin
                      mdl = pbm.mdl
                      tf = mdl.traj.tf
                      A = missing # TODO
                      A *= tf
                      return A
                      end,
                      # Jacobian df/du
                      (τ, x, u, p, pbm) -> begin
                      mdl = pbm.mdl
                      tf = mdl.traj.tf
                      B = missing # TODO
                      B *= tf
                      return B
                      end,
                      # Jacobian df/dp
                      (τ, x, u, p, pbm) -> begin
                      mdl = pbm.mdl
                      tf = mdl.traj.tf
                      F = missing # TODO
                      F *= tf
                      return F
                      end)

# Convex path constraints on the state
problem_set_X!(pbm, (x, mdl, pbm) -> begin
               X = [@constraint(mdl, x[1] <= 0.0)] # TODO
               return X
               end)

# Convex path constraints on the input
problem_set_U!(pbm, (u, mdl, pbm) -> begin
               # norm(x, 2) <= 2
               # SOC: { (t, x) : norm(x, 2) <= t}
               # @constraint(mdl, vcat(2, x) in MOI.SecondOrderCone(1+n))
               U = [@constraint(mdl, u[1] <= 0.0)] # TODO
               return U
               end)

# Nonconvex path inequality constraints
problem_set_s!(pbm,
               # Constraint s
               (x, u, p, pbm) -> begin
               s = missing # TODO
               # ρ(x[1:2])*norm(x[3:4])^3-p[1]
               return s
               end,
               # Jacobian ds/dx
               (x, u, p, pbm) -> begin
               C = missing # TODO
               return C
               end,
               # Jacobian ds/du
               (x, u, p, pbm) -> begin
               D = missing # TODO
               return D
               end,
               # Jacobian ds/dp
               (x, u, p, pbm) -> begin
               G = missing # TODO
               return G
               end)

# Initial boundary conditions
problem_set_bc!(pbm, :ic,
                # Constraint g
                (x, p, pbm) -> begin
                g_ic = 0.0 # TODO
                return g_ic
                end,
                # Jacobian dg/dx
                (x, p, pbm) -> begin
                H = 0.0 # TODO
                return H
                end,
                # Jacobian dg/dp
                (x, p, pbm) -> begin
                K = 0.0 # TODO
                return K
                end)

# Terminal boundary conditions
problem_set_bc!(pbm, :tc,
                # Constraint g
                (x, p, pbm) -> begin
                g_tc = 0.0 # TODO
                return g_tc
                end,
                # Jacobian dg/dx
                (x, p, pbm) -> begin
                H = 0.0 # TODO
                return H
                end,
                # Jacobian dg/dp
                (x, p, pbm) -> begin
                K = 0.0 # TODO
                return K
                end)

###############################################################################

###############################################################################
# ..:: Define the SCvx algorithm parameters ::..

N = 30
Nsub = 15
iter_max = 20
λ = 1e3
ρ_0 = 0.0
ρ_1 = 0.1
ρ_2 = 0.7
β_sh = 2.0
β_gr = 2.0
η_init = 1.0
η_lb = 1e-3
η_ub = 10.0
ε_abs = 0.0#1e-4
ε_rel = 0.01/100
feas_tol = 1e-3
q_tr = Inf
q_exit = Inf
solver = ECOS
solver_options = Dict("verbose"=>0)
pars = SCvxParameters(N, Nsub, iter_max, λ, ρ_0, ρ_1, ρ_2, β_sh, β_gr,
                      η_init, η_lb, η_ub, ε_abs, ε_rel, feas_tol, q_tr,
                      q_exit, solver, solver_options)

###############################################################################

###############################################################################
# ..:: Solve trajectory generation problem using SCvx ::..

scvx_pbm = SCvxProblem(pars, pbm)
sol, history = scvx_solve(scvx_pbm)

###############################################################################

###############################################################################
# ..:: Plot results ::..

pyplot()
plot_final_trajectory(mdl, sol) # TODO

###############################################################################
