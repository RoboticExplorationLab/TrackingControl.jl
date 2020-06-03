# Test Tracking Controller Files
# Test Interface

# TO DO; Adding Proper Test anc Checks using @Test

using StaticArrays
using LinearAlgebra
using ForwardDiff
using TrajectoryOptimization    # used for AbstractModel type
using Plots

const TO = TrajectoryOptimization

include("knotpoint.jl")
include("trajectories.jl")           # needs knotpoint.jl
include("rbstate.jl")
include("tracking_control.jl")       # needs rbstate.jl

# Test on Dubin's Car
struct DubinsCar{T} <: AbstractModel where{T}
    param::T
end

function TO.dynamics(model::DubinsCar, x, u)
    SA[u[1]*cos(x[3]),u[1]*sin(x[3]),u[2]]
end

Base.size(::DubinsCar) = 3,2

# Model and Discretization
model = DubinsCar(0.0)
n,m = size(model)
tf = 10.0  # sec
N = 51    # number of knot points

# Objective
x0 = SA[0.,0.,π/2]  # initial state
xf = SA[10.,5.0,π/2]  # final state

ρ = 1.0
Q = Diagonal(@SVector ones(n))
R = ρ*Diagonal(@SVector ones(m))
obj = LQRObjective(Q, R, N*Q, xf, N)

# Constraints
cons = TO.ConstraintSet(n,m,N)
add_constraint!(cons, GoalConstraint(xf), N:N)
add_constraint!(cons, BoundConstraint(n,m, u_min=-1, u_max=1), 1:N-1)

# Create and solve problem
prob = Problem(model, obj, xf, tf, x0=x0, constraints=cons)
solver = ALTROSolver(prob)
cost(solver)           # initial cost
solve!(solver)         # solve with ALTRO
max_violation(solver)  # max constraint violation
cost(solver)           # final cost
iterations(solver)     # total number of iterations

# Get the state and control trajectories
X = TO.states(solver)
U = TO.controls(solver)

# Plot ALTRO Results
plot([U[i][1] for i=1:N-1])   # First component of control input
plot!([U[i][2] for i=1:N-1])  # Second component of control input
plot([X[i][1] for i=1:N],[X[i][2] for i=1:N])  # Position of Dubins Car in 2D plane

Z = Traj(prob.Z)

typeof(prob.Z)
isa(prob.Z[1], AbstractKnotPoint)

Q = Matrix(Diagonal(ones(n)))
R = Matrix(Diagonal(ones(m)))

tracking_controller = TVLQR(model, Q, R, Z)

isa(model, AbstractModel)
