# Test on the Module Itself

#Pkg.add(PackageSpec(name="RobotDynamics", rev="master"))

# TO DO:
# Use TrajOpt and RobotDynamis last versions on last
# Right now incompatible because of Rotations package
# TO uses Rotations v0.13
# RobotDynamics uses Rotations v.1

# ATTENTION: Work in proper environment
using RobotDynamics
using StaticArrays
using TrajectoryOptimization
using LinearAlgebra

include("TrackingControl.jl")

const TC = TrackingControl

#const TO = TrajectoryOptimization

struct DubinsCar{T} <: TO.AbstractModel where{T}
    param::T
end

function TO.dynamics(model::DubinsCar, x, u)   # because we use "using" so need to add TO.dynamics to extend dynamics function from TO module
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

# Put into tracking controller (using TVLQR)
# Constructor Traj(X, U, dt)

Z = Traj(X, U, [prob.Z[i].dt for i=1:length(prob.Z)])
