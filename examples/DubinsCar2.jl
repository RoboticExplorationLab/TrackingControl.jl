# Example File for TVLQR on Dubin's Car
# Use reference trajectory generated using TrajectoryOptimization.jl

#STATE:
# NOT WORKING YET
#TO DO:
#Add TrajectoryOptimization master along with RobotDynamics.jl master

# Generate Optimal Trajectory using Trajopt ####################################
################################################################################

cd("C:\\Users\\33645\\Documents\\Stanford\\AA290ZAC - RA Fall 2019\\github\\TrackingControl.jl")

using TrajectoryOptimization
using StaticArrays
using LinearAlgebra
const TO = TrajectoryOptimization
using Plots

# Define Dubin's car Dynamics (Add suitable parameters to model later)
struct DubinsCar{T} <: AbstractModel
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
N = 101    # number of knot points

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
add_constraint!(cons, BoundConstraint(n,m, u_min=-1e10, u_max=1e10), 1:N-1)

# Create and solve problem
prob = Problem(model, obj, xf, tf, x0=x0, constraints=cons)
solver = ALTROSolver(prob)
cost(solver)           # initial cost
solve!(solver)         # solve with ALTRO
max_violation(solver)  # max constraint violation
cost(solver)           # final cost
iterations(solver)     # total number of iterations

# Get the state and control trajectories
X = states(solver)
U = controls(solver)

# Plot ALTRO Results
plot([U[i][1] for i=1:N-1])   # First component of control input
plot!([U[i][2] for i=1:N-1])  # Second component of control input
plot([X[i][1] for i=1:N],[X[i][2] for i=1:N])  # Position of Dubins Car in 2D plane

# Generate Trajectory Type for Reference Trajectory
Z = Traj(X, U, [prob.Z[i].dt for i=1:length(prob.Z)])

# Track Reference Trajectory using LQR #########################################
################################################################################

#using TrackingControl
#const TC = TrackingControl
include("..\\src\\TrackingControl.jl")

# Define TVLQR Cost Matrices
ρ = 1.0
Q = Diagonal(@SVector ones(n))
R = ρ*Diagonal(@SVector ones(m))

# Define TVLQR controller on model
tvlqr_cntrl = TC.TVLQR(model, Q, R, Z)

# Extract the Tracking trajectory for a given initial condition
function get_tracking_trajectory(cntrl::TVLQR, model, x0, dt)
    # Get tracking trajectory from TVLQR
    # Starting at position x0
    t = cntrl.t
    t0 = t[1]
    u0 = get_control(cntrl, x0, t0)
    Z_track = [KnotPoint(x0, u0, dt, t0)]
    for i=2:length(t)
        x_next = TC.discrete_dynamics(model, Z_track[end])
        u = get_control(cntrl, x_next, t[i])
        z_next = KnotPoint(x_next, u, dt, t[i])
        push!(Z_track, z_next)
    end
    return TC.Traj(Z_track)
end

# Define Initial Condition
x_start = [-1.0;0.0;π/2]

# Get Tracking Trejectory
Z_track = get_tracking_trajectory(tvlqr_cntrl, model, x_start, dt)

# Extract Position X and Y vectors
X_track = [Z_track.data[i].z[1] for i=1:length(Z_track.data)]
Y_track = [Z_track.data[i].z[2] for i=1:length(Z_track.data)]

# Plot Tracking Trajectory on top
plot!(X_track, Y_track, label = "Tracking Trajectory",
                        linewidth = 2.5)
