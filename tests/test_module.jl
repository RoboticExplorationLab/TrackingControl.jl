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

include("..\\src\\TrackingControl.jl")
using TrackingControl

const TC = TrackingControl

TrackingControl.SizedDynamicsExpansion{Float64}(2, 3)



TC.SizedDynamicsExpansion()
TC.SizedDynamicsExpansion(3, 2)


TC.Test(2, 3)
TC.Test(0.0, 1)

TC.SizedDynamicsExpansion(3, 2)
TC.AbstractController
TC.AbstractExpansion
TC.HFCA
TC.SizedDynamicsExpansion()

TC.SizedDynamicsExpansion
TC.SizedDynamicsExpansion(3, 2)

TC.SizedDynamicsExpansion(2, 3)
TC.SizedDynamicsExpansion(2, 3)

TC.SizedCostExpansion()

#const TO = TrajectoryOptimization

struct DubinsCar{T} <: TC.AbstractModel where{T}
    param::T
end

function TC.dynamics(model::DubinsCar, x, u)   # because we use "using" so need to add TO.dynamics to extend dynamics function from TO module
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

# ##############################################################################
# Test with hand-made trajectory ###############################################
# Dubin's Car model ############################################################

include("..\\src\\TrackingControl.jl")
using StaticArrays
using LinearAlgebra
using ForwardDiff
using Plots

G = TrackingControl


T = Float64

struct DubinsCar{T} <: G.AbstractModel where{T}
    param::T
end

function G.dynamics(model::DubinsCar, x, u)   # because we use "using" so need to add TO.dynamics to extend dynamics function from TO module
    SA[u[1]*cos(x[3]),u[1]*sin(x[3]),u[2]]
end

function discrete_dubins_dynamics(x, u, t, δ)
    derivative = [u[1]*cos(x[3]);
            u[1]*sin(x[3]);
            u[2]]
    return x+δ.*derivative
end

# Get a Nomial Feasible Trajectory (states and control to follow)

function controller(t::Float64)
    #u = [v; ω]
    return [1.0; sin(t)]
end

function discrete_propagate(x_ini, dynamics, Δt, N)
    U = Vector{Vector{Float64}}([])
    X = [x_ini]
    T = [0.0]
    t = 0.0
    x = x_ini
    for i=1:1:N
        u = controller(t)
        push!(U, u)
        x_next = dynamics(x, u, t, Δt)
        push!(X, x_next)
        t += Δt
        x = x_next
        push!(T, t)
    end
    return X, U, T
end

# Test discrete_propagate

model = DubinsCar(0.0)
x_0 = [0.0;0.0;π/2]
dynamics = discrete_dubins_dynamics
Δt = 0.1
N = 100
X, U, Ti = discrete_propagate(x_0, dynamics, Δt, N)
plot([X[i][1] for i=1:N+1],[X[i][2] for i=1:N+1])

Z = G.Traj(X, U, Δt*ones(N+1))

Z.data


ρ = 1.0
n = 3
m = 2
Q = Diagonal(@SVector ones(n))
R = ρ*Diagonal(@SVector ones(m))
control = G.TVLQR(model, Q, R, Z)

G.get_control(control, [0.0;0.0;π/2], 2.1)

function get_track_trajectory(cntrl::G.TVLQR, x0)
    # Get tracking trajectory from TVLQR
    # Starting at position x0
    # Starting at time t0
    XX = [x0]
    t = cntrl.t
    for i=1:length(t)-1
        u = G.get_control(cntrl, x0, t[i])
        x_next = discrete_dubins_dynamics(x0, u, t[i], t[i+1]-t[i])
        x0 = x_next
        push!(XX, x_next)
    end
    return XX
end

XX = get_track_trajectory(control, [-1.0;0.0;π/2])

plot!([XX[i][1] for i=1:length(XX)], [XX[i][2] for i=1:length(XX)])


# Now try to get discrete dynamics functions from RobotDynamics.jl

include("..\\src\\TrackingControl.jl")
using StaticArrays
using LinearAlgebra
using ForwardDiff
using Plots
using RobotDynamics

G = TrackingControl
T = Float64

struct DubinsCar{T} <: G.AbstractModel
    param::T
end

function G.dynamics(model::DubinsCar, x, u)   # because we use "using" so need to add TO.dynamics to extend dynamics function from TO module
    SA[u[1]*cos(x[3]),u[1]*sin(x[3]),u[2]]
end

@inline controller(t) = [1.0;sin(t)]

model = DubinsCar(0.0)
#n, m = size(model)  # need to do something
n, m = 3, 2
x0 = [0.0;0.0;π/2]
dt = 0.1
t0 = 0.0
z0 = KnotPoint(x0, controller(t), dt,t)
discrete_dynamics(RK3, model, z0)

RobotDynamics.state_dim(::DubinsCar) = 3
RobotDynamics.control_dim(::DubinsCar) = 2
n, m = size(model)

function generate_reference_trajectory(model, x0, t0, N)
    # Output a Traj type
    t = t0
    x = x0
    z0 = KnotPoint(x, controller(t), dt, t)
    Z = [z0]
    for i=1:N
        x = discrete_dynamics(RK3, model, z0)  # generate next state
        t = t+dt
        z = KnotPoint(x, controller(t), dt, t)
        push!(Z, z)
        z0 = z
    end
    return G.Traj(Z)
end

N = 100  # N propagation steps
Z = generate_reference_trajectory(model, x0, t0, N)
isa(Z, G.Traj)
X = [Z.data[i].z[1] for i=1:length(Z.data)]
Y = [Z.data[i].z[2] for i=1:length(Z.data)]
plot(X, Y, label = "Reference Trajectory")

ρ = 1.0
Q = Diagonal(@SVector ones(n))
R = ρ*Diagonal(@SVector ones(m))
tvlqr_cntrl = G.TVLQR(model, Q, R, Z)

function get_tracking_trajectory(cntrl::G.TVLQR, model, x0, dt)
    # Get tracking trajectory from TVLQR
    # Starting at position x0
    t = cntrl.t
    t0 = t[1]
    u0 = G.get_control(cntrl, x0, t0)
    Z_track = [KnotPoint(x0, u0, dt, t0)]
    for i=2:length(t)
        x_next = discrete_dynamics(model, Z_track[end])
        u = G.get_control(cntrl, x_next, t[i])
        z_next = KnotPoint(x_next, u, dt, t[i])
        push!(Z_track, z_next)
    end
    return G.Traj(Z_track)
end

x_start = [-1.0;0.0;π/2]
Z_track = get_tracking_trajectory(tvlqr_cntrl, model, x_start, dt)
X_track = [Z_track.data[i].z[1] for i=1:length(Z_track.data)]
Y_track = [Z_track.data[i].z[2] for i=1:length(Z_track.data)]
plot!(X_track, Y_track)

# Get the value function now
tvlqr_cntrl.P
