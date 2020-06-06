# Example File for TVLQR on Dubin's Car
# Generate arbitrary reference trajectory for this example

#TO DO
# Functions generate_reference_trajectory and get_tracking_trajectory
# will be made part of the package soon
# Will use discrete_dynamics on Traj soon
# Export Traj and discrete_dynamics

# WARN : Instantiate the environment

using LinearAlgebra
using StaticArrays
using Plots
using TrackingControl

TC = TrackingControl


# Define Dubins Car Model and Dynamics (relies on RobotDynamics.jl structures)
T = Float64

struct DubinsCar{T} <: TC.AbstractModel
    param::T
end

function TC.dynamics(model::DubinsCar, x, u)   # because we use "using" so need to add TO.dynamics to extend dynamics function from TO module
    SA[u[1]*cos(x[3]),u[1]*sin(x[3]),u[2]]
end

model = DubinsCar(0.0)
#Base.size(::DubinsCar) = 3, 2
#n,m = size(model)
n, m = 3, 2              # will use size

# Controller chosen (arbitrary) for generating reference trajectory
@inline controller(t) = [1.0;sin(t)]

# Set initial conditions
x0 = [0.0;0.0;π/2]
dt = 0.1
t0 = 0.0
N = 100  # N propagation steps

function generate_reference_trajectory(model, x0, t0, N)
    # Output a Traj type
    t = t0
    x = x0
    z0 = KnotPoint(x, controller(t), dt, t)
    Z = [z0]
    for i=1:N
        x = TC.discrete_dynamics(TC.RK3, model, z0)  # generate next state
        t = t+dt
        z = KnotPoint(x, controller(t), dt, t)
        push!(Z, z)
        z0 = z
    end
    return TC.Traj(Z)
end

Z = generate_reference_trajectory(model, x0, t0, N)
isa(Z, TC.Traj)
X = [Z.data[i].z[1] for i=1:length(Z.data)]
Y = [Z.data[i].z[2] for i=1:length(Z.data)]
plot(X, Y, label = "Reference Car Trajectory",
           xlabel = "X Position",
           ylabel = "Y Position",
           linewidth = 2.5)


# Define TVLQR Cost Matrices
ρ = 1.0
Q = Diagonal(@SVector ones(n))
R = ρ*Diagonal(@SVector ones(m))

# Define TVLQR controller on model
tvlqr_cntrl = TVLQR(model, Q, R, Z)

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
                        linewidth = 2.5)  # Plot tracking trajectory
