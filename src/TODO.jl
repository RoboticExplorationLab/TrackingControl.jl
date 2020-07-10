# TODO File

# TrackingControl.jl should
# 1) be able to take as an input
#    - a dynamical model (from RobotDynamics.jl)
#    - a reference trajectory (Obtained throught TrajectoryOptimization.jl or sth else)
#    - some control data (e.g. type of controller, cost functions etc...)
# 2) include a link with DifferentialEquations.jl enabling to go from simulation
#    to real control
# 3) include a link with Interpolations.jl for the same type of purposes
# 4) include some flavor of MPC controller (QP based MPC)
# 5) include tracking LQR controllers
#    - discrete time version
#    - continuous time version
#    - square root formulation of Riccati equation for efficiency
# 6) include some noise features (using Distributions.jl)
# 7) include an abstract type AbstractTrackingController for dispatch
# 8) rely on RobotDynamics.jl for Trajectory type and models
# 9) include some type of robust controllers (H_∞, LMI-based control)
# 10) 
