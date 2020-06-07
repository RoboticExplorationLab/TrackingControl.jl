module TrackingControl

# DONE:
# Removed dependency on TO by adding expansions.jl
# All exports from RobotDynamics can be used directly

#TO DO:
# Version Compatible on Rotations in TrajOpt
# question : Could we use expansions.jl in RobotDynamics ?

using StaticArrays   # Needs to be here, a module is a separate name frame
using LinearAlgebra
using ForwardDiff
using RobotDynamics
using Rotations
#using TrajectoryOptimization

# From trajectories.jl
export
    AbstractTrajectory
#    Traj
#    discrete_dynamics!
#    rollout!

# From expansions.jl
export
    AbstractExpansion

# From trackingcontrollers.jl
export
    get_control,
    TVLQR,
    LQR,
    MLQR,
    simulate,
    SE3Tracking,
    HFCA,
    test_ICs

export
    KnotPoint
    discrete_dynamics
    #AbstractTrajectory
    AbstractModel

#export
#    AbstractModel,
#    DynamicsExpansion,
#    dynamics,
#    jacobian!,
#    discrete_dynamics,
#    discrete_jacobian!,
#    linearize,
#    linearize!,
#    state_dim,
#    control_dim,
#    state_diff_size,
#    rollout!

# rigid bodies
#export
#    LieGroupModel,
#    RigidBody,
#    RBState,
#    orientation,
#    linear_velocity,
#    angular_velocity


# knotpoints
#export
#    AbstractKnotPoint,
#    KnotPoint,
#    StaticKnotPoint,
#    state,
#    control,
#    states,
#    controls,
#    set_states!,
#    set_controls!

# integration
#export
#    QuadratureRule,
#    RK2,
#    RK3,
#    RK4,
#    HermiteSimpson


#include("rbstate.jl")            # In RobotDynamics
#include("jacobian.jl")           # In RobotDynamics
#include("knotpoint.jl")          # In RobotDynamics
#include("model.jl")              # In RobotDynamics
#include("liestate.jl")           # In RobotDynamics
#include("rigidbody.jl")          # In RobotDynamics
#include("integration.jl")        # In RobotDynamics

include("trajectories.jl")        # needs to be in order of dependencies yes
include("expansions.jl")          # needed for SizedDynamicsExpansion
include("trackingcontrollers.jl")    # contains controllers


end # module
