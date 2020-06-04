module TrackingControl

# DONE:
# Removed dependency on TO by adding expansions.jl
# question : Could we use expansions.jl in RobotDynamics ?

#TO DO:
# Remove extra files already in RobotDynamics
# i.e. figure out a way to use the KnotPoint structures from Robot Dynamics
# for instance (need export in RobotDynamics ?)
# Version Compatible on Rotations 

using StaticArrays   # Needs to be here, a module is a separate name frame
using LinearAlgebra
using ForwardDiff
using RobotDynamics
using Rotations

const RD = RobotDynamics

# Robot Dynamics
export
    AbstractModel,
    DynamicsExpansion,
    dynamics,
    jacobian!,
    discrete_dynamics,
    discrete_jacobian!,
    linearize,
    linearize!,
    state_dim,
    control_dim,
    state_diff_size,
    rollout!

# rigid bodies
export
    LieGroupModel,
    RigidBody,
    RBState,
    orientation,
    linear_velocity,
    angular_velocity


# knotpoints
export
    AbstractKnotPoint,
    KnotPoint,
    StaticKnotPoint,
    state,
    control,
    states,
    controls,
    set_states!,
    set_controls!

# integration
export
    QuadratureRule,
    RK2,
    RK3,
    RK4,
    HermiteSimpson


#include("rbstate.jl")
#include("jacobian.jl")
include("knotpoint.jl")
include("model.jl")
include("trajectories.jl")        # needs to be in order of dependencies ?
include("tracking_control.jl")
include("expansions.jl")          # needed for SizedDynamicsExpansion from TO
#include("liestate.jl")
#include("rigidbody.jl")
#include("integration.jl")

end # module
