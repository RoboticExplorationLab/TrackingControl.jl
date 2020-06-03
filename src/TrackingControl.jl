module TrackingControl

#greet() = print("Hello World!")

using StaticArrays
using LinearAlgebra
using ForwardDiff

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


include("rbstate.jl")
include("jacobian.jl")
include("knotpoint.jl")
include("model.jl")
#include("liestate.jl")
include("rigidbody.jl")
include("integration.jl")

end # module


end # module
