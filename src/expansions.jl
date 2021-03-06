abstract type AbstractExpansion{T} end
abstract type AbstractTest end

"""
    Gradient Expansion
"""
struct GradientExpansion{T,N,M} <: AbstractExpansion{T}
	x::SizedVector{N,T}
	u::SizedVector{M,T}
	function GradientExpansion{T}(n::Int,m::Int) where T
		new{T,n,m}(SizedVector{n}(zeros(T,n)), SizedVector{m}(zeros(T,m)))
	end
end

"""
    Sized Cost Expansion
"""
struct SizedCostExpansion{T,N0,N,M} <: AbstractExpansion{T}
	# Cost Expansion Terms
	x ::SizedVector{N0,T,Vector{T}}
	xx::SizedMatrix{N0,N0,T,2,Matrix{T}}
	u ::SizedVector{M,T,Vector{T}}
	uu::SizedMatrix{M,M,T,2,Matrix{T}}
	ux::SizedMatrix{M,N0,T,2,Matrix{T}}

	# Error Expansion Terms
	x_ ::SizedVector{N,T,Vector{T}}
	xx_::SizedMatrix{N,N,T,2,Matrix{T}}
	u_ ::SizedVector{M,T,Vector{T}}
	uu_::SizedMatrix{M,M,T,2,Matrix{T}}
	ux_::SizedMatrix{M,N,T,2,Matrix{T}}

	tmp::SizedMatrix{N0,N,T,2,Matrix{T}}
	x0::SizedVector{N0,T,Vector{T}}  # gradient of cost function only (no multipliers)
end

function SizedCostExpansion{T}(n0::Int, n::Int, m::Int) where T
	x0  = SizedVector{n0}(zeros(T,n0))
	xx0 = SizedMatrix{n0,n0}(zeros(T,n0,n0))
	u0  = SizedVector{m}(zeros(T,m))
	uu0 = SizedMatrix{m,m}(zeros(T,m,m))
	ux0 = SizedMatrix{m,n0}(zeros(T,m,n0))

	x  = SizedVector{n}(zeros(T,n))
	xx = SizedMatrix{n,n}(zeros(T,n,n))
	u  = SizedVector{m}(zeros(T,m))
	uu = SizedMatrix{m,m}(zeros(T,m,m))
	ux = SizedMatrix{m,n}(zeros(T,m,n))
	tmp = SizedMatrix{n0,n}(zeros(T,n0,n))
	x_ = copy(x0)
	SizedCostExpansion(x0,xx0,u0,uu0,ux0, x, xx, u, uu, ux, tmp, x_)
end

function SizedCostExpansion{T}(n::Int, m::Int) where T
	x  = SizedVector{n}(zeros(T,n))
	xx = SizedMatrix{n,n}(zeros(T,n,n))
	u  = SizedVector{m}(zeros(T,m))
	uu = SizedMatrix{m,m}(zeros(T,m,m))
	ux = SizedMatrix{m,n}(zeros(T,m,n))
	tmp = SizedMatrix{n,n}(zeros(T,n,n))
	x_ = copy(x)
	SizedCostExpansion(x,xx,u,uu,ux, x, xx, u, uu, ux, tmp, x_)
end

"""
    Sized Expansion
"""
struct SizedExpansion{T,N0,N,M} <: AbstractExpansion{T}
	x::SizedVector{N,T,Vector{T}}
	xx::SizedMatrix{N,N,T,2,Matrix{T}}
	u::SizedVector{M,T,Vector{T}}
	uu::SizedMatrix{M,M,T,2,Matrix{T}}
	ux::SizedMatrix{M,N,T,2,Matrix{T}}
	tmp::SizedMatrix{N0,N,T,2,Matrix{T}}
	function SizedExpansion{T}(n::Int) where T
		x = SizedVector{n}(zeros(n))
		xx = SizedMatrix{n,n}(zeros(n,n))
		new{T,n,n,0}(x,xx)
	end
	function SizedExpansion{T}(n::Int,m::Int) where T
		x = SizedVector{n}(zeros(n))
		xx = SizedMatrix{n,n}(zeros(n,n))
		u = SizedVector{m}(zeros(m))
		uu = SizedMatrix{m,m}(zeros(m,m))
		ux = SizedMatrix{m,n}(zeros(m,n))
		new{T,n,n,m}(x,xx,u,uu,ux)
	end
	function SizedExpansion{T}(n0::Int,n::Int,m::Int) where T
		x = SizedVector{n}(zeros(n))
		xx = SizedMatrix{n,n}(zeros(n,n))
		u = SizedVector{m}(zeros(m))
		uu = SizedMatrix{m,m}(zeros(m,m))
		ux = SizedMatrix{m,n}(zeros(m,n))
		tmp = SizedMatrix{n0,n}(zeros(n0,n))
		new{T,n0,n,m}(x,xx,u,uu,ux,tmp)
	end
	function SizedExpansion(
			x::SizedVector{N,T,1},
			xx::SizedMatrix{N,N,T,2},
			u::SizedVector{M,T,1},
			uu::SizedMatrix{M,M,T,2},
			ux::SizedMatrix{M,N,T,2}) where {T,N,M}
		new{T,N,N,M}(x,xx,u,uu,ux)
	end
end

"""
    Static Expansion
"""
struct StaticExpansion{T,N,M,NN,MM,NM} <: AbstractExpansion{T}
	x::SVector{N,T}
	xx::SMatrix{N,N,T,NN}
	u::SVector{M,T}
	uu::SMatrix{M,M,T,MM}
	ux::SMatrix{M,N,T,NM}
end

function StaticExpansion(E::AbstractExpansion)
	StaticExpansion(SVector(E.x), SMatrix(E.xx),
		SVector(E.u), SMatrix(E.uu), SMatrix(E.ux))
end

function StaticExpansion(x,xx,u,uu,ux)
	StaticExpansion(SVector(x), SMatrix(xx), SVector(u), SMatrix(uu), SMatrix(ux))
end


"""
    Sized Dynamics Expansion
"""
struct SizedDynamicsExpansion{T,N,N̄,M} <: AbstractExpansion{T}
	∇f::Matrix{T} # n × (n+m+1)
	A_::SubArray{T,2,Matrix{T},Tuple{UnitRange{Int},UnitRange{Int}},false}
	B_::SubArray{T,2,Matrix{T},Tuple{UnitRange{Int},UnitRange{Int}},false}
	A::SizedMatrix{N̄,N̄,T,2,Matrix{T}}
	B::SizedMatrix{N̄,M,T,2,Matrix{T}}
	tmpA::SizedMatrix{N,N,T,2,Matrix{T}}
	tmpB::SizedMatrix{N,M,T,2,Matrix{T}}
	tmp::SizedMatrix{N,N̄,T,2,Matrix{T}}
	function SizedDynamicsExpansion{T}(∇f,A_,B_,A,B,tmpA,tmpB,tmp) where T
		n, k = size(∇f)
		m = k-n-1
		new{T,n,n,m}(∇f,A_,B_,A,B,tmpA,tmpB,tmp)
	end
	function SizedDynamicsExpansion{T}(n0::Int, n::Int, m::Int) where T
		∇f = zeros(n0,n0+m+1)
		ix = 1:n0
		iu = n0 .+ (1:m)
		A_ = view(∇f, ix, ix)
		B_ = view(∇f, ix, iu)
		A = SizedMatrix{n,n}(zeros(n,n))
		B = SizedMatrix{n,m}(zeros(n,m))
		tmpA = SizedMatrix{n0,n0}(zeros(n0,n0))
		tmpB = SizedMatrix{n0,m}(zeros(n0,m))
		tmp = zeros(n0,n)
		new{T,n0,n,m}(∇f,A_,B_,A,B,tmpA,tmpB,tmp)
	end
	function SizedDynamicsExpansion{T}(n::Int, m::Int) where T
		∇f = zeros(n,n+m)                                       # changed to n+m
		ix = 1:n
		iu = n .+ (1:m)
		A_ = view(∇f, ix, ix)
		B_ = view(∇f, ix, iu)
		A = SizedMatrix{n,n}(zeros(n,n))
		B = SizedMatrix{n,m}(zeros(n,m))
		tmpA = A
		tmpB = B
		tmp = zeros(n,n)
		new{T,n,n,m}(∇f,A_,B_,A,B,tmpA,tmpB,tmp)
	end
	#=
	function SizedDynamicsExpansion{T}(model::AbstractModel) where T  # ADDED
		n, m
		∇f = zeros(n,n+m+1)
		ix = 1:n
		iu = n .+ (1:m)
		A_ = view(∇f, ix, ix)
		B_ = view(∇f, ix, iu)
		A = SizedMatrix{n,n}(zeros(n,n))
		B = SizedMatrix{n,m}(zeros(n,m))
		tmpA = A
		tmpB = B
		tmp = zeros(n,n)
		new{T,n,n,m}(∇f,A_,B_,A,B,tmpA,tmpB,tmp)
	end=#
end

#=function SizedDynamicsExpansion(n::Int, m::Int)
	∇f = zeros(n,n+m+1)
	ix = 1:n
	iu = n .+ (1:m)
	A_ = view(∇f, ix, ix)
	B_ = view(∇f, ix, iu)
	A = SizedMatrix{n,n}(zeros(n,n))
	B = SizedMatrix{n,m}(zeros(n,m))
	tmpA = A
	tmpB = B
	tmp = zeros(n,n)
	SizedDynamicsExpansion(∇f,A_,B_,A,B,tmpA,tmpB,tmp)
end
=#
#=
function SizedDynamicsExpansion{T,N,N̄,M}(∇f::Matrix{T}, # n × (n+m+1)
	A_::SubArray{T,2,Matrix{T},Tuple{UnitRange{Int},UnitRange{Int}},false},
	B_::SubArray{T,2,Matrix{T},Tuple{UnitRange{Int},UnitRange{Int}},false},
	A::SizedMatrix{N̄,N̄,T,2},
	B::SizedMatrix{N̄,M,T,2},
	tmpA::SizedMatrix{N,N,T,2},
	tmpB::SizedMatrix{N,M,T,2},
	tmp::SizedMatrix{N,N̄,T,2}) where {T,N,N̄,M}
	return SizedDynamicsExpansion(∇f,A_,B_,A,B,tmpA,tmpB,tmp)
end

function SizedDynamicsExpansion(n0::Int, n::Int, m::Int)
	∇f = zeros(n0,n0+m+1)
	ix = 1:n0
	iu = n0 .+ (1:m)
	A_ = view(∇f, ix, ix)
	B_ = view(∇f, ix, iu)
	A = SizedMatrix{n,n}(zeros(n,n))
	B = SizedMatrix{n,m}(zeros(n,m))
	tmpA = SizedMatrix{n0,n0}(zeros(n0,n0))
	tmpB = SizedMatrix{n0,m}(zeros(n0,m))
	tmp = zeros(n0,n)
	SizedDynamicsExpansion(∇f,A_,B_,A,B,tmpA,tmpB,tmp)
end

function SizedDynamicsExpansion(n::Int, m::Int)
	∇f = zeros(n,n+m+1)
	ix = 1:n
	iu = n .+ (1:m)
	A_ = view(∇f, ix, ix)
	B_ = view(∇f, ix, iu)
	A = SizedMatrix{n,n}(zeros(n,n))
	B = SizedMatrix{n,m}(zeros(n,m))
	tmpA = A
	tmpB = B
	tmp = zeros(n,n)
	SizedDynamicsExpansion(∇f,A_,B_,A,B,tmpA,tmpB,tmp)
end
=#

"""
Test
"""

struct Test <: AbstractTest
	a::Int
	b::Int
	Test(a) = new(a, b)
	Test() = new(0, 0)
end


########################### Error Expansion ####################################
################################################################################

@inline error_expansion!(D::Vector{<:SizedDynamicsExpansion}, model::AbstractModel, G) = nothing

function error_expansion!(D::Vector{<:SizedDynamicsExpansion}, model::RigidBody, G)
	for k in eachindex(D)
		error_expansion!(D[k], G[k], G[k+1])
	end
end

function error_expansion!(D::SizedDynamicsExpansion,G1,G2)
    mul!(D.tmp, D.tmpA, G1)
    mul!(D.A, Transpose(G2), D.tmp)
    mul!(D.B, Transpose(G2), D.tmpB)
end

@inline error_expansion(D::SizedDynamicsExpansion, model::RigidBody) = D.A, D.B
@inline error_expansion(D::SizedDynamicsExpansion, model::AbstractModel) = D.tmpA, D.tmpB
@inline SizedDynamicsExpansion(model::AbstractModel) = SizedDynamicsExpansion{Float64}(model)
@inline function SizedDynamicsExpansion{T}(model::AbstractModel) where T
	n,m = size(model)
	n̄ = state_diff_size(model)
	SizedDynamicsExpansion{T}(n,n̄,m)
end
