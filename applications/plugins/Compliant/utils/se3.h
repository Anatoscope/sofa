#ifndef FLEXIBLE_UTILS_SE3_H
#define FLEXIBLE_UTILS_SE3_H

// SE(3) kinematics

// author: maxime.tournier@inria.fr
// license: LGPL 2.1

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/Quat.h>

#include <limits>

// TODO include boost::math in sofa or implement SE3::sinc more precisely
// #include <boost/math/special_functions/sinc.hpp>

template<class U>
struct SE3 {

	typedef U real;

	typedef ::sofa::defaulttype::RigidCoord<3, real> coord_type;
	typedef ::sofa::defaulttype::RigidDeriv<3, real> deriv_type;

	typedef Eigen::Matrix<real, 3, 1> vec3;
	typedef Eigen::Matrix<real, 6, 1> vec6;

	typedef Eigen::Matrix<real, 6, 6> mat66;
	typedef Eigen::Matrix<real, 3, 3> mat33;
    typedef Eigen::Matrix<real, 3, 6> mat36;
    typedef Eigen::Matrix<real, 6, 3> mat63;    

	typedef Eigen::Quaternion<real, Eigen::DontAlign> quat;

	// sofa order: (translation, rotation)
	typedef vec6 twist;
    
	// easy mappings between sofa/eigen vectors
	template<int I>
	static Eigen::Map< Eigen::Matrix<real, I, 1> > map(::sofa::defaulttype::Vec<I, real>& v) {
		return Eigen::Map< Eigen::Matrix<real, I, 1> >(v.ptr());
	}


	template<int I>
	static Eigen::Map<const Eigen::Matrix<real, I, 1> > map(const ::sofa::defaulttype::Vec<I, real>& v) {
		return Eigen::Map<const Eigen::Matrix<real, I, 1> >(v.ptr());
	}


    // rigidderiv
	static Eigen::Map< vec6 > map(deriv_type& v) {
		return Eigen::Map< vec6 >(v.getVCenter().ptr());
	}

	static Eigen::Map< const vec6 > map(const deriv_type& v) {
		return Eigen::Map< const vec6 >(v.getVCenter().ptr());
	}



	//  quaternion conversion
	static quat coord(const ::sofa::helper::Quater<real>& at) {
		return quat(at[3],
		            at[0],
		            at[1],
		            at[2]);
	}

	static ::sofa::helper::Quater<real> coord(const quat& at) {
		return ::sofa::helper::Quater<real>(at.x(),
                                            at.y(),
                                            at.z(),
                                            at.w());
	}
	
	// rotation quaternion
	static quat rotation(const coord_type& at) {
        // TODO paranoid normalization in debug mode
		return coord(at.getOrientation() );
	}

	
	// translation vector
	static vec3 translation(const coord_type& at) {
		return map( at.getCenter() );
	}


	// aliases
	static quat orient(const coord_type& at) {
		return rotation(at);
	}

	static vec3 pos(const coord_type& at) {
		return translation(at);
	}

    // apply rigid transformation to a vector
    static vec3 apply(const coord_type& g, const vec3& x) {
        return orient(g) * x + pos(g);
    }
    

	// standard coordinates for SE(3) tangent vectors are body and
	// spatial coordinates. SOFA uses its own custom coordinate system,
	// so here are a couple of conversion routines.

	// sofa -> body velocity coordinates conversion
	static twist body(const coord_type& at, const deriv_type& sofa) {
		twist res;

		const quat qT = rotation( at ).conjugate();
		
		// orientation
		res.template tail<3>() = qT * map(sofa.getVOrientation() );
		res.template head<3>() = qT * map(sofa.getVCenter() );

		return res;
	}


	// sofa -> body, matrix version
	static mat66 body(const coord_type& at) {
		mat66 res;

		const mat33 R = rotation(at).normalized().toRotationMatrix();

		res <<
			R.transpose(), mat33::Zero(),
			mat33::Zero(), R.transpose();

		return res;
	}


	// sofa -> spatial, matrix version
	static mat66 spatial(const coord_type& at) {
		// TODO optimize !!!
		return Ad(at) * body(at);
	}

	static mat66 spatial_to_sofa(const coord_type& at) {
		// TODO optimize !!!
		const coord_type i = inv(at);
		return body( i ) * Ad( i );
	}
	

	// body -> sofa velocity coordinates conversion
	static deriv_type sofa(const coord_type& at, const twist& body) {
		deriv_type res;

		const quat q = rotation(at);

		map( res.getVOrientation() )  = q * body.template tail<3>();
		map( res.getVCenter() ) = q * body.template head<3>();

		return res;
	}


	// body -> sofa, matrix version
	static mat66 sofa(const coord_type& at) { 
		mat66 res;

		mat33 R = rotation( at ).toRotationMatrix();

		res <<
			R,  mat33::Zero(),
			mat33::Zero(), R;

		return res;
    }

	
	// sofa adT
	static vec6 move_wrench(const vec6& w, const vec3& delta) {
		vec6 res;

		res.template head<3>() = w.template head<3>();
		res.template tail<3>() = w.template tail<3>() + delta.cross( w.template head<3>() );

		return res;
	}


	// skew-symmetric mapping: hat(v) * x = v.cross(x)
	static mat33 hat(const vec3& v) {
		mat33 res;

		res.diagonal().setZero();

		res(0, 1) = -v.z();
		res(1, 0) = v.z();

		res(0, 2) = v.y();
		res(2, 0) = -v.y();

		res(1, 2) = -v.x();
		res(2, 1) = v.x();

		return res;
	}

	// SE(3) adjoint map
	static twist Ad(const coord_type& at, const twist& v) {

		const quat q = rotation(at); 
		const vec3 t = translation(at);

		twist res;

		res.template tail<3>() = q * v.template tail<3>();
		res.template head<3>() = t.cross(res.template tail<3>()) + q * v.template head<3>();

		return res;
	}


	//  SO(3) adjoint map quaternion -> matrix conversion
	static mat33 Ad(const ::sofa::helper::Quater<real>& at) {
		return coord(at).toRotationMatrix();
	}

    
	// SE(3) adjoint, matrix version
	static mat66 Ad(const coord_type& at) {

		const mat33 R = rotation(at).toRotationMatrix();
		const mat33 T = hat( translation(at) );

		mat66 res;

		res <<
			R, T * R,
			mat33::Zero(), R;

		return res;
	}


	// SE(3) group operations
	static coord_type inv(const coord_type& g) {
		return ::sofa::defaulttype::Rigid3Types::inverse( g );
	}


	static coord_type prod(const coord_type& a, const coord_type& b) {
		return ::sofa::defaulttype::Rigid3Types::mult(a, b);
	}


    // SO(3) x R3 product
	static coord_type direct_prod(const coord_type& a, const coord_type& b) {
        coord_type res;
        res.getOrientation() = coord( rotation(a) * rotation(b) );
        res.getCenter() = a.getCenter() + b.getCenter();
        return res;
	}
    

	static const real epsilon() {
		return std::numeric_limits<real>::epsilon();
	}



	// SO(3) log
	static vec3 log(const quat& qq) {
		
		quat q = qq;
		q.normalize();

		// flip if needed
		if( q.w() < 0 ) q.coeffs() = -q.coeffs(); // needed to get theta between 0 and PI

        const real w = std::min<real>(1.0, q.w()); // is it really necessary to use the min on a normalized quat?

        real sin_half_theta; // note that sin(theta/2) == norm of the imaginary part for unit quaternion
        real theta;

        // max: is this really needed ?

        // to avoid numerical instabilities of acos for theta < 5°
        if(w>0.999) // theta < 5° -> _q[3] = cos(theta/2) > 0.999
        {
            sin_half_theta = q.vec().norm();
            theta = (real)(2.0 * asin(sin_half_theta)); // in (0, pi)
        }
        else
        {
            real half_theta = acos(w); // in (0, pi / 2)
            sin_half_theta = sin(half_theta);
            theta = 2*half_theta; // in (0, pi)
        }

        assert(sin_half_theta>=0);
        if( sin_half_theta < epsilon() ) {
            return 2 * q.vec(); // max: fixed
        } else {
            return theta * (q.vec()/sin_half_theta); 
        }

	}


	static real sinc( const real& x ) {
		
		if( std::abs(x) < epsilon() ) {
			// TODO Taylor series similar to boost::math::sinc instead
			return 1.0;
		}

		return std::sin(x) / x;
	}


	// SO(3) log derivative, body coordinates
	static mat33 dlog(const quat& q) {
        
		const vec3 x = log(q);
		const real theta = x.norm();
        
		if( theta < epsilon() ) {
            return mat33::Identity() + hat( x / 2 );
        }
        
		const vec3 n = x / theta;
        const real half_theta = theta / 2;
        
        const mat33 P = n * n.transpose();

        // checked from Bullo et al. 1995 (PD control in the Euclidean group)
        return P + (half_theta / std::tan(half_theta)) * (mat33::Identity() - P) + hat(x / 2);
	}



	// SO(3) exp derivative, body coordinates. 
	static mat33 dexp(const vec3& x, const quat& exp_v) {
		mat33 res = mat33::Zero();

		const mat33 xhat = hat(x);
		const mat33 R = exp_v.toRotationMatrix();
		
		res = mat33::Identity();

		const real theta2 = x.squaredNorm();

		if( theta2 > epsilon() ) {
            res.noalias() += (R.transpose() - mat33::Identity() + xhat) * xhat / theta2;
		}
		
		return res;
	}

	// TODO provide SE(3) exponential lol

    // SO(3) exponential
    static quat exp(const vec3& v) {
        const real theta = v.norm();

//        const real c = std::cos( theta / 2.0 );
        const real sc = sinc( theta / 2.0 ) / 2.0;
        
        quat res;

        res.w() = std::cos( theta / 2.0 );
        res.vec() = sc * v;
        
        return res;
    }
    
	// static mat33 dexp(const vec3& x) {
	// 	return dexp(x, exp(x) );
	// }


	// R(3) x SO(3) logarithm (i.e. *not* SE(3))
	static deriv_type product_log(const coord_type& g) {

		deriv_type res;

		res.getVCenter() = g.getCenter();
		map(res.getVOrientation() ) = log( rotation(g) );

		return res;
	}


	// R(3) x SO(3) logarithm derivative, in sofa coordinates
	static mat66 product_dlog(const coord_type& g) {
		mat66 res;

		const quat q = rotation(g);
		const mat33 R = q.toRotationMatrix();

		res <<
			mat33::Identity(), mat33::Zero(),
			mat33::Zero(), dlog( q ) * R.transpose();

		return res;
	}


	// left and right translation derivatives:

	// L_h(g) = h.g
	// R_h(g) = g.h
	
	// dL_h(g) in sofa coordinates
	static mat66 dL(const coord_type& h, const coord_type& ) {
		// dL_h(g) = sofa(hg) * body(g) = sofa(hg) * sofa(g-1) = sofa(h)
		return sofa(h);
	}


	// dR_h(g) in sofa coordinates
	static mat66 dR(const coord_type& h, const coord_type& g) {
		mat66 res; 

		res <<
			mat33::Identity(), - hat( rotation(g) * translation(h) ),
			mat33::Zero(), mat33::Identity();

		return res;

		// return sofa( prod(g, h) ) * Ad( inv(h) ) * body(g);
	}


    // d g^{-1} in sofa coordinates
    static mat66 dInv(const coord_type& g) {
        mat66 res;

        const mat33 R = rotation(g);
        
        res <<
            -R.transpose() , -R.transpose() * hat( translation(g) ),
            mat33::Zero(), -R.transpose();

        return res;

        // // this should be equal to
        // return sofa( inv(g) ) * (- Ad(g))  * body(g);
    }

    


};


#endif
