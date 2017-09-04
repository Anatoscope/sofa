#ifndef SOFA_HELPER_SSE_HPP
#define SOFA_HELPER_SSE_HPP

// takes care of including intrinsics
#include <Eigen/Core>

#include <sofa/defaulttype/Vec.h>

#ifdef EIGEN_VECTORIZE_SSE
namespace sse {

static inline float rsqrt(const float& x ) {
    // fast 1 / sqrt(x), see https://stackoverflow.com/q/1528727/1753545
    float res;
    __m128 in = _mm_load_ss( &x );
    _mm_store_ss( &res,  _mm_rsqrt_ss( in ) );
    return res;
}

}

#endif



namespace sofa {
namespace defaulttype {

template<int N, class U>
static void fast_normalize(Vec<N, U>& self) {
#ifdef EIGEN_VECTORIZE_SSE
    self *= sse::rsqrt(self.norm2());
#else
    self /= self.norm();
#endif
}

}
}


#endif
