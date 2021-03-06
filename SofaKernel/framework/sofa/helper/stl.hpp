#ifndef SOFA_HELPER_STL_HPP
#define SOFA_HELPER_STL_HPP

#include <sofa/helper/helper.h>

#include <istream>

namespace sofa {
namespace helper {
namespace stl {

// TODO merge the two implementations (WTF) of STL loading
// (MeshSTL.cpp/MeshSTLLoader.cpp) here)

// determine if a given opened stl file is binary or not by trying binary first
// and comparing file size. seekpos is left unspecified.
bool SOFA_HELPER_API is_binary(std::istream& in);

}
}
}


#endif
