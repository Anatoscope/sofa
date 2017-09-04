#include "stl.hpp"

#include <stdint.h>

namespace sofa {
namespace helper {
namespace stl {

union uint32 {
    char bytes[4];
    std::uint32_t value;
};

struct stream_pos {
    std::istream& in;
    const std::istream::pos_type pos;
    
    stream_pos(std::istream& in)
        : in(in),
          pos(in.tellg()) {

    }

    ~stream_pos() {
        in.clear();
        in.seekg(pos);
    }
};


bool SOFA_HELPER_API is_binary(std::istream& in) {
    const stream_pos backup(in);
    
    in.seekg(0, std::ios::end);
    const std::size_t file_size = in.tellg();

    in.seekg(0, std::ios::beg);    
    char header[80];

    if(!in.read(header, sizeof(header)) ) {
        return false;
    }

    uint32 triangles;
    if(!in.read(triangles.bytes, sizeof(triangles.bytes))) {
        return false;
    }
    
    static const std::size_t float_size = 4;
    static const std::size_t vec3_size = 3 * float_size;
    static const std::size_t triangle_size = 4 * vec3_size + 2;
    static const std::size_t header_size = 80 + 4;

    const std::size_t expected_size = header_size + triangles.value * triangle_size;

    return file_size == expected_size;
}



}
}
}

