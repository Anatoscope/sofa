#include <sofa/defaulttype/TemplatesAliases.h>

#include "ImageTypes.h"

namespace sofa
{
namespace defaulttype
{


#ifndef SOFA_FLOAT
RegisterTemplateAlias ImageRAlias("ImageR", "ImageD");
#else
RegisterTemplateAlias ImageRAlias("ImageR", "ImageF");
#endif


template struct Image<unsigned int>;
template struct Image<long double>;
template struct Image<bool>;
template struct Image<unsigned char>;

template struct Image<char>;
template struct Image<int>;

template struct Image<short>;
template struct Image<unsigned short>;

template struct Image<long>;
template struct Image<unsigned long>;

template struct Image<float>;
template struct Image<double>;



template struct Histogram<unsigned int>;
template struct Histogram<long double>;
template struct Histogram<bool>;
template struct Histogram<unsigned char>;

template struct Histogram<char>;
template struct Histogram<int>;

template struct Histogram<short>;
template struct Histogram<unsigned short>;

template struct Histogram<long>;
template struct Histogram<unsigned long>;

template struct Histogram<float>;
template struct Histogram<double>;



template struct ImagePlane<unsigned int>;
template struct ImagePlane<long double>;
template struct ImagePlane<bool>;
template struct ImagePlane<unsigned char>;

template struct ImagePlane<char>;
template struct ImagePlane<int>;

template struct ImagePlane<short>;
template struct ImagePlane<unsigned short>;

template struct ImagePlane<long>;
template struct ImagePlane<unsigned long>;

template struct ImagePlane<float>;
template struct ImagePlane<double>;



}
}
