#ifndef SOFA_IMAGE_CIMG_TYPES_H
#define SOFA_IMAGE_CIMG_TYPES_H

#if  defined (SOFA_HAVE_FFMPEG)  || defined (SOFA_EXTLIBS_FFMPEG)
#define cimg_use_ffmpeg
#endif
#ifdef SOFA_IMAGE_HAVE_OPENCV // should be "SOFA_HAVE_OPENCV" -> use "SOFA_IMAGE_HAVE_OPENCV" until the opencv plugin is fixed..
#define cimg_use_opencv
#endif

#define cimg_display 0

#include <CImg/SOFACImg.h>

// extern templates
namespace cimg_library {

extern template struct CImg<unsigned int>;
extern template struct CImg<long double>;
extern template struct CImg<bool>;
extern template struct CImg<unsigned char>;

extern template struct CImg<char>;
extern template struct CImg<int>;

extern template struct CImg<short>;
extern template struct CImg<unsigned short>;

extern template struct CImg<long>;
extern template struct CImg<unsigned long>;

extern template struct CImg<float>;
extern template struct CImg<double>;

}





#endif
