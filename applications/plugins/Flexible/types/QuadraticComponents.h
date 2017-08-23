#ifndef FLEXIBLE_QuadraticComponents_H
#define FLEXIBLE_QuadraticComponents_H



#include "QuadraticTypes.h"

#define TYPEABSTRACTNAME Quadratic

#ifdef FLEXIBLE_QuadraticComponents_constraint_CPP
    #define FLEXIBLE_COMPILING_CONSTRAINT_CPP
#endif
#ifdef FLEXIBLE_QuadraticComponents_core_CPP
    #define FLEXIBLE_COMPILING_CORE_CPP
#endif
#ifdef FLEXIBLE_QuadraticComponents_forcefield_CPP
    #define FLEXIBLE_COMPILING_FORCEFIELD_CPP
#endif
#ifdef FLEXIBLE_QuadraticComponents_mapping_CPP
    #define FLEXIBLE_COMPILING_MAPPING_CPP
#endif
#ifdef FLEXIBLE_QuadraticComponents_misc_CPP
    #define FLEXIBLE_COMPILING_MISC_CPP
#endif

#include "ComponentSpecializations.h.inl"

#endif // FLEXIBLE_QuadraticComponents_H
