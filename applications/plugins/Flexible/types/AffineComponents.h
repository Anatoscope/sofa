#ifndef FLEXIBLE_AffineComponents_H
#define FLEXIBLE_AffineComponents_H



#include "AffineTypes.h"

#define TYPEABSTRACTNAME Affine


#ifdef FLEXIBLE_AffineComponents_constraint_CPP
    #define FLEXIBLE_COMPILING_CONSTRAINT_CPP
#endif
#ifdef FLEXIBLE_AffineComponents_core_CPP
    #define FLEXIBLE_COMPILING_CORE_CPP
#endif
#ifdef FLEXIBLE_AffineComponents_forcefield_CPP
    #define FLEXIBLE_COMPILING_FORCEFIELD_CPP
#endif
#ifdef FLEXIBLE_AffineComponents_mapping_CPP
    #define FLEXIBLE_COMPILING_MAPPING_CPP
#endif
#ifdef FLEXIBLE_AffineComponents_misc_CPP
    #define FLEXIBLE_COMPILING_MISC_CPP
#endif


#include "ComponentSpecializations.h.inl"



#endif // FLEXIBLE_AffineComponents_H
