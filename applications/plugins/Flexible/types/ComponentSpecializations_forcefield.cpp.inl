

#include <Flexible/config.h>


#include "ComponentSpecializationsDefines.h"

#include <sofa/core/ObjectFactory.h>


#include <SofaBaseMechanics/UniformMass.inl>

#include <SofaDeformable/RestShapeSpringsForceField.inl>
#include <SofaBoundaryCondition/ConstantForceField.inl>
#include <SofaBoundaryCondition/UniformVelocityDampingForceField.inl>



namespace sofa
{

namespace component
{

namespace mass
{

//#ifndef SOFA_FLOAT
//template<> SOFA_Flexible_API
//void UniformMass<TYPEABSTRACTNAME3dTypes, TYPEABSTRACTNAME3dMass>::reinit()
//{
//    if (this->totalMass.getValue()>0 && this->mstate!=NULL)
//    {
//        MassType* m = this->mass.beginWriteOnly();
//        *m = ((Real)this->totalMass.getValue() / this->mstate->getSize());
//        this->mass.endEdit();
//    }
//    else
//    {
//        this->totalMass.setValue( this->mstate->getSize() * this->mass.getValue().getUniformValue() );
//    }
//}
//#endif
//#ifndef SOFA_DOUBLE
//template<> SOFA_Flexible_API
//void UniformMass<TYPEABSTRACTNAME3fTypes, TYPEABSTRACTNAME3fMass>::reinit()
//{
//    if (this->totalMass.getValue()>0 && this->mstate!=NULL)
//    {
//        MassType* m = this->mass.beginWriteOnly();
//        *m = ((Real)this->totalMass.getValue() / this->mstate->getSize());
//        this->mass.endEdit();
//    }
//    else
//    {
//        this->totalMass.setValue( this->mstate->getSize() * this->mass.getValue().getUniformValue() );
//    }
//}
//#endif

#ifndef SOFA_FLOAT
template <> SOFA_Flexible_API
void UniformMass<defaulttype::TYPEABSTRACTNAME3dTypes, defaulttype::TYPEABSTRACTNAME3dMass>::constructor_message()
{
    serr << "UniformMass on '" << this->templateName() << "' is for debug purpose only and should NOT be used for simulation" << sendl;
}
template <> SOFA_Flexible_API
void UniformMass<defaulttype::TYPEABSTRACTNAME3dTypes, defaulttype::TYPEABSTRACTNAME3dMass>::draw(const core::visual::VisualParams* /*vparams*/)
{
}
template <> SOFA_Flexible_API
SReal UniformMass<defaulttype::TYPEABSTRACTNAME3dTypes, defaulttype::TYPEABSTRACTNAME3dMass>::getPotentialEnergy ( const core::MechanicalParams*, const DataVecCoord& vx  ) const
{
    helper::ReadAccessor<DataVecCoord> x = vx;

    unsigned int ibegin = 0;
    unsigned int iend = x.size();

    if ( d_localRange.getValue() [0] >= 0 )
        ibegin = d_localRange.getValue() [0];

    if ( d_localRange.getValue() [1] >= 0 && ( unsigned int ) d_localRange.getValue() [1]+1 < iend )
        iend = d_localRange.getValue() [1]+1;

    SReal e = 0;
    const MassType& m = d_mass.getValue();
    // gravity
    defaulttype::Vec3d g ( this->getContext()->getGravity() );
    Deriv theGravity;
    theGravity[0]=g[0], theGravity[1]=g[1], theGravity[2]=g[2];

    Deriv mg = m * theGravity;

    for ( unsigned int i=ibegin; i<iend; i++ )
    {
        Deriv translation;
        translation[0]=(float)x[i].getCenter()[0],  translation[0]=(float)x[1].getCenter()[1], translation[2]=(float)x[i].getCenter()[2];
        e -= translation * mg;
    }
    return e;
}
#endif
#ifndef SOFA_DOUBLE
template <> SOFA_Flexible_API
void UniformMass<defaulttype::TYPEABSTRACTNAME3fTypes, defaulttype::TYPEABSTRACTNAME3fMass>::constructor_message()
{
    serr << "UniformMass on '" << this->templateName() << "' is for debug purpose only and should NOT be used for simulation" << sendl;
}
template <> SOFA_Flexible_API
void UniformMass<defaulttype::TYPEABSTRACTNAME3fTypes, defaulttype::TYPEABSTRACTNAME3fMass>::draw(const core::visual::VisualParams* /*vparams*/)
{
}
template <> SOFA_Flexible_API
SReal UniformMass<defaulttype::TYPEABSTRACTNAME3fTypes, defaulttype::TYPEABSTRACTNAME3fMass>::getPotentialEnergy ( const core::MechanicalParams*, const DataVecCoord& vx  ) const
{
    helper::ReadAccessor<DataVecCoord> x = vx;

    unsigned int ibegin = 0;
    unsigned int iend = x.size();

    if ( d_localRange.getValue() [0] >= 0 )
        ibegin = d_localRange.getValue() [0];

    if ( d_localRange.getValue() [1] >= 0 && ( unsigned int ) d_localRange.getValue() [1]+1 < iend )
        iend = d_localRange.getValue() [1]+1;

    SReal e = 0;
    const MassType& m = d_mass.getValue();
    // gravity
    defaulttype::Vec3d g ( this->getContext()->getGravity() );
    Deriv theGravity;
    theGravity[0]=g[0], theGravity[1]=g[1], theGravity[2]=g[2];

    Deriv mg = m * theGravity;

    for ( unsigned int i=ibegin; i<iend; i++ )
    {
        Deriv translation;
        translation[0]=(float)x[i].getCenter()[0],  translation[0]=(float)x[1].getCenter()[1], translation[2]=(float)x[i].getCenter()[2];
        e -= translation * mg;
    }
    return e;
}
#endif

    // ==========================================================================
    // Instanciation

    using namespace sofa::defaulttype;

    SOFA_DECL_CLASS ( EVALUATOR(TYPEABSTRACTNAME,UniformMass) )

    int EVALUATOR(TYPEABSTRACTNAME,UniformMassClass) = core::RegisterObject ( "Define the same mass for all the particles" )
#ifndef SOFA_FLOAT
    .add< UniformMass<TYPEABSTRACTNAME3dTypes,TYPEABSTRACTNAME3dMass> >()
#endif
#ifndef SOFA_DOUBLE
    .add< UniformMass<TYPEABSTRACTNAME3fTypes,TYPEABSTRACTNAME3fMass> >()
#endif
            ;


#ifndef SOFA_FLOAT
    template class SOFA_Flexible_API UniformMass<TYPEABSTRACTNAME3dTypes,TYPEABSTRACTNAME3dMass>;
#endif
#ifndef SOFA_DOUBLE
    template class SOFA_Flexible_API UniformMass<TYPEABSTRACTNAME3fTypes,TYPEABSTRACTNAME3fMass>;
#endif


} // namespace mass


namespace forcefield
{

    SOFA_DECL_CLASS(EVALUATOR(TYPEABSTRACTNAME,RestShapeSpringsForceField))

    // Register in the Factory
    int EVALUATOR(TYPEABSTRACTNAME,RestShapeSpringsForceFieldClass) = core::RegisterObject("Spring attached to rest position")
    #ifndef SOFA_FLOAT
            .add< RestShapeSpringsForceField< defaulttype::TYPEABSTRACTNAME3dTypes > >()
    #endif
    #ifndef SOFA_DOUBLE
            .add< RestShapeSpringsForceField< defaulttype::TYPEABSTRACTNAME3fTypes > >()
    #endif
    ;

    #ifndef SOFA_FLOAT
        template class SOFA_Flexible_API RestShapeSpringsForceField< defaulttype::TYPEABSTRACTNAME3dTypes >;
    #endif
    #ifndef SOFA_DOUBLE
        template class SOFA_Flexible_API RestShapeSpringsForceField< defaulttype::TYPEABSTRACTNAME3fTypes >;
    #endif





    SOFA_DECL_CLASS(EVALUATOR(TYPEABSTRACTNAME,ConstantForceField))

    // Register in the Factory
    int EVALUATOR(TYPEABSTRACTNAME,ConstantForceFieldClass) = core::RegisterObject("Constant forces applied to given degrees of freedom")
    #ifndef SOFA_FLOAT
            .add< ConstantForceField< defaulttype::TYPEABSTRACTNAME3dTypes > >()
    #endif
    #ifndef SOFA_DOUBLE
            .add< ConstantForceField< defaulttype::TYPEABSTRACTNAME3fTypes > >()
    #endif
    ;

    #ifndef SOFA_FLOAT
        template class SOFA_Flexible_API ConstantForceField< defaulttype::TYPEABSTRACTNAME3dTypes >;
    #endif
    #ifndef SOFA_DOUBLE
        template class SOFA_Flexible_API ConstantForceField< defaulttype::TYPEABSTRACTNAME3fTypes >;
    #endif

    SOFA_DECL_CLASS(EVALUATOR(TYPEABSTRACTNAME,UniformVelocityDampingForceField))

    // Register in the Factory
    int EVALUATOR(TYPEABSTRACTNAME,UniformVelocityDampingForceFieldClass) = core::RegisterObject("Uniform velocity damping")
    #ifndef SOFA_FLOAT
            .add< UniformVelocityDampingForceField< defaulttype::TYPEABSTRACTNAME3dTypes > >()
    #endif
    #ifndef SOFA_DOUBLE
            .add< UniformVelocityDampingForceField< defaulttype::TYPEABSTRACTNAME3fTypes > >()
    #endif
    ;

    #ifndef SOFA_FLOAT
    template class SOFA_Flexible_API UniformVelocityDampingForceField<defaulttype::TYPEABSTRACTNAME3dTypes>;
    #endif
    #ifndef SOFA_DOUBLE
    template class SOFA_Flexible_API UniformVelocityDampingForceField<defaulttype::TYPEABSTRACTNAME3fTypes>;
    #endif

} // namespace forcefield


} // namespace component


} // namespace sofa


#include "ComponentSpecializationsUndef.h"
