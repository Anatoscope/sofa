
#include <Flexible/config.h>

#include "ComponentSpecializationsDefines.h"

#include <SofaBoundaryCondition/ProjectToPointConstraint.h>
#include <SofaBoundaryCondition/ProjectToLineConstraint.h>
#include <SofaBoundaryCondition/ProjectToPlaneConstraint.h>
#include <SofaBoundaryCondition/ProjectDirectionConstraint.h>

#include <SofaBoundaryCondition/FixedConstraint.h>
#include <SofaBoundaryCondition/PartialFixedConstraint.h>

#include <SofaConstraint/UncoupledConstraintCorrection.h>

#include <sofa/core/visual/VisualParams.h>


namespace sofa {
namespace component
{
namespace projectiveconstraintset
{

using namespace sofa::defaulttype;
using namespace sofa::helper;

// ==========================================================================
// FixedConstraint

#ifndef SOFA_FLOAT
template<>
void FixedConstraint< TYPEABSTRACTNAME3dTypes >::draw(const core::visual::VisualParams* vparams)
{
    if (!vparams->displayFlags().getShowBehaviorModels()) return;

    const SetIndexArray & indices = f_indices.getValue();
    const VecCoord& x = mstate->read(core::ConstVecCoordId::position())->getValue();

    if( f_drawSize.getValue() == 0) // old classical drawing by points
    {
        std::vector< Vector3 > points;

        if( f_fixAll.getValue()==true )
            for (unsigned i=0; i<x.size(); i++ )
                points.push_back(x[i].getCenter());
        else
        {
            if( x.size() < indices.size() ) for (unsigned i=0; i<x.size(); i++ ) points.push_back(x[indices[i]].getCenter());
            else for (SetIndex::const_iterator it = indices.begin(); it != indices.end(); ++it) points.push_back(x[*it].getCenter());
        }

        vparams->drawTool()->drawPoints(points, 10, Vec<4,float>(1,0.5,0.5,1));
    }
    else
//        vparams->drawTool()->drawSpheres(points, (float)f_drawSize.getValue(), Vec<4,float>(0.2f,0.1f,0.9f,1.0f));
    {
        if( f_fixAll.getValue()==true )
            for (unsigned i=0; i<x.size(); i++ )
            {
                vparams->drawTool()->pushMatrix();
                float glTransform[16];
                x[i].writeOpenGlMatrix ( glTransform );
                vparams->drawTool()->multMatrix( glTransform );
                vparams->drawTool()->scale ( f_drawSize.getValue() );
                vparams->drawTool()->drawFrame ( Vector3(), Quat(), Vector3 ( 1,1,1 ), Vec4f(0,0,1,1) );
                vparams->drawTool()->popMatrix();
            }
        else
        {
            if( x.size() < indices.size() )
                for (unsigned i=0; i<x.size(); i++ )
                {
                    vparams->drawTool()->pushMatrix();
                    float glTransform[16];
                    x[indices[i]].writeOpenGlMatrix ( glTransform );
                    vparams->drawTool()->multMatrix( glTransform );
                    vparams->drawTool()->scale ( f_drawSize.getValue() );
                    vparams->drawTool()->drawFrame ( Vector3(), Quat(), Vector3 ( 1,1,1 ), Vec4f(0,0,1,1) );
                    vparams->drawTool()->popMatrix();
                }
            else for (SetIndex::const_iterator it = indices.begin(); it != indices.end(); ++it)
            {
                vparams->drawTool()->pushMatrix();
                float glTransform[16];
                x[*it].writeOpenGlMatrix ( glTransform );
                vparams->drawTool()->multMatrix( glTransform );
                vparams->drawTool()->scale ( f_drawSize.getValue() );
                vparams->drawTool()->drawFrame ( Vector3(), Quat(), Vector3 ( 1,1,1 ), Vec4f(0,0,1,1) );
                vparams->drawTool()->popMatrix();
            }
        }
    }
}
#endif
#ifndef SOFA_DOUBLE
template<>
void FixedConstraint< TYPEABSTRACTNAME3fTypes >::draw(const core::visual::VisualParams* vparams)
{
    if (!vparams->displayFlags().getShowBehaviorModels()) return;

    const SetIndexArray & indices = f_indices.getValue();
    const VecCoord& x = mstate->read(core::ConstVecCoordId::position())->getValue();

    if( f_drawSize.getValue() == 0) // old classical drawing by points
    {
        std::vector< Vector3 > points;

        if( f_fixAll.getValue()==true )
            for (unsigned i=0; i<x.size(); i++ )
                points.push_back(x[i].getCenter());
        else
        {
            if( x.size() < indices.size() ) for (unsigned i=0; i<x.size(); i++ ) points.push_back(x[indices[i]].getCenter());
            else for (SetIndex::const_iterator it = indices.begin(); it != indices.end(); ++it) points.push_back(x[*it].getCenter());
        }

        vparams->drawTool()->drawPoints(points, 10, Vec<4,float>(1,0.5,0.5,1));
    }
    else
//        vparams->drawTool()->drawSpheres(points, (float)f_drawSize.getValue(), Vec<4,float>(0.2f,0.1f,0.9f,1.0f));
    {
        if( f_fixAll.getValue()==true )
            for (unsigned i=0; i<x.size(); i++ )
            {
                vparams->drawTool()->pushMatrix();
                float glTransform[16];
                x[i].writeOpenGlMatrix ( glTransform );
                vparams->drawTool()->multMatrix( glTransform );
                vparams->drawTool()->scale ( f_drawSize.getValue() );
                vparams->drawTool()->drawFrame ( Vector3(), Quat(), Vector3 ( 1,1,1 ), Vec4f(0,0,1,1) );
                vparams->drawTool()->popMatrix();
            }
        else
        {
            if( x.size() < indices.size() )
                for (unsigned i=0; i<x.size(); i++ )
                {
                    vparams->drawTool()->pushMatrix();
                    float glTransform[16];
                    x[indices[i]].writeOpenGlMatrix ( glTransform );
                    vparams->drawTool()->multMatrix( glTransform );
                    vparams->drawTool()->scale ( f_drawSize.getValue() );
                    vparams->drawTool()->drawFrame ( Vector3(), Quat(), Vector3 ( 1,1,1 ), Vec4f(0,0,1,1) );
                    vparams->drawTool()->popMatrix();
                }
            else for (SetIndex::const_iterator it = indices.begin(); it != indices.end(); ++it)
            {
                vparams->drawTool()->pushMatrix();
                float glTransform[16];
                x[*it].writeOpenGlMatrix ( glTransform );
                vparams->drawTool()->multMatrix( glTransform );
                vparams->drawTool()->scale ( f_drawSize.getValue() );
                vparams->drawTool()->drawFrame ( Vector3(), Quat(), Vector3 ( 1,1,1 ), Vec4f(0,0,1,1) );
                vparams->drawTool()->popMatrix();
            }
        }
    }
}
#endif


// ==========================================================================
// PartialFixedConstraint


#ifndef SOFA_FLOAT
template <>
void PartialFixedConstraint<TYPEABSTRACTNAME3dTypes>::draw(const core::visual::VisualParams* vparams)
{
    if (!vparams->displayFlags().getShowBehaviorModels()) return;

    const SetIndexArray & indices = f_indices.getValue();
    const VecCoord& x = mstate->read(core::ConstVecCoordId::position())->getValue();

    if( _drawSize.getValue() == 0) // old classical drawing by points
    {
        std::vector< Vector3 > points;

        if( f_fixAll.getValue()==true )
            for (unsigned i=0; i<x.size(); i++ )
                points.push_back(x[i].getCenter());
        else
        {
            if( x.size() < indices.size() ) for (unsigned i=0; i<x.size(); i++ ) points.push_back(x[indices[i]].getCenter());
            else for (SetIndex::const_iterator it = indices.begin(); it != indices.end(); ++it) points.push_back(x[*it].getCenter());
        }

        vparams->drawTool()->drawPoints(points, 10, Vec<4,float>(1,0.5,0.5,1));
    }
    else
//        vparams->drawTool()->drawSpheres(points, (float)f_drawSize.getValue(), Vec<4,float>(0.2f,0.1f,0.9f,1.0f));
    {
        if( f_fixAll.getValue()==true )
            for (unsigned i=0; i<x.size(); i++ )
            {
                vparams->drawTool()->pushMatrix();
                float glTransform[16];
                x[i].writeOpenGlMatrix ( glTransform );
                vparams->drawTool()->multMatrix( glTransform );
                vparams->drawTool()->scale ( _drawSize.getValue() );
                vparams->drawTool()->drawFrame ( Vector3(), Quat(), Vector3 ( 1,1,1 ), Vec4f(0,0,1,1) );
                vparams->drawTool()->popMatrix();
            }
        else
        {
            if( x.size() < indices.size() )
                for (unsigned i=0; i<x.size(); i++ )
                {
                    vparams->drawTool()->pushMatrix();
                    float glTransform[16];
                    x[indices[i]].writeOpenGlMatrix ( glTransform );
                    vparams->drawTool()->multMatrix( glTransform );
                    vparams->drawTool()->scale ( _drawSize.getValue() );
                    vparams->drawTool()->drawFrame ( Vector3(), Quat(), Vector3 ( 1,1,1 ), Vec4f(0,0,1,1) );
                    vparams->drawTool()->popMatrix();
                }
            else for (SetIndex::const_iterator it = indices.begin(); it != indices.end(); ++it)
            {
                vparams->drawTool()->pushMatrix();
                float glTransform[16];
                x[*it].writeOpenGlMatrix ( glTransform );
                vparams->drawTool()->multMatrix( glTransform );
                vparams->drawTool()->scale ( _drawSize.getValue() );
                vparams->drawTool()->drawFrame ( Vector3(), Quat(), Vector3 ( 1,1,1 ), Vec4f(0,0,1,1) );
                vparams->drawTool()->popMatrix();
            }
        }
    }
}
#endif
#ifndef SOFA_DOUBLE
template <>
void PartialFixedConstraint<TYPEABSTRACTNAME3fTypes>::draw(const core::visual::VisualParams* vparams)
{
    if (!vparams->displayFlags().getShowBehaviorModels()) return;

    const SetIndexArray & indices = f_indices.getValue();
    const VecCoord& x = mstate->read(core::ConstVecCoordId::position())->getValue();

    if( _drawSize.getValue() == 0) // old classical drawing by points
    {
        std::vector< Vector3 > points;

        if( f_fixAll.getValue()==true )
            for (unsigned i=0; i<x.size(); i++ )
                points.push_back(x[i].getCenter());
        else
        {
            if( x.size() < indices.size() ) for (unsigned i=0; i<x.size(); i++ ) points.push_back(x[indices[i]].getCenter());
            else for (SetIndex::const_iterator it = indices.begin(); it != indices.end(); ++it) points.push_back(x[*it].getCenter());
        }

        vparams->drawTool()->drawPoints(points, 10, Vec<4,float>(1,0.5,0.5,1));
    }
    else
//        vparams->drawTool()->drawSpheres(points, (float)f_drawSize.getValue(), Vec<4,float>(0.2f,0.1f,0.9f,1.0f));
    {
        if( f_fixAll.getValue()==true )
            for (unsigned i=0; i<x.size(); i++ )
            {
                vparams->drawTool()->pushMatrix();
                float glTransform[16];
                x[i].writeOpenGlMatrix ( glTransform );
                vparams->drawTool()->multMatrix( glTransform );
                vparams->drawTool()->scale ( _drawSize.getValue() );
                vparams->drawTool()->drawFrame ( Vector3(), Quat(), Vector3 ( 1,1,1 ), Vec4f(0,0,1,1) );
                vparams->drawTool()->popMatrix();
            }
        else
        {
            if( x.size() < indices.size() )
                for (unsigned i=0; i<x.size(); i++ )
                {
                    vparams->drawTool()->pushMatrix();
                    float glTransform[16];
                    x[indices[i]].writeOpenGlMatrix ( glTransform );
                    vparams->drawTool()->multMatrix( glTransform );
                    vparams->drawTool()->scale ( _drawSize.getValue() );
                    vparams->drawTool()->drawFrame ( Vector3(), Quat(), Vector3 ( 1,1,1 ), Vec4f(0,0,1,1) );
                    vparams->drawTool()->popMatrix();
                }
            else for (SetIndex::const_iterator it = indices.begin(); it != indices.end(); ++it)
            {
                vparams->drawTool()->pushMatrix();
                float glTransform[16];
                x[*it].writeOpenGlMatrix ( glTransform );
                vparams->drawTool()->multMatrix( glTransform );
                vparams->drawTool()->scale ( _drawSize.getValue() );
                vparams->drawTool()->drawFrame ( Vector3(), Quat(), Vector3 ( 1,1,1 ), Vec4f(0,0,1,1) );
                vparams->drawTool()->popMatrix();
            }
        }
    }
}
#endif




// ==========================================================================
// ProjectToPointConstraint


// TODO: jacobians need to be adjusted to complex types for following projective constaints

//// ==========================================================================
//// ProjectToLineConstraint
//SOFA_DECL_CLASS ( EVALUATOR(TYPEABSTRACTNAME,ProjectToLineConstraint) )
//int EVALUATOR(TYPEABSTRACTNAME,ProjectToLineConstraintClass) = core::RegisterObject ( "Project particles to a line" )
//#ifndef SOFA_FLOAT
//.add< ProjectToLineConstraint<defaulttype::TYPEABSTRACTNAME3dTypes> >()
//#endif
//#ifndef SOFA_DOUBLE
//.add< ProjectToLineConstraint<defaulttype::TYPEABSTRACTNAME3fTypes> >()
//#endif
//;
//#ifndef SOFA_FLOAT
//template class SOFA_Flexible_API ProjectToLineConstraint<TYPEABSTRACTNAME3dTypes>;
//#endif
//#ifndef SOFA_DOUBLE
//template class SOFA_Flexible_API ProjectToLineConstraint<TYPEABSTRACTNAME3fTypes>;
//#endif

//// ==========================================================================
//// ProjectToPlaneConstraint
//SOFA_DECL_CLASS ( EVALUATOR(TYPEABSTRACTNAME,ProjectToPlaneConstraint) )
//int EVALUATOR(TYPEABSTRACTNAME,ProjectToPlaneConstraintClass) = core::RegisterObject ( "Project particles to a plane" )
//#ifndef SOFA_FLOAT
//.add< ProjectToPlaneConstraint<defaulttype::TYPEABSTRACTNAME3dTypes> >()
//#endif
//#ifndef SOFA_DOUBLE
//.add< ProjectToPlaneConstraint<defaulttype::TYPEABSTRACTNAME3fTypes> >()
//#endif
//;

//#ifndef SOFA_FLOAT
//template class SOFA_Flexible_API ProjectToPlaneConstraint<TYPEABSTRACTNAME3dTypes>;
//#endif
//#ifndef SOFA_DOUBLE
//template class SOFA_Flexible_API ProjectToPlaneConstraint<TYPEABSTRACTNAME3fTypes>;
//#endif

//// ==========================================================================
//// ProjectDirectionConstraint
//SOFA_DECL_CLASS ( EVALUATOR(TYPEABSTRACTNAME,ProjectDirectionConstraint) )
//int EVALUATOR(TYPEABSTRACTNAME,ProjectDirectionConstraintClass) = core::RegisterObject ( "Project particles to a line" )
//#ifndef SOFA_FLOAT
//.add< ProjectDirectionConstraint<defaulttype::TYPEABSTRACTNAME3dTypes> >()
//#endif
//#ifndef SOFA_DOUBLE
//.add< ProjectDirectionConstraint<defaulttype::TYPEABSTRACTNAME3fTypes> >()
//#endif
//    ;
//#ifndef SOFA_FLOAT
//template class SOFA_Flexible_API ProjectDirectionConstraint<TYPEABSTRACTNAME3dTypes>;
//#endif
//#ifndef SOFA_DOUBLE
//template class SOFA_Flexible_API ProjectDirectionConstraint<TYPEABSTRACTNAME3fTypes>;
//#endif


} // namespace projectiveconstraintset
} // namespace component
} // namespace sofa



#include <sofa/helper/gl/Axis.h>
namespace sofa
{
namespace component
{
namespace container
{

using defaulttype::Vector3;
using defaulttype::Quat;
using defaulttype::Vec4f;

// ==========================================================================
// Draw Specializations
#ifndef SOFA_FLOAT
template <> SOFA_Flexible_API
void MechanicalObject<defaulttype::TYPEABSTRACTNAME3dTypes>::draw(const core::visual::VisualParams* vparams)
{
#ifndef SOFA_NO_OPENGL

    if (!vparams->displayFlags().getShowBehaviorModels()) return;

    if ( showIndices.getValue() )
    {
        drawIndices(vparams);
    }


    if (showObject.getValue())
    {
        const float& scale = showObjectScale.getValue();
        const defaulttype::TYPEABSTRACTNAME3dTypes::VecCoord& x = ( read(core::ConstVecCoordId::position())->getValue() );

        for (size_t i = 0; i < this->getSize(); ++i)
        {
            vparams->drawTool()->pushMatrix();
            float glTransform[16];
            x[i].writeOpenGlMatrix ( glTransform );
            vparams->drawTool()->multMatrix( glTransform );
            vparams->drawTool()->scale ( scale);

            switch( drawMode.getValue() )
            {
                case 1:
                    vparams->drawTool()->drawFrame ( Vector3(), Quat(), Vector3 ( 1,1,1 ), Vec4f(0,1,0,1) );
                    break;
                case 2:
                    vparams->drawTool()->drawFrame ( Vector3(), Quat(), Vector3 ( 1,1,1 ), Vec4f(1,0,0,1) );
                    break;
                case 3:
                    vparams->drawTool()->drawFrame ( Vector3(), Quat(), Vector3 ( 1,1,1 ), Vec4f(0,0,1,1) );
                    break;
                default:
                    vparams->drawTool()->drawFrame ( Vector3(), Quat(), Vector3 ( 1,1,1 ) );
            }

            vparams->drawTool()->popMatrix();
        }
    }
#endif /* SOFA_NO_OPENGL */
}
#endif
#ifndef SOFA_DOUBLE
template <> SOFA_Flexible_API
void MechanicalObject<defaulttype::TYPEABSTRACTNAME3fTypes>::draw(const core::visual::VisualParams* vparams)
{
#ifndef SOFA_NO_OPENGL

    if (!vparams->displayFlags().getShowBehaviorModels()) return;

    if ( showIndices.getValue() )
    {
        drawIndices(vparams);
    }


    if (showObject.getValue())
    {
        const float& scale = showObjectScale.getValue();
        const defaulttype::TYPEABSTRACTNAME3fTypes::VecCoord& x = read(core::ConstVecCoordId::position())->getValue();

        for (size_t i = 0; i < this->getSize(); ++i)
        {
            vparams->drawTool()->pushMatrix();
            float glTransform[16];
            x[i].writeOpenGlMatrix ( glTransform );
            vparams->drawTool()->multMatrix( glTransform );
            vparams->drawTool()->scale ( scale);

            switch( drawMode.getValue() )
            {
                case 1:
                    vparams->drawTool()->drawFrame ( Vector3(), Quat(), Vector3 ( 1,1,1 ), Vec4f(0,1,0,1) );
                    break;
                case 2:
                    vparams->drawTool()->drawFrame ( Vector3(), Quat(), Vector3 ( 1,1,1 ), Vec4f(1,0,0,1) );
                    break;
                case 3:
                    vparams->drawTool()->drawFrame ( Vector3(), Quat(), Vector3 ( 1,1,1 ), Vec4f(0,0,1,1) );
                    break;
                default:
                    vparams->drawTool()->drawFrame ( Vector3(), Quat(), Vector3 ( 1,1,1 ) );
            }

            vparams->drawTool()->popMatrix();
        }
    }
#endif /* SOFA_NO_OPENGL */
}
#endif
// ==========================================================================
// Instanciation


} // namespace container

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


} // namespace mass


namespace constraintset
{
#ifndef SOFA_FLOAT
template<> SOFA_Flexible_API
void UncoupledConstraintCorrection< defaulttype::TYPEABSTRACTNAME3dTypes >::init()
{
    Inherit::init();

    const SReal dt = this->getContext()->getDt();

    const SReal dt2 = dt * dt;

    defaulttype::TYPEABSTRACTNAME3dMass massValue;
    VecReal usedComp;

    sofa::component::mass::UniformMass< defaulttype::TYPEABSTRACTNAME3dTypes, defaulttype::TYPEABSTRACTNAME3dMass >* uniformMass;

    this->getContext()->get( uniformMass, core::objectmodel::BaseContext::SearchUp );
    if( uniformMass )
    {
        massValue = uniformMass->getMass();

        Real H = dt2 / (Real)massValue;

        //for( int i=0 ; i<12 ; ++i )
            usedComp.push_back( H );
    }
    // todo add ImageDensityMass
    /*else
    {
        for( int i=0 ; i<1 ; ++i )
            usedComp.push_back( defaultCompliance.getValue() );
    }*/

    compliance.setValue(usedComp);
}
#endif
#ifndef SOFA_DOUBLE
template<> SOFA_Flexible_API
void UncoupledConstraintCorrection< defaulttype::TYPEABSTRACTNAME3fTypes >::init()
{
    Inherit::init();

    const SReal dt = this->getContext()->getDt();

    const SReal dt2 = dt * dt;

    defaulttype::TYPEABSTRACTNAME3fMass massValue;
    VecReal usedComp;

    sofa::component::mass::UniformMass< defaulttype::TYPEABSTRACTNAME3fTypes, defaulttype::TYPEABSTRACTNAME3fMass >* uniformMass;

    this->getContext()->get( uniformMass, core::objectmodel::BaseContext::SearchUp );
    if( uniformMass )
    {
        massValue = uniformMass->getMass();

        Real H = dt2 / (Real)massValue;

        //for( int i=0 ; i<12 ; ++i )
            usedComp.push_back( H );
    }
    // todo add ImageDensityMass
    /*else
    {
        for( int i=0 ; i<1 ; ++i )
            usedComp.push_back( defaultCompliance.getValue() );
    }*/

    compliance.setValue(usedComp);
}
#endif


} // namespace constraintset


} // namespace component


} // namespace sofa


#include "ComponentSpecializationsUndef.h"
