

#include <Flexible/config.h>


#include "ComponentSpecializationsDefines.h"

#include <SofaBoundaryCondition/ProjectToPointConstraint.inl>
#include <SofaBoundaryCondition/ProjectToLineConstraint.inl>
#include <SofaBoundaryCondition/ProjectToPlaneConstraint.inl>
#include <SofaBoundaryCondition/ProjectDirectionConstraint.inl>

#include <sofa/core/ObjectFactory.h>

#include <SofaBoundaryCondition/FixedConstraint.inl>
#include <SofaBoundaryCondition/PartialFixedConstraint.inl>
#include <sofa/core/behavior/ProjectiveConstraintSet.inl>

#include <SofaConstraint/UncoupledConstraintCorrection.inl>


namespace sofa
{

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


SOFA_DECL_CLASS ( EVALUATOR(TYPEABSTRACTNAME,FixedConstraint) )
int EVALUATOR(TYPEABSTRACTNAME,FixedConstraintClass) = core::RegisterObject ( "Attach given dofs to their initial positions" )
        #ifndef SOFA_FLOAT
                .add< FixedConstraint<defaulttype::TYPEABSTRACTNAME3dTypes> >()
    #endif
    #ifndef SOFA_DOUBLE
                .add< FixedConstraint<defaulttype::TYPEABSTRACTNAME3fTypes> >()
    #endif
        ;
#ifndef SOFA_FLOAT
template class SOFA_Flexible_API FixedConstraint<TYPEABSTRACTNAME3dTypes>;
#endif
#ifndef SOFA_DOUBLE
template class SOFA_Flexible_API FixedConstraint<TYPEABSTRACTNAME3fTypes>;
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

SOFA_DECL_CLASS ( EVALUATOR(TYPEABSTRACTNAME,PartialFixedConstraint) )
int EVALUATOR(TYPEABSTRACTNAME,PartialFixedConstraintClass) = core::RegisterObject ( "Attach given cinematic dofs to their initial positions" )
        #ifndef SOFA_FLOAT
        .add< PartialFixedConstraint<defaulttype::TYPEABSTRACTNAME3dTypes> >()
    #endif
    #ifndef SOFA_DOUBLE
        .add< PartialFixedConstraint<defaulttype::TYPEABSTRACTNAME3fTypes> >()
    #endif
;
#ifndef SOFA_FLOAT
template class SOFA_Flexible_API PartialFixedConstraint<TYPEABSTRACTNAME3dTypes>;
#endif
#ifndef SOFA_DOUBLE
template class SOFA_Flexible_API PartialFixedConstraint<TYPEABSTRACTNAME3fTypes>;
#endif




// ==========================================================================
// ProjectToPointConstraint
SOFA_DECL_CLASS ( EVALUATOR(TYPEABSTRACTNAME,ProjectToPointConstraint) )
int EVALUATOR(TYPEABSTRACTNAME,ProjectToPointConstraintClass) = core::RegisterObject ( "Project particles to a point" )
#ifndef SOFA_FLOAT
        .add< ProjectToPointConstraint<defaulttype::TYPEABSTRACTNAME3dTypes> >()
#endif
#ifndef SOFA_DOUBLE
.add< ProjectToPointConstraint<defaulttype::TYPEABSTRACTNAME3fTypes> >()
#endif
        ;
#ifndef SOFA_FLOAT
template class SOFA_Flexible_API ProjectToPointConstraint<TYPEABSTRACTNAME3dTypes>;
#endif
#ifndef SOFA_DOUBLE
template class SOFA_Flexible_API ProjectToPointConstraint<TYPEABSTRACTNAME3fTypes>;
#endif


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

SOFA_DECL_CLASS( EVALUATOR(TYPEABSTRACTNAME,UncoupledConstraintCorrection) )
// Register in the Factory
int EVALUATOR(TYPEABSTRACTNAME,UncoupledConstraintCorrectionClass) = core::RegisterObject("Component computing contact forces within a simulated body using the compliance method.")
#ifndef SOFA_FLOAT
    .add< UncoupledConstraintCorrection<defaulttype::TYPEABSTRACTNAME3dTypes> >()
#endif
#ifndef SOFA_DOUBLE
    .add< UncoupledConstraintCorrection<defaulttype::TYPEABSTRACTNAME3fTypes> >()
#endif
        ;

#ifndef SOFA_FLOAT
    template class SOFA_Flexible_API UncoupledConstraintCorrection<defaulttype::TYPEABSTRACTNAME3dTypes>;
#endif
#ifndef SOFA_DOUBLE
    template class SOFA_Flexible_API UncoupledConstraintCorrection<defaulttype::TYPEABSTRACTNAME3fTypes>;
#endif


} // namespace constraintset


} // namespace component


} // namespace sofa


#include "ComponentSpecializationsUndef.h"
