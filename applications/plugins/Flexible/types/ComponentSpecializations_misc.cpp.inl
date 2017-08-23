

#include <Flexible/config.h>


#include "ComponentSpecializationsDefines.h"

#include <sofa/core/ObjectFactory.h>


#include <SofaBaseMechanics/MechanicalObject.inl>


#include <SofaEngine/BoxROI.inl>


#include <SofaValidation/Monitor.inl>
#include <SofaValidation/ExtraMonitor.inl>

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

SOFA_DECL_CLASS ( EVALUATOR(TYPEABSTRACTNAME,MechanicalObject) )

using namespace sofa::defaulttype;

int EVALUATOR(TYPEABSTRACTNAME,MechanicalObjectClass) = core::RegisterObject ( "mechanical state vectors" )
    #ifndef SOFA_FLOAT
        .add< MechanicalObject<TYPEABSTRACTNAME3dTypes> >()
    #endif
    #ifndef SOFA_DOUBLE
        .add< MechanicalObject<TYPEABSTRACTNAME3fTypes> >()
    #endif
        ;



#ifndef SOFA_FLOAT
    template class SOFA_Flexible_API MechanicalObject<TYPEABSTRACTNAME3dTypes>;
#endif
#ifndef SOFA_DOUBLE
    template class SOFA_Flexible_API MechanicalObject<TYPEABSTRACTNAME3fTypes>;
#endif



} // namespace container

namespace misc
{


SOFA_DECL_CLASS( EVALUATOR(TYPEABSTRACTNAME,Monitor) )
// Register in the Factory
int EVALUATOR(TYPEABSTRACTNAME,MonitorClass) = core::RegisterObject("Monitoring of particles")
#ifndef SOFA_FLOAT
        .add< Monitor<defaulttype::TYPEABSTRACTNAME3dTypes> >()
#endif
#ifndef SOFA_DOUBLE
        .add< Monitor<defaulttype::TYPEABSTRACTNAME3fTypes> >()
#endif
    ;

#ifndef SOFA_FLOAT
    template class SOFA_Flexible_API Monitor<defaulttype::TYPEABSTRACTNAME3dTypes>;
#endif
#ifndef SOFA_DOUBLE
    template class SOFA_Flexible_API Monitor<defaulttype::TYPEABSTRACTNAME3fTypes>;
#endif





SOFA_DECL_CLASS( EVALUATOR(TYPEABSTRACTNAME,ExtraMonitor) )
// Register in the Factory
int EVALUATOR(TYPEABSTRACTNAME,ExtraMonitorClass) = core::RegisterObject("Monitoring of particles")
#ifndef SOFA_FLOAT
    .add< ExtraMonitor<defaulttype::TYPEABSTRACTNAME3dTypes> >()
#endif
#ifndef SOFA_DOUBLE
    .add< ExtraMonitor<defaulttype::TYPEABSTRACTNAME3fTypes> >()
#endif
;



#ifndef SOFA_FLOAT
    template class SOFA_Flexible_API ExtraMonitor<defaulttype::TYPEABSTRACTNAME3dTypes>;
#endif
#ifndef SOFA_DOUBLE
    template class SOFA_Flexible_API ExtraMonitor<defaulttype::TYPEABSTRACTNAME3fTypes>;
#endif



} // namespace misc

namespace engine
{
    SOFA_DECL_CLASS(EVALUATOR(TYPEABSTRACTNAME,BoxROI))

    // Register in the Factory
    int EVALUATOR(TYPEABSTRACTNAME,BoxROIClass) = core::RegisterObject("Find the primitives (vertex/edge/triangle/tetrahedron) inside a given box")
#ifndef SOFA_FLOAT
            .add< BoxROI< defaulttype::TYPEABSTRACTNAME3dTypes > >()
#endif
#ifndef SOFA_DOUBLE
            .add< BoxROI< defaulttype::TYPEABSTRACTNAME3fTypes > >()
#endif
    ;


#ifndef SOFA_FLOAT
    template class SOFA_Flexible_API boxroi::BoxROI< defaulttype::TYPEABSTRACTNAME3dTypes >;
#endif
#ifndef SOFA_DOUBLE
    template class SOFA_Flexible_API boxroi::BoxROI< defaulttype::TYPEABSTRACTNAME3fTypes >;
#endif

} // namespace engine

} // namespace component

} // namespace sofa


#include "ComponentSpecializationsUndef.h"
