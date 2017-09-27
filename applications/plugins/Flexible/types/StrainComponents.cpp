/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2017 INRIA, USTL, UJF, CNRS, MGH                    *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#define FLEXIBLE_StrainComponents_CPP

#include "../types/StrainComponents.h"
#include <sofa/core/ObjectFactory.h>

#include <sofa/core/State.inl>
#include <SofaBaseMechanics/MechanicalObject.inl>


using namespace sofa::defaulttype;

namespace sofa
{


namespace core
{

#ifndef SOFA_FLOAT

template <> void State<E331dTypes>::computeBBox(const core::ExecParams*, bool) {}
template <> void State<E321dTypes>::computeBBox(const core::ExecParams*, bool) {}
template <> void State<E311dTypes>::computeBBox(const core::ExecParams*, bool) {}
template <> void State<E332dTypes>::computeBBox(const core::ExecParams*, bool) {}
template <> void State<E333dTypes>::computeBBox(const core::ExecParams*, bool) {}
template <> void State<E221dTypes>::computeBBox(const core::ExecParams*, bool) {}
template <> void State<I331dTypes>::computeBBox(const core::ExecParams*, bool) {}
template <> void State<U331dTypes>::computeBBox(const core::ExecParams*, bool) {}
template <> void State<U321dTypes>::computeBBox(const core::ExecParams*, bool) {}


template class SOFA_Flexible_API State<E331dTypes>;
template class SOFA_Flexible_API State<E321dTypes>;
template class SOFA_Flexible_API State<E311dTypes>;
template class SOFA_Flexible_API State<E332dTypes>;
template class SOFA_Flexible_API State<E333dTypes>;
template class SOFA_Flexible_API State<E221dTypes>;
template class SOFA_Flexible_API State<I331dTypes>;
template class SOFA_Flexible_API State<U331dTypes>;
template class SOFA_Flexible_API State<U321dTypes>;
#endif
#ifndef SOFA_DOUBLE


template <> void State<E331fTypes>::computeBBox(const core::ExecParams*, bool) {}
template <> void State<E321fTypes>::computeBBox(const core::ExecParams*, bool) {}
template <> void State<E311fTypes>::computeBBox(const core::ExecParams*, bool) {}
template <> void State<E332fTypes>::computeBBox(const core::ExecParams*, bool) {}
template <> void State<E333fTypes>::computeBBox(const core::ExecParams*, bool) {}
template <> void State<E221fTypes>::computeBBox(const core::ExecParams*, bool) {}
template <> void State<I331fTypes>::computeBBox(const core::ExecParams*, bool) {}
template <> void State<U331fTypes>::computeBBox(const core::ExecParams*, bool) {}
template <> void State<U321fTypes>::computeBBox(const core::ExecParams*, bool) {}

template class SOFA_Flexible_API State<E331fTypes>;
template class SOFA_Flexible_API State<E321fTypes>;
template class SOFA_Flexible_API State<E311fTypes>;
template class SOFA_Flexible_API State<E332fTypes>;
template class SOFA_Flexible_API State<E333fTypes>;
template class SOFA_Flexible_API State<E221fTypes>;
template class SOFA_Flexible_API State<I331fTypes>;
template class SOFA_Flexible_API State<U331fTypes>;
template class SOFA_Flexible_API State<U321fTypes>;
#endif

} // namespace core


namespace component
{
namespace container
{

// ==========================================================================
// Instanciation

SOFA_DECL_CLASS ( StrainMechanicalObject )


int StrainMechanicalObjectClass = core::RegisterObject ( "mechanical state vectors" )
#ifndef SOFA_FLOAT
        .add< MechanicalObject<E331dTypes> >()
        .add< MechanicalObject<E321dTypes> >()
        .add< MechanicalObject<E311dTypes> >()
        .add< MechanicalObject<E332dTypes> >()
        .add< MechanicalObject<E333dTypes> >()
        .add< MechanicalObject<E221dTypes> >()

//        .add< MechanicalObject<D331dTypes> >()
//        .add< MechanicalObject<D321dTypes> >()
//        .add< MechanicalObject<D332dTypes> >()
//        .add< MechanicalObject<D333dTypes> >()

        .add< MechanicalObject<I331dTypes> >()
//.add< MechanicalObject<I332dTypes> >()
//.add< MechanicalObject<I333dTypes> >()

        .add< MechanicalObject<U331dTypes> >()
        .add< MechanicalObject<U321dTypes> >()
#endif
#ifndef SOFA_DOUBLE
        .add< MechanicalObject<E331fTypes> >()
        .add< MechanicalObject<E321fTypes> >()
        .add< MechanicalObject<E311fTypes> >()
        .add< MechanicalObject<E332fTypes> >()
        .add< MechanicalObject<E333fTypes> >()
        .add< MechanicalObject<E221fTypes> >()

//        .add< MechanicalObject<D331fTypes> >()
//        .add< MechanicalObject<D321fTypes> >()
//        .add< MechanicalObject<D332fTypes> >()
//        .add< MechanicalObject<D333fTypes> >()

        .add< MechanicalObject<I331fTypes> >()
//.add< MechanicalObject<I332fTypes> >()
//.add< MechanicalObject<I333fTypes> >()

        .add< MechanicalObject<U331fTypes> >()
        .add< MechanicalObject<U321fTypes> >()
#endif
        ;

#ifndef SOFA_FLOAT

template <> void MechanicalObject<E331dTypes>::draw(const core::visual::VisualParams*) {}
template <> void MechanicalObject<E321dTypes>::draw(const core::visual::VisualParams*) {}
template <> void MechanicalObject<E311dTypes>::draw(const core::visual::VisualParams*) {}
template <> void MechanicalObject<E332dTypes>::draw(const core::visual::VisualParams*) {}
template <> void MechanicalObject<E333dTypes>::draw(const core::visual::VisualParams*) {}
template <> void MechanicalObject<E221dTypes>::draw(const core::visual::VisualParams*) {}
template <> void MechanicalObject<I331dTypes>::draw(const core::visual::VisualParams*) {}
template <> void MechanicalObject<U331dTypes>::draw(const core::visual::VisualParams*) {}
template <> void MechanicalObject<U321dTypes>::draw(const core::visual::VisualParams*) {}


template class SOFA_Flexible_API MechanicalObject<E331dTypes>;
template class SOFA_Flexible_API MechanicalObject<E321dTypes>;
template class SOFA_Flexible_API MechanicalObject<E311dTypes>;
template class SOFA_Flexible_API MechanicalObject<E332dTypes>;
template class SOFA_Flexible_API MechanicalObject<E333dTypes>;
template class SOFA_Flexible_API MechanicalObject<E221dTypes>;

//template class SOFA_Flexible_API MechanicalObject<D331dTypes>;
//template class SOFA_Flexible_API MechanicalObject<D321dTypes>;
//template class SOFA_Flexible_API MechanicalObject<D332dTypes>;
//template class SOFA_Flexible_API MechanicalObject<D333dTypes>;

template class SOFA_Flexible_API MechanicalObject<I331dTypes>;
//template class SOFA_Flexible_API MechanicalObject<I332dTypes>;
//template class SOFA_Flexible_API MechanicalObject<I333dTypes>;

template class SOFA_Flexible_API MechanicalObject<U331dTypes>;
template class SOFA_Flexible_API MechanicalObject<U321dTypes>;
#endif
#ifndef SOFA_DOUBLE

template <> void MechanicalObject<E331fTypes>::draw(const core::visual::VisualParams*) {}
template <> void MechanicalObject<E321fTypes>::draw(const core::visual::VisualParams*) {}
template <> void MechanicalObject<E311fTypes>::draw(const core::visual::VisualParams*) {}
template <> void MechanicalObject<E332fTypes>::draw(const core::visual::VisualParams*) {}
template <> void MechanicalObject<E333fTypes>::draw(const core::visual::VisualParams*) {}
template <> void MechanicalObject<E221fTypes>::draw(const core::visual::VisualParams*) {}
template <> void MechanicalObject<I331fTypes>::draw(const core::visual::VisualParams*) {}
template <> void MechanicalObject<U331fTypes>::draw(const core::visual::VisualParams*) {}
template <> void MechanicalObject<U321fTypes>::draw(const core::visual::VisualParams*) {}

template class SOFA_Flexible_API MechanicalObject<E331fTypes>;
template class SOFA_Flexible_API MechanicalObject<E321fTypes>;
template class SOFA_Flexible_API MechanicalObject<E311fTypes>;
template class SOFA_Flexible_API MechanicalObject<E332fTypes>;
template class SOFA_Flexible_API MechanicalObject<E333fTypes>;
template class SOFA_Flexible_API MechanicalObject<E221fTypes>;

//template class SOFA_Flexible_API MechanicalObject<D331fTypes>;
//template class SOFA_Flexible_API MechanicalObject<D321fTypes>;
//template class SOFA_Flexible_API MechanicalObject<D332fTypes>;
//template class SOFA_Flexible_API MechanicalObject<D333fTypes>;

template class SOFA_Flexible_API MechanicalObject<I331fTypes>;
//template class SOFA_Flexible_API MechanicalObject<I332fTypes>;
//template class SOFA_Flexible_API MechanicalObject<I333fTypes>;

template class SOFA_Flexible_API MechanicalObject<U331fTypes>;
template class SOFA_Flexible_API MechanicalObject<U321fTypes>;
#endif

} // namespace container
} // namespace component
} // namespace sofa
