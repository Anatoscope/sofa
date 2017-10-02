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
#ifndef SOFA_CORE_STATE_INL
#define SOFA_CORE_STATE_INL

#include <sofa/core/State.h>

namespace sofa
{

namespace core
{

template<class DataTypes>
objectmodel::BaseData* State<DataTypes>::baseWrite(VecId v)
{
    switch (v.getType())
    {
    case V_ALL: return NULL;
    case V_COORD: return write(VecCoordId(v));
    case V_DERIV: return write(VecDerivId(v));
    case V_MATDERIV: return write(MatrixDerivId(v));
    }
    return NULL;
}

template<class DataTypes>
const objectmodel::BaseData* State<DataTypes>::baseRead(ConstVecId v) const
{
    switch (v.getType())
    {
    case V_ALL: return NULL;
    case V_COORD: return read(ConstVecCoordId(v));
    case V_DERIV: return read(ConstVecDerivId(v));
    case V_MATDERIV: return read(ConstMatrixDerivId(v));
    }
    return NULL;
}

template<class DataTypes>
void State<DataTypes>::computeBBox(const core::ExecParams* params, bool)
{
    const VecCoord& x = read(ConstVecCoordId::position())->getValue(params);

    sofa::defaulttype::BoundingBox& bbox = *this->f_bbox.beginWriteOnly(params);
    bbox.setFromPointVector( x );
    this->f_bbox.endEdit(params);
}

} // namespace core

} // namespace sofa

#endif
