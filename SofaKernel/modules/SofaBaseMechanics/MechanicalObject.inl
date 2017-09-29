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
#ifndef SOFA_COMPONENT_MECHANICALOBJECT_INL
#define SOFA_COMPONENT_MECHANICALOBJECT_INL

#include <SofaBaseMechanics/MechanicalObject.h>
#include <sofa/core/visual/VisualParams.h>
#ifdef SOFA_SMP
#include <SofaBaseMechanics/MechanicalObjectTasks.inl>
#endif
#include <SofaBaseLinearSolver/SparseMatrix.h>
#include <sofa/core/topology/BaseTopology.h>
#include <sofa/core/topology/TopologyChange.h>

#include <sofa/defaulttype/DataTypeInfo.h>

#include <sofa/helper/accessor.h>

#include <sofa/simulation/Node.h>
#include <sofa/simulation/Simulation.h>
#ifdef SOFA_DUMP_VISITOR_INFO
#include <sofa/simulation/Visitor.h>
#endif

#include <assert.h>
#include <iostream>

namespace {

template<class V>
void renumber(V* v, V* tmp, const sofa::helper::vector< unsigned int > &index )
{
    if (v == NULL)
        return;

    if (v->empty())
        return;

    *tmp = *v;
    for (unsigned int i = 0; i < v->size(); ++i)
        (*v)[i] = (*tmp)[index[i]];
}

} // anonymous namespace


namespace sofa {

namespace component {

namespace container {

template <class DataTypes>
MechanicalObject<DataTypes>::MechanicalObject()
    : x(initData(&x, "position", "position coordinates of the degrees of freedom"))
    , v(initData(&v, "velocity", "velocity coordinates of the degrees of freedom"))
    , f(initData(&f, "force", "force vector of the degrees of freedom"))
    , externalForces(initData(&externalForces, "externalForce", "externalForces vector of the degrees of freedom"))
    , dx(initData(&dx, "derivX", "dx vector of the degrees of freedom"))
    , x0(initData(&x0, "rest_position", "rest position coordinates of the degrees of freedom"))
    , reset_position(initData(&reset_position, "reset_position", "reset position coordinates of the degrees of freedom"))
    , reset_velocity(initData(&reset_velocity, "reset_velocity", "reset velocity coordinates of the degrees of freedom"))
    , restScale(initData(&restScale, (SReal)1.0, "restScale", "optional scaling of rest position coordinates (to simulated pre-existing internal tension).(default = 1.0)"))
    , showObject(initData(&showObject, (bool) false, "showObject", "Show objects. (default=false)"))
    , showObjectScale(initData(&showObjectScale, (float) 0.1, "showObjectScale", "Scale for object display. (default=0.1)"))
    , showIndices(initData(&showIndices, (bool) false, "showIndices", "Show indices. (default=false)"))
    , showIndicesScale(initData(&showIndicesScale, (float) 0.02, "showIndicesScale", "Scale for indices display. (default=0.02)"))
    , showVectors(initData(&showVectors, (bool) false, "showVectors", "Show velocity. (default=false)"))
    , showVectorsScale(initData(&showVectorsScale, (float) 0.0001, "showVectorsScale", "Scale for vectors display. (default=0.0001)"))
    , drawMode(initData(&drawMode,0,"drawMode","The way vectors will be drawn:\n- 0: Line\n- 1:Cylinder\n- 2: Arrow.\n\nThe DOFS will be drawn:\n- 0: point\n- >1: sphere. (default=0)"))
    , d_color(initData(&d_color, defaulttype::Vec4f(1,1,1,1), "showColor", "Color for object display. (default=[1 1 1 1])"))
    , isToPrint( initData(&isToPrint, false, "isToPrint", "suppress somes data before using save as function. (default=false)"))
    , filename(initData(&filename, std::string(""), "filename", "File corresponding to the Mechanical Object", false))
    , ignoreLoader(initData(&ignoreLoader, (bool) false, "ignoreLoader", "Is the Mechanical Object do not use a loader. (default=false)"))
    , f_reserve(initData(&f_reserve, 0, "reserve", "Size to reserve when creating vectors. (default=0)"))
    , vsize(0)
{
    // HACK
    if (!restScale.isSet())
    {
        restScale.setValue(1);
    }

    m_initialized = false;

    data = MechanicalObjectInternalData<DataTypes>(this);

    x               .setGroup("Vector");
    v               .setGroup("Vector");
    f               .setGroup("Vector");
    externalForces  .setGroup("Vector");
    dx              .setGroup("Vector");
    x0              .setGroup("Vector");
    reset_position  .setGroup("Vector");
    reset_velocity  .setGroup("Vector");

    setVecCoord(core::VecCoordId::position().index, &x);
    setVecCoord(core::VecCoordId::restPosition().index, &x0);

    setVecDeriv(core::VecDerivId::velocity().index, &v);
    setVecDeriv(core::VecDerivId::dx().index, &dx);
    
    setVecDeriv(core::VecDerivId::force().index, &f);
    setVecDeriv(core::VecDerivId::externalForce().index, &externalForces);
    
    setVecCoord(core::VecCoordId::resetPosition().index, &reset_position);    
    setVecDeriv(core::VecDerivId::resetVelocity().index, &reset_velocity);
    
    // These vectors are set as modified as they are mandatory in the MechanicalObject.
    x               .forceSet();
    v               .forceSet();
    f               .forceSet();
    externalForces  .forceSet();

    // there are not
    //  x0              .forceSet();
    //  dx              .forceSet();

    resize(1);
}


template <class DataTypes>
MechanicalObject<DataTypes>::~MechanicalObject()
{

    for(unsigned i=core::VecCoordId::V_FIRST_DYNAMIC_INDEX; i<vectorsCoord.size(); i++)
        if( vectorsCoord[i] != NULL ) { delete vectorsCoord[i]; vectorsCoord[i]=NULL; }
    if( vectorsCoord[core::VecCoordId::null().getIndex()] != NULL )
        { delete vectorsCoord[core::VecCoordId::null().getIndex()]; vectorsCoord[core::VecCoordId::null().getIndex()] = NULL; }

    for(unsigned i=core::VecDerivId::V_FIRST_DYNAMIC_INDEX; i<vectorsDeriv.size(); i++)
        if( vectorsDeriv[i] != NULL )  { delete vectorsDeriv[i]; vectorsDeriv[i]=NULL; }
    if( vectorsDeriv[core::VecDerivId::null().getIndex()] != NULL )
        { delete vectorsDeriv[core::VecDerivId::null().getIndex()]; vectorsDeriv[core::VecDerivId::null().getIndex()] = NULL; }
    if( core::VecDerivId::dforce().getIndex()<vectorsDeriv.size() && vectorsDeriv[core::VecDerivId::dforce().getIndex()] != NULL )
        { delete vectorsDeriv[core::VecDerivId::dforce().getIndex()]; vectorsDeriv[core::VecDerivId::dforce().getIndex()] = NULL; }

    // for(unsigned i=core::MatrixDerivId::V_FIRST_DYNAMIC_INDEX; i<vectorsMatrixDeriv.size(); i++)
    //     if( vectorsMatrixDeriv[i] != NULL )  { delete vectorsMatrixDeriv[i]; vectorsMatrixDeriv[i]=NULL; }
}

template <class DataTypes>
MechanicalObject<DataTypes> &MechanicalObject<DataTypes>::operator = (const MechanicalObject& obj)
{
    resize(obj.getSize());

    return *this;
}


template <class DataTypes>
void MechanicalObject<DataTypes>::parse ( sofa::core::objectmodel::BaseObjectDescription* arg )
{
    Inherited::parse(arg);

    if (arg->getAttribute("size") != NULL)
    {
        int newsize = arg->getAttributeAsInt("size", 1) ;
        if(newsize>=0) {
            resize(newsize) ;
        } else {
            msg_warning() << "The attribute 'size' cannot have a negative value.  "
                "The value "<<newsize<<" is ignored. Current value is " <<getSize()<< ".  "
                "To remove this warning you need to fix your scene.";
        }
    }
    

}



template <class DataTypes>
void MechanicalObject<DataTypes>::handleStateChange()
{
    using sofa::core::topology::TopologyChange;
    using sofa::core::topology::TopologyChangeType;
    using sofa::core::topology::PointsAdded;
    using sofa::core::topology::PointsMoved;
    using sofa::core::topology::PointsRemoved;
    using sofa::core::topology::PointsRenumbering;
    sofa::core::topology::GeometryAlgorithms *geoAlgo = NULL;
    this->getContext()->get(geoAlgo, sofa::core::objectmodel::BaseContext::Local);

    std::list< const TopologyChange * >::const_iterator itBegin = m_topology->beginStateChange();
    std::list< const TopologyChange * >::const_iterator itEnd = m_topology->endStateChange();

    while( itBegin != itEnd )
    {
        TopologyChangeType changeType = (*itBegin)->getChangeType();

        switch( changeType )
        {
        case core::topology::POINTSADDED:
        {
            using sofa::helper::vector;
            const PointsAdded &pointsAdded = *static_cast< const PointsAdded * >( *itBegin );

            unsigned int prevSizeMechObj = getSize();
            unsigned int nbPoints = pointsAdded.getNbAddedVertices();

            if (pointsAdded.pointIndexArray.size() != nbPoints)
            {
                serr << "TOPO STATE EVENT POINTSADDED SIZE MISMATCH: "
                     << nbPoints << " != " << pointsAdded.pointIndexArray.size() << sendl;
            }
            for (unsigned int i=0; i<pointsAdded.pointIndexArray.size(); ++i)
            {
                unsigned int p1 = prevSizeMechObj + i;
                unsigned int p2 = pointsAdded.pointIndexArray[i];
                if (p1 != p2)
                {
                    dmsg_error(this) << "TOPO STATE EVENT POINTSADDED INDEX " << i << " MISMATCH: "
                                     << p1 << " != " << p2 << ".\n";
                }
            }

            vector< vector< unsigned int > > ancestors = pointsAdded.ancestorsList;
            vector< vector< double       > > coefs     = pointsAdded.coefs;

            resize(prevSizeMechObj + nbPoints);

            if (!ancestors.empty() )
            {
                vector< vector< double > > coefs2;
                coefs2.resize(ancestors.size());

                for (unsigned int i = 0; i < ancestors.size(); ++i)
                {
                    coefs2[i].resize(ancestors[i].size());

                    for (unsigned int j = 0; j < ancestors[i].size(); ++j)
                    {
                        // constructng default coefs if none were defined
                        if (coefs == (const vector< vector< double > >)0 || coefs[i].size() == 0)
                            coefs2[i][j] = 1.0f / ancestors[i].size();
                        else
                            coefs2[i][j] = coefs[i][j];
                    }
                }

                for (unsigned int i = 0; i < ancestors.size(); ++i)
                {
                    computeWeightedValue( prevSizeMechObj + i, ancestors[i], coefs2[i] );
                }
            }

            if (!pointsAdded.ancestorElems.empty() && (geoAlgo != NULL))
            {
                helper::vector< core::VecCoordId > coordVecs;
                helper::vector< core::VecDerivId > derivVecs;

                for (unsigned int k = 0; k < vectorsCoord.size(); k++)
                {
                    if (vectorsCoord[k] != NULL)
                    {
                        const VecCoord &vecCoord = vectorsCoord[k]->getValue();

                        if (vecCoord.size() != 0)
                        {
                            coordVecs.push_back(k);
                        }
                    }
                }

                for (unsigned int k = 0; k < vectorsDeriv.size(); k++)
                {
                    if (vectorsDeriv[k] != NULL)
                    {
                        const VecDeriv &vecDeriv = vectorsDeriv[k]->getValue();

                        if (vecDeriv.size() != 0)
                        {
                            derivVecs.push_back(k);
                        }
                    }
                }

                geoAlgo->initPointsAdded(pointsAdded.pointIndexArray, pointsAdded.ancestorElems, coordVecs, derivVecs);
            }

            break;
        }
        case core::topology::POINTSREMOVED:
        {
            const sofa::helper::vector<unsigned int> tab = ( static_cast< const PointsRemoved * >( *itBegin ) )->getArray();

            unsigned int prevSizeMechObj   = getSize();
            unsigned int lastIndexMech = prevSizeMechObj - 1;
            for (unsigned int i = 0; i < tab.size(); ++i)
            {
                replaceValue(lastIndexMech, tab[i] );

                --lastIndexMech;
            }
            resize( prevSizeMechObj - tab.size() );
            break;
        }
        case core::topology::POINTSMOVED:
        {
            using sofa::helper::vector;

            const vector< unsigned int > indicesList = ( static_cast <const PointsMoved *> (*itBegin))->indicesList;
            const vector< vector< unsigned int > > ancestors = ( static_cast< const PointsMoved * >( *itBegin ) )->ancestorsList;
            const vector< vector< double > > coefs = ( static_cast< const PointsMoved * >( *itBegin ) )->baryCoefsList;

            if (ancestors.size() != indicesList.size() || ancestors.empty())
            {
                this->serr << "Error ! MechanicalObject::POINTSMOVED topological event, bad inputs (inputs don't share the same size or are empty)."<<this->sendl;
                break;
            }

            vector< vector < double > > coefs2;
            coefs2.resize (coefs.size());

            for (unsigned int i = 0; i<ancestors.size(); ++i)
            {
                coefs2[i].resize(ancestors[i].size());

                for (unsigned int j = 0; j < ancestors[i].size(); ++j)
                {
                    // constructng default coefs if none were defined
                    if (coefs == (const vector< vector< double > >)0 || coefs[i].size() == 0)
                        coefs2[i][j] = 1.0f / ancestors[i].size();
                    else
                        coefs2[i][j] = coefs[i][j];
                }
            }

            for (unsigned int i = 0; i < indicesList.size(); ++i)
            {
                computeWeightedValue( indicesList[i], ancestors[i], coefs2[i] );
            }

            break;
        }
        case core::topology::POINTSRENUMBERING:
        {
            const sofa::helper::vector<unsigned int> &tab = ( static_cast< const PointsRenumbering * >( *itBegin ) )->getIndexArray();

            renumberValues( tab );
            break;
        }
        default:
            // Ignore events that are not Point-related.
            break;
        };

        ++itBegin;
    }
    //#endif
}

template <class DataTypes>
void MechanicalObject<DataTypes>::replaceValue (const int inputIndex, const int outputIndex)
{
    //const unsigned int maxIndex = std::max(inputIndex, outputIndex);
    const unsigned int maxIndex = inputIndex<outputIndex ? outputIndex : inputIndex;
    const unsigned int vecCoordSize = vectorsCoord.size();
    for (unsigned int i = 0; i < vecCoordSize; i++)
    {
        if (vectorsCoord[i] != NULL)
        {
            VecCoord& vector = *(vectorsCoord[i]->beginEdit());

            if (vector.size() > maxIndex)
                vector[outputIndex] = vector[inputIndex];

            vectorsCoord[i]->endEdit();
        }
    }

    const unsigned int vecDerivSize = vectorsDeriv.size();
    for (unsigned int i = 0; i < vecDerivSize; i++)
    {
        if (vectorsDeriv[i] != NULL)
        {
            VecDeriv& vector = *(vectorsDeriv[i]->beginEdit());

            if (vector.size() > maxIndex)
                vector[outputIndex] = vector[inputIndex];

            vectorsDeriv[i]->endEdit();
        }
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::swapValues (const int idx1, const int idx2)
{
    //const unsigned int maxIndex = std::max(idx1, idx2);
    const unsigned int maxIndex = idx1<idx2 ? idx2 : idx1;

    Coord tmp;
    Deriv tmp2;
    unsigned int i;
    for (i=0; i<vectorsCoord.size(); i++)
    {
        if(vectorsCoord[i] != NULL)
        {
            VecCoord& vector = *vectorsCoord[i]->beginEdit();
            if(vector.size() > maxIndex)
            {
                tmp = vector[idx1];
                vector[idx1] = vector[idx2];
                vector[idx2] = tmp;
            }
            vectorsCoord[i]->endEdit();
        }
    }
    for (i=0; i<vectorsDeriv.size(); i++)
    {
        if(vectorsDeriv[i] != NULL)
        {
            VecDeriv& vector = *vectorsDeriv[i]->beginEdit();
            if(vector.size() > maxIndex)
            {
                tmp2 = vector[idx1];
                vector[idx1] = vector[idx2];
                vector[idx2] = tmp2;
            }
            vectorsDeriv[i]->endEdit();
        }
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::renumberValues( const sofa::helper::vector< unsigned int > &index )
{
    VecDeriv dtmp;
    VecCoord ctmp;

    for (unsigned int i = 0; i < vectorsCoord.size(); ++i)
    {
        if (vectorsCoord[i] != NULL)
        {
            renumber(vectorsCoord[i]->beginEdit(), &ctmp, index);
            vectorsCoord[i]->endEdit();
        }
    }

    for (unsigned int i = 0; i < vectorsDeriv.size(); ++i)
    {
        if (vectorsDeriv[i] != NULL)
        {
            renumber(vectorsDeriv[i]->beginEdit(), &dtmp, index);
            vectorsDeriv[i]->endEdit();
        }
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::resize(const size_t size)
{

    if(size>0)
    {
        //if (size!=vsize)
        {
            vsize = size;
            for (unsigned int i = 0; i < vectorsCoord.size(); i++)
            {
                if (vectorsCoord[i] != NULL && vectorsCoord[i]->isSet())
                {
                    vectorsCoord[i]->beginWriteOnly()->resize(size);
                    vectorsCoord[i]->endEdit();
                }
            }

            for (unsigned int i = 0; i < vectorsDeriv.size(); i++)
            {
                if (vectorsDeriv[i] != NULL && vectorsDeriv[i]->isSet())
                {
                    vectorsDeriv[i]->beginWriteOnly()->resize(size);
                    vectorsDeriv[i]->endEdit();
                }
            }
        }
        this->forceMask.resize(size);
    }
    else // clear
    {
        vsize = 0;
        for (unsigned int i = 0; i < vectorsCoord.size(); i++)
        {
            if (vectorsCoord[i] != NULL && vectorsCoord[i]->isSet())
            {
                vectorsCoord[i]->beginWriteOnly()->clear();
                vectorsCoord[i]->endEdit();
            }
        }

        for (unsigned int i = 0; i < vectorsDeriv.size(); i++)
        {
            if (vectorsDeriv[i] != NULL && vectorsDeriv[i]->isSet())
            {
                vectorsDeriv[i]->beginWriteOnly()->clear();
                vectorsDeriv[i]->endEdit();
            }
        }
        this->forceMask.clear();
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::reserve(const size_t size)
{
    if (size == 0) return;

    for (unsigned int i = 0; i < vectorsCoord.size(); i++)
    {
        if (vectorsCoord[i] != NULL && vectorsCoord[i]->isSet())
        {
            vectorsCoord[i]->beginWriteOnly()->reserve(size);
            vectorsCoord[i]->endEdit();
        }
    }

    for (unsigned int i = 0; i < vectorsDeriv.size(); i++)
    {
        if (vectorsDeriv[i] != NULL && vectorsDeriv[i]->isSet())
        {
            vectorsDeriv[i]->beginWriteOnly()->reserve(size);
            vectorsDeriv[i]->endEdit();
        }
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::applyTranslation (const SReal dx, const SReal dy, const SReal dz)
{
    helper::WriteAccessor< Data<VecCoord> > x_wA = *this->write(core::VecCoordId::position());

    for (unsigned int i = 0; i < x_wA.size(); i++)
    {
        DataTypes::add(x_wA[i], dx, dy, dz);
    }
}

//Apply Rotation from Euler angles (in degree!)
template <class DataTypes>
void MechanicalObject<DataTypes>::applyRotation (const SReal rx, const SReal ry, const SReal rz)
{
    sofa::defaulttype::Quaternion q =
            helper::Quater< SReal >::createQuaterFromEuler(sofa::defaulttype::Vec< 3, SReal >(rx, ry, rz) * M_PI / 180.0);
    applyRotation(q);
}

template <class DataTypes>
void MechanicalObject<DataTypes>::applyRotation (const defaulttype::Quat q)
{
    helper::WriteAccessor< Data<VecCoord> > x_wA = *this->write(core::VecCoordId::position());

    for (unsigned int i = 0; i < x_wA.size(); i++)
    {
        sofa::defaulttype::Vec<3,Real> pos;
        DataTypes::get(pos[0], pos[1], pos[2], x_wA[i]);
        sofa::defaulttype::Vec<3,Real> newposition = q.rotate(pos);
        DataTypes::set(x_wA[i], newposition[0], newposition[1], newposition[2]);
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::applyScale(const SReal sx,const SReal sy,const SReal sz)
{
    helper::WriteAccessor< Data<VecCoord> > x_wA = this->writePositions();

    const sofa::defaulttype::Vec<3,Real> s((Real)sx, (Real)sy, (Real)sz);
    for (unsigned int i=0; i<x_wA.size(); i++)
    {
        x_wA[i][0] = x_wA[i][0] * s[0];
        x_wA[i][1] = x_wA[i][1] * s[1];
        x_wA[i][2] = x_wA[i][2] * s[2];
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::getIndicesInSpace(sofa::helper::vector<unsigned>& indices, Real xmin, Real xmax, Real ymin, Real ymax, Real zmin, Real zmax) const
{
    helper::ReadAccessor< Data<VecCoord> > x_rA = this->readPositions();

    for( unsigned i=0; i<x_rA.size(); ++i )
    {
        Real x=0.0,y=0.0,z=0.0;
        DataTypes::get(x,y,z,x_rA[i]);
        if( x >= xmin && x <= xmax && y >= ymin && y <= ymax && z >= zmin && z <= zmax )
        {
            indices.push_back(i);
        }
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::computeWeightedValue( const unsigned int i, const sofa::helper::vector< unsigned int >& ancestors, const sofa::helper::vector< double >& coefs)
{
    // HD interpolate position, speed,force,...
    // assume all coef sum to 1.0
    unsigned int j;

    const unsigned int ancestorsSize = ancestors.size();

    helper::vector< Coord > ancestorsCoord(ancestorsSize);
    helper::vector< Deriv > ancestorsDeriv(ancestorsSize);
    helper::vector< Real > ancestorsCoefs(ancestorsSize);

    for (unsigned int k = 0; k < vectorsCoord.size(); k++)
    {
        if (vectorsCoord[k] != NULL)
        {
            VecCoord &vecCoord = *(vectorsCoord[k]->beginEdit());

            if (vecCoord.size() != 0)
            {
                for (j = 0; j < ancestorsSize; ++j)
                {
                    ancestorsCoord[j] = vecCoord[ancestors[j]];
                    ancestorsCoefs[j] = (Real)coefs[j];
                }

                vecCoord[i] = DataTypes::interpolate(ancestorsCoord, ancestorsCoefs);
            }

            vectorsCoord[k]->endEdit();
        }
    }

    for (unsigned int k = 0; k < vectorsDeriv.size(); k++)
    {
        if (vectorsDeriv[k] != NULL)
        {
            VecDeriv &vecDeriv = *(vectorsDeriv[k]->beginEdit());

            if (vecDeriv.size() != 0)
            {
                for (j = 0; j < ancestorsSize; ++j)
                {
                    ancestorsDeriv[j] = vecDeriv[ancestors[j]];
                    ancestorsCoefs[j] = (Real)coefs[j];
                }

                vecDeriv[i] = DataTypes::interpolate(ancestorsDeriv, ancestorsCoefs);
            }

            vectorsDeriv[k]->endEdit();
        }
    }
}

// Force the position of a point (and force its velocity to zero value)
template <class DataTypes>
void MechanicalObject<DataTypes>::forcePointPosition(const unsigned int i, const sofa::helper::vector< double >& m_x)
{
    helper::WriteAccessor< Data<VecCoord> > x_wA = this->writePositions();
    helper::WriteAccessor< Data<VecDeriv> > v_wA = this->writeVelocities();

    DataTypes::set(x_wA[i], m_x[0], m_x[1], m_x[2]);
    DataTypes::set(v_wA[i], (Real) 0.0, (Real) 0.0, (Real) 0.0);
}

template <class DataTypes>
void MechanicalObject<DataTypes>::copyToBaseVector(defaulttype::BaseVector * dest, core::ConstVecId src, unsigned int &offset)
{
    if (src.type == sofa::core::V_COORD)
    {
        helper::ReadAccessor< Data<VecCoord> > vSrc = *this->read(sofa::core::ConstVecCoordId(src));
        const unsigned int coordDim = sofa::defaulttype::DataTypeInfo<Coord>::size();

        for (unsigned int i = 0; i < vSrc.size(); i++)
        {
            for (unsigned int j = 0; j < coordDim; j++)
            {
                Real tmp = (Real)0.0;
                sofa::defaulttype::DataTypeInfo<Coord>::getValue(vSrc[i], j, tmp);
                dest->set(offset + i * coordDim + j, tmp);
            }
        }

        offset += vSrc.size() * coordDim;
    }
    else
    {
        helper::ReadAccessor< Data<VecDeriv> > vSrc = *this->read(sofa::core::ConstVecDerivId(src));
        const unsigned int derivDim = defaulttype::DataTypeInfo<Deriv>::size();

        for (unsigned int i = 0; i < vSrc.size(); i++)
        {
            for (unsigned int j = 0; j < derivDim; j++)
            {
                Real tmp;
                sofa::defaulttype::DataTypeInfo<Deriv>::getValue(vSrc[i], j, tmp);
                dest->set(offset + i * derivDim + j, tmp);
            }
        }

        offset += vSrc.size() * derivDim;
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::copyFromBaseVector(sofa::core::VecId dest, const defaulttype::BaseVector *src, unsigned int &offset)
{
    if (dest.type == sofa::core::V_COORD)
    {
        helper::WriteOnlyAccessor< Data<VecCoord> > vDest = *this->write(sofa::core::VecCoordId(dest));
        const unsigned int coordDim = defaulttype::DataTypeInfo<Coord>::size();

        for (unsigned int i = 0; i < vDest.size(); i++)
        {
            for (unsigned int j = 0; j < coordDim; j++)
            {
                Real tmp;
                tmp = (Real)src->element(offset + i * coordDim + j);
                sofa::defaulttype::DataTypeInfo<Coord>::setValue(vDest[i], j, tmp);
            }
        }

        offset += vDest.size() * coordDim;
    }
    else
    {
        helper::WriteOnlyAccessor< Data<VecDeriv> > vDest = *this->write(sofa::core::VecDerivId(dest));
        const unsigned int derivDim = sofa::defaulttype::DataTypeInfo<Deriv>::size();

        for (unsigned int i = 0; i < vDest.size(); i++)
        {
            for (unsigned int j = 0; j < derivDim; j++)
            {
                Real tmp;
                tmp = (Real)src->element(offset + i * derivDim + j);
                defaulttype::DataTypeInfo<Deriv>::setValue(vDest[i], j, tmp);
            }
        }

        offset += vDest.size() * derivDim;
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::addToBaseVector(defaulttype::BaseVector* dest, sofa::core::ConstVecId src, unsigned int &offset)
{
    if (src.type == sofa::core::V_COORD)
    {
        helper::ReadAccessor< Data<VecCoord> > vSrc = *this->read(core::ConstVecCoordId(src));
        const unsigned int coordDim = defaulttype::DataTypeInfo<Coord>::size();

        for (unsigned int i = 0; i < vSrc.size(); i++)
        {
            for (unsigned int j = 0; j < coordDim; j++)
            {
                Real tmp = (Real)0.0;
                defaulttype::DataTypeInfo<Coord>::getValue(vSrc[i], j, tmp);
                dest->add(offset + i * coordDim + j, tmp);
            }
        }

        offset += vSrc.size() * coordDim;
    }
    else
    {
        helper::ReadAccessor< Data<VecDeriv> > vSrc = *this->read(core::ConstVecDerivId(src));
        const unsigned int derivDim = defaulttype::DataTypeInfo<Deriv>::size();

        for (unsigned int i = 0; i < vSrc.size(); i++)
        {
            for (unsigned int j = 0; j < derivDim; j++)
            {
                Real tmp;
                defaulttype::DataTypeInfo<Deriv>::getValue(vSrc[i], j, tmp);
                dest->add(offset + i * derivDim + j, tmp);
            }
        }

        offset += vSrc.size() * derivDim;
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::addFromBaseVectorSameSize(sofa::core::VecId dest, const defaulttype::BaseVector *src, unsigned int &offset)
{
    if (dest.type == sofa::core::V_COORD)
    {
        helper::WriteAccessor< Data<VecCoord> > vDest = *this->write(core::VecCoordId(dest));
        const unsigned int coordDim = defaulttype::DataTypeInfo<Coord>::size();

        for (unsigned int i = 0; i < vDest.size(); i++)
        {
            for (unsigned int j = 0; j < coordDim; j++)
            {
                Real tmp = (Real)0.0;
                defaulttype::DataTypeInfo<Coord>::getValue(vDest[i], j, tmp);
                defaulttype::DataTypeInfo<Coord>::setValue(vDest[i], j, tmp + src->element(offset + i * coordDim + j));
            }
        }

        offset += vDest.size() * coordDim;
    }
    else
    {
        helper::WriteAccessor< Data<VecDeriv> > vDest = *this->write(core::VecDerivId(dest));
        const unsigned int derivDim = defaulttype::DataTypeInfo<Deriv>::size();

        for (unsigned int i = 0; i < vDest.size(); i++)
        {
            for (unsigned int j = 0; j < derivDim; j++)
            {
                Real tmp = (Real)0.0;
                defaulttype::DataTypeInfo<Deriv>::getValue(vDest[i], j, tmp);
                defaulttype::DataTypeInfo<Deriv>::setValue(vDest[i], j, tmp + src->element(offset + i * derivDim + j));
            }
        }

        offset += vDest.size() * derivDim;
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::addFromBaseVectorDifferentSize(sofa::core::VecId dest, const defaulttype::BaseVector* src, unsigned int &offset )
{
    if (dest.type == sofa::core::V_COORD)
    {
        helper::WriteAccessor< Data<VecCoord> > vDest = *this->write(core::VecCoordId(dest));
        const unsigned int coordDim = defaulttype::DataTypeInfo<Coord>::size();
        const unsigned int nbEntries = src->size()/coordDim;
        for (unsigned int i=0; i<nbEntries; i++)
        {
            for (unsigned int j=0; j<coordDim; ++j)
            {
                Real tmp = (Real)0.0;
                defaulttype::DataTypeInfo<Coord>::getValue(vDest[i+offset],j,tmp);
                defaulttype::DataTypeInfo<Coord>::setValue(vDest[i+offset],j, tmp + src->element(i*coordDim+j));
            }
        }
        offset += nbEntries;
    }
    else
    {
        helper::WriteAccessor< Data<VecDeriv> > vDest = *this->write(core::VecDerivId(dest));

        const unsigned int derivDim = defaulttype::DataTypeInfo<Deriv>::size();
        const unsigned int nbEntries = src->size()/derivDim;
        for (unsigned int i=0; i<nbEntries; i++)
        {
            for (unsigned int j=0; j<derivDim; ++j)
            {
                Real tmp = (Real)0.0;
                defaulttype::DataTypeInfo<Deriv>::getValue(vDest[i+offset],j,tmp);
                defaulttype::DataTypeInfo<Deriv>::setValue(vDest[i+offset],j, tmp + src->element(i*derivDim+j));
            }
        }
        offset += nbEntries;
    }


}


template <class DataTypes>
void MechanicalObject<DataTypes>::init()
{

    //Look at a topology associated to this instance of MechanicalObject by a tag
    this->getContext()->get(m_topology, this->getTags());
    // If no topology found, no association, then look to the nearest one
    if(m_topology == NULL)
        m_topology = this->getContext()->getMeshTopology();


    //helper::WriteAccessor< Data<VecCoord> > x_wA = *this->write(VecCoordId::position());
    //helper::WriteAccessor< Data<VecDeriv> > v_wA = *this->write(VecDerivId::velocity());
    Data<VecCoord>* x_wAData = this->write(sofa::core::VecCoordId::position());
    Data<VecDeriv>* v_wAData = this->write(sofa::core::VecDerivId::velocity());
    VecCoord& x_wA = *x_wAData->beginEdit();
    VecDeriv& v_wA = *v_wAData->beginEdit();

    //case if X0 has been set but not X
    if (read(core::ConstVecCoordId::restPosition())->getValue().size() > x_wA.size())
    {
        vOp(core::ExecParams::defaultInstance(), core::VecId::position(), core::VecId::restPosition());
    }

    // the given position and velocity vectors are empty
    // note that when a vector is not  explicitly specified, its size won't change (1 by default)
    if( x_wA.size() <= 1 && v_wA.size() <= 1 )
    {
        // if a topology is present, implicitly copy position from it
        if (m_topology != NULL && m_topology->getNbPoints() && m_topology->getContext() == this->getContext())
        {
            int nbp = m_topology->getNbPoints();
            //std::cout<<"Setting "<<nbp<<" points from topology. " << this->getName() << " topo : " << m_topology->getName() <<std::endl;
            // copy the last specified velocity to all points
            if (v_wA.size() >= 1 && v_wA.size() < (unsigned)nbp)
            {
                unsigned int i = v_wA.size();
                Deriv v1 = v_wA[i-1];
                v_wA.resize(nbp);
                while (i < v_wA.size())
                    v_wA[i++] = v1;
            }
            this->resize(nbp);
            for (int i=0; i<nbp; i++)
            {
                x_wA[i] = Coord();
                DataTypes::set(x_wA[i], m_topology->getPX(i), m_topology->getPY(i), m_topology->getPZ(i));
            }
        }
        else if( x_wA.size() == 0 )
        {
            // special case when the user manually explicitly defined an empty position vector
            // (e.g. linked to an empty vector)
            resize(0);
        }
    }
    else if (x_wA.size() != vsize || v_wA.size() != vsize)
    {
        // X and/or V were user-specified
        // copy the last specified velocity to all points

        const unsigned int xSize = x_wA.size();
        const unsigned int vSize = v_wA.size();

        if (vSize >= 1 && vSize < xSize)
        {
            unsigned int i = vSize;
            Deriv v1 = v_wA[i-1];
            v_wA.resize(xSize);
            while (i < xSize)
                v_wA[i++] = v1;
        }

        resize(xSize > v_wA.size() ? xSize : v_wA.size());
    }

    x_wAData->endEdit();
    v_wAData->endEdit();

    reinit();

    // storing X0 must be done after reinit() that possibly applies transformations
    if( read(core::ConstVecCoordId::restPosition())->getValue().size()!=x_wA.size() )
    {
        // storing X0 from X
        if( restScale.getValue()!=1 )
            vOp(core::ExecParams::defaultInstance(), core::VecId::restPosition(), core::ConstVecId::null(), core::VecId::position(), restScale.getValue());
        else
            vOp(core::ExecParams::defaultInstance(), core::VecId::restPosition(), core::VecId::position());
    }

    m_initialized = true;

    if (f_reserve.getValue() > 0)
        reserve(f_reserve.getValue());

    if(isToPrint.getValue()==true) {
        x.setPersistent(false);
        v.setPersistent(false);
        f.setPersistent(false);
        externalForces.setPersistent(false);
        dx.setPersistent(false);
        x0.setPersistent(false);
        reset_position.setPersistent(false);}
}

template <class DataTypes>
void MechanicalObject<DataTypes>::reinit() { }

template <class DataTypes>
void MechanicalObject<DataTypes>::storeResetState()
{
    // store a reset state only for independent dofs (mapped dofs are deduced from independent dofs)
    if( !isIndependent() ) return;

    // Save initial state for reset button
    vOp(core::ExecParams::defaultInstance(), core::VecId::resetPosition(), core::VecId::position());

    //vOp(VecId::resetVelocity(), VecId::velocity());
    // we only store a resetVelocity if the velocity is not zero
    helper::ReadAccessor< Data<VecDeriv> > v = *this->read(core::VecDerivId::velocity());
    bool zero = true;
    for (unsigned int i=0; i<v.size(); ++i)
    {
        const Deriv& vi = v[i];
        for (unsigned int j=0; j<vi.size(); ++j)
            if (vi[j] != 0) zero = false;
        if (!zero) break;
    }
    if (!zero)
        vOp(core::ExecParams::defaultInstance(), core::VecId::resetVelocity(), core::VecId::velocity());
}


template <class DataTypes>
void MechanicalObject<DataTypes>::reset()
{
    // resetting force for every dofs, even mapped ones
    vOp(core::ExecParams::defaultInstance(), core::VecId::force());

    if (!reset_position.isSet()) // mapped states are deduced from independent ones
        return;

    vOp(core::ExecParams::defaultInstance(), core::VecId::position(), core::VecId::resetPosition());

    if (!reset_velocity.isSet()) {
        vOp(core::ExecParams::defaultInstance(), core::VecId::velocity());
    } else  {
        vOp(core::ExecParams::defaultInstance(), core::VecId::velocity(), core::VecId::resetVelocity());
    }

}


template <class DataTypes>
void MechanicalObject<DataTypes>::writeVec(core::ConstVecId v, std::ostream &out)
{
    switch (v.type)
    {
    case sofa::core::V_COORD:
        out << this->read(core::ConstVecCoordId(v))->getValue();
        break;
    case sofa::core::V_DERIV:
        out << this->read(core::ConstVecDerivId(v))->getValue();
        break;
    default:
        throw std::logic_error("unimplemented");
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::readVec(core::VecId v, std::istream &in)
{
    size_t i = 0;

    switch (v.type)
    {
    case sofa::core::V_COORD:
    {
        Coord coord;
        helper::WriteOnlyAccessor< Data< VecCoord > > vec = *this->write(core::VecCoordId(v));

        while (in >> coord)
        {
            if (i >= getSize())
                resize(i+1);
            vec[i++] = coord;
        }

        break;
    }
    case sofa::core::V_DERIV:
    {
        Deriv deriv;
        helper::WriteOnlyAccessor< Data< VecDeriv > > vec = *this->write(core::VecDerivId(v));

        while (in >> deriv)
        {
            if (i >= getSize())
                resize(i+1);
            vec[i++] = deriv;
        }

        break;
    }
    case sofa::core::V_MATDERIV:
        //TODO
        break;
    default:
        break;
    }

    if (i < getSize())
        resize(i);
}

template <class DataTypes>
SReal MechanicalObject<DataTypes>::compareVec(core::ConstVecId v, std::istream &in)
{
    std::string ref,cur;
    getline(in, ref);

    std::ostringstream out;

    switch (v.type)
    {
    case sofa::core::V_COORD:
        out << this->read(core::ConstVecCoordId(v))->getValue();
        break;
    case sofa::core::V_DERIV:
        out << this->read(core::ConstVecDerivId(v))->getValue();
        break;
    default:
        throw std::logic_error("unimplemented");
    }

    cur = out.str();

    SReal error=0;
    std::istringstream compare_ref(ref);
    std::istringstream compare_cur(cur);

    Real value_ref, value_cur;
    unsigned int count=0;
    while (compare_ref >> value_ref && compare_cur >> value_cur )
    {
        error += fabs(value_ref-value_cur);
        count ++;
    }
    if( count == 0 ) return 0; //both vector are empy, so we return 0;

    return error/count;
}

template <class DataTypes>
void MechanicalObject<DataTypes>::writeState(std::ostream& out)
{
    writeVec(core::VecId::position(),out);
    out << " ";
    writeVec(core::VecId::velocity(),out);
}

template <class DataTypes>
void MechanicalObject<DataTypes>::beginIntegration(SReal /*dt*/)
{
    this->forceMask.activate(false);
}

template <class DataTypes>
void MechanicalObject<DataTypes>::endIntegration(const core::ExecParams*
                                                 #ifdef SOFA_SMP
                                                 params
                                                 #endif
                                                , SReal /*dt*/    )
{
    this->forceMask.assign( this->getSize(), false );

    {
        this->externalForces.beginWriteOnly()->clear();
        this->externalForces.endEdit();
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::accumulateForce(const core::ExecParams* params, core::VecDerivId fId)
{
    {
        helper::ReadAccessor< Data<VecDeriv> > extForces_rA( params, *this->read(core::ConstVecDerivId::externalForce()) );

        if (!extForces_rA.empty())
        {
            helper::WriteAccessor< Data<VecDeriv> > f_wA ( params, *this->write(fId) );

            for (unsigned int i=0; i < extForces_rA.size(); i++)
            {
                if( extForces_rA[i] != Deriv() )
                {
                    f_wA[i] += extForces_rA[i];
                    this->forceMask.insertEntry(i); // if an external force is applied on the dofs, it must be added to the mask
                }
            }
        }
    }
}

template <class DataTypes>
Data<typename MechanicalObject<DataTypes>::VecCoord>* MechanicalObject<DataTypes>::write(core::VecCoordId v)
{

    if (v.index >= vectorsCoord.size())
    {
        vectorsCoord.resize(v.index + 1, 0);
    }

    if (vectorsCoord[v.index] == NULL)
    {
        vectorsCoord[v.index] = new Data< VecCoord >;
        if (f_reserve.getValue() > 0)
        {
            vectorsCoord[v.index]->beginWriteOnly()->reserve(f_reserve.getValue());
            vectorsCoord[v.index]->endEdit();
        }
        if (vectorsCoord[v.index]->getValue().size() != (size_t)getSize())
        {
            vectorsCoord[v.index]->beginWriteOnly()->resize( getSize() );
            vectorsCoord[v.index]->endEdit();
        }
    }
    Data<typename MechanicalObject<DataTypes>::VecCoord>* d = vectorsCoord[v.index];
#if defined(SOFA_DEBUG) || !defined(NDEBUG)
    const typename MechanicalObject<DataTypes>::VecCoord& val = d->getValue();
    if (!val.empty() && val.size() != (unsigned int)this->getSize())
    {
        serr << "Writing to State vector " << v << " with incorrect size : " << val.size() << " != " << this->getSize() << sendl;
    }
#endif
    return d;
}



template <class DataTypes>
const Data<typename MechanicalObject<DataTypes>::VecCoord>* MechanicalObject<DataTypes>::read(core::ConstVecCoordId v) const
{
    if (v.isNull())
    {
        serr << "Accessing null VecCoord" << sendl;
    }
    if (v.index < vectorsCoord.size() && vectorsCoord[v.index] != NULL)
    {
        const Data<typename MechanicalObject<DataTypes>::VecCoord>* d = vectorsCoord[v.index];
#if defined(SOFA_DEBUG) || !defined(NDEBUG)
        const typename MechanicalObject<DataTypes>::VecCoord& val = d->getValue();
        if (!val.empty() && val.size() != (unsigned int)this->getSize())
        {
            serr << "Accessing State vector " << v << " with incorrect size : " << val.size() << " != " << this->getSize() << sendl;
        }
#endif
        return d;
    }
    else
    {
        serr << "Vector " << v << " does not exist" << sendl;
        return NULL;
    }
}

template <class DataTypes>
Data<typename MechanicalObject<DataTypes>::VecDeriv>* MechanicalObject<DataTypes>::write(core::VecDerivId v)
{

    if (v.index >= vectorsDeriv.size())
    {
        vectorsDeriv.resize(v.index + 1, 0);
    }

    if (vectorsDeriv[v.index] == NULL)
    {
        vectorsDeriv[v.index] = new Data< VecDeriv >;
        if (f_reserve.getValue() > 0)
        {
            vectorsDeriv[v.index]->beginWriteOnly()->reserve(f_reserve.getValue());
            vectorsDeriv[v.index]->endEdit();
        }
        if (vectorsDeriv[v.index]->getValue().size() != (size_t)getSize())
        {
            vectorsDeriv[v.index]->beginWriteOnly()->resize( getSize() );
            vectorsDeriv[v.index]->endEdit();
        }
    }
    Data<typename MechanicalObject<DataTypes>::VecDeriv>* d = vectorsDeriv[v.index];
#if defined(SOFA_DEBUG) || !defined(NDEBUG)
    const typename MechanicalObject<DataTypes>::VecDeriv& val = d->getValue();
    if (!val.empty() && val.size() != (unsigned int)this->getSize())
    {
        serr << "Writing to State vector " << v << " with incorrect size : " << val.size() << " != " << this->getSize() << sendl;
    }
#endif
    return d;
}

template <class DataTypes>
const Data<typename MechanicalObject<DataTypes>::VecDeriv>* MechanicalObject<DataTypes>::read(core::ConstVecDerivId v) const
{
    if (v.index < vectorsDeriv.size())
    {
        const Data<typename MechanicalObject<DataTypes>::VecDeriv>* d = vectorsDeriv[v.index];
#if defined(SOFA_DEBUG) || !defined(NDEBUG)
        const typename MechanicalObject<DataTypes>::VecDeriv& val = d->getValue();
        if (!val.empty() && val.size() != (unsigned int)this->getSize())
        {
            serr << "Accessing State vector " << v << " with incorrect size : " << val.size() << " != " << this->getSize() << sendl;
        }
#endif
        return d;
    }
    else
    {
        serr << "Vector " << v << "does not exist" << sendl;
        return NULL;
    }
}

template <class DataTypes>
Data<typename MechanicalObject<DataTypes>::MatrixDeriv>* MechanicalObject<DataTypes>::write(core::MatrixDerivId )
{
    throw std::logic_error("unimplemented");
}

template <class DataTypes>
const Data<typename MechanicalObject<DataTypes>::MatrixDeriv>* MechanicalObject<DataTypes>::read(core::ConstMatrixDerivId ) const
{
    throw std::logic_error("unimplemented");    
}

template <class DataTypes>
void MechanicalObject<DataTypes>::setVecCoord(unsigned int index, Data< VecCoord > *v)
{
    if (index >= vectorsCoord.size())
    {
        vectorsCoord.resize(index + 1, 0);
    }

    vectorsCoord[index] = v;
}

template <class DataTypes>
void MechanicalObject<DataTypes>::setVecDeriv(unsigned int index, Data< VecDeriv > *v)
{
    if (index >= vectorsDeriv.size())
    {
        vectorsDeriv.resize(index + 1, 0);
    }

    vectorsDeriv[index] = v;
}


// template <class DataTypes>
// void MechanicalObject<DataTypes>::setVecMatrixDeriv(unsigned int index, Data < MatrixDeriv > *m)
// {
//     throw std::logic_error("unimplemented");        
// }

template <class DataTypes>
void MechanicalObject<DataTypes>::vAvail(const core::ExecParams* /* params */, core::VecCoordId& v)
{
    for (unsigned int i = v.index; i < vectorsCoord.size(); ++i)
    {
        if (vectorsCoord[i] && vectorsCoord[i]->isSet())
            v.index = i+1;
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::vAvail(const core::ExecParams* /* params */, core::VecDerivId& v)
{
    for (unsigned int i = v.index; i < vectorsDeriv.size(); ++i)
    {
        if (vectorsDeriv[i] && vectorsDeriv[i]->isSet())
            v.index = i+1;
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::vAlloc(const core::ExecParams* params, core::VecCoordId v)
{
    if (v.index >= sofa::core::VecCoordId::V_FIRST_DYNAMIC_INDEX)
    {
        Data<VecCoord>* vec_d = this->write(v);
        vec_d->beginWriteOnly(params)->resize(vsize);
        vec_d->endEdit(params);
    }

    //vOp(v); // clear vector
}

template <class DataTypes>
void MechanicalObject<DataTypes>::vAlloc(const core::ExecParams* params, core::VecDerivId v)
{

    if (v.index >= sofa::core::VecDerivId::V_FIRST_DYNAMIC_INDEX)
    {
        Data<VecDeriv>* vec_d = this->write(v);
        vec_d->beginWriteOnly(params)->resize(vsize);
        vec_d->endEdit(params);
    }

    //vOp(v); // clear vector
}

template <class DataTypes>
void MechanicalObject<DataTypes>::vRealloc(const core::ExecParams* params, core::VecCoordId v)
{
    Data<VecCoord>* vec_d = this->write(v);

    if ( !vec_d->isSet(params) /*&& v.index >= sofa::core::VecCoordId::V_FIRST_DYNAMIC_INDEX*/ )
    {
        vec_d->beginWriteOnly(params)->resize(vsize);
        vec_d->endEdit(params);
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::vRealloc(const core::ExecParams* params, core::VecDerivId v)
{
    Data<VecDeriv>* vec_d = this->write(v);

    if ( !vec_d->isSet(params) /*&& v.index >= sofa::core::VecDerivId::V_FIRST_DYNAMIC_INDEX*/ )
    {
        vec_d->beginWriteOnly(params)->resize(vsize);
        vec_d->endEdit(params);
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::vFree(const core::ExecParams* params, core::VecCoordId vId)
{
    if (vId.index >= sofa::core::VecCoordId::V_FIRST_DYNAMIC_INDEX)
    {
        Data< VecCoord >* vec_d = this->write(vId);

        VecCoord *vec = vec_d->beginWriteOnly(params);
        vec->resize(0);
        vec_d->endEdit(params);

        vec_d->unset(params);
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::vFree(const core::ExecParams* params, core::VecDerivId vId)
{
    if (vId.index >= sofa::core::VecDerivId::V_FIRST_DYNAMIC_INDEX)
    {
        Data< VecDeriv >* vec_d = this->write(vId);

        VecDeriv *vec = vec_d->beginWriteOnly(params);
        vec->resize(0);
        vec_d->endEdit(params);

        vec_d->unset(params);
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::vInit(const core::ExecParams* params
                                        , core::VecCoordId vId
                                        , core::ConstVecCoordId vSrcId)
{
    Data< VecCoord >* vec_d = this->write(vId);

    if (!vec_d->isSet(params) || vec_d->getValue().empty())
    {
        vec_d->forceSet(params);
        vOp(params, vId, vSrcId);
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::vInit(const core::ExecParams* params,
                                        core::VecDerivId vId,
                                        core::ConstVecDerivId vSrcId)
{
    Data< VecDeriv >* vec_d = this->write(vId);

    if (!vec_d->isSet(params) || vec_d->getValue().empty()) {
        vec_d->forceSet(params);
        vOp(params, vId, vSrcId);
    }
}


// expression templates for v = a + b * f
namespace expr {

// zero
template<class T>
struct null {
    const std::size_t s;
    std::size_t size() const { return s; }

    using value_type = T;
    value_type operator()(std::size_t) const { return {}; }
};

// sum
template<class LHS, class RHS>
struct sum_type {
    const LHS lhs;
    const RHS rhs;

    sum_type(LHS lhs, RHS rhs) : lhs(lhs), rhs(rhs) {
        if(lhs.size() != rhs.size()) throw std::runtime_error("size error");
    }
    
    std::size_t size() const { return lhs.size(); }

    using lhs_value_type = typename LHS::value_type;
    using rhs_value_type = typename RHS::value_type;    
    
    using value_type = decltype( lhs_value_type() + rhs_value_type() );
    value_type operator()(std::size_t i) const { return lhs(i) + rhs(i); }
};


template<class LHS, class RHS>
static sum_type<LHS, RHS> operator+(LHS lhs, RHS rhs) { return {lhs, rhs}; }

// scalar product
template<class Expr>
struct scalar_type {
    const SReal scalar;
    const Expr expr;

    std::size_t size() const { return expr.size(); }
    
    using value_type = typename Expr::value_type;
    value_type operator()(std::size_t i) const { return expr(i) * scalar; }
};

template<class Expr>
static scalar_type<Expr> operator*(SReal scalar, Expr expr) { return {scalar, expr}; }


// ast optimizations (TODO check size consistency?)
template<class T, class Expr>
// static typename std::enable_if< std::is_same<T, typename Expr::value_type>::value, Expr>::type
static Expr operator+(null<T>, Expr expr) { return expr; }

template<class Expr, class T>
// static typename std::enable_if< std::is_same<T, typename Expr::value_type>::value, Expr>::type
static Expr operator+(Expr expr, null<T>) { return expr; }

// note: null<T> + null<U> *must* remain ambiguous.
template<class T> static null<T> operator+(null<T> lhs, null<T>) { return lhs; }

template<class T> static null<T> operator*(SReal, null<T> x) { return x; }
template<class T> static null<T> operator*(null<T> x, SReal) { return x; }

// container reference
template<class Container>
struct ref_type {
    Container& container;

    std::size_t size() const { return container.size(); }

    using value_type = typename Container::value_type;
    const value_type& operator()(std::size_t i) const { return container[i]; }

    template<class Expr>
    ref_type& operator=(const Expr& expr) {
        container.resize(expr.size());
        
        for(std::size_t i = 0, n = size(); i < n; ++i) {
            container[i] = expr(i);
        }

        return *this;
    }
    
};

template<class Container>
static ref_type<Container> ref(Container& container) { return {container}; }

}

// visit a (Const)VecId with proper type: (Const)CoordVecId or
// (Const)DerivVecId, or the (unsigned) vector index for V_ALL

// the visitor gets called with: visitor(vecid, args...);
template<core::VecAccess MODE, class Visitor, class ... Args>
static void visit(core::TVecId<core::V_ALL, MODE> id, const Visitor& visitor, Args&& ... args) {
    switch(id.type) {
    case core::V_COORD: 
        visitor(core::TVecId<core::V_COORD, MODE>(id), std::forward<Args>(args)...);
        break;
    case core::V_DERIV:
        visitor(core::TVecId<core::V_DERIV, MODE>(id), std::forward<Args>(args)...);
        break;
    case core::V_ALL:
        // amidoinitrite?
        assert(id.isNull() && "actual V_ALL vectors must be null!");
        visitor(id.getIndex(), std::forward<Args>(args)...);
        break;
    default:
        throw std::runtime_error("unimplemented vop for vecid:" + std::to_string(id.type));
    }
}




struct dispatch_visitor {

    // first
    template<class V, class Dispatch>
    void operator()(V v, core::ConstVecId a, core::ConstVecId b, const Dispatch& dispatch) const {
        visit(a, *this, v, b, dispatch);
    }

    // second
    template<class A, class V, class Dispatch>
    void operator()(A a, V v, core::ConstVecId b, const Dispatch& dispatch) const {
        visit(b, *this, v, a, dispatch);
    }

    // third
    template<class B, class A, class V, class Dispatch>
    void operator()(B b, V v, A a, const Dispatch& dispatch) const {
        const prune_type<Dispatch> prune = { dispatch };
        prune(v, a, b);
    }

    template<class Dispatch>
    struct prune_type {
        const Dispatch& dispatch;

        // prune illegal patterns + normalize legal ones
        template<class V, class A, class B>
        void operator()(V, A, B) const {
            throw std::runtime_error("illegal vop");
        } 

        // allowed patterns
        void operator()(core::VecCoordId v, core::ConstVecCoordId a, core::ConstVecCoordId b) const {
            // coord = coord + coord        
            dispatch(v, a, b);
        }


        void operator()(core::VecCoordId v, core::ConstVecCoordId a, core::ConstVecDerivId b) const {
            // coord = coord + deriv
            dispatch(v, a, b);
        }


        void operator()(core::VecDerivId v, core::ConstVecDerivId a, core::ConstVecDerivId b) const {
            // deriv = deriv + deriv        
            dispatch(v, a, b);
        }

        // fixing v_all crap
        void operator()(core::VecCoordId v, unsigned a, core::ConstVecCoordId b) const {
            // coord = 0 + coord        
            dispatch(v, core::ConstVecCoordId(a), b);
        }

        void operator()(core::VecCoordId v, core::ConstVecCoordId a, unsigned b) const {
            // coord = coord + 0
            dispatch(v, a, core::ConstVecCoordId(b));
        }

        void operator()(core::VecCoordId v, unsigned a, unsigned b) const {
            // coord = 0 + 0
            dispatch(v, core::ConstVecCoordId(a), core::ConstVecCoordId(b));
        }

        
        void operator()(core::VecDerivId v, unsigned a, core::ConstVecDerivId b) const {
            // deriv = 0 + deriv
            dispatch(v, core::ConstVecDerivId(a), b);
        }
        
        void operator()(core::VecDerivId v, core::ConstVecDerivId a, unsigned b) const {
            // deriv = deriv + 0
            dispatch(v, a, core::ConstVecDerivId(b));
        }

        void operator()(core::VecDerivId v, unsigned a, unsigned b) const {
            // deriv = 0 + 0
            dispatch(v, core::ConstVecDerivId(a), core::ConstVecDerivId(b));
        }
        

        
        
    };
    
    
};



template<class T>
const typename T::VecCoord& read_vector(const core::State<T>* state, core::ConstVecCoordId id)  {
    assert(!id.isNull());
    return state->read(id)->getValue();
}

template<class T>
const typename T::VecDeriv& read_vector(const core::State<T>* state, core::ConstVecDerivId id) {
    assert(!id.isNull());    
    return state->read(id)->getValue();
}


template<class T>
typename T::VecCoord& write_vector(core::State<T>* state, core::VecCoordId id) {
    assert(!id.isNull());    
    return const_cast<typename T::VecCoord&>(state->read(id)->getValue());
}

template<class T>
typename T::VecDeriv& write_vector(core::State<T>* state, core::VecDerivId id)  {
    assert(!id.isNull());        
    return const_cast<typename T::VecDeriv&>(state->read(id)->getValue());
}

template<class T>
struct dispatch_type {
    core::State<T>* const state;
    const SReal f;
    
    // properly typed null exprs
    expr::null< typename T::Coord > null(core::ConstVecCoordId ) const { return {state->getSize()}; }
    expr::null< typename T::Deriv > null(core::ConstVecDerivId ) const { return {state->getSize()}; }

    // first: build proper expression types
    template<class V, class A, class B>
    void operator()(V v, A a, B b) const {
        auto vref = expr::ref(write_vector(state, v));
        
        const bool lhs_null = a.isNull();
        const bool rhs_null = b.isNull() || f == 0;
        
        // dispatch on a == 0, b == 0
        if(lhs_null && rhs_null ) {
            assign(vref, null(v));
        } else if(rhs_null) {
            assign(vref, expr::ref(read_vector(state, a)) );
        } else if(lhs_null) {
            assign(vref, f * expr::ref(read_vector(state, b)));
        } else {
            assign(vref, expr::ref(read_vector(state, a)) + f * expr::ref(read_vector(state, b)));
        }
        
        // signal modifications
        auto data = state->write(v);
        data->beginWriteOnly();
        data->endEdit();
    }

    // we add an extra safety layer since (coord + f * deriv) may be a coord or
    // deriv depending on coord being null (wtf) "almost typesafe" lol.
    template<class V, class E>
    static typename std::enable_if< std::is_assignable<typename V::value_type,
                                                       typename E::value_type>::value  >::type
    assign(V v, E e) { v = e; }

    template<class V, class E>
    static typename std::enable_if< !std::is_assignable<typename V::value_type,
                                                        typename E::value_type>::value  >::type
    assign(V, E) {
        std::cerr << typeid( typename V::value_type ).name() << " "
                  << typeid( typename E::value_type ).name() << std::endl;
        std::stringstream ss;

        ss << "type error when assigning \"" << defaulttype::DataTypeInfo<typename V::value_type>::name()
           << "\" and \"" << defaulttype::DataTypeInfo<typename V::value_type>::name() << "\"";
        
        throw std::runtime_error(ss.str());
    }
    
        
};


template <class DataTypes>
void MechanicalObject<DataTypes>::vOp(const core::ExecParams*, core::VecId v,
                                      core::ConstVecId a,
                                      core::ConstVecId b, SReal f) {
    try {
        if(v.isNull()) throw std::runtime_error("cannot assign to null");

        const dispatch_type<DataTypes> dispatch = {this, f};
        visit(v, dispatch_visitor(), a, b, dispatch);
        
    } catch (std::runtime_error& e) {
        msg_error() << e.what() << " in vop \"" << v << " = " << a << " + " << b << " * " << f << "\"";
    }

}



template <class DataTypes>
void MechanicalObject<DataTypes>::vMultiOp(const core::ExecParams* params, const VMultiOp& ops) {
    // optimize common integration case: v += a*dt, x += v*dt
    if (ops.size() == 2
            && ops[0].second.size() == 2
            && ops[0].first.getId(this) == ops[0].second[0].first.getId(this)
            && ops[0].first.getId(this).type == sofa::core::V_DERIV
            && ops[0].second[1].first.getId(this).type == sofa::core::V_DERIV
            && ops[1].second.size() == 2
            && ops[1].first.getId(this) == ops[1].second[0].first.getId(this)
            && ops[0].first.getId(this) == ops[1].second[1].first.getId(this)
            && ops[1].first.getId(this).type == sofa::core::V_COORD) {
        helper::ReadAccessor< Data<VecDeriv> > va( params, *this->read(core::ConstVecDerivId(ops[0].second[1].first.getId(this))) );
        helper::WriteAccessor< Data<VecDeriv> > vv( params, *this->write(core::VecDerivId(ops[0].first.getId(this))) );
        helper::WriteAccessor< Data<VecCoord> > vx( params, *this->write(core::VecCoordId(ops[1].first.getId(this))) );

        const unsigned int n = vx.size();
        const Real f_v_v = (Real)(ops[0].second[0].second);
        const Real f_v_a = (Real)(ops[0].second[1].second);
        const Real f_x_x = (Real)(ops[1].second[0].second);
        const Real f_x_v = (Real)(ops[1].second[1].second);

        if (f_v_v == 1.0 && f_x_x == 1.0) // very common case
        {
            if (f_v_a == 1.0) // used by euler implicit and other integrators that directly computes a*dt
            {
                for (unsigned int i=0; i<n; ++i)
                {
                    vv[i] += va[i];
                    vx[i] += vv[i]*f_x_v;
                }
            }
            else
            {
                for (unsigned int i=0; i<n; ++i)
                {
                    vv[i] += va[i]*f_v_a;
                    vx[i] += vv[i]*f_x_v;
                }
            }
        }
        else if (f_x_x == 1.0) // some damping is applied to v
        {
            for (unsigned int i=0; i<n; ++i)
            {
                vv[i] *= f_v_v;
                vv[i] += va[i];
                vx[i] += vv[i]*f_x_v;
            }
        }
        else // general case
        {
            for (unsigned int i=0; i<n; ++i)
            {
                vv[i] *= f_v_v;
                vv[i] += va[i]*f_v_a;
                vx[i] *= f_x_x;
                vx[i] += vv[i]*f_x_v;
            }
        }
    }
    else if(ops.size()==2 //used in the ExplicitBDF solver only (Electrophysiology)
            && ops[0].second.size()==1
            && ops[0].second[0].second == 1.0
            && ops[1].second.size()==3
            )
    {
        helper::ReadAccessor< Data<VecCoord> > v11( params, *this->read(core::ConstVecCoordId(ops[0].second[0].first.getId(this))) );
        helper::ReadAccessor< Data<VecCoord> > v21( params, *this->read(core::ConstVecCoordId(ops[1].second[0].first.getId(this))) );
        helper::ReadAccessor< Data<VecCoord> > v22( params, *this->read(core::ConstVecCoordId(ops[1].second[1].first.getId(this))) );
        helper::ReadAccessor< Data<VecDeriv> > v23( params, *this->read(core::ConstVecDerivId(ops[1].second[2].first.getId(this))) );

        helper::WriteAccessor< Data<VecCoord> > previousPos( params, *this->write(core::VecCoordId(ops[0].first.getId(this))) );
        helper::WriteAccessor< Data<VecCoord> > newPos( params, *this->write(core::VecCoordId(ops[1].first.getId(this))) );

        const unsigned int n = v11.size();
        const Real f_1 = (Real)(ops[1].second[0].second);
        const Real f_2 = (Real)(ops[1].second[1].second);
        const Real f_3 = (Real)(ops[1].second[2].second);

        for (unsigned int i=0; i<n; ++i)
        {
            previousPos[i] = v11[i];
            newPos[i]  = v21[i]*f_1;
            newPos[i] += v22[i]*f_2;
            newPos[i] += v23[i]*f_3;
        }
    }
    else // no optimization for now for other cases
        Inherited::vMultiOp(params, ops);
}

template <class T> static inline void clear( T& t ) {
    t.clear();
}

static inline void clear( float& t ) {
    t=0;
}

static inline void clear( double& t ) {
    t=0;
}

template <class DataTypes>
void MechanicalObject<DataTypes>::vThreshold(core::VecId v, SReal t)
{
    if( v.type==sofa::core::V_DERIV)
    {
        helper::WriteAccessor< Data<VecDeriv> > vv = *this->write(core::VecDerivId(v));
        Real t2 = (Real)(t*t);
        for (unsigned int i=0; i<vv.size(); i++)
        {
            if( vv[i]*vv[i] < t2 )
                clear(vv[i]);
        }
    }
    else
    {
        serr<<"vThreshold does not apply to coordinate vectors"<<sendl;
    }
}

template <class DataTypes>
SReal MechanicalObject<DataTypes>::vDot(const core::ExecParams* params, core::ConstVecId a, core::ConstVecId b)
{
    Real r = 0.0;

    if (a.type == sofa::core::V_COORD && b.type == sofa::core::V_COORD)
    {
        const VecCoord &va = this->read(core::ConstVecCoordId(a))->getValue(params);
        const VecCoord &vb = this->read(core::ConstVecCoordId(b))->getValue(params);

        for (unsigned int i=0; i<va.size(); i++)
        {
            r += va[i] * vb[i];
        }
    }
    else if (a.type == sofa::core::V_DERIV && b.type == sofa::core::V_DERIV)
    {
        const VecDeriv &va = this->read(core::ConstVecDerivId(a))->getValue(params);
        const VecDeriv &vb = this->read(core::ConstVecDerivId(b))->getValue(params);

        for (unsigned int i=0; i<va.size(); i++)
        {
            r += va[i] * vb[i];
        }
    }
    else
    {
        serr << "Invalid dot operation ("<<a<<','<<b<<")" << sendl;
    }

    return r;
}

typedef std::size_t nat;

template <class DataTypes>
SReal MechanicalObject<DataTypes>::vSum(const core::ExecParams* params, core::ConstVecId a, unsigned l)
{
    Real r = 0.0;

    if (a.type == sofa::core::V_COORD )
    {
        serr << "Invalid vSum operation: can not compute the sum of V_Coord terms in vector "<< a << sendl;
    }
    else if (a.type == sofa::core::V_DERIV)
    {
        const VecDeriv &va = this->read(core::ConstVecDerivId(a))->getValue(params);

        if( l==0 ) for (nat i=0; i<va.size(); i++)
        {
            for(unsigned j=0; j<DataTypes::deriv_total_size; j++)
                if ( fabs(va[i][j])>r) r=fabs(va[i][j]);
        }
        else for (unsigned int i=0; i<va.size(); i++)
        {
            for(unsigned j=0; j<DataTypes::deriv_total_size; j++)
                r += (Real) exp(va[i][j]/l);
        }
    }
    else
    {
        serr << "Invalid vSum operation ("<<a<<")" << sendl;
    }

    return r;
}

template <class DataTypes>
SReal MechanicalObject<DataTypes>::vMax(const core::ExecParams* params, core::ConstVecId a )
{
    Real r = 0.0;

    if (a.type == sofa::core::V_COORD )
    {
        const VecCoord &va = this->read(core::ConstVecCoordId(a))->getValue(params);

        for (nat i=0; i<va.size(); i++)
        {
            for(unsigned j=0; j<DataTypes::coord_total_size; j++)
                if (fabs(va[i][j])>r) r=fabs(va[i][j]);
        }
    }
    else if (a.type == sofa::core::V_DERIV)
    {
        const VecDeriv &va = this->read(core::ConstVecDerivId(a))->getValue(params);

        for (nat i=0; i<va.size(); i++)
        {
            for(unsigned j=0; j<DataTypes::deriv_total_size; j++)
                if (fabs(va[i][j])>r) r=fabs(va[i][j]);
        }
    }
    else
    {
        serr << "Invalid vMax operation ("<<a<<")" << sendl;
    }

    return r;
}

template <class DataTypes>
size_t MechanicalObject<DataTypes>::vSize(const core::ExecParams* params, core::ConstVecId v)
{
    if (v.type == sofa::core::V_COORD)
    {
        const VecCoord &vv = this->read(core::ConstVecCoordId(v))->getValue(params);
        return vv.size() * Coord::total_size;
    }
    else if (v.type == sofa::core::V_DERIV)
    {
        const VecDeriv &vv = this->read(core::ConstVecDerivId(v))->getValue(params);
        return vv.size() * Deriv::total_size;
    }
    else
    {
        serr << "Invalid size operation ("<<v<<")" << sendl;
        return 0;
    }
}




#ifndef SOFA_SMP
template <class DataTypes>
void MechanicalObject<DataTypes>::printDOF( core::ConstVecId v, std::ostream& out, int firstIndex, int range) const
{
    const unsigned int size=this->getSize();
    if ((unsigned int) (abs(firstIndex)) >= size) return;
    const unsigned int first=((firstIndex>=0)?firstIndex:size+firstIndex);
    const unsigned int max=( ( (range >= 0) && ( (range+first)<size) ) ? (range+first):size);

    if( v.type==sofa::core::V_COORD)
    {
        const Data<VecCoord>* d_x = this->read(core::ConstVecCoordId(v));
        if (d_x == NULL) return;
        helper::ReadAccessor< Data<VecCoord> > x = *d_x;

        if (x.empty())
            return;

        for (unsigned i=first; i<max; ++i)
        {
            out << x[i];
            if (i != max-1)
                out <<" ";
        }
    }
    else if( v.type==sofa::core::V_DERIV)
    {
        const Data<VecDeriv>* d_x = this->read(core::ConstVecDerivId(v));
        if (d_x == NULL) return;
        helper::ReadAccessor< Data<VecDeriv> > x = *d_x;

        if (x.empty())
            return;

        for (unsigned i=first; i<max; ++i)
        {
            out << x[i];
            if (i != max-1)
                out <<" ";
        }
    }
    else
        out<<"MechanicalObject<DataTypes>::printDOF, unknown v.type = "<<v.type<<std::endl;
}
#endif

template <class DataTypes>
unsigned MechanicalObject<DataTypes>::printDOFWithElapsedTime(core::ConstVecId v, unsigned count, unsigned time, std::ostream& out)
{
    if (v.type == sofa::core::V_COORD)
    {
        const Data<VecCoord>* d_x = this->read(core::ConstVecCoordId(v));
        if (d_x == NULL) return 0;
        helper::ReadAccessor< Data<VecCoord> > x = *d_x;

        for (unsigned i = 0; i < x.size(); ++i)
        {
            out << count + i << "\t" << time << "\t" << x[i] << std::endl;
        }
        out << std::endl << std::endl;

        return x.size();
    }
    else if (v.type == sofa::core::V_DERIV)
    {
        const Data<VecDeriv>* d_x = this->read(core::ConstVecDerivId(v));
        if (d_x == NULL) return 0;
        helper::ReadAccessor< Data<VecDeriv> > x = *d_x;

        for (unsigned i = 0; i < x.size(); ++i)
        {
            out << count + i << "\t" << time << "\t" << x[i] << std::endl;
        }
        out << std::endl << std::endl;

        return x.size();
    }
    else
        out << "MechanicalObject<DataTypes>::printDOFWithElapsedTime, unknown v.type = " << v.type << std::endl;

    return 0;
}

template <class DataTypes>
void MechanicalObject<DataTypes>::resetForce(const core::ExecParams* params, core::VecDerivId fid)
{
    {
        helper::WriteOnlyAccessor< Data<VecDeriv> > f( params, *this->write(fid) );
        for (unsigned i = 0; i < f.size(); ++i)
//          if( this->forceMask.getEntry(i) ) // safe getter or not?
                f[i] = Deriv();
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::resetAcc(const core::ExecParams* params, core::VecDerivId aId)
{
    {
        helper::WriteOnlyAccessor< Data<VecDeriv> > a( params, *this->write(aId) );
        for (unsigned i = 0; i < a.size(); ++i)
        {
            a[i] = Deriv();
        }
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::resetConstraint(const core::ExecParams*)
{
    throw std::logic_error("unimplemented");        
}

template <class DataTypes>
void MechanicalObject<DataTypes>::getConstraintJacobian(const core::ExecParams* /*params*/, sofa::defaulttype::BaseMatrix*,unsigned int &)
{
    throw std::logic_error("unimplemented");            
}

#if(SOFA_WITH_EXPERIMENTAL_FEATURES==1)
template <class DataTypes>
void MechanicalObject<DataTypes>::buildIdentityBlocksInJacobian(const sofa::helper::vector<unsigned int>& , core::MatrixDerivId &)
{
    throw std::logic_error("unimplemented");                
}
#endif


template <class DataTypes>
inline void MechanicalObject<DataTypes>::drawIndices(const core::visual::VisualParams* vparams)
{
    defaulttype::Vec4f color(1.0, 1.0, 1.0, 1.0);

    float scale = (float)((vparams->sceneBBox().maxBBox() - vparams->sceneBBox().minBBox()).norm() * showIndicesScale.getValue());

    helper::vector<defaulttype::Vector3> positions;
    for (size_t i = 0; i < vsize; ++i)
        positions.push_back(defaulttype::Vector3(getPX(i), getPY(i), getPZ(i)));

    vparams->drawTool()->draw3DText_Indices(positions, scale, color);
}

template <class DataTypes>
inline void MechanicalObject<DataTypes>::drawVectors(const core::visual::VisualParams* vparams)
{
    float scale = showVectorsScale.getValue();
    sofa::helper::ReadAccessor< Data<VecDeriv> > v_rA = *this->read(core::ConstVecDerivId::velocity());
    helper::vector<Vector3> points;
    points.resize(2);
    for( unsigned i=0; i<v_rA.size(); ++i )
    {
        Real vx=0.0,vy=0.0,vz=0.0;
        DataTypes::get(vx,vy,vz,v_rA[i]);
        //v = DataTypes::getDPos(v_rA[i]);
        //Real vx = v[0]; Real vy = v[1]; Real vz = v[2];
        //std::cout << "v=" << vx << ", " << vy << ", " << vz << std::endl;
        Vector3 p1 = Vector3(getPX(i), getPY(i), getPZ(i));
        Vector3 p2 = Vector3(getPX(i)+scale*vx, getPY(i)+scale*vy, getPZ(i)+scale*vz);

        float rad = (float)( (p1-p2).norm()/20.0 );
        switch (drawMode.getValue())
        {
        case 0:
            points[0] = p1;
            points[1] = p2;
            vparams->drawTool()->drawLines(points, 1, defaulttype::Vec<4,float>(1.0,1.0,1.0,1.0));
            break;
        case 1:
            vparams->drawTool()->drawCylinder(p1, p2, rad, defaulttype::Vec<4,float>(1.0,1.0,1.0,1.0));
            break;
        case 2:
            vparams->drawTool()->drawArrow(p1, p2, rad, defaulttype::Vec<4,float>(1.0,1.0,1.0,1.0));
            break;
        default:
            serr << "No proper drawing mode found!" << sendl;
            break;
        }
    }
}

template <class DataTypes>
inline void MechanicalObject<DataTypes>::draw(const core::visual::VisualParams* vparams)
{
    vparams->drawTool()->saveLastState();
    vparams->drawTool()->setLightingEnabled(false);

    if (showIndices.getValue())
    {
        drawIndices(vparams);
    }

    if (showVectors.getValue())
    {
        drawVectors(vparams);
    }

    if (showObject.getValue())
    {
        const float& scale = showObjectScale.getValue();
        helper::vector<Vector3> positions(vsize);
        for (size_t i = 0; i < vsize; ++i)
            positions[i] = Vector3(getPX(i), getPY(i), getPZ(i));

        switch (drawMode.getValue())
        {
        case 0:
            vparams->drawTool()->drawPoints(positions,scale,defaulttype::Vec<4,float>(d_color.getValue()));
            break;
        case 1:
            vparams->drawTool()->setLightingEnabled(true);
            vparams->drawTool()->drawSpheres(positions,scale,defaulttype::Vec<4,float>(d_color.getValue()));
            break;
        case 2:
            vparams->drawTool()->setLightingEnabled(true);
            vparams->drawTool()->drawSpheres(positions,scale,defaulttype::Vec<4,float>(1.0,0.0,0.0,1.0));
            break;
        case 3:
            vparams->drawTool()->setLightingEnabled(true);
            vparams->drawTool()->drawSpheres(positions,scale,defaulttype::Vec<4,float>(0.0,1.0,0.0,1.0));
            break;
        case 4:
           vparams->drawTool()->setLightingEnabled(true);
            vparams->drawTool()->drawSpheres(positions,scale,defaulttype::Vec<4,float>(0.0,0.0,1.0,1.0));
            break;
        default:
            serr << "No proper drawing mode found!" << sendl;
            break;
        }
    }
    vparams->drawTool()->restoreLastState();
}


/// Find mechanical particles hit by the given ray.
/// A mechanical particle is defined as a 2D or 3D, position or rigid DOF
/// Returns false if this object does not support picking
template <class DataTypes>
bool MechanicalObject<DataTypes>::pickParticles(const core::ExecParams* /* params */, double rayOx, double rayOy, double rayOz, double rayDx, double rayDy, double rayDz, double radius0, double dRadius,
                                                std::multimap< double, std::pair<sofa::core::behavior::BaseMechanicalState*, int> >& particles)
{
    if (defaulttype::DataTypeInfo<Coord>::size() == 2 || defaulttype::DataTypeInfo<Coord>::size() == 3
            || (defaulttype::DataTypeInfo<Coord>::size() == 7 && defaulttype::DataTypeInfo<Deriv>::size() == 6))
        // TODO: this verification is awful and should be done by template specialization
    {
        // seems to be valid DOFs
        const VecCoord& x =this->read(core::ConstVecCoordId::position())->getValue();

        defaulttype::Vec<3,Real> origin((Real)rayOx, (Real)rayOy, (Real)rayOz);
        defaulttype::Vec<3,Real> direction((Real)rayDx, (Real)rayDy, (Real)rayDz);
        for (size_t i=0; i< vsize; ++i)
        {
            defaulttype::Vec<3,Real> pos;
            DataTypes::get(pos[0],pos[1],pos[2],x[i]);

            if (pos == origin) continue;
            SReal dist = (pos-origin)*direction;
            if (dist < 0) continue; // discard particles behind the camera, such as mouse position

            defaulttype::Vec<3,Real> vecPoint = (pos-origin) - direction*dist;
            SReal distToRay = vecPoint.norm2();
            SReal maxr = radius0 + dRadius*dist;
            if (distToRay <= maxr*maxr)
            {
                particles.insert(std::make_pair(distToRay,std::make_pair(this,i)));
            }
        }
        return true;
    }
    else
        return false;
}

template <class DataTypes>
bool MechanicalObject<DataTypes>::closestParticle(const core::ExecParams* /*params*/, defaulttype::Vector3 const& point,
            defaulttype::Vector3 const& origin, double radius0, double dRadius,
            sofa::core::behavior::BaseMechanicalState*& ms, int& index, SReal& distance)
{
    if (defaulttype::DataTypeInfo<Coord>::size() == 2 || defaulttype::DataTypeInfo<Coord>::size() == 3
            || (defaulttype::DataTypeInfo<Coord>::size() == 7 && defaulttype::DataTypeInfo<Deriv>::size() == 6)) {
        // TODO: this verification is awful and should be done by template specialization
        // seems to be valid DOFs
        const VecCoord& x =this->read(core::ConstVecCoordId::position())->getValue();
        for (size_t i=0; i< vsize; ++i) {
            defaulttype::Vec<3,Real> pos;
            DataTypes::get(pos[0],pos[1],pos[2],x[i]);
            SReal d = (pos-point).norm();
            if (d>radius0+(pos-origin).norm()*dRadius) // points too far away are ignored
                continue;
            if (d < distance) {
                ms = this;
                index = i;
                distance = d;
            }
        }
        return true;
    }
    else {
        return false;
    }
}

template <class DataTypes>
bool MechanicalObject<DataTypes>::addBBox(SReal* minBBox, SReal* maxBBox)
{
    // participating to bbox only if it is drawn
    if( !showObject.getValue() ) return false;

    static const unsigned spatial_dimensions = std::min( (unsigned)DataTypes::spatial_dimensions, 3u );

    const VecCoord& x = read(core::ConstVecCoordId::position())->getValue();
    for( std::size_t i=0; i<x.size(); i++ )
    {
        defaulttype::Vec<3,Real> p;
        DataTypes::get( p[0], p[1], p[2], x[i] );

        for( unsigned int j=0 ; j<spatial_dimensions; ++j )
        {
            if(p[j]<minBBox[j]) minBBox[j]=p[j];
            if(p[j]>maxBBox[j]) maxBBox[j]=p[j];
        }
    }
    return true;
}


template <class DataTypes>
void MechanicalObject<DataTypes>::computeBBox(const core::ExecParams* params, bool onlyVisible)
{
    // participating to bbox only if it is drawn
    if( onlyVisible && !showObject.getValue() ) return;
    Inherited::computeBBox( params );
}

template <class DataTypes>
bool MechanicalObject<DataTypes>::isIndependent() const
{
    return static_cast<const simulation::Node*>(this->getContext())->mechanicalMapping.empty();
}


} // namespace container

} // namespace component

} // namespace sofa

#endif

