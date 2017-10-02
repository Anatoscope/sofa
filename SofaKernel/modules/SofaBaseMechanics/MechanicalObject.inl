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

#include <SofaBaseLinearSolver/SparseMatrix.h>
#include <sofa/core/topology/BaseTopology.h>
#include <sofa/core/topology/TopologyChange.h>

#include <sofa/defaulttype/Quat.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/DataTypeInfo.h>

#include <sofa/helper/accessor.h>

#include <sofa/simulation/Node.h>
#include <sofa/simulation/Simulation.h>

#ifdef SOFA_DUMP_VISITOR_INFO
#include <sofa/simulation/Visitor.h>
#endif

#include <cassert>
#include <iostream>



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
    , showObject(initData(&showObject, (bool) false, "showObject", "Show objects. (default=false)"))
    , showObjectScale(initData(&showObjectScale, (float) 0.1, "showObjectScale", "Scale for object display. (default=0.1)"))
    , showIndices(initData(&showIndices, (bool) false, "showIndices", "Show indices. (default=false)"))
    , showIndicesScale(initData(&showIndicesScale, (float) 0.02, "showIndicesScale", "Scale for indices display. (default=0.02)"))
    , showVectors(initData(&showVectors, (bool) false, "showVectors", "Show velocity. (default=false)"))
    , showVectorsScale(initData(&showVectorsScale, (float) 0.0001, "showVectorsScale", "Scale for vectors display. (default=0.0001)"))
    , drawMode(initData(&drawMode,0,"drawMode","The way vectors will be drawn:\n- 0: Line\n- 1:Cylinder\n- 2: Arrow.\n\nThe DOFS will be drawn:\n- 0: point\n- >1: sphere. (default=0)"))
    , d_color(initData(&d_color, defaulttype::Vec4f(1,1,1,1), "showColor", "Color for object display. (default=[1 1 1 1])"))
    , f_reserve(initData(&f_reserve, 0, "reserve", "Size to reserve when creating vectors. (default=0)"))
    , vsize(0)
{

    m_initialized = false;

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

    // TODO wtf?
    // These vectors are set as modified as they are mandatory in the
    // MechanicalObject.
    x.forceSet();
    v.forceSet();
    f.forceSet();
    externalForces.forceSet();

    // apparently, these are not
    //  x0              .forceSet();
    //  dx              .forceSet();

    resize(1);
}


template <class DataTypes>
MechanicalObject<DataTypes>::~MechanicalObject() {

    // TODO oh and by the way deleting a null pointer is a noop.
    
    // TODO auto pointers for the love of god
    // TODO at the very least package this shit up in a class so we don't
    // duplicate code between coord/deriv
    for(unsigned i = core::VecCoordId::V_FIRST_DYNAMIC_INDEX; i<vectorsCoord.size(); i++) {
        if( vectorsCoord[i] ) {
            delete vectorsCoord[i];
        }
    }

    // TODO are you fucking kidding me? we may allocate for zero vector? 
    if( vectorsCoord[core::VecCoordId::null().getIndex()] ) {
        delete vectorsCoord[core::VecCoordId::null().getIndex()];
    }

    // TODO auto pointers for the love of god
    // TODO at the very least package this shit up in a class so we don't
    // duplicate code between coord/deriv
    for(unsigned i = core::VecDerivId::V_FIRST_DYNAMIC_INDEX; i<vectorsDeriv.size(); i++) {
        if( vectorsDeriv[i] )  {
            delete vectorsDeriv[i];
        }
    }

    // TODO are you fucking kidding me? we may allocate for zero vector? 
    if( vectorsDeriv[core::VecDerivId::null().getIndex()] ) {
        delete vectorsDeriv[core::VecDerivId::null().getIndex()];
    }

    // TODO wtf is this shit
    if( core::VecDerivId::dforce().getIndex() < vectorsDeriv.size() && vectorsDeriv[core::VecDerivId::dforce().getIndex()] ) {
        delete vectorsDeriv[core::VecDerivId::dforce().getIndex()];
    }

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
void MechanicalObject<DataTypes>::resize(const size_t size) {
    vsize = size;

    // TODO there is something *REALLLY* fishy going on here
    // TODO why do we need to check isSet ? (if we don't, it does not work)
    
    for (auto* vi : vectorsCoord) {
        if (vi && vi->isSet() ) {

            if(size) vi->beginWriteOnly()->resize(size);
            else vi->beginWriteOnly()->clear();
        
            vi->endEdit();
        }
    }

    for (auto* vi : vectorsDeriv) {
        if(vi && vi->isSet() ) {

            if(size) vi->beginWriteOnly()->resize(size);
            else vi->beginWriteOnly()->clear();
        
            vi->endEdit();
        }
    }

    if(size) this->forceMask.resize(size);
    else this->forceMask.clear();
}


template <class DataTypes>
void MechanicalObject<DataTypes>::reserve(const size_t size) {
    if (size == 0) return;

    for (auto* vi : vectorsCoord) {
        if (vi && vi->isSet()) {
            vi->beginWriteOnly()->reserve(size);
            vi->endEdit();
        }
    }

    for (auto* vi : vectorsDeriv ){
        
        if (vi && vi->isSet() ) {
            vi->beginWriteOnly()->reserve(size);
            vi->endEdit();
        }
    }
}


template <class DataTypes>
void MechanicalObject<DataTypes>::getIndicesInSpace(sofa::helper::vector<unsigned>& indices, 
                                                    Real xmin, Real xmax,
                                                    Real ymin, Real ymax,  
                                                    Real zmin, Real zmax) const {
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
void MechanicalObject<DataTypes>::init()
{

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


    // // TODO these should all be errors, period.
    sofa::core::topology::BaseMeshTopology* m_topology = nullptr;

    //Look at a topology associated to this instance of MechanicalObject by a tag
    this->getContext()->get(m_topology, this->getTags());
     
    // // If no topology found, no association, then look to the nearest one
    if(!m_topology) {
        m_topology = this->getContext()->getMeshTopology();
    }
    
    // the given position and velocity vectors are empty
    // note that when a vector is not  explicitly specified, its size won't change (1 by default)
    if( x_wA.size() <= 1 && v_wA.size() <= 1 ) {

        // if a topology is present, implicitly copy position from it
        if (m_topology != NULL && m_topology->getNbPoints() && m_topology->getContext() == this->getContext())
        {
            msg_warning() << "doing funny stuff during mechanical state init";
            
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
        } else if( x_wA.size() == 0 ) {
            // special case when the user manually explicitly defined an empty position vector
            // (e.g. linked to an empty vector)
            resize(0);
        }
    } else if (x_wA.size() != vsize || v_wA.size() != vsize) {
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

    // TODO there are no longer transformations lol
    // storing X0 must be done after reinit() that possibly applies transformations
    if( read(core::ConstVecCoordId::restPosition())->getValue().size()!=x_wA.size() ) {
        // storing X0 from X
        vOp(core::ExecParams::defaultInstance(), core::VecId::restPosition(), core::VecId::position());
    }

    m_initialized = true;

    if (f_reserve.getValue() > 0) {
        reserve(f_reserve.getValue());
    }
}

template <class DataTypes>
void MechanicalObject<DataTypes>::reinit() { }

template <class DataTypes>
void MechanicalObject<DataTypes>::storeResetState()
{
    if(!static_cast<const simulation::Node*>(this->getContext())->mechanicalMapping.empty() ) {
        msg_warning() << "storeResetState on mapped dofs";
    }
    
    // Save initial state for reset button
    vOp(core::ExecParams::defaultInstance(), core::VecId::resetPosition(), core::VecId::position());

    //vOp(VecId::resetVelocity(), VecId::velocity());
    // we only store a resetVelocity if the velocity is not zero
    helper::ReadAccessor< Data<VecDeriv> > v = *this->read(core::VecDerivId::velocity());
    bool zero = true;
    for (unsigned int i=0; i<v.size(); ++i) {
        const Deriv& vi = v[i];
        for (unsigned int j=0; j<vi.size(); ++j)
            if (vi[j] != 0) zero = false;
        if (!zero) break;
    }
    if (!zero) {
        vOp(core::ExecParams::defaultInstance(), core::VecId::resetVelocity(), core::VecId::velocity());
    }
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

    this->externalForces.beginWriteOnly()->clear();
    this->externalForces.endEdit();

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
    if (!val.empty() && val.size() != (unsigned int)this->getSize()) {
        msg_error() << "writing to State vector " << v << " with incorrect size : " 
                    << val.size() << " != " << this->getSize();
    }
#endif
    return d;
}



template <class DataTypes>
const Data<typename MechanicalObject<DataTypes>::VecCoord>* MechanicalObject<DataTypes>::read(core::ConstVecCoordId v) const
{
    if (v.isNull())
    {
        msg_error() << "Accessing null VecCoord";
    }
    if (v.index < vectorsCoord.size() && vectorsCoord[v.index] != NULL)
    {
        const Data<typename MechanicalObject<DataTypes>::VecCoord>* d = vectorsCoord[v.index];
#if defined(SOFA_DEBUG) || !defined(NDEBUG)
        const typename MechanicalObject<DataTypes>::VecCoord& val = d->getValue();
        if (!val.empty() && val.size() != (unsigned int)this->getSize())
        {
            msg_error() << "Accessing State vector " << v << " with incorrect size : " << val.size() << " != " << this->getSize();
        }
#endif
        return d;
    }
    else
    {
        msg_error() << "Vector " << v << " does not exist";
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
        msg_error() << "Writing to State vector " << v << " with incorrect size : " << val.size() << " != " << this->getSize();
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
            msg_error() << "Accessing State vector " << v << " with incorrect size : " << val.size() << " != " << this->getSize();
        }
#endif
        return d;
    }
    else
    {
        msg_error() << "Vector " << v << "does not exist";
        return NULL;
    }
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

} //

// visit a (Const)VecId with proper type: (Const)CoordVecId or
// (Const)DerivVecId, or the (unsigned) vector index for V_ALL

// visitor gets called with: visitor(vecid, args...);
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
        throw std::runtime_error("unimplemented vop for vecid: " + std::to_string(id.type));
    }
}



// 1. do 3 dispatches to obtain proper types for arguments v, a, b
// 2. prune non-legal v = a + b patterns through the various prune overloads
// 3. call the dispatch operator on the resulting typed v, a, b
struct dispatch_visitor {

    // first dispatch got type for v, continuing on a
    template<class V, class Dispatch>
    void operator()(V v, core::ConstVecId a, core::ConstVecId b, const Dispatch& dispatch) const {
        visit(a, *this, v, b, dispatch);
    }

    // second dispatch got type for a, continuing on b
    template<class A, class V, class Dispatch>
    void operator()(A a, V v, core::ConstVecId b, const Dispatch& dispatch) const {
        visit(b, *this, v, a, dispatch);
    }

    // third dispatch got type for b, continue with pruning v, a, b
    template<class B, class A, class V, class Dispatch>
    void operator()(B b, V v, A a, const Dispatch& dispatch) const {
        const prune_type<Dispatch> prune = { dispatch };
        prune(v, a, b);
    }

    template<class Dispatch>
    struct prune_type {
        const Dispatch& dispatch;

        // prune illegal patterns
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

        // properly type zero vectors
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




// map between VecTypes and actual types
template<class T, core::VecType> struct vector_traits;

template<class T> struct vector_traits<T, core::V_COORD> {
    using type = typename T::VecCoord;
};

template<class T> struct vector_traits<T, core::V_DERIV> {
    using type = typename T::VecDeriv;
};


// return a const expr::ref for the given vecid in state
template<class T, core::VecType type>
static expr::ref_type< const typename vector_traits<T, type>::type > vec_ref(const core::State<T>* state,
                                                                             core::TVecId<type, core::V_READ> id) {
    assert(!id.isNull());
    return expr::ref(state->read(id)->getValue());
}


// return a writeable expr::ref for the given vecid in state
template<class T, core::VecType type>
static expr::ref_type< typename vector_traits<T, type>::type > vec_ref(core::State<T>* state,
                                                                       core::TVecId<type, core::V_WRITE> id) {
    assert(!id.isNull());
    auto& cast = const_cast<typename vector_traits<T, type>::type&>(state->read(id)->getValue());
    return expr::ref(cast);
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
        auto lvalue = vec_ref(state, v);
        
        const bool a_null = a.isNull();
        const bool b_null = b.isNull() || f == 0;
        
        // note: we need to dispatch on a == 0 since coord + deriv yields coord
        // or deriv depending on a == zero (lolwat)
        if(a_null && b_null ) {
            assign(lvalue, null(v));
        } else if(b_null) {
            assign(lvalue, vec_ref(state, a));
        } else if(a_null) {
            assign(lvalue, f * vec_ref(state, b));
        } else {
            assign(lvalue, vec_ref(state, a) + f * vec_ref(state, b));
        }
        
        // signal modifications
        auto data = state->write(v);
        data->beginWriteOnly();
        data->endEdit();
    }

    // need an extra layer since assignment might trigger type errors (ie. coord
    // = coord + deriv when coord == 0), which we detect and turn into runtime
    // errors.
    template<class V, class E>
    static typename std::enable_if< std::is_assignable<typename V::value_type,
                                                       typename E::value_type>::value  >::type
    assign(V v, E e) { v = e; }

    template<class V, class E>
    static typename std::enable_if< !std::is_assignable<typename V::value_type,
                                                        typename E::value_type>::value  >::type
    assign(V, E) {
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
        if(v.isNull()) throw std::runtime_error("cannot assign to null vector");

        const dispatch_type<DataTypes> dispatch = {this, f};
        visit(v, dispatch_visitor(), a, b, dispatch);
        
    } catch (std::runtime_error& e) {
        msg_error() << "in vop " << v << " = " << a << " + " << b << " * " << f << ": " << e.what();
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
        msg_error()<<"vThreshold does not apply to coordinate vectors";
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
        msg_error() << "Invalid dot operation ("<<a<<','<<b<<")";
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
        msg_error() << "Invalid vSum operation: can not compute the sum of V_Coord terms in vector "<< a;
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
        msg_error() << "Invalid vSum operation ("<<a<<")";
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
        msg_error() << "Invalid vMax operation ("<<a<<")";
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
        msg_error() << "Invalid size operation ("<<v<<")";
        return 0;
    }
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
inline void MechanicalObject<DataTypes>::drawIndices(const core::visual::VisualParams* vparams)
{
    const defaulttype::Vec4f color(1.0, 1.0, 1.0, 1.0);

    const float scale = ((vparams->sceneBBox().maxBBox() - vparams->sceneBBox().minBBox()).norm() * showIndicesScale.getValue());
    
    helper::vector<defaulttype::Vector3> positions;
    
    const VecCoord& pos = this->read(core::ConstVecCoordId::position())->getValue();
    
    for (const Coord& pi : pos) {
        Real x, y, z;
        DataTypes::get(x, y, z, pi);
        positions.push_back( {x, y, z} );
    }

    vparams->drawTool()->draw3DText_Indices(positions, scale, color);
}

template <class DataTypes>
inline void MechanicalObject<DataTypes>::drawVectors(const core::visual::VisualParams* vparams)
{
    float scale = showVectorsScale.getValue();

    const VecCoord& pos = this->read(core::ConstVecCoordId::position())->getValue();
    const VecDeriv& vel = this->read(core::ConstVecDerivId::velocity())->getValue();    

    // this is for drawLines (lol)
    helper::vector<Vector3> points;
    points.resize(2);
    
    for(std::size_t i = 0, n = this->getSize(); i < n; ++i) {
        Real x, y, z;
        DataTypes::get(x, y, z, pos[i]);
        
        Real vx, vy, vz;
        DataTypes::get(vx, vy, vz, vel[i]);
        
        const Vector3 p1 = {x, y, z};
        const Vector3 p2 = {x + scale * vx,
                            y + scale * vy,
                            z + scale * vz};

        const float rad = ( (p1 - p2).norm() / 20.0 ); // lolwat
        switch (drawMode.getValue()) {
        case 0:
            points[0] = p1;
            points[1] = p2;
            
            vparams->drawTool()->drawLines(points, 1, defaulttype::Vec4f(1.0,1.0,1.0,1.0));
            break;
        case 1:
            vparams->drawTool()->drawCylinder(p1, p2, rad, defaulttype::Vec4f(1.0,1.0,1.0,1.0));
            break;
        case 2:
            vparams->drawTool()->drawArrow(p1, p2, rad, defaulttype::Vec4f(1.0,1.0,1.0,1.0));
            break;
        default:
            msg_error() << "unknown drawing mode found!";
        }
    }
}

template <class DataTypes>
inline void MechanicalObject<DataTypes>::draw(const core::visual::VisualParams* vparams)
{
    vparams->drawTool()->saveLastState();
    vparams->drawTool()->setLightingEnabled(false);

    if (showIndices.getValue()) { drawIndices(vparams); }

    if (showVectors.getValue()) { drawVectors(vparams); }

    if (showObject.getValue()) {
        const float scale = showObjectScale.getValue();

        const VecCoord& pos = this->read(core::ConstVecCoordId::position())->getValue();        

        helper::vector<Vector3> positions(vsize);
        for (size_t i = 0; i < vsize; ++i) {
            Real x, y, z;
            DataTypes::get(x, y, z, pos[i]);
            positions[i] = {x, y, z};
        }
        
        switch (drawMode.getValue()) {
        case 0:
            vparams->drawTool()->drawPoints(positions,scale,defaulttype::Vec4f(d_color.getValue()));
            break;
        case 1:
            vparams->drawTool()->setLightingEnabled(true);
            vparams->drawTool()->drawSpheres(positions,scale,defaulttype::Vec4f(d_color.getValue()));
            break;
        case 2:
            vparams->drawTool()->setLightingEnabled(true);
            vparams->drawTool()->drawSpheres(positions,scale,defaulttype::Vec4f(1.0,0.0,0.0,1.0));
            break;
        case 3:
            vparams->drawTool()->setLightingEnabled(true);
            vparams->drawTool()->drawSpheres(positions,scale,defaulttype::Vec4f(0.0,1.0,0.0,1.0));
            break;
        case 4:
           vparams->drawTool()->setLightingEnabled(true);
            vparams->drawTool()->drawSpheres(positions,scale,defaulttype::Vec4f(0.0,0.0,1.0,1.0));
            break;
        default:
            msg_error() << "unknown drawing mode";
        }
    }
    vparams->drawTool()->restoreLastState();
}

// TODO this should probably go into standard traits anyways
template<class T> struct pickable { static const bool value = false; };

template<class U> struct pickable< defaulttype::Vec<2, U> > {
    static const bool value = true;
};

template<class U> struct pickable< defaulttype::Vec<3, U> > {
    static const bool value = true;
};

template<class U> struct pickable< defaulttype::RigidCoord<2, U> > {
    static const bool value = true;
};

template<class U> struct pickable< defaulttype::RigidCoord<3, U> > {
    static const bool value = true;
};


/// Find mechanical particles hit by the given ray.
/// A mechanical particle is defined as a 2D or 3D, position or rigid DOF
/// Returns false if this object does not support picking
template <class DataTypes>
bool MechanicalObject<DataTypes>::pickParticles(const core::ExecParams* /* params */, double rayOx, double rayOy, double rayOz, double rayDx, double rayDy, double rayDz, double radius0, double dRadius,
                                                std::multimap< double, std::pair<sofa::core::behavior::BaseMechanicalState*, int> >& particles)
{
    if ( pickable<Coord>::value ) {
        // seems to be valid DOFs
        const VecCoord& x =this->read(core::ConstVecCoordId::position())->getValue();

        defaulttype::Vec<3,Real> origin((Real)rayOx, (Real)rayOy, (Real)rayOz);
        defaulttype::Vec<3,Real> direction((Real)rayDx, (Real)rayDy, (Real)rayDz);

        for (size_t i=0; i< vsize; ++i) {
            defaulttype::Vec<3,Real> pos;
            DataTypes::get(pos[0], pos[1], pos[2], x[i]);

            if (pos == origin) continue;
            const SReal dist = (pos - origin) * direction;
            if (dist < 0) continue; // discard particles behind the camera, such as mouse position

            defaulttype::Vec<3,Real> vecPoint = (pos-origin) - direction * dist;
            const SReal distToRay = vecPoint.norm2();
            const SReal maxr = radius0 + dRadius * dist;
            if (distToRay <= maxr*maxr) {
                particles.insert( { distToRay, {this, i} } );
            }
        }
        return true;
    }

    return false;
}

template <class DataTypes>
bool MechanicalObject<DataTypes>::closestParticle(const core::ExecParams* /*params*/, defaulttype::Vector3 const& point,
            defaulttype::Vector3 const& origin, double radius0, double dRadius,
            sofa::core::behavior::BaseMechanicalState*& ms, int& index, SReal& distance)
{
    if (pickable<Coord>::value ) {

        const VecCoord& x = this->read(core::ConstVecCoordId::position())->getValue();
        for (size_t i=0; i< vsize; ++i) {
            
            defaulttype::Vec<3,Real> pos;
            DataTypes::get(pos[0], pos[1], pos[2], x[i]);
            
            const SReal d = (pos-point).norm();
            if (d > radius0 + (pos-origin).norm() * dRadius) // points too far away are ignored
                continue;
            if (d < distance) {
                ms = this;
                index = i;
                distance = d;
            }
        }
        return true;
    }

    return false;
}

template <class DataTypes>
bool MechanicalObject<DataTypes>::addBBox(SReal* minBBox, SReal* maxBBox)
{
    // participating to bbox only if it is drawn
    if( !showObject.getValue() ) return false;

    static const unsigned spatial_dimensions = std::min( (unsigned)DataTypes::spatial_dimensions, 3u );

    const VecCoord& x = read(core::ConstVecCoordId::position())->getValue();
    for( std::size_t i=0; i<x.size(); i++ ) {
        defaulttype::Vec<3,Real> p;
        DataTypes::get( p[0], p[1], p[2], x[i] );

        for( unsigned int j=0 ; j<spatial_dimensions; ++j )
        {
            if(p[j] < minBBox[j]) minBBox[j]=p[j];
            if(p[j] > maxBBox[j]) maxBBox[j]=p[j];
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



} // namespace container

} // namespace component

} // namespace sofa

#endif

