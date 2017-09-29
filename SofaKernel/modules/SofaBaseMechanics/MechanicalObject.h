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
#ifndef SOFA_COMPONENT_MECHANICALOBJECT_H
#define SOFA_COMPONENT_MECHANICALOBJECT_H
#include "config.h"

#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/objectmodel/DataFileName.h>
#include <sofa/core/topology/BaseMeshTopology.h>

#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/BaseVector.h>

// #include <sofa/defaulttype/MapMapSparseMatrix.h>

#include <vector>

namespace sofa {

namespace component {

namespace container {


// TODO remove this crap
template<class DataTypes>
class MechanicalObjectInternalData { };

/** 
 * @brief MechanicalObject class TODO no shit sherlock
 */
template <class DataTypes>
class MechanicalObject : public sofa::core::behavior::MechanicalState<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(MechanicalObject, DataTypes),
               SOFA_TEMPLATE(sofa::core::behavior::MechanicalState, DataTypes));

    typedef sofa::core::behavior::MechanicalState<DataTypes>      Inherited;
    typedef typename Inherited::VMultiOp    VMultiOp;
    typedef typename Inherited::ForceMask   ForceMask;
    typedef typename DataTypes::Real        Real;
    typedef typename DataTypes::Coord       Coord;
    typedef typename DataTypes::Deriv       Deriv;
    typedef typename DataTypes::VecCoord    VecCoord;
    typedef typename DataTypes::VecDeriv    VecDeriv;
    typedef typename DataTypes::MatrixDeriv						MatrixDeriv;

    // TODO remove
    typedef typename core::behavior::BaseMechanicalState::ConstraintBlock ConstraintBlock;

    typedef sofa::defaulttype::Vector3 Vector3;

protected:
    // TODO public ffs
    MechanicalObject();
public:
    // TODO bad idea
    MechanicalObject& operator = ( const MechanicalObject& );
protected:
    virtual ~MechanicalObject();
public:
    virtual void parse ( core::objectmodel::BaseObjectDescription* arg );

    // TODO find a way of NOT allocating each of these separately
    Data< VecCoord > x;
    Data< VecDeriv > v;
    Data< VecDeriv > f;
    Data< VecDeriv > externalForces;
    Data< VecDeriv > dx;
    Data< VecCoord > x0;

    Data< VecCoord > reset_position;
    Data< VecDeriv > reset_velocity;

    Data< bool >  showObject;
    Data< float > showObjectScale;

    Data< bool >  showIndices;
    Data< float > showIndicesScale;

    Data< bool >  showVectors;
    Data< float > showVectorsScale;
    
    Data< int > drawMode;
    Data< defaulttype::Vec4f > d_color;  ///< drawing color

    virtual void init();
    virtual void reinit();

    virtual void storeResetState();

    virtual void reset();

    virtual void writeVec(core::ConstVecId v, std::ostream &out);
    virtual void readVec(core::VecId v, std::istream &in);

    // TODO wtf is this doing 
    virtual void writeState( std::ostream& out );    

    // TODO WTF
    virtual SReal compareVec(core::ConstVecId , std::istream &) {
        throw std::logic_error("unimplemented");
    }
    
    void setIgnoreLoader(bool) {
        throw std::logic_error("unimplemented");
    }

    /// @name New vectors access API based on VecId
    /// @{

    virtual Data< VecCoord >* write(core::VecCoordId v);
    virtual const Data< VecCoord >* read(core::ConstVecCoordId v) const;

    virtual Data< VecDeriv >* write(core::VecDerivId v);
    virtual const Data< VecDeriv >* read(core::ConstVecDerivId v) const;

    virtual Data< MatrixDeriv >* write(core::MatrixDerivId) {
        throw std::logic_error("unimplemented");
    }
    virtual const Data< MatrixDeriv >* read(core::ConstMatrixDerivId) const  {
        throw std::logic_error("unimplemented");
    }

    /// @}

    virtual void resize( size_t vsize);
    virtual void reserve(size_t vsize);

    size_t getSize() const { return vsize; }

    virtual SReal getPX(std::size_t i) const {
        SReal x, y, z;
        DataTypes::get(x, y, z, read( core::ConstVecCoordId::position() )->getValue()[i]);
        return x;
    }
    
    virtual SReal getPY(std::size_t i) const {
        SReal x, y, z;
        DataTypes::get(x, y, z, read( core::ConstVecCoordId::position() )->getValue()[i]);
        return y;
    }
    
    virtual SReal getPZ(std::size_t i) const {
        SReal x, y, z;
        DataTypes::get(x, y, z, read( core::ConstVecCoordId::position() )->getValue()[i]);
        return z;
    }
    


    /** \brief Reorder values according to parameter.
     *
     * Result of this method is :
     * newValue[ i ] = oldValue[ index[i] ];
     */
    void renumberValues( const sofa::helper::vector<unsigned int> &index );

    /** \brief Replace the value at index by the sum of the ancestors values
     * weithed by the coefs.
     *
     * Sum of the coefs should usually equal to 1.0
     */
    void computeWeightedValue( const unsigned int, 
                               const sofa::helper::vector< unsigned int >&, 
                               const sofa::helper::vector< double >& ) {
        throw std::logic_error("unimplemented");
    }

    /// Force the position of a point (and force its velocity to zero value)
    void forcePointPosition( const unsigned int, const sofa::helper::vector< double >&) {
        throw std::logic_error("unimplemented");
    }

    /// @name Initial transformations application methods.
    /// @{

    /// Apply translation vector to the position.
    
    virtual void applyTranslation (const SReal, const SReal, const SReal) {
        throw std::logic_error("unimplemented");
    }

    /// Rotation using Euler Angles in degree.
    virtual void applyRotation (const SReal, const SReal, const SReal) {
        throw std::logic_error("unimplemented");
    }

    virtual void applyRotation (const defaulttype::Quat) {
        throw std::logic_error("unimplemented");
    }

    virtual void applyScale (const SReal, const SReal, const SReal) {
        throw std::logic_error("unimplemented");
    }

    /// @}

    /// Get the indices of the particles located in the given bounding box
    void getIndicesInSpace(sofa::helper::vector<unsigned>& indices, Real xmin, Real xmax, 
                           Real ymin, Real ymax, Real zmin, Real zmax) const;


    
    // TODO fix this broken API
    /// update the given bounding box, to include this
    virtual bool addBBox(SReal* minBBox, SReal* maxBBox);
    /// Bounding Box computation method.
    virtual void computeBBox(const core::ExecParams* params, bool onlyVisible=false);




    
    /// @name Base Matrices and Vectors Interface
    /// @{

    /// Copy data to a global BaseVector the state stored in a local vector
    /// @param offset the offset in the BaseVector where the scalar values will
    /// be used. It will be updated to the first scalar value after the ones
    /// used by this operation when this method returns
    virtual void copyToBaseVector(defaulttype::BaseVector* dest, core::ConstVecId src, unsigned int &offset);

    /// Copy data to a local vector the state stored in a global BaseVector
    /// @param offset the offset in the BaseVector where the scalar values will
    /// be used. It will be updated to the first scalar value after the ones
    /// used by this operation when this method returns
    virtual void copyFromBaseVector(core::VecId dest, const defaulttype::BaseVector* src, unsigned int &offset);

    /// Add data to a global BaseVector from the state stored in a local vector
    /// @param offset the offset in the BaseVector where the scalar values will
    /// be used. It will be updated to the first scalar value after the ones
    /// used by this operation when this method returns
    virtual void addToBaseVector(defaulttype::BaseVector* dest, core::ConstVecId src, unsigned int &offset);


    // TODO deprecated & remove this shit
    virtual void addFromBaseVectorSameSize(core::VecId ,  const defaulttype::BaseVector*, unsigned int &) {
        throw std::logic_error("unimplemented");
    }

    virtual void addFromBaseVectorDifferentSize(core::VecId, const defaulttype::BaseVector*, unsigned int& ) {
        throw std::logic_error("unimplemented");
    }


    /// @}

    void setTranslation(SReal, SReal, SReal) { throw std::logic_error("unimplemented"); }
    void setRotation(SReal, SReal, SReal) { throw std::logic_error("unimplemented"); }
    void setScale(SReal, SReal, SReal) { throw std::logic_error("unimplemented"); }
    Vector3 getTranslation() const { return {};}
    
    // virtual Vector3 getRotation() const {return rotation.getValue();}
    // virtual Vector3 getScale() const {return scale.getValue();}

    /// @}


    
    /// Renumber the constraint ids with the given permutation vector
    
    void renumberConstraintId(const sofa::helper::vector< unsigned >&) {
        throw std::logic_error("unimplemented");
    }
    

    /// @name Integration related methods
    /// @{

    virtual void beginIntegration(SReal dt);

    virtual void endIntegration(const core::ExecParams* params, SReal dt);

    // see BaseMechanicalState::accumulateForce(const ExecParams*, VecId)    
    virtual void accumulateForce(const core::ExecParams* params, 
                                 core::VecDerivId f = core::VecDerivId::force()); 

    /// Increment the index of the given VecCoordId, so that all 'allocated'
    /// vectors in this state have a lower index
    virtual void vAvail(const core::ExecParams* params, core::VecCoordId& v);
    
    /// Increment the index of the given VecDerivId, so that all 'allocated'
    /// vectors in this state have a lower index
    virtual void vAvail(const core::ExecParams* params, core::VecDerivId& v);

    /// Allocate a new temporary vector
    virtual void vAlloc(const core::ExecParams* params, core::VecCoordId v);
    /// Allocate a new temporary vector
    virtual void vAlloc(const core::ExecParams* params, core::VecDerivId v);

    // TODO WTF is this supposed to mean
    /// Reallocate a new temporary vector
    virtual void vRealloc(const core::ExecParams* params, core::VecCoordId v);
    /// Reallocate a new temporary vector
    virtual void vRealloc(const core::ExecParams* params, core::VecDerivId v);


    /// Free a temporary vector
    virtual void vFree(const core::ExecParams* params, core::VecCoordId v);
    /// Free a temporary vector
    virtual void vFree(const core::ExecParams* params, core::VecDerivId v);
    
    // TODO WTF is this supposed to mean
    
    /// Initialize an unset vector
    virtual void vInit(const core::ExecParams* params, core::VecCoordId v, core::ConstVecCoordId vSrc);
    virtual void vInit(const core::ExecParams* params, core::VecDerivId v, core::ConstVecDerivId vSrc);

    virtual void vOp(const core::ExecParams* params, core::VecId v, 
                     core::ConstVecId a = core::ConstVecId::null(), 
                     core::ConstVecId b = core::ConstVecId::null(), SReal f=1.0);
    
    virtual void vMultiOp(const core::ExecParams* params, const VMultiOp& ops);

    // TODO WTF IS THIS
    virtual void vThreshold(core::VecId a, SReal threshold );

    virtual SReal vDot(const core::ExecParams* params, core::ConstVecId a, core::ConstVecId b);

    /// Sum of the entries of state vector a at the power of l>0. This is used
    /// to compute the l-norm of the vector.
    virtual SReal vSum(const core::ExecParams* params, core::ConstVecId a, unsigned l);

    /// Maximum of the absolute values of the entries of state vector a. This is
    /// used to compute the infinite-norm of the vector.
    virtual SReal vMax(const core::ExecParams* params, core::ConstVecId a);

    // TODO remove these duplicates
    virtual size_t vSize( const core::ExecParams* params, core::ConstVecId v );
    virtual void resetForce(const core::ExecParams* params, core::VecDerivId f = core::VecDerivId::force());
    virtual void resetAcc(const core::ExecParams* params, core::VecDerivId a = core::VecDerivId::dx());
    
    // TODO remove
    virtual void resetConstraint(const core::ExecParams*) {
        throw std::logic_error("unimplemented");
    }
    
    virtual void getConstraintJacobian(const core::ExecParams* , 
                                       sofa::defaulttype::BaseMatrix*, 
                                       unsigned int & ) {
        throw std::logic_error("unimplemented");
    }
    
    /// @name Debug
    /// @{

    // // TODO remove
    virtual void printDOF(core::ConstVecId, std::ostream&, int, int) const {
        throw std::logic_error("unimplemented");
    }
    
    virtual unsigned printDOFWithElapsedTime(core::ConstVecId, unsigned, unsigned, std::ostream& ) {
        throw std::logic_error("unimplemented");
    }

    void draw(const core::visual::VisualParams* vparams);

    /// @}

    /// Find mechanical particles hit by the given ray.
    /// A mechanical particle is defined as a 2D or 3D, position or rigid DOF
    /// Returns false if this object does not support picking
    virtual bool pickParticles(const core::ExecParams* params, 
                               double rayOx, double rayOy, double rayOz, 
                               double rayDx, double rayDy, double rayDz, 
                               double radius0, double dRadius,
                               std::multimap< double, std::pair<sofa::core::behavior::BaseMechanicalState*, int> >& particles);

    virtual bool closestParticle(const core::ExecParams* params,
                                 Vector3 const& point, defaulttype::Vector3 const& origin,
                                 double radius0, double dRadius,
                                 sofa::core::behavior::BaseMechanicalState*& ms, int& index, SReal& distance);


protected :

    Data< int > f_reserve;

    bool m_initialized;

    /// @name Integration-related data
    /// @{

    ///< Coordinates DOFs vectors table (static and dynamic allocated)    
    sofa::helper::vector< Data< VecCoord >		* > vectorsCoord;
    
    ///< Derivates DOFs vectors table (static and dynamic allocated)    
    sofa::helper::vector< Data< VecDeriv >		* > vectorsDeriv;		
    
    size_t vsize; ///< Number of elements to allocate in vectors

    /**
     * @brief Inserts VecCoord DOF coordinates vector at index in the vectorsCoord container.
     */
    void setVecCoord(unsigned int /*index*/, Data< VecCoord >* /*vCoord*/);

    /**
     * @brief Inserts VecDeriv DOF derivates vector at index in the vectorsDeriv container.
     */
    void setVecDeriv(unsigned int /*index*/, Data< VecDeriv >* /*vDeriv*/);

    /**
    * @brief Internal function : Draw indices in 3d coordinates.
    */
    void drawIndices(const core::visual::VisualParams* vparams);

    /**
    * @brief Internal function : Draw vectors
    */
    void drawVectors(const core::visual::VisualParams* vparams);


    // TODO this is ridiculous
    MechanicalObjectInternalData<DataTypes> data;
    friend class MechanicalObjectInternalData<DataTypes>;
};


// TODO we should have drawing components for this
#ifndef SOFA_FLOAT
template<> SOFA_BASE_MECHANICS_API
void MechanicalObject<defaulttype::Rigid3dTypes>::draw(const core::visual::VisualParams* vparams);
#endif
#ifndef SOFA_DOUBLE
template<> SOFA_BASE_MECHANICS_API
void MechanicalObject<defaulttype::Rigid3fTypes>::draw(const core::visual::VisualParams* vparams);
#endif



#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_COMPONENT_CONTAINER_MECHANICALOBJECT_CPP)
#ifndef SOFA_FLOAT
extern template class SOFA_BASE_MECHANICS_API MechanicalObject<defaulttype::Vec3dTypes>;
extern template class SOFA_BASE_MECHANICS_API MechanicalObject<defaulttype::Vec2dTypes>;
extern template class SOFA_BASE_MECHANICS_API MechanicalObject<defaulttype::Vec1dTypes>;
extern template class SOFA_BASE_MECHANICS_API MechanicalObject<defaulttype::Vec6dTypes>;
extern template class SOFA_BASE_MECHANICS_API MechanicalObject<defaulttype::Rigid3dTypes>;
extern template class SOFA_BASE_MECHANICS_API MechanicalObject<defaulttype::Rigid2dTypes>;
#endif
#ifndef SOFA_DOUBLE
extern template class SOFA_BASE_MECHANICS_API MechanicalObject<defaulttype::Vec3fTypes>;
extern template class SOFA_BASE_MECHANICS_API MechanicalObject<defaulttype::Vec2fTypes>;
extern template class SOFA_BASE_MECHANICS_API MechanicalObject<defaulttype::Vec1fTypes>;
extern template class SOFA_BASE_MECHANICS_API MechanicalObject<defaulttype::Vec6fTypes>;
extern template class SOFA_BASE_MECHANICS_API MechanicalObject<defaulttype::Rigid3fTypes>;
extern template class SOFA_BASE_MECHANICS_API MechanicalObject<defaulttype::Rigid2fTypes>;
#endif
#endif

} // namespace container

} // namespace component

} // namespace sofa

#endif
