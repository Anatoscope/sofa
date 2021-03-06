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
#ifndef SOFA_COMPONENT_ENGINE_MeshSplittingEngine_H
#define SOFA_COMPONENT_ENGINE_MeshSplittingEngine_H
#include "config.h"

#include <map>

#include <sofa/core/DataEngine.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/defaulttype/Vec3Types.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/helper/vectorData.h>
#include <array>

namespace sofa
{

namespace component
{

namespace engine
{

/**
 * This class breaks a mesh in multiple parts, based on selected vertices or cells.
 * It provide a map to be used in subsetMultiMapping, and vertex positions for each parts.
 */
template <class DataTypes>
class MeshSplittingEngine : public core::DataEngine
{
public:
    typedef core::DataEngine Inherited;
    SOFA_CLASS(SOFA_TEMPLATE(MeshSplittingEngine,DataTypes),Inherited);

    typedef typename DataTypes::Real Real;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef VecCoord SeqPositions;
    typedef typename core::topology::BaseMeshTopology::index_type index_type;
    typedef typename core::topology::BaseMeshTopology::Edge Edge;
    typedef typename core::topology::BaseMeshTopology::SeqEdges SeqEdges;
    typedef typename core::topology::BaseMeshTopology::Triangle Triangle;
    typedef typename core::topology::BaseMeshTopology::SeqTriangles SeqTriangles;
    typedef typename core::topology::BaseMeshTopology::Quad Quad;
    typedef typename core::topology::BaseMeshTopology::SeqQuads SeqQuads;
    typedef typename core::topology::BaseMeshTopology::Tetrahedron Tetrahedron;
    typedef typename core::topology::BaseMeshTopology::SeqTetrahedra SeqTetrahedra;
    typedef typename core::topology::BaseMeshTopology::Hexahedron Hexahedron;
    typedef typename core::topology::BaseMeshTopology::SeqHexahedra SeqHexahedra;
    typedef typename core::topology::BaseMeshTopology::PointID PointID;
    typedef typename core::topology::BaseMeshTopology::SetIndices SetIndices;

    typedef std::map<index_type, index_type> IndexToIndexMap;

    /// inputs
    Data< SeqPositions > inputPosition;
    Data< SeqEdges > inputEdges;
    Data< SeqTriangles > inputTriangles;
    Data< SeqQuads > inputQuads;
    Data< SeqTetrahedra > inputTets;
    Data< SeqHexahedra > inputHexa;
    Data<unsigned int> nbInputs;
    helper::vectorData<SetIndices> indices;
    helper::vectorData<SetIndices> edgeIndices;
    helper::vectorData<SetIndices> triangleIndices;
    helper::vectorData<SetIndices> quadIndices;
    helper::vectorData<SetIndices> tetrahedronIndices;
    helper::vectorData<SetIndices> hexahedronIndices;
    Data< bool > doOutputTopology;

    /// outputs
    Data< helper::vector<index_type> > indexPairs;
    helper::vectorData<SeqPositions> position;
    helper::vectorData<SeqEdges> edges;
    helper::vectorData<SeqTriangles> triangles;
    helper::vectorData<SeqQuads> quads;

    virtual std::string getTemplateName() const    { return templateName(this);    }
    static std::string templateName(const MeshSplittingEngine<DataTypes>* = NULL) { return DataTypes::Name();    }

protected:

    MeshSplittingEngine()    : Inherited()
      , inputPosition(initData(&inputPosition,"position","input vertices"))
      , inputEdges(initData(&inputEdges,"edges","input edges"))
      , inputTriangles(initData(&inputTriangles,"triangles","input triangles"))
      , inputQuads(initData(&inputQuads,"quads","input quads"))
      , inputTets(initData(&inputTets,"tetrahedra","input tetrahedra"))
      , inputHexa(initData(&inputHexa,"hexahedra","input hexahedra"))
      , nbInputs (initData(&nbInputs, (unsigned)0, "nbInputs", "Number of input vectors"))
      , indices(this, "indices", "input vertex indices", helper::DataEngineInput)
      , edgeIndices(this, "edgeIndices", "input edge indices", helper::DataEngineInput)
      , triangleIndices(this, "triangleIndices", "input triangle indices", helper::DataEngineInput)
      , quadIndices(this, "quadIndices", "input quad indices", helper::DataEngineInput)
      , tetrahedronIndices(this, "tetrahedronIndices", "input tetrahedron indices", helper::DataEngineInput)
      , hexahedronIndices(this, "hexahedronIndices", "input hexahedron indices", helper::DataEngineInput)
      , doOutputTopology(initData(&doOutputTopology, false, "doOutputTopologies", "output or not topologies for each mesh"))
      , indexPairs( initData( &indexPairs, helper::vector<unsigned>(), "indexPairs", "couples for input vertices: ROI index + index in the ROI"))
      , position(this, "position", "output vertices", helper::DataEngineOutput)
      , edges(this, "edges", "output edges", helper::DataEngineOutput)
      , triangles(this, "triangles", "output triangles", helper::DataEngineOutput)
      , quads(this, "quads", "output quads", helper::DataEngineOutput)
    {
        resizeData();
    }

    virtual ~MeshSplittingEngine()
    {

    }

    template <class Shape>
    void fillTopology(helper::vector<Shape>& output,
                      helper::vector<Shape> const& input, IndexToIndexMap const& indexMap)
    {
        std::array<IndexToIndexMap::const_iterator,Shape::static_size> shapeIndices;
        bool shapeInSubMesh;
        for (Shape const& shape: input) {
            shapeInSubMesh=true;
            for (index_type i=0; i<Shape::static_size; ++i) {
                shapeIndices[i] = indexMap.find(shape[i]);
                if (indexMap.end() == shapeIndices[i]) {
                    shapeInSubMesh=false;
                    break;
                }
            }
            if (!shapeInSubMesh)
                continue;
            output.push_back(Shape());
            for (index_type i=0; i<Shape::static_size; ++i)
                output.back()[i] = shapeIndices[i]->second;
        }
    }


public:
    virtual void init()
    {
        addInput(&inputPosition);
        addInput(&inputEdges);
        addInput(&inputTriangles);
        addInput(&inputQuads);
        addInput(&inputTets);
        addInput(&inputHexa);
        addInput(&nbInputs);
        addOutput(&indexPairs);
        addOutput(&doOutputTopology);
        resizeData();

        setDirtyValue();
    }

    virtual void reinit()    { resizeData(); update();  }

    /// Parse the given description to assign values to this object's fields and potentially other parameters
    void parse ( sofa::core::objectmodel::BaseObjectDescription* arg )
    {
        const char* p = arg->getAttribute(nbInputs.getName().c_str());
        if (p) {
            std::string nbStr = p;
            nbInputs.read(nbStr);
            resizeData();
        }
        Inherit1::parse(arg);
    }

    /// Assign the field values stored in the given map of name -> value pairs
    void parseFields ( const std::map<std::string,std::string*>& str )
    {
        std::map<std::string,std::string*>::const_iterator it = str.find(nbInputs.getName());
        if (it != str.end() && it->second)
        {
            std::string nbStr = *it->second;
            nbInputs.read(nbStr);
            resizeData();
        }
    }


    void update();

protected:
    void resizeData()
    {
        indices.resize(nbInputs.getValue());
        edgeIndices.resize(nbInputs.getValue());
        triangleIndices.resize(nbInputs.getValue());
        quadIndices.resize(nbInputs.getValue());
        tetrahedronIndices.resize(nbInputs.getValue());
        hexahedronIndices.resize(nbInputs.getValue());
        position.resize(nbInputs.getValue()+1); // one more to store the remaining sub mesh
        if (doOutputTopology.getValue()) {
            edges.resize(nbInputs.getValue()+1);
            triangles.resize(nbInputs.getValue()+1);
            quads.resize(nbInputs.getValue()+1);
        }
    }

};

#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_COMPONENT_ENGINE_MeshSplittingEngine_CPP)
#ifndef SOFA_FLOAT
extern template class SOFA_GENERAL_ENGINE_API MeshSplittingEngine<defaulttype::Vec3dTypes>;
#endif //SOFA_FLOAT
#ifndef SOFA_DOUBLE
extern template class SOFA_GENERAL_ENGINE_API MeshSplittingEngine<defaulttype::Vec3fTypes>;
#endif //SOFA_DOUBLE
#endif

} // namespace engine

} // namespace component

} // namespace sofa

#endif
