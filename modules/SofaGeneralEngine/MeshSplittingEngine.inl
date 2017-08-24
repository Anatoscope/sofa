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
#ifndef SOFA_COMPONENT_ENGINE_MeshSplittingEngine_INL
#define SOFA_COMPONENT_ENGINE_MeshSplittingEngine_INL

#include "MeshSplittingEngine.h"

namespace sofa
{

namespace component
{

namespace engine
{

template <class container>
inline void parseIndices(helper::vector<unsigned int>& pairs, const container& indices, const unsigned int parentIndex)
{
    for(size_t i=0;i<indices.size();++i)
        if(2*indices[i]<pairs.size())
            pairs[2*indices[i]]=parentIndex;
}

template <class container1,class container2>
inline void parseIndices(helper::vector<unsigned int>& pairs, const container1& indices,const container2& cells, const unsigned int parentIndex)
{
    for(size_t i=0;i<indices.size();++i)
        for(size_t j=0;j<cells[indices[i]].size();++j)
            if(2*cells[indices[i]][j]<pairs.size())
                pairs[2*cells[indices[i]][j]]=parentIndex;
}


template <class DataTypes>
void MeshSplittingEngine<DataTypes>::update()
{
    updateAllInputsIfDirty();

    SeqEdges const& inEdges = inputEdges.getValue();
    SeqTriangles const& inTriangles = inputTriangles.getValue();
    SeqQuads const& inQuads = inputQuads.getValue();
    SeqTetrahedra const& inTets = inputTets.getValue();
    SeqHexahedra const& inHexa = inputHexa.getValue();

    cleanDirty();

    helper::ReadAccessor<Data< SeqPositions > > i_pos(this->inputPosition);
    const size_t& nb = nbInputs.getValue();

    helper::WriteOnlyAccessor<Data< helper::vector<unsigned int> > > indPairs(this->indexPairs);
    indPairs.resize(2*i_pos.size());
    for(size_t i=0;i<i_pos.size();++i) indPairs[2*i]=nb; // assign to remaining sub mesh

    // get parent of each child
    for(int i=nb-1;i>=0;--i) // use reverse order to prioritize the first ROIs
    {
        parseIndices(indPairs.wref(), indices[i]->getValue(), i);
        parseIndices(indPairs.wref(), edgeIndices[i]->getValue(), inEdges, i);
        parseIndices(indPairs.wref(), triangleIndices[i]->getValue(), inTriangles, i);
        parseIndices(indPairs.wref(), quadIndices[i]->getValue(), inQuads, i);
        parseIndices(indPairs.wref(), tetrahedronIndices[i]->getValue(), inTets, i);
        parseIndices(indPairs.wref(), hexahedronIndices[i]->getValue(), inHexa, i);
    }

    // add positions
    std::vector<SeqPositions*> o_pos(nb+1);
    for(size_t i=0;i<nb+1;++i) { o_pos[i]=position[i]->beginWriteOnly(); o_pos[i]->clear(); }

    for(size_t i=0;i<i_pos.size();++i)
    {
        indPairs[2*i+1]=o_pos[indPairs[2*i]]->size();
        o_pos[indPairs[2*i]]->push_back(i_pos[i]);
    }
    for(size_t i=0;i<nb+1;++i) position[i]->endEdit();

    if (doOutputTopology.getValue()) {
        // add topology

        // map to convert from mesh indices to sub mesh indices
        std::vector<IndexToIndexMap> meshToSub(nb+1);
        for (std::size_t i=0; i_pos.size();++i)
            meshToSub[indPairs[2*i]][i] = indPairs[2*i+1];

        std::vector<SeqEdges*> o_edges(nb+1);
        std::vector<SeqTriangles*> o_triangles(nb+1);
        std::vector<SeqQuads*> o_quads(nb+1);
        for(size_t i=0;i<nb+1;++i) {
            o_edges[i]=edges[i]->beginWriteOnly();
            o_edges[i]->clear();
            o_triangles[i]=triangles[i]->beginWriteOnly();
            o_triangles[i]->clear();
            o_quads[i]=quads[i]->beginWriteOnly();
            o_quads[i]->clear();
        }

        for(size_t i=0;i<nb+1;++i) {
            fillTopology(*(o_edges[i]), inEdges, meshToSub[i]);
//            fillTopology(o_edges[i], inputEdges, meshToSub[i]);
        }

        for(size_t i=0;i<nb+1;++i) {
            edges[i]->endEdit();
            triangles[i]->endEdit();
            quads[i]->endEdit();
        }
    }

    msg_info() <<this->name<<":"<<"updated" ;
}


} // namespace engine

} // namespace component

} // namespace sofa

#endif
