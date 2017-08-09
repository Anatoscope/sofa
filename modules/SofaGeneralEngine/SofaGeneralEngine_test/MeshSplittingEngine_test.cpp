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

#include <SofaTest/DataEngine_test.h>

#include <SofaGeneralEngine/MeshSplittingEngine.h>

namespace sofa {

typedef sofa::component::engine::MeshSplittingEngine<defaulttype::Vec3Types> MeshSplittingEngineT;

class MeshSplittingEngine_test : public DataEngine_test<MeshSplittingEngineT>
{
public:
    void run_output_topology_test() {
        init();

        // set inputs
        m_engineInput->inputPosition.setValue({{0,0,0}, {1,0,0}, {0,1,0}, {1,1,0}});
        m_engineInput->inputTriangles.setValue({{0,1,2},{1,3,2}});
        m_engineInput->inputQuads.setValue({{0,1,2,3}});
        m_engineInput->nbInputs.setValue(1);
        m_engineInput->indices[0]->setValue({1,2,3});

        // test output

        ASSERT_EQ((std::size_t)2, m_engine->position.size());
        ASSERT_EQ((std::size_t)2, m_engine->triangles.size());
        ASSERT_EQ((std::size_t)2, m_engine->quads.size());

        {
            // the sub mesh
            typename MeshSplittingEngineT::VecCoord const& positions = m_engine->position[0]->getValue();
            ASSERT_EQ((std::size_t)3, positions.size());
            EXPECT_VEC_DOUBLE_EQ(typename MeshSplittingEngineT::Coord({{1,0,0}}), positions[0]);
            EXPECT_VEC_DOUBLE_EQ(typename MeshSplittingEngineT::Coord({{0,1,0}}), positions[1]);
            EXPECT_VEC_DOUBLE_EQ(typename MeshSplittingEngineT::Coord({{1,1,0}}), positions[2]);

            typename MeshSplittingEngineT::SeqTriangles const& triangles = m_engine->triangles[0]->getValue();
            ASSERT_EQ((std::size_t)1, triangles.size());
            EXPECT_VEC_EQ(typename MeshSplittingEngineT::Triangle({0,2,1}), triangles[0]);

            typename MeshSplittingEngineT::SeqQuads const& quads = m_engine->quads[0]->getValue();
            ASSERT_EQ((std::size_t)0, quads.size());
        }

        {
            // the remaining mesh
            typename MeshSplittingEngineT::VecCoord const& positions = m_engine->position[1]->getValue();
            ASSERT_EQ((std::size_t)1, positions.size());
            EXPECT_VEC_DOUBLE_EQ(typename MeshSplittingEngineT::Coord({{0,0,0}}), positions[0]);

            typename MeshSplittingEngineT::SeqTriangles const& triangles = m_engine->triangles[1]->getValue();
            ASSERT_EQ((std::size_t)0, triangles.size());

            typename MeshSplittingEngineT::SeqQuads const& quads = m_engine->quads[1]->getValue();
            ASSERT_EQ((std::size_t)0, quads.size());
        }
    }
};

TEST_F(MeshSplittingEngine_test, output_topology_test)
{
//    this->run_output_topology_test();
}

}

