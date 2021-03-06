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

#include "STLExporter.h"

#include <sstream>
#include <string>

#include <sofa/helper/io/File.h>
#include <sofa/core/ObjectFactory.h>

#include <sofa/core/objectmodel/Event.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/core/objectmodel/KeyreleasedEvent.h>


namespace sofa
{

namespace component
{

namespace misc
{

SOFA_DECL_CLASS(STLExporter)

int STLExporterClass = core::RegisterObject("Read State vectors from file at each timestep")
        .add< STLExporter >();

STLExporter::STLExporter()
    : stepCounter(0)
    , maxStep(0)
    , stlFilename( initData(&stlFilename, "filename", "output STL file name"))
    , d_binaryFormat( initData(&d_binaryFormat, true, "binaryformat", "if true, save in binary format, otherwise in ascii"))
    , m_position( initData(&m_position, "position", "points coordinates"))
    , m_triangle( initData(&m_triangle, "triangle", "triangles indices"))
    , m_quad( initData(&m_quad, "quad", "quads indices"))
    , exportEveryNbSteps( initData(&exportEveryNbSteps, (unsigned int)0, "exportEveryNumberOfSteps", "export file only at specified number of steps (0=disable)"))
    , exportAtBegin( initData(&exportAtBegin, false, "exportAtBegin", "export file at the initialization"))
    , exportAtEnd( initData(&exportAtEnd, false, "exportAtEnd", "export file when the simulation is finished"))
    , set_extension( initData(&set_extension, true, "set_extension", "automatically add/set extension to .stl"))
{
        this->addAlias(&m_triangle, "triangles");
        this->addAlias(&m_quad, "quads");
}

STLExporter::~STLExporter()
{

}

void STLExporter::init()
{
    sofa::core::objectmodel::BaseContext* context = this->getContext();

    context->get(topology, sofa::core::objectmodel::BaseContext::Local);
    context->get(mstate, sofa::core::objectmodel::BaseContext::Local);
    context->get(vmodel, sofa::core::objectmodel::BaseContext::Local);

    // Test if the position has not been modified
    if(!m_position.isSet())
    {
        sofa::core::objectmodel::BaseData* pos = NULL;
        sofa::core::objectmodel::BaseData* tri = NULL;
        sofa::core::objectmodel::BaseData* qua = NULL;
        if(vmodel)
        {
            pos = vmodel->findData("position");
            tri = vmodel->findData("triangles");
            qua = vmodel->findData("quads");
        }
        else if(topology)
        {
            pos = topology->findData("position");
            tri = topology->findData("triangles");
            qua = topology->findData("quads");
        }
        else
        {
            serr << "STLExporter : error, STLExporter needs VisualModel or Topology" << sendl;
            return;
        }

        m_position.setParent(pos);
        m_triangle.setParent(tri);
        m_quad.setParent(qua);
    }

    // Activate the listening to the event in order to be able to export file at the nth-step
    if(exportEveryNbSteps.getValue() != 0)
        this->f_listening.setValue(true);

    nbFiles = 0;

}

std::string STLExporter::getFilename() const {
    std::string filename = stlFilename.getFullPath();

    if( set_extension.getValue() ) {
        // mtournier: why???
        if (maxStep) {
            // if extension given, remove it
            if (".stl" == filename.substr(filename.size()-4, 4)) {
                filename = filename.substr(0, filename.size()-4);
            }
            std::ostringstream oss;
            oss.width(5);
            oss.fill('0');
            oss << nbFiles;
            filename += oss.str();
            filename += ".stl";
        } else {
            // add the file extension if necessary
            if (".stl" != filename.substr(filename.size()-4, 4)) {
                filename += ".stl";
            }
        }
    }
    
    return filename;
}

void STLExporter::writeSTL()
{
    std::string filename = getFilename();

    sofa::helper::io::File outFile;
    if(!outFile.open(filename.c_str(), std::ios_base::out))
    {
        serr << "Error creating file " << filename << sendl;
        return;
    }
    std::ostream outStream(outFile.streambuf());
    
    helper::ReadAccessor< Data< helper::vector< core::topology::BaseMeshTopology::Triangle > > > triangleIndices = m_triangle;
    helper::ReadAccessor< Data< helper::vector< core::topology::BaseMeshTopology::Quad > > > quadIndices = m_quad;
    helper::ReadAccessor<Data<defaulttype::Vec3Types::VecCoord> > positionIndices = m_position;

    helper::vector< core::topology::BaseMeshTopology::Triangle > vecTri;

    if(positionIndices.empty())
    {
        serr << "STLExporter::writeSTL : error, no positions in topology" << sendl;
        return;
    }
    if(!triangleIndices.empty())
    {
        for(unsigned int i=0;i<triangleIndices.size();i++)
        {
            vecTri.push_back(triangleIndices[i]);
        }
    }
    else if(!quadIndices.empty())
    {
        core::topology::BaseMeshTopology::Triangle tri;
        for(unsigned int i=0;i<quadIndices.size();i++)
        {
            for(int j=0;j<3;j++)
            {
                tri[j] = quadIndices[i][j];
            }
            vecTri.push_back(tri);
            tri[0] = quadIndices[i][0];
            tri[1] = quadIndices[i][2];
            tri[2] = quadIndices[i][3];
            vecTri.push_back(tri);
        }
    }
    else
    {
        serr << "STLExporter::writeSTLBinary : error, neither triangles nor quads" << sendl;
        return;
    }

    /* Get number of facets */
    const int nbt = vecTri.size();

    // Sets the floatfield format flag for the str stream to fixed
    std::cout.precision(6);

    /* solid */
    outStream << "solid Exported from Sofa" << std::endl;
    
    
    for(int i=0;i<nbt;i++)
    {
        /* normal */
        outStream << "facet normal 0 0 0" << std::endl;
        outStream << "outer loop" << std::endl;
        for (int j=0;j<3;j++)
        {
            /* vertices */
            outStream << "vertex " << std::fixed << positionIndices[ vecTri[i][j] ] << std::endl;
        }
        outStream << "endloop" << std::endl;
        outStream << "endfacet" << std::endl;
    }

    /* endsolid */

    outStream << "endsolid Exported from Sofa" << std::endl;
    
    msg_info("STLExporter") << filename << " written" ;

    nbFiles++;
}

void STLExporter::writeSTLBinary()
{
    std::string filename = getFilename();

    sofa::helper::io::File outFile;
    if(!outFile.open(filename.c_str(), std::ios_base::out | std::ios::binary))
    {
        serr << "Error creating file " << filename << sendl;
        return;
    }
    std::ostream outStream(outFile.streambuf());

    helper::ReadAccessor< Data< helper::vector< core::topology::BaseMeshTopology::Triangle > > > triangleIndices = m_triangle;
    helper::ReadAccessor< Data< helper::vector< core::topology::BaseMeshTopology::Quad > > > quadIndices = m_quad;
    helper::ReadAccessor<Data<defaulttype::Vec3Types::VecCoord> > positionIndices = m_position;

    helper::vector< core::topology::BaseMeshTopology::Triangle > vecTri;

    if(positionIndices.empty())
    {
        serr << "STLExporter::writeSTLBinary : error, no positions in topology" << sendl;
        return;
    }
    if(!triangleIndices.empty())
    {
        for(unsigned int i=0;i<triangleIndices.size();i++)
        {
            vecTri.push_back(triangleIndices[i]);
        }
    }
    if(!quadIndices.empty())
    {
        core::topology::BaseMeshTopology::Triangle tri;
        for(unsigned int i=0;i<quadIndices.size();i++)
        {
            for(int j=0;j<3;j++)
            {
                tri[j] = quadIndices[i][j];
            }
            vecTri.push_back(tri);
            tri[0] = quadIndices[i][0];
            tri[1] = quadIndices[i][2];
            tri[2] = quadIndices[i][3];
            vecTri.push_back(tri);
        }
    }
    if(quadIndices.empty() && triangleIndices.empty())
    {
        serr << "STLExporter::writeSTLBinary : error, neither triangles nor quads" << sendl;
        return;
    }

    // Sets the floatfield format flag for the str stream to fixed
    std::cout.precision(6);

    /* Creating header file */
    char* buffer = new char [80];
    // Cleaning buffer
    for(int i=0;i<80;i++)
    {
        buffer[i]='\0';
    }
    strcpy(buffer, "Exported from Sofa");
    outStream.write(buffer,80);

    delete [] buffer;

    /* Number of facets */
    const unsigned int nbt = vecTri.size();
    outStream.write((char*)&nbt,4);
    
    // Parsing facets
    for(unsigned long i=0;i<nbt;i++)
    {
        /* normal */
        float nul = 0.; // normals are set to 0
        outStream.write((char*)&nul, 4);
        outStream.write((char*)&nul, 4);
        outStream.write((char*)&nul, 4);
        for (int j=0;j<3;j++)
        {
            /* vertices */
            float iOne = (float)positionIndices[ vecTri[i][j] ][0];
            float iTwo = (float)positionIndices[ vecTri[i][j] ][1];
            float iThree = (float)positionIndices[ vecTri[i][j] ][2];
            outStream.write( (char*)&iOne, 4);
            outStream.write( (char*)&iTwo, 4);
            outStream.write( (char*)&iThree, 4);
        }

        /* Attribute byte count */
        // attribute count is currently not used, it's garbage
        unsigned int zero = 0;
        outStream.write((char*)&zero, 2);
    }

    msg_info() << filename << " written." ;

    nbFiles++;
}

void STLExporter::handleEvent(sofa::core::objectmodel::Event *event)
{
    if (sofa::core::objectmodel::KeypressedEvent::checkEventType(event))
    {
        sofa::core::objectmodel::KeypressedEvent *ev = static_cast<sofa::core::objectmodel::KeypressedEvent *>(event);
        switch(ev->getKey())
        {

        case 'E':
        case 'e':
            if(d_binaryFormat.getValue())
                writeSTLBinary();
            else
                writeSTL();
            break;

        case 'F':
        case 'f':
            break;
        }
    }


    if ( /*simulation::AnimateEndEvent* ev =*/ simulation::AnimateEndEvent::checkEventType(event))
    {
        maxStep = exportEveryNbSteps.getValue();
        if (maxStep == 0) return;

        stepCounter++;
        if(stepCounter % maxStep == 0)
        {
            if(d_binaryFormat.getValue())
                writeSTLBinary();
            else
                writeSTL();
        }
    }


    if (sofa::core::objectmodel::GUIEvent::checkEventType(event))
    {
        maxStep = 0; // to keep the name of the file set as input

        sofa::core::objectmodel::GUIEvent *guiEvent = static_cast<sofa::core::objectmodel::GUIEvent *>(event);

        if (guiEvent->getValueName().compare("STLExport") == 0)
        {
            if(d_binaryFormat.getValue())
                writeSTLBinary();
            else
                writeSTL();
        }
    }
}

void STLExporter::cleanup()
{
    if (exportAtEnd.getValue())
        (d_binaryFormat.getValue()) ? writeSTLBinary() : writeSTL();

}

void STLExporter::bwdInit()
{
    if (exportAtBegin.getValue())
        (d_binaryFormat.getValue()) ? writeSTLBinary() : writeSTL();
}

} // namespace misc

} // namespace component

} // namespace sofa

