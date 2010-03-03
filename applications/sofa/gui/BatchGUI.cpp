/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 4      *
*                (c) 2006-2009 MGH, INRIA, USTL, UJF, CNRS                    *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU General Public License as published by the Free  *
* Software Foundation; either version 2 of the License, or (at your option)   *
* any later version.                                                          *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for    *
* more details.                                                               *
*                                                                             *
* You should have received a copy of the GNU General Public License along     *
* with this program; if not, write to the Free Software Foundation, Inc., 51  *
* Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.                   *
*******************************************************************************
*                            SOFA :: Applications                             *
*                                                                             *
* Authors: M. Adam, J. Allard, B. Andre, P-J. Bensoussan, S. Cotin, C. Duriez,*
* H. Delingette, F. Falipou, F. Faure, S. Fonteneau, L. Heigeas, C. Mendoza,  *
* M. Nesme, P. Neumann, J-P. de la Plata Alcade, F. Poyer and F. Roy          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include "BatchGUI.h"
#include <sofa/simulation/common/Simulation.h>
#include <sofa/helper/system/thread/CTime.h>
#include <iostream>
#include <sstream>

namespace sofa
{

namespace gui
{

const unsigned int BatchGUI::DEFAULT_NUMBER_OF_ITERATIONS = 1000;
unsigned int BatchGUI::nbIter = BatchGUI::DEFAULT_NUMBER_OF_ITERATIONS;

BatchGUI::BatchGUI()
    : groot(NULL)
{
}

BatchGUI::~BatchGUI()
{
}

int BatchGUI::mainLoop()
{
    if (groot)
    {

        sofa::simulation::getSimulation()->animate(groot);
        //As no visualization is done by the Batch GUI, these two lines are not necessary.
        sofa::simulation::getSimulation()->updateVisual(groot);
        sofa::simulation::getSimulation()->updateVisual(simulation::getSimulation()->getVisualRoot());

        std::cout << "Computing "<<nbIter<<" iterations." << std::endl;
        sofa::simulation::Node::ctime_t rtfreq = sofa::helper::system::thread::CTime::getRefTicksPerSec();
        sofa::simulation::Node::ctime_t tfreq = sofa::helper::system::thread::CTime::getTicksPerSec();
        sofa::simulation::Node::ctime_t rt = sofa::helper::system::thread::CTime::getRefTime();
        sofa::simulation::Node::ctime_t t = sofa::helper::system::thread::CTime::getFastTime();
        for (unsigned int i=0; i<nbIter; i++)
        {
            sofa::simulation::getSimulation()->animate(groot);
            //As no visualization is done by the Batch GUI, these two lines are not necessary.
            sofa::simulation::getSimulation()->updateVisual(groot);
            sofa::simulation::getSimulation()->updateVisual(simulation::getSimulation()->getVisualRoot());
        }
        t = sofa::helper::system::thread::CTime::getFastTime()-t;
        rt = sofa::helper::system::thread::CTime::getRefTime()-rt;

        std::cout << nbIter << " iterations done in "<< ((double)t)/((double)tfreq) << " s ( " << (((double)tfreq)*nbIter)/((double)t) << " FPS)." << std::endl;
        std::cout << nbIter << " iterations done in "<< ((double)rt)/((double)rtfreq) << " s ( " << (((double)rtfreq)*nbIter)/((double)rt) << " FPS)." << std::endl;
    }
    return 0;
}

void BatchGUI::redraw()
{
}

int BatchGUI::closeGUI()
{
    delete this;
    return 0;
}

void BatchGUI::setScene(sofa::simulation::Node* groot, const char* filename, bool )
{
    this->groot = groot;
    this->filename = (filename?filename:"");
}

sofa::simulation::Node* BatchGUI::currentSimulation()
{
    return groot;
}


int BatchGUI::InitGUI(const char* /*name*/, const std::vector<std::string>& options)
{
    setNumIterations(DEFAULT_NUMBER_OF_ITERATIONS);

    //parse options
    for (unsigned int i=0 ; i<options.size() ; i++)
    {
        size_t cursor = 0;
        std::string opt = options[i];
        //Set number of iterations
        //(option = "nbIterations=N where N is the number of iterations)
        if ( (cursor = opt.find("nbIterations=")) != std::string::npos )
        {
            unsigned int nbIterations;
            std::istringstream iss;
            iss.str(opt.substr(cursor+std::string("nbIterations=").length(), std::string::npos));
            iss >> nbIterations;
            setNumIterations(nbIterations);
        }
    }
    return 0;
}

SofaGUI* BatchGUI::CreateGUI(const char* name, const std::vector<std::string>& /*options*/, sofa::simulation::Node* groot, const char* filename)
{
    BatchGUI::guiName = name;
    BatchGUI* gui = new BatchGUI();
    gui->setScene(groot, filename);
    return gui;
}

} // namespace gui

} // namespace sofa
