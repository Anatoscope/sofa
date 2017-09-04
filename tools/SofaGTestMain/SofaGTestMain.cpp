#include <sofa/helper/Utils.h>
#include <sofa/helper/system/FileRepository.h>
#include <sofa/helper/system/FileSystem.h>
#include <sofa/helper/system/PluginManager.h>
#include <sofa/simulation/config.h> // #defines SOFA_HAVE_DAG (or not)
#ifdef SOFA_HAVE_DAG
#  include <SofaSimulationGraph/init.h>
#endif
#include <SofaSimulationTree/init.h>

#include <gtest/gtest.h>

using sofa::helper::system::PluginManager;
using sofa::helper::system::PluginRepository;
using sofa::helper::system::DataRepository;
using sofa::helper::system::FileSystem;
using sofa::helper::Utils;

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);

    sofa::simulation::tree::init();
#ifdef SOFA_HAVE_DAG
    sofa::simulation::graph::init();
#endif

    // do not try to load plugin's gui libraries
    PluginManager::s_gui_postfix = "";

    PluginRepository.addFirstPath(Utils::getPluginDirectory());

    // default DataRepository
    // Read the paths to the share/ directory from etc/sofa.ini,
    const std::string etcDir = Utils::getSofaPathPrefix() + "/etc";
    const std::string sofaIniFilePath = etcDir + "/sofa.ini";
    std::map<std::string, std::string> iniFileValues = Utils::readBasicIniFile(sofaIniFilePath);
    // and add them to DataRepository
    if (iniFileValues.find("SHARE_DIR") != iniFileValues.end())
    {
        std::string shareDir = iniFileValues["SHARE_DIR"];
        if (!FileSystem::isAbsolute(shareDir))
            shareDir = etcDir + "/" + shareDir;
        DataRepository.addFirstPath(shareDir);
    }


    int ret =  RUN_ALL_TESTS();

#ifdef SOFA_HAVE_DAG
    sofa::simulation::graph::cleanup();
#endif
    sofa::simulation::tree::cleanup();

    return ret;
}
