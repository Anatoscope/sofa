#include <SofaPython/PythonScriptEvent.h>

#include "Python_test.h"

#include <sofa/helper/system/PluginManager.h>

#include <sofa/simulation/Simulation.h>
#include <SofaSimulationGraph/DAGSimulation.h>

#include <sofa/helper/logging/Messaging.h>
#include <sofa/helper/system/FileSystem.h>

namespace sofa {



Python_test::Python_test()
{
    static const std::string plugin = "SofaPython";
    sofa::helper::system::PluginManager::getInstance().loadPlugin(plugin);
}



void Python_test::run( const Python_test_data& data ) {

    msg_info("Python_test") << "running " << data.filepath;

    {
        // Check the file exists
        std::ifstream file(data.filepath.c_str());
        bool scriptFound = file.good();
        ASSERT_TRUE(scriptFound);
    }

    ASSERT_TRUE( loader.loadTestWithArguments(data.filepath.c_str(),data.arguments) );

}


static bool ends_with(const std::string& suffix, const std::string& full){
    const std::size_t lf = full.length();
    const std::size_t ls = suffix.length();
    
    if(lf < ls) return false;
    
    return (0 == full.compare(lf - ls, ls, suffix));
}

static bool starts_with(const std::string& prefix, const std::string& full){
    const std::size_t lf = full.length();
    const std::size_t lp = prefix.length();
    
    if(lf < lp) return false;
    
    return (0 == full.compare(0, lp, prefix));
}




void Python_test_list::addTestDir(const std::string& dir, const std::string& prefix) {

    std::vector<std::string> files;
    helper::system::FileSystem::listDirectory(dir, files);
    
    for(const std::string& file : files) {
        if( starts_with(prefix, file) && ends_with(".py", file) ) {
            addTest(file, dir);
        }
    }
    
}



////////////////////////////////////////////////////
////////////////////////////////////////////////////
////////////////////////////////////////////////////


struct Listener : core::objectmodel::BaseObject {

    Listener() {
        f_listening = true;
    }

    virtual void handleEvent(core::objectmodel::Event * event) {
        if (core::objectmodel::PythonScriptEvent::checkEventType(event)
              || core::objectmodel::ScriptEvent::checkEventType(event) )
       {
            core::objectmodel::ScriptEvent* e = static_cast<core::objectmodel::ScriptEvent*>(event);
            std::string name = e->getEventName();
            if( name == "success" ) {
                throw Python_scene_test::result(true);
            } else if (name == "failure") {
                throw Python_scene_test::result(false);
            }
        }
    }

};


static simulation::Node::SPtr root;

void Python_scene_test::run( const Python_test_data& data ) {

    msg_info("Python_scene_test") << "running "<< data.filepath;

    {
        // Check the file exists
        std::ifstream file(data.filepath.c_str());
        bool scriptFound = file.good();
        ASSERT_TRUE(scriptFound);
    }

    if( !simulation::getSimulation() ) {
        simulation::setSimulation( new sofa::simulation::graph::DAGSimulation() );
    }
    
    loader.loadSceneWithArguments(data.filepath.c_str(),
                                  data.arguments,
                                  &root);

    ASSERT_TRUE(bool(root)) << "scene creation failed!";
    
	root->addObject( new Listener );

	simulation::getSimulation()->init(root.get());

	try {
		while(root->isActive()) {
			simulation::getSimulation()->animate(root.get(), root->getDt());
		}
	} catch( const result& test_result ) {
        ASSERT_TRUE(test_result.value);
        simulation::getSimulation()->unload( root.get() );
	}
}



} // namespace sofa


extern "C" {

    void finish() {
        if(sofa::root) {
            sofa::root->setActive(false);
        }
    }

    void expect_true(bool test, const char* msg) {
	EXPECT_TRUE(test) << msg;
    }
    
    void assert_true(bool test, const char* msg) {
        auto trigger = [&] {
            ASSERT_TRUE(test) << msg;
        };

        trigger();
        if(!test) finish();
    }
    

}
