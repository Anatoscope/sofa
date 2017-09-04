#include <SofaPython/PythonScriptEvent.h>

#include "Python_test.h"

#include <sofa/helper/system/PluginManager.h>

#include <sofa/simulation/Simulation.h>
#include <SofaSimulationGraph/DAGSimulation.h>

#include <sofa/helper/logging/Messaging.h>
#include <sofa/helper/system/FileSystem.h>

#include <SofaPython/PythonMacros.h>
#include <SofaPython/PythonFactory.h>

#include <SofaPython/helper.hpp>

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
        ASSERT_TRUE(scriptFound) << " ("<<data.filepath<<")";
    }

    ASSERT_TRUE( loader.loadTestWithArguments(data.filepath.c_str(),data.arguments) ) << " ("<<data.filepath<<")";

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



struct fail {
    const char* message;
    fail(const char* message)
        : message(message) { }
};

static PyObject* operator||(PyObject* obj, const fail& error) {
    if(obj) return obj;
    throw std::runtime_error(error.message);
    return nullptr;
}

static void operator||(int code, const fail& error) {
    if(code >= 0) return;
    throw std::runtime_error(error.message);
}


enum flag : char {
    stop = 1 << 1,
    test_failure = 1 << 2,
    python_error = 1 << 3
};





// shared data between except_hook and simulation loop
struct shared_type {
    char flags = 0;
    std::string message;
    PyObject* default_excepthook = nullptr;
};


// TODO FIXME: there's probably a MEMLEAK hiding in there, figure it out
static PyObject* except_hook(PyObject* self, PyObject* args) {

    // raise the stop flag
    shared_type* shared = reinterpret_cast<shared_type*>(PyCapsule_GetPointer(self, NULL));
    
    assert(shared && "cannot get shared_type pointer (wtf?)");
    if(!shared) std::exit(1);

    shared->flags |= flag::stop;
    
    // switch on exception type
    PyObject* type;
    PyObject* value;
    PyObject* traceback;

    if( !PyArg_ParseTuple(args, "OOO", &type, &value, &traceback) ) {
        assert(false && "cannot parse excepthook args (wtf?)");
        std::exit(1);
    }

    if( type == PyExc_AssertionError ) {
        // test failure
        shared->flags |= flag::test_failure;
    } else {
        // other python error
        shared->flags |= flag::python_error;
    }

    {
        // TODO find a way to push traceback as well
        const py::object value_string = py::ptr(value).str();
        assert(value_string.c_str());        
        shared->message = value_string.c_str();
    }
    
    // call default excepthook to get traceback etc
    return PyObject_CallObject(shared->default_excepthook, args);
}

static PyMethodDef except_hook_def = {
    "sofa_excepthook",
    except_hook,
    METH_VARARGS,
    NULL
};


// save/restore excepthook
class excepthook_installer {
    PyObject* default_excepthook = nullptr;
public:
    excepthook_installer(shared_type* shared) {
        default_excepthook = PySys_GetObject((char*)"excepthook") || fail("cannot get default excepthook");

        shared->default_excepthook = default_excepthook;
        
        PyObject* self = PyCapsule_New(shared, NULL, NULL) || fail("cant wrap shared_type pointer");

        // build closure
        PyObject* excepthook = PyCFunction_NewEx(&except_hook_def, self, NULL)
                || fail("cannot create excepthook closure");
        
        PySys_SetObject((char*)"excepthook", excepthook) || fail("cannot set sys.excepthook");
    }

    ~excepthook_installer() {
        PySys_SetObject((char*)"excepthook", default_excepthook) || fail("cannot restore default excepthook");
    }
};


Python_scene_test::Python_scene_test()
    : max_steps(1000) {

}

void Python_scene_test::run( const Python_test_data& data ) {

    msg_info("Python_scene_test") << "running "<< data.filepath;

    {
        // Check the file exists
        std::ifstream file(data.filepath.c_str());
        bool scriptFound = file.good();
        ASSERT_TRUE(scriptFound) << " ("<<data.filepath<<")";
    }

    if( !simulation::getSimulation() ) {
        simulation::setSimulation( new sofa::simulation::graph::DAGSimulation() );
    }

    simulation::Node::SPtr root;
    
    try {
        shared_type shared;
        const excepthook_installer lock(&shared);
        
        loader.loadSceneWithArguments(data.filepath.c_str(),
                                      data.arguments,
                                      &root);
        ASSERT_TRUE(bool(root)) << "scene creation failed!" << " ("<<data.filepath<<")";

        root->addObject( new Listener );
        simulation::getSimulation()->init(root.get());


        // TODO eventually tests should only stop by throwing SystemExit
        unsigned i;
        for(i = 0; (i < max_steps) && !(shared.flags & flag::stop) && root->isActive(); ++i) {
            simulation::getSimulation()->animate(root.get(), root->getDt());
        }

        ASSERT_TRUE(i != max_steps) << "maximum allowed steps reached: " << max_steps << " ("<<data.filepath<<")";

        if( shared.flags & flag::test_failure ) {
            FAIL() << "test failure:\n" << shared.message;
        }

        if( shared.flags & flag::python_error) {
            FAIL() << "python error:\n" << shared.message;
        }
        
    } catch( simulation::PythonEnvironment::system_exit& e) {
        SUCCEED() << "test terminated normally";
    } catch( const result& test_result ) {
        ASSERT_TRUE(test_result.value) << " ("<<data.filepath<<")";

        // TODO raii for unloading
        simulation::getSimulation()->unload( root.get() );
        
    } catch( std::runtime_error& e) {
        ASSERT_TRUE(false) << "error setting up python excepthook, aborting test" << " ("<<data.filepath<<")";
    }
}



} // namespace sofa

