
/// CREATING A NEW PYTHON MODULE: _SofaTest
///
/// @author Matthieu Nesme
/// @date 2017


#include <SofaPython/PythonMacros.h>
#include <SofaPython/PythonFactory.h>
#include <SofaPython/Binding_Data.h>

#include <sofa/helper/cast.h>
#include <sofa/helper/logging/Messaging.h>





/// args are node + factors m,b,k to return the linear combinaison mM+bB+kK
extern "C" PyObject * _SofaTest_ASSERT(PyObject * /*self*/, PyObject * args)
{
    PyObject* pyCondition;
    if (!PyArg_ParseTuple(args, "O", &pyCondition) || !PyBool_Check(pyCondition) )
    {
        PyErr_BadArgument();
        return NULL;
    }

    bool condition = pyCondition==Py_True;

    // TODO gtest assert
    msg_info("_SofaTest")<<"needs to gtest-assert "<<condition;



    Py_RETURN_NONE;
}



// Methods of the module
SP_MODULE_METHODS_BEGIN(_SofaTest)
SP_MODULE_METHOD(_SofaTest,ASSERT)
SP_MODULE_METHODS_END

