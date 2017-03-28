
/// CREATING A NEW PYTHON MODULE: _SofaTest
///
/// @author Matthieu Nesme
/// @date 2017


#include <SofaPython/PythonMacros.h>
#include <SofaPython/PythonFactory.h>
#include <SofaPython/Binding_Data.h>

#include <sofa/helper/cast.h>
#include <sofa/helper/logging/Messaging.h>

#include <SofaTest/Python_test.h>
#include <gtest/gtest.h>


template< void (*action)(bool, const char* ) >
static PyObject * predicate_check(PyObject * /*self*/, PyObject * args) {

    PyObject* condition = 0;
    const char* message = "";
    
    if (!PyArg_ParseTuple(args, "O|s", &condition, &message) ) {
        PyErr_BadArgument();
        return NULL;
    }

    const int test = PyObject_IsTrue(condition);
    if(test < 0) {
        PyErr_BadArgument();    // should we set error ?
        return NULL;
    }

    action(test, message);
    
    Py_RETURN_NONE;
}


static void assert_true(bool condition, const char* message) {
    ASSERT_TRUE(condition) << message;
    sofa::Python_scene_test::finish();    
}


static void expect_true(bool condition, const char* message) {
    EXPECT_TRUE(condition) << message;
}


static PyCFunction _SofaTest_assert_true = predicate_check<assert_true>;
static PyCFunction _SofaTest_expect_true = predicate_check<expect_true>;

static PyObject * _SofaTest_finish(PyObject * /*self*/, PyObject * /*args*/) {
    sofa::Python_scene_test::finish();
    Py_RETURN_NONE;
}

static PyObject * _SofaTest_excepthook(PyObject * /*self*/, PyObject * /*args*/) {
    auto fail = []{
        const bool no_uncaught_exception = false;
        ASSERT_TRUE(no_uncaught_exception) << "python error, aborting test";
    };
    
    fail();
    sofa::Python_scene_test::finish();
    Py_RETURN_NONE;
}



SP_MODULE_METHODS_BEGIN(_SofaTest)
SP_MODULE_METHOD(_SofaTest, assert_true)
SP_MODULE_METHOD(_SofaTest, expect_true)
SP_MODULE_METHOD(_SofaTest, finish)
SP_MODULE_METHOD(_SofaTest, excepthook)
SP_MODULE_METHODS_END
