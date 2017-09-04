#include "helper.hpp"

namespace sofa {
namespace py {

object ptr::str() const {
    return PyObject_Str(get());
}

object ptr::repr() const {
    return PyObject_Repr(get());
}


const char* ptr::c_str() const {
    if(!PyString_Check(get())) return nullptr;

    return PyString_AsString(get());
}


ref::~ref() {
    Py_XDECREF(obj);
}

ref::ref(ref&& other) : ptr(other) {
    other.obj = nullptr;
}

ref::ref(PyObject* obj) : ptr(obj) {
    Py_XINCREF(obj);
}


object::object(PyObject* obj) : ptr(obj) { }

object::~object() {
    Py_XDECREF(obj);
}

object::object(object&& other) : ptr(other) {
    other.obj = nullptr;
}


}
}

