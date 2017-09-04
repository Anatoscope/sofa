#ifndef SOFA_SOFAPYTHON_HELPER_HPP
#define SOFA_SOFAPYTHON_HELPER_HPP

#include <SofaPython/PythonEnvironment.h>

namespace sofa {
namespace py {

class object;
class ref;

// a simple pyobject pointer wrapper
class SOFA_SOFAPYTHON_API ptr {
protected:
    PyObject* obj;
public:
    ptr(PyObject* obj) : obj(obj) { }

    explicit operator bool() const { return obj; }
    
    PyObject* get() const { return obj; }
    PyObject* operator->() const { return obj; }    

    // TODO make these free functions to mimic python?
    object repr() const;
    object str() const;    

    const char* c_str() const;
};


// a scoped owning reference to a new object: refcount is decreased at
// desctruction
class SOFA_SOFAPYTHON_API object : public ptr {
public:
    object(PyObject* obj);

    ~object();

    object(object&& other);

    object(const object&) = delete;
    object& operator=(const object&) = delete;

    object& operator=(object&&) = delete;
};


// a scoped shared reference: refcount is increased on construction, decreased
// on destruction.
class SOFA_SOFAPYTHON_API ref : public ptr {
public:
    
    ref(PyObject* obj);

    ~ref();
    ref(ref&& other);

    // TODO implement these?
    ref(const ref&) = delete;
    ref& operator=(const ref&) = delete;

    ref& operator=(ref&&) = delete;
};




}
}


#endif
