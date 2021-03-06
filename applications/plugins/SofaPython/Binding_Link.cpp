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
#include "Binding_Link.h"

#include <sofa/core/objectmodel/BaseLink.h>
#include <sofa/core/objectmodel/Link.h>
#include "PythonToSofa.inl"


using namespace sofa::core::objectmodel;
using namespace sofa::defaulttype;

//TODO(PR:304) remove this todo or do it.
//TODO:
// se servir du LinkTypeInfo pour utiliser directement les bons type :-)
// Il y a un seul type "Link" exposé en python, le transtypage est géré automatiquement

static inline BaseLink* get_baselink(PyObject* obj) {
    return sofa::py::unwrap<BaseLink>(obj);
}




SP_CLASS_ATTR_GET(Link,name)(PyObject *self, void*)
{
    BaseLink* link = get_baselink(self);
    return PyString_FromString(link->getName().c_str());
}


SP_CLASS_ATTR_SET(Link,name)(PyObject *self, PyObject * args, void*)
{
    BaseLink* link = get_baselink( self );
    char *str = PyString_AsString(args); /// for setters, only one object and not a tuple....
    link->setName(str);
    return 0;
}


PyObject *GetLinkValuePython(BaseLink* link)
{
    /// only by string for now
    return PyString_FromString(link->getValueString().c_str());
}


int SetLinkValuePython(BaseLink* link, PyObject* args)
{
    if( PyString_Check(args) )
    {
        char *str = PyString_AsString(args); /// for setters, only one object and not a tuple....
        link->read(str);
        return 0;
    }

    return -1;
}


SP_CLASS_ATTR_GET(Link,value)(PyObject *self, void*)
{
    BaseLink* link = get_baselink( self );
    return GetLinkValuePython(link);
}


SP_CLASS_ATTR_SET(Link,value)(PyObject *self, PyObject * args, void*)
{
    BaseLink* link = get_baselink( self );
    return SetLinkValuePython(link,args);
}


static PyObject * Link_getValueTypeString(PyObject *self, PyObject * /*args*/)
{
    BaseLink* link = get_baselink( self );
    return PyString_FromString(link->getValueTypeString().c_str());
}


static PyObject * Link_getValueString(PyObject *self, PyObject * /*args*/)
{
    BaseLink* link = get_baselink( self );
    return PyString_FromString(link->getValueString().c_str());
}


static PyObject * Link_getSize(PyObject *self, PyObject * /*args*/)
{
    BaseLink* link = get_baselink( self );
    return PyInt_FromLong( link->getSize() );
}

static PyObject* Link_base_link_ptr(PyObject* self, PyObject* args) {
    if (!PyArg_ParseTuple(args, "")) return NULL;

    BaseLink* component = get_baselink(self);
    return PyLong_FromVoidPtr(component);
}



SP_CLASS_METHODS_BEGIN(Link)
SP_CLASS_METHOD(Link,base_link_ptr)
SP_CLASS_METHOD(Link,getValueTypeString)
SP_CLASS_METHOD(Link,getValueString)
SP_CLASS_METHOD(Link,getSize)
SP_CLASS_METHODS_END


SP_CLASS_ATTRS_BEGIN(Link)
SP_CLASS_ATTR(Link,name)
SP_CLASS_ATTR(Link,value)
SP_CLASS_ATTRS_END

SP_CLASS_TYPE_BASE_PTR_ATTR(Link,BaseLink)

