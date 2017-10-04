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


#include "Binding_ImageViewer.h"
#include <SofaPython/Binding_BaseObject.h>
#include <SofaPython/PythonToSofa.inl>
#include <sofa/defaulttype/Vec.h>

using namespace sofa::component::misc;
using namespace sofa::core::objectmodel;


/// getting a BaseImageViewer* from a PyObject*
static inline BaseImageViewer* get_ImageViewer(PyObject* obj) {
    return sofa::py::unwrap<BaseImageViewer>(obj);
}


static PyObject * ImageViewer_getPlaneQuads(PyObject *self, PyObject * /*args*/)
{
    BaseImageViewer* v = get_ImageViewer( self );
    BaseImageViewer::PlaneQuads pq;
    v->getPlaneQuads( pq );

    // @TODO check decref when creating lists

    PyObject *py_pq = PyList_New(pq.size());

    for( size_t i=0, n=pq.size() ; i<n ; ++i )
    {
        const BaseImageViewer::PlaneQuad& q = pq[i];
        PyObject *py_q = PyList_New(4);

        for (int j = 0; j < 4; ++j)
        {
            const sofa::defaulttype::Vector3& p = q[j];
            PyObject *py_p = PyList_New(3);

            for (int k = 0; k < 3; ++k)
            {
                PyList_SetItem(py_p, k, PyFloat_FromDouble(p[k]));
            }

            PyList_SetItem(py_q,j,py_p);
        }

        PyList_SetItem(py_pq,i,py_q);
    }

    return py_pq;
}


SP_CLASS_METHODS_BEGIN(ImageViewer)
SP_CLASS_METHOD(ImageViewer,getPlaneQuads)
SP_CLASS_METHODS_END


SP_CLASS_TYPE_SPTR(ImageViewer,BaseImageViewer,BaseObject)


