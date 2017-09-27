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
#ifndef SOFA_DEFAULTTYPE_BoundingBox_H
#define SOFA_DEFAULTTYPE_BoundingBox_H

#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/defaulttype.h>
#include <vector>

namespace sofa
{
namespace defaulttype
{

template<int N=3, class Real=SReal>
class TBoundingBox
{

public:

    enum { spatial_dimensions = N };
    typedef Real value_type;

    typedef Vec<N,Real> Coord;

    typedef std::pair< Coord, Coord > bbox_t;

    TBoundingBox();
    /// Define using the endpoints of the main diagonal
    TBoundingBox(const Coord& minBBox, const Coord& maxBBox);
    TBoundingBox(const bbox_t& bbox);

    static TBoundingBox neutral_bbox();

    template<class Vector>
    void setFromPointVector( const Vector& points )
    {
        const size_t size = points.size();
        if( !size ) return invalidate();

        const int n = std::min<int>( N, Vector::value_type::spatial_dimensions );

        int j=0;
        for( ; j<n ; ++j )
        {
            bbox.first[j] = points[0][j];
            bbox.second[j] = points[0][j];
        }
        for( ; j<N ; ++j )
        {
            bbox.first[j] = 0;
            bbox.second[j] = 0;
        }

        for( size_t i=1 ; i<size ; ++i )
        {
            for( j=0 ; j<n ; ++j )
            {
                if( points[i][j] < bbox.first[j] ) bbox.first[j] = points[i][j];
                else if( points[i][j] > bbox.second[j] ) bbox.second[j] = points[i][j];
            }
        }
    }


    operator bbox_t() const;

    void invalidate();
    bool isValid() const;
    bool isFlat()  const;
    bool isNegligeable() const; // !valid || flat
    bool isNull()  const;

    Real* minBBoxPtr();
    Real* maxBBoxPtr();
    const Real* minBBoxPtr() const;
    const Real* maxBBoxPtr() const;
    const Coord&  minBBox() const;
    const Coord&  maxBBox() const;
    Coord& minBBox();
    Coord& maxBBox();





    bool contains( const Coord& point) const;
    bool contains( const TBoundingBox& other) const;

    bool intersect( const TBoundingBox& other) const;
    void intersection( const TBoundingBox& other);

    void include( const Coord& point);
    void include( const TBoundingBox& other);

    void inflate( Real amount );

    TBoundingBox getIntersection( const TBoundingBox& other ) const;
    TBoundingBox getInclude( const Coord& point ) const;
    TBoundingBox getInclude( const TBoundingBox& other ) const;
    TBoundingBox getInflate( Real amount ) const;


    friend std::ostream& operator << ( std::ostream& out, const TBoundingBox& bbox)
    {
        out << bbox.minBBox() << " " <<  bbox.maxBBox();
        return out;
    }

    friend std::istream& operator >> ( std::istream& in, TBoundingBox& bbox)
    {
        in >> bbox.minBBox() >> bbox.maxBBox();
        return in;
    }


protected:


    bbox_t bbox;


    static bbox_t make_neutralBBox();

};





#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_DEFAULTTYPE_TBoundingBox_CPP)

extern template class SOFA_DEFAULTTYPE_API TBoundingBox<3,SReal>;
extern template class SOFA_DEFAULTTYPE_API TBoundingBox<2,SReal>;
extern template class SOFA_DEFAULTTYPE_API TBoundingBox<1,SReal>;


typedef TBoundingBox<3,SReal> BoundingBox3D; ///< bounding box
typedef BoundingBox3D BoundingBox; ///< bounding box
typedef TBoundingBox<2,SReal> BoundingBox2D; ///< bounding rectangle
typedef TBoundingBox<1,SReal> BoundingBox1D; ///< bounding interval
#endif


}
}

#endif // SOFA_DEFAULTTYPE_BoundingBox_H
