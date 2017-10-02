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
#include <sofa/defaulttype/BoundingBox.h>
#include <limits>

namespace sofa
{
namespace defaulttype
{



template<int N,class Real>
typename TBoundingBox<N,Real>::bbox_t TBoundingBox<N,Real>::make_neutralBBox()
{
    const Real max_real = std::numeric_limits<Real>::max();
    Coord minBBox, maxBBox;
    minBBox.fill(max_real);
    maxBBox.fill(-max_real);
    return std::make_pair(minBBox,maxBBox);
}

template<int N,class Real>
TBoundingBox<N,Real>::TBoundingBox()
    :bbox(make_neutralBBox())
{
}

template<int N,class Real>
TBoundingBox<N,Real>::TBoundingBox(const bbox_t& bbox)
    :bbox(bbox)
{
}

template<int N,class Real>
TBoundingBox<N,Real>::TBoundingBox(const Coord& minBBox, const Coord& maxBBox)
    :bbox(std::make_pair(minBBox,maxBBox))
{
}


/*static*/
template<int N,class Real>
TBoundingBox<N,Real> TBoundingBox<N,Real>::neutral_bbox()
{
    return TBoundingBox(make_neutralBBox());
}

template<int N,class Real>
void TBoundingBox<N,Real>::invalidate()
{
    this->bbox = make_neutralBBox();
}

template<int N,class Real>
bool TBoundingBox<N,Real>::isNegligeable() const
{
    for( int i=0 ; i<N ; ++i )
        if( minBBox()[i] < maxBBox()[i] )
            return false;
    return true;
}

template<int N,class Real>
bool TBoundingBox<N,Real>::isValid() const
{
    for( int i=0 ; i<N ; ++i )
        if( minBBox()[i] > maxBBox()[i] )
            return false;
    return true;
}

template<int N,class Real>
bool TBoundingBox<N,Real>::isFlat() const
{
    for( int i=0 ; i<N ; ++i )
        if( minBBox()[i] != maxBBox()[i] )
            return false;
    return true;
}

template<int N,class Real>
bool TBoundingBox<N,Real>::isNull() const
{
    return isFlat();
}

template<int N,class Real>
TBoundingBox<N,Real>::operator bbox_t() const
{
    return bbox;
}

template<int N,class Real>
Real* TBoundingBox<N,Real>::minBBoxPtr()
{
    return bbox.first.elems;
}

template<int N,class Real>
Real* TBoundingBox<N,Real>::maxBBoxPtr()
{
    return bbox.second.elems;
}

template<int N,class Real>
const Real* TBoundingBox<N,Real>::minBBoxPtr() const
{
    return bbox.first.elems;
}

template<int N,class Real>
const Real* TBoundingBox<N,Real>::maxBBoxPtr() const
{
    return bbox.second.elems;
}

template<int N,class Real>
const typename TBoundingBox<N,Real>::Coord& TBoundingBox<N,Real>::minBBox() const
{
    return bbox.first;
}

template<int N,class Real>
const typename TBoundingBox<N,Real>::Coord& TBoundingBox<N,Real>::maxBBox() const
{
    return bbox.second;
}

template<int N,class Real>
typename TBoundingBox<N,Real>::Coord& TBoundingBox<N,Real>::minBBox()
{
    return bbox.first;
}

template<int N,class Real>
typename TBoundingBox<N,Real>::Coord& TBoundingBox<N,Real>::maxBBox()
{
    return bbox.second;
}

template<int N,class Real>
bool TBoundingBox<N,Real>::contains(const Coord& point) const
{
    for( int i=0 ; i<N ; ++i )
        if( point[i] < minBBox()[i] || point[i] > maxBBox()[i] )
            return false;
    return true;
}

template<int N,class Real>
bool TBoundingBox<N,Real>::contains(const TBoundingBox& other) const
{
    return contains(other.minBBox()) && contains(other.maxBBox());
}

template<int N,class Real>
bool TBoundingBox<N,Real>::intersect(const TBoundingBox& other) const
{
    for( int i=0 ; i<N ; ++i )
        if( other.minBBox()[i] > maxBBox()[i] || other.maxBBox()[i] < minBBox()[i] ) return false;
    return true;
}

template<int N,class Real>
void TBoundingBox<N,Real>::intersection(const TBoundingBox& other)
{

    for( int i=0 ; i<N ; ++i )
    {
        minBBox()[i] = std::max(minBBox()[i], other.minBBox()[i]);
        maxBBox()[i] = std::min(maxBBox()[i], other.maxBBox()[i]);
    }
}

template<int N,class Real>
void TBoundingBox<N,Real>::include(const Coord& point)
{
    for( int i=0 ; i<N ; ++i )
    {
        minBBox()[i] = std::min( minBBox()[i], point[i]);
        maxBBox()[i] = std::max( maxBBox()[i], point[i]);
    }
}

template<int N,class Real>
void TBoundingBox<N,Real>::include(const TBoundingBox& other)
{
    for( int i=0 ; i<N ; ++i )
    {
        minBBox()[i] = std::min( minBBox()[i], other.minBBox()[i]);
        maxBBox()[i] = std::max( maxBBox()[i], other.maxBBox()[i]);
    }
}

template<int N,class Real>
void TBoundingBox<N,Real>::inflate(const Real amount)
{
    Coord size;
    size.fill(amount);
    minBBox() -= size;
    maxBBox() += size;
}


template<int N,class Real>
TBoundingBox<N,Real> TBoundingBox<N,Real>::getIntersection( const TBoundingBox& other ) const
{
    TBoundingBox<N,Real> result;

    for( int i=0 ; i<N ; ++i )
    {
        result.minBBox()[i] = std::max(minBBox()[i], other.minBBox()[i]);
        result.maxBBox()[i] = std::min(maxBBox()[i], other.maxBBox()[i]);
    }

    return result;
}

template<int N,class Real>
TBoundingBox<N,Real> TBoundingBox<N,Real>::getInclude( const Coord& point ) const
{
    TBoundingBox<N,Real> result;

    for( int i=0 ; i<N ; ++i )
    {
        result.minBBox()[i] = std::min( minBBox()[i], point[i]);
        result.maxBBox()[i] = std::max( maxBBox()[i], point[i]);
    }

    return result;
}

template<int N,class Real>
TBoundingBox<N,Real> TBoundingBox<N,Real>::getInclude( const TBoundingBox& other ) const
{
    TBoundingBox<N,Real> result;

    for( int i=0 ; i<N ; ++i )
    {
        result.minBBox()[i] = std::min( minBBox()[i], other.minBBox()[i]);
        result.maxBBox()[i] = std::max( maxBBox()[i], other.maxBBox()[i]);
    }

    return result;
}

template<int N,class Real>
TBoundingBox<N,Real> TBoundingBox<N,Real>::getInflate( Real amount ) const
{
    TBoundingBox<N,Real> result;

    Coord size;
    size.fill(amount);
    result.minBBox() = minBBox() - size;
    result.maxBBox() = maxBBox() + size;

    return result;
}




} // namespace defaulttype

} // namespace sofa
