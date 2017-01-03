#include "DataTracker.h"
#include "objectmodel/BaseData.h"

namespace sofa
{

namespace core
{


static std::hash<std::string> s_stringHasher;


void DataTracker::trackData( const objectmodel::BaseData& data, bool activateValueHash )
{
    m_dataTrackers[&data] = data.getCounter();
    if( activateValueHash ) m_dataValueTrackers[&data] = s_stringHasher(data.getValueString());
}

bool DataTracker::wasModified( const objectmodel::BaseData& data ) const
{
    return m_dataTrackers.at(&data) != data.getCounter();
}

bool DataTracker::wasModified() const
{
    for( DataTrackers::const_iterator it=m_dataTrackers.begin(),itend=m_dataTrackers.end() ; it!=itend ; ++it )
        if( it->second != it->first->getCounter() ) return true;
    return false;
}

bool DataTracker::isDirty( const objectmodel::BaseData& data ) const
{
    return wasModified(data) || data.isDirty();
}

bool DataTracker::isDirty() const
{
    for( DataTrackers::const_iterator it=m_dataTrackers.begin(),itend=m_dataTrackers.end() ; it!=itend ; ++it )
        if( it->second != it->first->getCounter() || it->first->isDirty() ) return true;
    return false;
}

bool DataTracker::hasChanged( const objectmodel::BaseData& data ) const
{
    return wasModified(data) && m_dataValueTrackers.at(&data) != s_stringHasher(data.getValueString());
}


void DataTracker::clean( const objectmodel::BaseData& data )
{
    m_dataTrackers.at(&data) = data.getCounter();

    if( m_dataValueTrackers.at(&data) ) m_dataValueTrackers[&data] = s_stringHasher(data.getValueString());
}

void DataTracker::clean()
{
    for( DataTrackers::iterator it=m_dataTrackers.begin(),itend=m_dataTrackers.end() ; it!=itend ; ++it )
        it->second = it->first->getCounter();

    for( DataValueTrackers::iterator it=m_dataValueTrackers.begin(),itend=m_dataValueTrackers.end() ; it!=itend ; ++it )
        it->second = s_stringHasher(it->first->getValueString());
}



////////////////////



void DataTrackerDDGNode::cleanDirty(const core::ExecParams* params)
{
    core::objectmodel::DDGNode::cleanDirty(params);

    // it is also time to clean the tracked Data
    m_dataTracker.clean();

}



void DataTrackerDDGNode::updateAllInputsIfDirty()
{
    const DDGLinkContainer& inputs = DDGNode::getInputs();
    for(size_t i=0, iend=inputs.size() ; i<iend ; ++i )
    {
        static_cast<core::objectmodel::BaseData*>(inputs[i])->updateIfDirty();
    }
}



///////////////////////


void DataTrackerEngine::setUpdateCallback( void (*f)(DataTrackerEngine*) )
{
    m_updateCallback = f;
}

}

}
