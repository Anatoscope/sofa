#ifndef SOFA_GUI_QT_QSOFALISTVIEW_H
#define SOFA_GUI_QT_QSOFALISTVIEW_H

#ifdef SOFA_QT4
#include <QWidget>
#include <Q3ListView>
#include <Q3ListViewItem>
#include <Q3Header>
#include <QPushButton>
#else
#include <qwidget.h>
#include <qlistview.h>
#include <qheader.h>
#include <qpushbutton.h>
#endif

#include <sofa/simulation/common/Node.h>
#include <sofa/core/objectmodel/BaseData.h>
#include <sofa/core/objectmodel/BaseObject.h>

#ifndef SOFA_QT4
typedef QListView Q3ListView;
typedef QListViewItem Q3ListViewItem;

#endif
#include <map>

namespace sofa
{

namespace gui
{
namespace qt
{

class AddObject;
class GraphListenerQListView;


enum ObjectModelType { typeNode, typeObject, typeData };
typedef union ObjectModelPtr
{
    sofa::simulation::Node* Node;
    core::objectmodel::BaseObject* Object;
    core::objectmodel::BaseData* Data;
} ObjectModelPtr;

typedef struct ObjectModel
{
public:
    ObjectModelType type;
    ObjectModelPtr ptr;
    bool isNode()   { return type == typeNode;   }
    bool isObject() { return type == typeObject; }
    bool isData()   { return type == typeData;   }
} ObjectModel;

enum SofaListViewAttribute
{
    SIMULATION,
    VISUAL,
    MODELER
};

class QSofaListView : public Q3ListView
{
    Q_OBJECT
public:
    QSofaListView(const SofaListViewAttribute& attribute,
            QWidget* parent=0,
            const char* name=0,
            Qt::WFlags f = 0 );
    ~QSofaListView();

    GraphListenerQListView* getListener() const { return  graphListener_; };
    void Clear(sofa::simulation::Node* rootNode);
    void Freeze();
    void Unfreeze();
    SofaListViewAttribute getAttribute() const { return attribute_; };
public slots:
    void Export();
    void CloseAllDialogs();
    void UpdateOpenedDialogs();
signals:
    void Close();
    void Lock(bool);
    void RequestSaving(sofa::simulation::Node*);
    void RequestExportOBJ(sofa::simulation::Node* node, bool exportMTL);
    void RequestActivation(sofa::simulation::Node*,bool);
    void RootNodeChanged(sofa::simulation::Node* newroot, const char* newpath);
    void NodeRemoved();
    void Updated();
    void NodeAdded();

protected slots:
    void SaveNode();
    void exportOBJ();
    void collapseNode();
    void expandNode();
    void modifyUnlock(void* Id);
    void RaiseAddObject();
    void RemoveNode();
    void Modify();
    void HideDatas();
    void ShowDatas();
    void DeactivateNode();
    void ActivateNode();
    void loadObject ( std::string path, double dx, double dy, double dz,  double rx, double ry, double rz,double scale );
#ifdef SOFA_QT4
    void updateMatchingObjectmodel(Q3ListViewItem* item);
    void RunSofaRightClicked( Q3ListViewItem *item, const QPoint& point, int index );
    void RunSofaDoubleClicked( Q3ListViewItem*);
#else
    void updateMatchingObjectmodel(QListViewItem* item);
    void RunSofaRightClicked( QListViewItem *item, const QPoint& point, int index );
    void RunSofaDoubleClicked( QListViewItem*);
#endif
    void nodeNameModification( simulation::Node*);
protected:
    void collapseNode(Q3ListViewItem* item);
    void expandNode(Q3ListViewItem* item);
    void transformObject ( sofa::simulation::Node *node, double dx, double dy, double dz,  double rx, double ry, double rz, double scale );
    bool isNodeErasable( core::objectmodel::BaseNode* node);
    void updateMatchingObjectmodel();
    std::list<core::objectmodel::BaseNode*> collectNodesToChange(core::objectmodel::BaseNode* node);
    std::map< void*, Q3ListViewItem* > map_modifyDialogOpened;
    std::map< void*, QDialog* > map_modifyObjectWindow;
    GraphListenerQListView* graphListener_;
    std::vector< std::string > list_object;
    AddObject* AddObjectDialog_;
    ObjectModel object_;
    SofaListViewAttribute attribute_;

};

} //sofa
} //gui
}//qt

#endif // SOFA_GUI_QT_QSOFALISTVIEW_H


