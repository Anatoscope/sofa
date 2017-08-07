import Sofa

import sys
import os.path
import math
import xml.etree.ElementTree as etree

import Quaternion
import Tools
from Tools import listToStr as concat
import units
import mass
import DAGValidation
import SofaPython.MeshLoader

printLog = True

def parseIdName(obj, objXml):
    """ set id and name of obj
    """
    obj.id = objXml.attrib["id"]
    obj.name = obj.id
    if not objXml.find("name") is None:
        obj.name = objXml.find("name").text

def parseTag(obj, objXml):
    """ set tags of the object
    """
    for xmlTag in objXml.findall("tag"):
        obj.tags.add(xmlTag.text)

def parseData(xmlData):
    """ return the list of data in xmlData
    """
    if xmlData.attrib["type"]=="float":
        return Tools.strToListFloat(xmlData.text)
    elif xmlData.attrib["type"]=="int":
        return Tools.strToListInt(xmlData.text)
    elif xmlData.attrib["type"]=="string":
        return xmlData.text.split()

def _getObjectsByTags(objects, tags):
    """ internal function to return a list of objects with given tags
    \todo add an explicit option atLeastOneTag, allTags,...
    """
    taggedObjects = list()
    for obj in objects:
        if len(obj.tags & tags) > 0:
            taggedObjects.append(obj)
    return taggedObjects

class Model:
    """ This class stores a Sofa Model read from a sml/xml (Sofa Modelling Language) file.
    """

    class Mesh:

        class Group:
            def __init__(self):
                self.index=list()
                self.data=dict()
                self.tags=set()
            def __len__(self):
                return len(self.index)

        def __init__(self, meshXml=None):
            self.id=None
            self.name=None
            self.format=None
            self.source=None
            self.group=dict() # should be groups with *s*
            if not meshXml is None:
                self.parseXml(meshXml)

        def parseXml(self, meshXml):
            parseIdName(self,meshXml)

            if not meshXml.find("source") is None:
                self.format = meshXml.find("source").attrib["format"]
                self.source = meshXml.find("source").text

            for g in meshXml.findall("group"):
                self.group[g.attrib["id"]] = Model.Mesh.Group()


                if not g.find("index").text:
                    Sofa.msg_warning("SofaPython.sml","Group: group '"+g.attrib["id"]+"' of mesh '"+self.name+"' is empty")
                else:
                    self.group[g.attrib["id"]].index = Tools.strToListInt(g.find("index").text)
                    for d in g.findall("data"):
                        self.group[g.attrib["id"]].data[d.attrib["name"]]=parseData(d)
                    parseTag(self.group[g.attrib["id"]], g)

        def load(self):
            if self.format.lower() == "obj":
                return SofaPython.MeshLoader.loadOBJ(self.source)
            else:
                Sofa.msg_error("SofaPython.sml","Mesh: format "+self.format+" not yet loadable")
                return SofaPython.MeshLoader.Mesh()

    class MeshAttributes:
        def __init__(self,objXml=None):
            self.mesh = None
            self.collision=True
            self.simulation=True
            self.visual=True
            self.tags = set()
            if not objXml is None:
                self.parseXml(objXml)

        def parseXml(self, objXml):
            if "collision" in objXml.attrib:
                self.collision = False if objXml.attrib["collision"] in {'False','0','false'} else True
            if "simulation" in objXml.attrib:
                self.simulation = False if objXml.attrib["simulation"] in {'False','0','false'} else True
            if "visual" in objXml.attrib:
                self.visual = False if objXml.attrib["visual"] in {'False','0','false'} else True
            parseTag(self, objXml)

        def getValueByTag(self, valuesByTag, noDefault=False):
            """
            \sa getValueByTag()
            """
            return getValueByTag(valuesByTag, self.tags, noDefault)

    class Image:
        def __init__(self, imageXml=None):
            self.format=None
            self.source=None
            if not imageXml is None:
                self.parseXml(imageXml)

        def parseXml(self, imageXml):
            parseIdName(self,imageXml)
            self.format = imageXml.find("source").attrib["format"]
            self.source = imageXml.find("source").text

    class Solid:
        def __init__(self, solidXml=None):
            self.id = None
            self.name = None
            self.tags = set()
            self.position = [0,0,0,0,0,0,1]
            self.keyPositions = {} # optional animated keyframed positions {name(string):position(6 floats)}, note 'name' can represent a time (that would need to be casted as a float in your sml moulinette)
            self.mesh = list() # list of meshes TODO: should be self.meshes to be consistent
            self.meshAttributes = dict() # attributes associated with each mesh
            self.image = list() # list of images
            self.offsets = list()  # list of rigid offsets

            #TODO replace this with a MassInfo?
            self.mass = None
            self.com = None # x,y,z
            self.inertia = None # Ixx, Ixy, Ixz, Iyy, Iyz, Izz or Ixx, Iyy, Izz
            self.inertia_rotation = None # only useful for diagonal (3 values) inertia
            self.massInfo = None # store the SofaPython.mass.RigidMassInfo() once computed

            self.parent = None # Parent Id for armature
            self.skinnings=list()
            if not solidXml is None:
                self.parseXml(solidXml)

        def getRigidMassInfo(self, density, scale=1):
            if self.massInfo is None:
                self.massInfo = computeRigidMassInfo(self, density, scale)
            return self.massInfo

        def addMesh(self, mesh, attr=None):
            self.mesh.append(mesh)
            if not attr is None:
                self.meshAttributes[mesh.id]=attr
            else:
                self.meshAttributes[mesh.id]= Model.MeshAttributes()
            self.meshAttributes[mesh.id].mesh = mesh

        def addImage(self, image):
            self.image.append(image)

        def getValueByTag(self, valuesByTag, noDefault=False):
            """
            \sa getValueByTag()
            """
            return getValueByTag(valuesByTag, self.tags, noDefault)

        def getMeshesByTags(self, tags):
            """ \return a list of solids which contains at least one tag from tags
            """
            meshes = list()
            for ma in _getObjectsByTags(self.meshAttributes.values(), tags):
                meshes.append(ma.mesh)
            return meshes

        def parseXml(self, objXml):
            parseIdName(self, objXml)
            parseTag(self,objXml)
            if not objXml.find("position") is None:
                self.position=Tools.strToListFloat(objXml.find("position").text)
            if not objXml.find("mass") is None:
                self.mass = float(objXml.find("mass").text)
            if not objXml.find("parent") is None:
                self.parent = objXml.find("parent").text
            if not objXml.find("com") is None:
                self.com = Tools.strToListFloat(objXml.find("com").text)
            if not objXml.find("inertia") is None:
                self.inertia = Tools.strToListFloat(objXml.find("inertia").text)
            if not objXml.find("inertia_rotation") is None:
                self.inertia_rotation = Tools.strToListFloat(objXml.find("inertia_rotation").text)
            for o in objXml.findall("offset"):
                self.offsets.append( Model.Offset(o) )
            for o in objXml.findall("keyPosition"):
                assert( o.attrib["name"] )
                self.keyPositions[o.attrib["name"]] = Tools.strToListFloat(o.text)

    class Offset:
        def __init__(self, offsetXml=None):
            self.name = "offset"
            self.value = [0., 0., 0., 0., 0., 0., 1.] # x y z qx qy qz qw
            self.type = "absolute"
            if not offsetXml is None:
                self.parseXml(offsetXml)

        def parseXml(self, offsetXml):
            self.value = Tools.strToListFloat(offsetXml.text)
            self.type = offsetXml.attrib["type"]
            if "name" in offsetXml.attrib:
                self.name = offsetXml.attrib["name"]

        def isAbsolute(self):
            return self.type == "absolute"

    class Dof:
        def __init__(self, dofXml=None, dof=None, min=None, max=None):
            self.index = None
            if not dof is None:
                self.index = Model.dofIndex[dof]
            self.min = min
            self.max = max
            if not dofXml is None:
                self.parseXml(dofXml)

        def parseXml(self, dofXml):
            self.index = Model.dofIndex[dofXml.attrib["index"]]
            if "min" in dofXml.attrib:
                self.min = float(dofXml.attrib["min"])
            if "max" in dofXml.attrib:
                self.max = float(dofXml.attrib["max"])

    class JointGeneric:
        def __init__(self, jointXml=None):
            self.id = None
            self.name = None
            self.solids = [None,None]
            # offsets
            self.offsets = [None,None]
            # dofs
            self.dofs = []
            self.tags = set() # user-defined tags
            if not jointXml is None:
                self.parseXml(jointXml)

        def parseXml(self, jointXml):
            parseIdName(self, jointXml)
            parseTag(self, jointXml)
            solidsRef = jointXml.findall("jointSolidRef")
            for i in range(0,2):
                if not solidsRef[i].find("offset") is None:
                    self.offsets[i] = Model.Offset(solidsRef[i].find("offset"))
                    self.offsets[i].name = "offset_{0}".format(self.name)
            for dof in jointXml.findall("dof"):
                self.dofs.append(Model.Dof(dof))

    class Skinning:
        """ Skinning definition, vertices index influenced by bone with weight
        """
        def __init__(self):
            self.solid = None    # id of the parent bone # WARNING it rather seems to be directly a pointer to the Solid
            self.mesh = None     # the target mesh
            self.index = list()  # indices of target mesh
            self.weight = list() # weights for these vertices with respect with this bone

    class Surface:
        def __init__(self):
            self.solid = None # a Model.Solid object
            self.mesh = None  # a Model.Mesh object
            self.group = None # the name of the group defined in the mesh

    class SurfaceLink:
        def __init__(self,objXml=None):
            self.id = None
            self.name = None
            self.tags = set()           # user-defined tags
            self.surfaces = [None,None] # two Model.Surface
            self.distance=None
            if not objXml is None:
                self.parseXml(objXml)

        def parseXml(self, objXml):
            parseIdName(self,objXml)
            parseTag(self,objXml)
            if objXml.find("distance"):
                self.distance=float(objXml.findText("distance"))

    dofIndex={"x": 0, "y": 1, "z": 2, "rx": 3, "ry": 4, "rz": 5}

    def __init__(self, filename=None, name=None):
        self.name = name
        self.modelDir = None
        self.units = dict()
        self.meshes = dict()
        self.images = dict()
        self.solids = dict()
        self.genericJoints = dict()
        self.surfaceLinks = dict()

        if not filename is None:
            self.open(filename)

    def open(self, filename):
        Sofa.msg_info("SofaPython.sml","Model: opening file: " + filename)
        self.modelDir = os.path.abspath(os.path.dirname(filename))
        with open(filename,'r') as f:
            # TODO automatic DTD validation could go here, not available in python builtin ElementTree module
            modelXml = etree.parse(f).getroot()
            if self.name is None and "name" in modelXml.attrib:
                self.name = modelXml.attrib["name"]
            else:
                self.name = os.path.basename(filename)

            # units
            self.parseUnits(modelXml)

            # meshes
            for m in modelXml.iter("mesh"):
                if not m.find("source") is None:
                    if m.attrib["id"] in self.meshes:
                        Sofa.msg_warning("SofaPython.sml","Model: mesh id {0} already defined".format(m.attrib["id"]) )
                    mesh = Model.Mesh(m)
                    if not mesh.source is None:
                        sourceFullPath = os.path.join(self.modelDir,mesh.source)
                        if os.path.exists(sourceFullPath):
                            mesh.source=sourceFullPath
                        else:
                            Sofa.msg_warning("SofaPython.sml","Model: mesh not found: "+mesh.source )
                    self.meshes[m.attrib["id"]] = mesh

            # images
            for m in modelXml.iter("image"):
                if not m.find("source") is None:
                    if m.attrib["id"] in self.images:
                        Sofa.msg_warning("SofaPython.sml","Model: image id {0} already defined".format(m.attrib["id"]) )
                    image = Model.Image(m)
                    sourceFullPath = os.path.join(self.modelDir,image.source)
                    if os.path.exists(sourceFullPath):
                        image.source=sourceFullPath
                    else:
                        Sofa.msg_warning("SofaPython.sml","Model: image not found: "+image.source )
                    self.images[m.attrib["id"]] = image

            # solids
            for objXml in modelXml.findall("solid"):
                if objXml.attrib["id"] in self.solids:
                    Sofa.msg_error("SofaPython.sml","Model: solid defined twice, id:" + objXml.attrib["id"])
                    continue
                solid=Model.Solid(objXml)
                self.parseMeshes(solid, objXml)
                self.parseImages(solid, objXml)
                self.solids[solid.id]=solid

            # skinning
            for objXml in modelXml.findall("solid"):
                solid=self.solids[objXml.attrib["id"]]
                # TODO: support multiple meshes for skinning (currently only the first mesh is skinned)
                for s in objXml.findall("skinning"):
                    if not s.attrib["solid"] in self.solids:
                        Sofa.msg_error("SofaPython.sml","Model: skinning for solid {0}: solid {1} is not defined".format(solid.name, s.attrib["solid"]) )
                        continue
                    skinning = Model.Skinning()
                    if not s.attrib["solid"] in self.solids :
                        Sofa.msg_error("SofaPython.sml","Model: skinning for solid {0}: bone (solid) {1} not defined".format(solid.name, s.attrib["solid"]) )
                        continue
                    skinning.solid = self.solids[s.attrib["solid"]]
                    if not s.attrib["mesh"] in self.meshes :
                        Sofa.msg_error("SofaPython.sml","Model: skinning for solid {0}: mesh {1} not defined".format(solid.name, s.attrib["mesh"]) )
                        continue
                    skinning.mesh = self.meshes[s.attrib["mesh"]]
                    #TODO: check that this mesh is also part of the solid
                    if not (s.attrib["group"] in skinning.mesh.group and s.attrib["weight"] in skinning.mesh.group[s.attrib["group"]].data):
                        Sofa.msg_error("SofaPython.sml","Model: skinning for solid {0}: mesh {1} - group {2} - weight {3} is not defined".format(solid.name, s.attrib["mesh"], s.attrib["group"], s.attrib["weight"]))
                        continue
                    skinning.index = skinning.mesh.group[s.attrib["group"]].index
                    skinning.weight = skinning.mesh.group[s.attrib["group"]].data[s.attrib["weight"]]
                    solid.skinnings.append(skinning)

            # joints
            self.parseJointGenerics(modelXml)

            # contacts
            for c in modelXml.findall("surfaceLink"):
                if c.attrib["id"] in self.surfaceLinks:
                    Sofa.msg_error("SofaPython.sml","Model: surfaceLink defined twice, id:", c.attrib["id"])
                    continue
                surfaceLink = Model.SurfaceLink(c)
                surfaces=c.findall("surface")
                for i,s in enumerate(surfaces):
                    # a surfaceLink has at least two surfaces (initialized to None)
                    if i>= 2:
                        surfaceLink.surfaces.append( Model.Surface() )
                    else:
                        surfaceLink.surfaces[i] = Model.Surface()
                    if s.attrib["solid"] in self.solids:
                        surfaceLink.surfaces[i].solid = self.solids[s.attrib["solid"]]
                    else:
                        Sofa.msg_error("SofaPython.sml","Model: in contact {0}, unknown solid {1} referenced".format(surfaceLink.name, s.attrib["solid"]))
                    if s.attrib["mesh"] in self.meshes:
                        surfaceLink.surfaces[i].mesh = self.meshes[s.attrib["mesh"]]
                    else:
                        Sofa.msg_error("SofaPython.sml","Model: in contact {0}, unknown mesh {1} referenced".format(surfaceLink.name, s.attrib["mesh"]))
                    if "group" in s.attrib: # optional
                        if len(s.attrib["group"]): # discard empty string
                            surfaceLink.surfaces[i].group = s.attrib["group"]
                    #if "image" in s.attrib: # optional
                    #    if len(s.attrib["image"]): # discard empty string
                    #        if s.attrib["image"] in self.images:
                    #           reg.surfaces[i].image = self.images[s.attrib["image"]]
                self.surfaceLinks[surfaceLink.id]=surfaceLink

    def parseUnits(self, modelXml):
        xmlUnits = modelXml.find("units")
        if not xmlUnits is None:
            for u in xmlUnits.attrib:
                self.units[u]=xmlUnits.attrib[u]

    def parseMeshes(self, obj, objXml):
        meshes=objXml.findall("mesh")
        for i,m in enumerate(meshes):
            meshId = m.attrib["id"]
            attr = Model.MeshAttributes(m)
            if meshId in self.meshes:
                obj.addMesh(self.meshes[meshId], attr)
            else:
                Sofa.msg_error("SofaPython.sml","Model: solid {0} references undefined mesh {1}".format(obj.name, meshId))

    def parseImages(self, obj, objXml):
        images=objXml.findall("image")
        for i,m in enumerate(images):
            imageId = m.attrib["id"]
            if imageId in self.images:
                obj.addImage(self.images[imageId])
            else:
                Sofa.msg_error("SofaPython.sml","Model: solid {0} references undefined image {1}".format(obj.name, imageId))

    def parseJointGenerics(self,modelXml):
        for j in modelXml.findall("jointGeneric"):
            if j.attrib["id"] in self.genericJoints:
                Sofa.msg_error("SofaPython.sml","Model: jointGeneric defined twice, id:", j.attrib["id"])
                continue

            joint = Model.JointGeneric(j)
            solids = j.findall("jointSolidRef")
            for i, o in enumerate(solids):
                if o.attrib["id"] in self.solids:
                    joint.solids[i] = self.solids[o.attrib["id"]]
                else:
                    Sofa.msg_error("SofaPython.sml","Model: in joint {0}, unknown solid {1} referenced".format(joint.name, o.attrib["id"]))
            self.genericJoints[joint.id]=joint

    def setMeshDirectoryPath(self, path):
        """ Change all the meshes directory path to be path.
        If path is not absolute, model directory is appended to it
        """
        _path = None
        if os.path.isabs(path):
            _path = path
        else:
            _path = os.path.join(self.modelDir, path)
        for mesh in self.meshes.itervalues():
            if not mesh.source is None:
                mesh.source = os.path.join(_path, os.path.basename(mesh.source))
                if not os.path.exists(mesh.source):
                    Sofa.msg_warning("SofaPython.sml","Model: mesh not found: "+mesh.source)

    def getSolidsByTags(self, tags):
        """ \return a list of solids which contains at least one tag from tags
        """
        return _getObjectsByTags(self.solids.values(), tags)

    def getSurfaceLinksByTags(self, tags):
        """ \return a list of solids which contains at least one tag from tags
        """
        return _getObjectsByTags(self.surfaceLinks.values(), tags)

def insertVisual(parentNode, solid, color):
    node = parentNode.createChild("node_"+solid.name)
    translation=solid.position[:3]
    rotation = Quaternion.to_euler(solid.position[3:])  * 180.0 / math.pi
    for m in solid.mesh:
        Tools.meshLoader(node, m.source, name="loader_"+m.id)
        node.createObject("OglModel",src="@loader_"+m.id, translation=concat(translation),rotation=rotation.tolist(), color=color)

def setupUnits(myUnits):
    message = "units:"
    for quantity,unit in myUnits.iteritems():
        exec("units.local_{0} = units.{0}_{1}".format(quantity,unit))
        message+=" "+quantity+":"+unit
    if printLog:
        Sofa.msg_info("SofaPython.sml",message)

def computeRigidMassInfo(solid, density, scale=1):
    massInfo = mass.RigidMassInfo()
    for mesh in solid.mesh:
        if solid.meshAttributes[mesh.id].simulation is True:
            # mesh mass info
            mmi = mass.RigidMassInfo()
            mmi.setFromMesh(mesh.source, density=density, scale3d=[scale]*3)
            massInfo += mmi
    return massInfo

class BaseScene:
    """ Base class for Scene class, creates a node for this Scene
    """
    class Param:
        pass

    def __init__(self,parentNode,model,name=None):
        self.root = parentNode
        self.model = model
        self.param = BaseScene.Param()
        self.material = Tools.Material() # a default material set
        self.solidMaterial = dict() # assign a material to a solid
        self.nodes = dict() # to store special nodes
        self.meshExporters = list() # optional components to exports meshes
        self.collisions = dict() # to store collisions of the scene by [solidId][meshId]
        self.visuals = dict() # to store visuals of the scene by [solidId][meshId]

        n=name
        if n is None:
            n=self.model.name
        self.node=parentNode.createChild(n)
        setupUnits(self.model.units)

    def createChild(self, parent, childName):
        """Creates a child node and store it in the Scene nodes dictionary"""
        """ if parent is a list of Nodes, the child is created in the fist valid parent """
        """ and then added to every other valid parents """
        childNode = None
        if isinstance(parent, list): # we have a list of parent nodes
            for p in parent:
                if not p is None: # p is valid
                    if childNode is None: # childNode is not yet created
                        childNode = p.createChild( childName )
                    else:
                        p.addChild( childNode )
        else: # only one parent
            childNode = parent.createChild(childName)
        self.nodes[childName] = childNode
        return childNode

    def setMaterial(self, solid, material):
        """ assign material to solid
        """
        self.solidMaterial[solid]=material

    def setMaterialByTag(self, tag, material):
        """ assign material to all solids with tag
        """
        for solid in self.model.getSolidsByTags({tag}):
            self.solidMaterial[solid.id] = material

    def getMaterial(self, solid):
        """ return the solid material, "default" if none is defined
        """
        if solid in self.solidMaterial:
            return self.solidMaterial[solid]
        else :
            return "default"

    def getCollision(self,solidId,meshId):
        """ returns a collision object identified by solidId/meshId
        """
        Sofa.msg_deprecated("SofaPython.sml.BaseScene","getCollision is deprecated, access directly self.collisions[solidId][meshId]")
        mesh=None
        if hasattr(self, 'rigids'):  # inserted by Compliant.sml FIXME: parent class should not know child class !
            if solidId in self.rigids:
                if meshId in self.rigids[solidId].collisions:
                    mesh = self.rigids[solidId].collisions[meshId]
        if hasattr(self, 'collisions'):  # inserted by Anatomy.sml FIXME: parent class should not know child class !
            if solidId in self.collisions:
                if meshId in self.collisions[solidId]:
                    mesh = self.collisions[solidId][meshId]
        return mesh

    def getVisual(self,solidId,meshId):
        """ returns a visual object identified by solidId/meshId
        """
        Sofa.msg_deprecated("SofaPython.sml.BaseScene","getCollision is deprecated, access directly self.visuals[solidId][meshId]")
        mesh=None
        if hasattr(self, 'rigids'):  # inserted by Compliant.sml FIXME: parent class should not know child class !
            if solidId in self.rigids:
                if meshId in self.rigids[solidId].visuals:
                    mesh = self.rigids[solidId].visuals[meshId]
        if hasattr(self, 'visuals'):  # inserted by Anatomy.sml FIXME: parent class should not know child class !
            if solidId in self.visuals:
                if meshId in self.visuals[solidId]:
                    mesh = self.visuals[solidId][meshId]
        return mesh

    def getVisualsByTags(self, solidId=None, solidTags=set(), meshTags=set()):
        """ \return a selection of visual models, based on solidTags and meshTags
        an empty tag set means all solids or meshes
        """
        visuals = list()
        if not solidId is None and len(solidTags):
            Sofa.msg_warning("sml.Model.getVisualsByTags", "solidId and solidTags both specified, solidTags ignored")
        solids = None
        if not solidId is None:
            solids = [self.model.solids[solidId]]
        elif len(solidTags) :
            solids = self.model.getSolidsByTags(solidTags)
        else :
            solids = self.model.solids.values()
        for solid in solids:
            if not solid.id in self.visuals:
                if printLog:
                    Sofa.msg_info("SofaPython.sml", "No visual for solid "+solid.id)
                continue
            solidVisuals = self.visuals[solid.id]
            meshes = solid.getMeshesByTags(meshTags) if len(meshTags) else solid.mesh
            for mesh in meshes:
                if not mesh.id in solidVisuals:
                    if printLog:
                        Sofa.msg_info("SofaPython.sml", "No visual for solid "+solid.id+", mesh: "+mesh.id)
                    continue
                visual = solidVisuals[mesh.id]
                visual.mesh = mesh # make sure we known which mesh is attached to this visual
                visuals.append(visual)
        return visuals

    def addMeshExporters(self, dir, ExportAtEnd=False, solidId=None, solidTags=set(), meshTags=set()):
        """ add obj exporters to selected visual models,
        selection is done by solidTags and meshTags, empty set means all solids or meshes
        \todo make ExportAtEnd consistant with exportAtEnd data
        """
        for visual in self.getVisualsByTags(solidId, solidTags, meshTags):
            filename = os.path.join(dir, os.path.basename(visual.mesh.source))
            e = visual.node.createObject('ObjExporter', name='ObjExporter', filename=filename, printLog=True, exportMTL=False, exportAtEnd=ExportAtEnd)
            self.meshExporters.append(e)

    def addVisualStyles(self, displayFlags="", solidId=None, solidTags=set(), meshTags=set()):
        """ add visual styles to selected visual models,
        selection is done by solidTags and meshTags, empty set means all solids or meshes
        """
        for visual in self.getVisualsByTags(solidId, solidTags, meshTags):
            visual.node.createObject("VisualStyle", name="visualStyle", displayFlags=displayFlags)

    def setVisualStyles(self, displayFlags="showVisual", solidId=None, solidTags=set(), meshTags=set()):
        """ set visual tags to selected visual models,
        a visualStyle must have been already created
        selection is done by solidTags and meshTags, empty set means all solids or meshes
        """
        for visual in self.getVisualsByTags(solidId, solidTags, meshTags):
            vs = visual.node.getObject("visualStyle")
            if vs is None:
                Sofa.msg_warning("sml.BaseScene", "Missing VisualStyle component in "+visual.node.getPathName())
                continue
            vs.displayFlags=displayFlags
            if "showVisual" in displayFlags:
                visual.node.propagatePositionAndVelocity()

    def exportMeshes(self):
        for e in self.meshExporters:
            e.writeOBJ()

    def dagValidation(self):
        err = DAGValidation.test( self.root, True )
        if not len(err) is 0:
            Sofa.msg_error("SofaPython.sml","BaseScene: your DAG scene is not valid")
            for e in err:
                Sofa.msg_error("SofaPython.sml",e)

def getValueByTag(valueByTag, tags, noDefault=False):
    """ look into the valueByTag dictionary for a tag contained in tags
        \return the corresponding value, or the "default" value if none is found
        \todo print a warning if several matching tags are found in valueByTag
    """
    if "default" in tags:
        Sofa.msg_error("SofaPython.sml.getValueByTag", "default tag has a special meaning, it should not be defined in {0}".format(tags))
    tag = tags & set(valueByTag.keys())
    if len(tag)>1:
        Sofa.msg_warning("SofaPython.sml.getValueByTag", "sevaral tags from {0} are defined in values {1}".format(tags, valueByTag))
    if not len(tag)==0:
        return valueByTag[tag.pop()]
    elif noDefault:
        return None
    else:
        if "default" in valueByTag:
            return valueByTag["default"]
        else:
            Sofa.msg_error("SofaPython.sml.getValueByTag", "No default value, and no tag from {0} found in {1}".format(tags, valueByTag))
            return None

class SceneDisplay(BaseScene):
    """ Creates a scene to display solid meshes
    """
    def __init__(self,parentNode,model):
        BaseScene.__init__(self,parentNode,model)
        self.param.colorByTag=dict()
        self.param.colorByTag["default"]="1. 1. 1."

    def createScene(self):
        model=self.model # shortcut
        for solid in model.solids.values():
            if printLog:
                Sofa.msg_info("SofaPython.sml","SceneDisplay: Display solid:" + solid.name)
            color = solid.getValueByTag(self.param.colorByTag)
            insertVisual(self.node, solid, color)
