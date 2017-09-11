import Sofa

import sys
import os.path
import math
import xml.etree.cElementTree as etree

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
    if not objXml.find("name") is None:
        obj.name = objXml.find("name").text
    else:
        obj.name = obj.id

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
    if tags is None:
        return objects
    taggedObjects = list()
    for obj in objects:
        if len(obj.tags & tags) > 0:
            taggedObjects.append(obj)
    return taggedObjects

class Model(object):
    """ This class stores a Sofa Model read from a sml/xml (Sofa Modelling Language) file.
    """

    class Mesh(object):

        class Group(object):
            def __init__(self, id):
                self.id=id
                # indices of the vertices in this group
                self.index=list()
                # optionnaly, data vectors can be associated to these vertices (for instance skinning weight)
                self.data=dict()
                # tags to classify this group
                self.tags=set()
            def __len__(self):
                return len(self.index)

        def __init__(self, meshXml=None):
            self.id=None
            self.name=None
            self.format=None
            self.source=None
            # vertex groups defined on this mesh
            self.groups=dict()
            self.group=self.groups # for compatibility
            if not meshXml is None:
                self.parseXml(meshXml)

        def parseXml(self, meshXml):
            parseIdName(self,meshXml)

            if not meshXml.find("source") is None:
                self.format = meshXml.find("source").attrib["format"]
                self.source = meshXml.find("source").text

            for g in meshXml.findall("group"):
                self.group[g.attrib["id"]] = Model.Mesh.Group(id=g.attrib["id"])
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
            
        def getGroupsByTags(self, tags):
            """ \return a list of groups which contains at least one tag from tags
            """
            return _getObjectsByTags(self.groups.values(), tags)

    class MeshAttributes(object):
        """ This class contains attributes for a given mesh associated to a given solid.
        """
        def __init__(self,objXml=None):
            self.mesh = None
            # is this mesh a collision mesh for this solid
            self.collision=True
            # is this mesh to be simulated for this solid (depends on the sml moulinette)
            self.simulation=True
            # is this mesh a visual mesh for this solid
            self.visual=True
            # other characteristics of how this mesh participates to this solid simulation
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

    class Image(object):
        def __init__(self, imageXml=None):
            self.format=None
            self.source=None
            if not imageXml is None:
                self.parseXml(imageXml)

        def parseXml(self, imageXml):
            parseIdName(self,imageXml)
            self.format = imageXml.find("source").attrib["format"]
            self.source = imageXml.find("source").text

    class Solid(object):
        
        def __init__(self, solidXml=None):
            # id, used to retrieve this solid
            self.id = None
            # if specified, a user friendly name, default to id
            self.name = None
            # set of tags of this solid, to be used in sml moulinette to know what to do with this solid
            self.tags = set()
            # solid position [X,q], default to origin
            self.position = [0,0,0,0,0,0,1]
            self.keyPositions = {} # optional animated keyframed positions {name(string):position(6 floats)}, note 'name' can represent a time (that would need to be casted as a float in your sml moulinette)
            # list of meshes which participate to this solid
            self.meshes = list()
            self.mesh = self.meshes # alias for compatibility
            # for each mesh.id, its attributes associated to this solid
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
            self.meshes.append(mesh)
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
            """ \return a list of meshes which contains at least one tag from tags
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

    class Offset(object):
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

    class Dof(object):
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

    class JointGeneric(object):
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

        def symmetrize(self,plane_center,plane_normal):
            Sofa.msg_warning('SofaPython.sml','JointGeneric.symmetrize is not yet implemented')
            # not trivial, need to check which dof, to select important axis, and be careful about limits...
            # but doable

    _joint_type_from_tag = {'jointGeneric':JointGeneric } # todo how to makes this automatic?

    class JointSpecific(JointGeneric):
        """ @internal
            convenience class to build a JointGeneric based on a center and a direction
            only for a selected subset of joint types, always based on x-axis of the offsets
            These joints are symmetrizable.
        """

        def __init__(self, jointXml=None):
            self.center = None
            self.direction = [1, 0, 0]
            Model.JointGeneric.__init__(self, jointXml)

        @staticmethod
        def joint_quaternion(u, n):
            import numpy as np
            import Quaternion
            
            # basis vector
            e = np.identity(3)[u]
            
            return Quaternion.from_vectors(e, n)
            

            ## mtournier: ????
            ## returns a quaternion that orients the u-th axis to the given axis n
            # build an orthogonal basis as a column matrix and convert it to a quaternion
            v=(u+1)%3
            w=(v+1)%3
            import numpy as np
            from numpy.linalg import norm
            m=np.zeros((3,3))
            m[:,u]=n
            m[:,u] = m[:,u]/norm(m[:,u]) # normalize to be sure
            threshold = 1e-8 # TODO what is the right way to get numeric limits in python?
            if   abs(n[1]) > threshold: m[:,v] = [0, -n[2], n[1]]
            elif abs(n[2]) > threshold: m[:,v] = [n[2], 0, -n[0]]
            elif abs(n[0]) > threshold: m[:,v] = [n[1], -n[0], 0]
            else: raise Exception("Null normal")
            m[:,v] = m[:,v]/norm(m[:,v]) # normalize
            m[:,w] = np.cross(m[:,u],m[:,v])
            import Quaternion
            return Quaternion.from_matrix(m)


        def parseXml(self, jointXml):
            parseIdName(self, jointXml)
            parseTag(self, jointXml)
            solidsRef = jointXml.findall("jointSolidRef")
            for i in range(0,2):
                if not solidsRef[i].find("offset") is None:
                    Sofa.msg_warning("SofaPython.sml","JointSpecific: offets are undesirable and won't be used.")
                self.offsets[i] = Model.Offset()
                self.offsets[i].name = "offset_{0}".format(self.name)
            for dof in jointXml.findall("dof"):
                Sofa.msg_warning("SofaPython.sml","JointSpecific: dof are undesirable and won't be used.")
            self.center = Tools.strToListFloat(jointXml.attrib["center"])
            if "direction" in jointXml.attrib:
                self.direction = Tools.strToListFloat(jointXml.attrib["direction"])
            self.update()


        def update(self):
            for off in self.offsets:
                off.value = self.center + Model.JointSpecific.joint_quaternion(0, self.direction) 
                
        def symmetrize(self,plane_center,plane_normal):
            # based on specific joint axis, the symmetrization should always work
            import numpy as np
            from numpy.linalg import norm
            plane_center = np.asarray(plane_center)
            plane_normal = np.asarray(plane_normal); plane_normal=plane_normal/norm(plane_normal) # normalize to be sure
            self.center = Tools.planarSymmetrization( np.asarray(self.center), plane_center, plane_normal ).tolist()
            self.direction = Tools.planarSymmetrization(np.asarray(self.direction), np.array([0, 0, 0]), plane_normal).tolist()
            self.update()


    class JointHinge(JointSpecific):

        def __init__(self, jointXml=None):
            Model.JointSpecific.__init__(self, jointXml)

        def parseXml(self, jointXml):
            Model.JointSpecific.parseXml(self, jointXml)
            dof = Model.Dof(); dof.index = Model.dofIndex['rx']; self.dofs = [dof]
            if "angle_min" in jointXml.attrib and "angle_max" in jointXml.attrib:
                self.dofs[0].min = math.radians(float(jointXml.attrib["angle_min"]))
                self.dofs[0].max = math.radians(float(jointXml.attrib["angle_max"]))

    _joint_type_from_tag['jointHinge'] = JointHinge


    class JointSlider(JointSpecific):

        def __init__(self, jointXml=None):
            Model.JointSpecific.__init__(self, jointXml)

        def parseXml(self, jointXml):
            Model.JointSpecific.parseXml(self, jointXml)
            dof = Model.Dof(); dof.index = Model.dofIndex['x']; self.dofs = [dof]
            if "min" in jointXml.attrib and "max" in jointXml.attrib:
                self.dofs[0].min = float(jointXml.attrib["min"])
                self.dofs[0].max = float(jointXml.attrib["max"])


    _joint_type_from_tag['jointSlider'] = JointSlider

    class JointCylindrical(JointSpecific):

        def __init__(self, jointXml=None):
            Model.JointSpecific.__init__(self, jointXml)

        def parseXml(self, jointXml):
            Model.JointSpecific.parseXml(self, jointXml)
            dof_t = Model.Dof(); dof_t.index = Model.dofIndex['x']; self.dofs = [dof_t]
            dof_r = Model.Dof(); dof_r.index = Model.dofIndex['rx']; self.dofs.append(dof_r)
            if "min" in jointXml.attrib and "max" in jointXml.attrib:
                self.dofs[0].min = float(jointXml.attrib["min"])
                self.dofs[0].max = float(jointXml.attrib["max"])
            if "angle_min" in jointXml.attrib and "angle_max" in jointXml.attrib:
                self.dofs[1].min = math.radians(float(jointXml.attrib["angle_min"]))
                self.dofs[1].max = math.radians(float(jointXml.attrib["angle_max"]))

    _joint_type_from_tag['jointCylindrical'] = JointCylindrical

    class JointSpherical(JointSpecific):
        def __init__(self, jointXml=None):
            Model.JointSpecific.__init__(self, jointXml)

        def parseXml(self, jointXml):
            Model.JointSpecific.parseXml(self, jointXml)
            dof_x = Model.Dof(); dof_x.index = Model.dofIndex['rx']; self.dofs = [dof_x]
            dof_y = Model.Dof(); dof_y.index = Model.dofIndex['ry']; self.dofs.append(dof_y)
            dof_z = Model.Dof(); dof_z.index = Model.dofIndex['rz']; self.dofs.append(dof_z)

    _joint_type_from_tag['jointSpherical'] = JointSpherical



    class Skinning(object):
        """ Skinning definition, vertices index influenced by bone with weight
        """
        def __init__(self):
            self.solid = None    # id of the parent bone # WARNING it rather seems to be directly a pointer to the Solid
            self.mesh = None     # the target mesh
            self.index = list()  # indices of target mesh
            self.weight = list() # weights for these vertices with respect with this bone

    class Surface(object):
        def __init__(self):
            self.solid = None # a Model.Solid object
            self.mesh = None  # a Model.Mesh object
            self.group = None # the name of the group defined in the mesh

    class SurfaceLink(object):
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
        # where the (last) .sml was read, all relative path in sml files (mesh files) are understood from this path
        self.modelDir = None
        self.units = dict()
        # all meshes of this model (whether or not they are included in a solid), by mesh.id
        self.meshes = dict()
        self.images = dict()
        self.solids = dict()
        self.genericJoints = dict()
        self.surfaceLinks = dict()

        if not filename is None:
            self.open( filename )

    def open(self, filename, checkMeshFilePresence=True):
        """ parse \a filename and append its content to the current model
        """
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
                        elif checkMeshFilePresence :
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
                    if not "solid" in s.attrib: # required
                        Sofa.msg_error("SofaPython.sml", "Missing solid attribute in surface definition - surfaceLink: "+surfaceLink.id)
                        continue
                    if not "mesh" in s.attrib: # required
                        Sofa.msg_error("SofaPython.sml", "Missing mesh attribute in surface definition - surfaceLink: "+surfaceLink.id)
                        continue
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

        for k in Model._joint_type_from_tag.iterkeys():
            for j in modelXml.findall(k):

                if j.attrib["id"] in self.genericJoints:
                    Sofa.msg_error("SofaPython.sml","Model: jointGeneric defined twice, id:", j.attrib["id"])
                    continue

                joint = Model._joint_type_from_tag[j.tag](j)

                solids = j.findall("jointSolidRef")
                for i, o in enumerate(solids):
                    if o.attrib["id"] in self.solids:
                        joint.solids[i] = self.solids[o.attrib["id"]]
                    else:
                        Sofa.msg_error("SofaPython.sml","Model: in joint {0}, unknown solid {1} referenced".format(joint.name, o.attrib["id"]))
                self.genericJoints[joint.id]=joint


    def _setMeshDirectory(self, path, mesh):
        if not mesh.source is None:
            mesh.source = os.path.join(path, os.path.basename(mesh.source))
            if not os.path.exists(mesh.source):
                Sofa.msg_warning("SofaPython.sml","Model: mesh not found: "+mesh.source)

    def setMeshDirectoryPath(self, path, solidId=None, solidTags=None, meshTags=None):
        """ Change all the meshes directory path to be path. If solidId is given, only meshes part of this solid get their path updated
        If path is not absolute, model directory is appended to it
        """
        _path = None
        if os.path.isabs(path):
            _path = path
        else:
            _path = os.path.join(self.modelDir, path)
        meshesIter = None

        if not solidId is None:
            for mesh in self.solids[solidId].getMeshesByTags(meshTags):
                self._setMeshDirectory(_path, mesh)
        elif not solidTags is None or not meshTags is None:
            for solid in self.getSolidsByTags(solidTags):
                for mesh in solid.getMeshesByTags(meshTags):
                    self._setMeshDirectory(_path, mesh)
        else:
            for mesh in self.meshes.itervalues():
                self._setMeshDirectory(_path, mesh)

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

class BaseScene(object):
    """ Base class for Scene class, creates a node for this Scene
    """
    class Param(object):
        """ Empty class to store parameters (dynamically created)
        \todo add some serialization if needed
        \todo add a virtual method check() that would be called in __init__() to check parameters consistency
        """
        pass

    def __init__(self,parentNode,model,name=None):
        self.root = parentNode
        self.model = model
        # to store scene parameters
        self.param = BaseScene.Param()
        # a default material set
        self.material = Tools.Material()
        # assign a material to a solid
        self.solidMaterial = dict()
        # to store special nodes and have access to them
        self.nodes = dict()
        # optional components to exports meshes
        self.meshExporters = list()
        # to store collisions of the scene by [solidId][meshId]
        self.collisions = dict()
        # to store visuals of the scene by [solidId][meshId]
        self.visuals = dict()

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
        Sofa.msg_deprecated("SofaPython.sml.BaseScene","getVisual is deprecated, access directly self.visuals[solidId][meshId]")
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

    def setVisualColors(self, color, solidId=None, solidTags=set(), meshTags=set()):
        """ set colors of selected visual models,
        selection is done by solidTags and meshTags, empty set means all solids or meshes
        """
        for visual in self.getVisualsByTags(solidId, solidTags, meshTags):
            visual.visual.setColor(color[0],color[1],color[2],color[3])

    def exportMeshes(self):
        '''make all meshExporters export their meshes'''
        
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
