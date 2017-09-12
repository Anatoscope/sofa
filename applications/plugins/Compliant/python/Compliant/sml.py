import os.path
import math
import xml.etree.cElementTree as etree
import sys

from Compliant import StructuralAPI

import SofaPython.Tools
import SofaPython.units
from SofaPython import Quaternion
from SofaPython.Tools import listToStr as concat
import SofaPython.mass
import SofaPython.sml
import Compliant.StructuralAPI
import Flexible.API
import Flexible.sml

import Sofa

printLog = True


def log_info(*args): Sofa.msg_info("Compliant.sml", ' '.join(args))
def log_warning(*args): Sofa.msg_warning("Compliant.sml", ' '.join(args))


def model_mass_info(model):
    '''construct a mass info from a rigid model with all the needed info'''
    assert model.mass is not None and model.com is not None and model.inertia is not None
    mi = SofaPython.mass.RigidMassInfo()

    mi.mass = model.mass # TODO: convert units ?
    mi.com = model.com # TODO: convert units ?

    rotation = model.inertia_rotation
    if rotation is None: rotation = [0, 0, 0, 1]
    
    assert len(model.inertia) in [3, 6]

    if len(model.inertia) == 6:
        mi.setFromInertia(*rigidModel.inertia)
    elif model.inertia_rotation is not None:
        mi.diagonal_inertia = model.inertia
        mi.inertia_rotation = rotation
        
    return mi

        
def rigid_mass_info_offset(rigidModel, density, scale):
    '''compute mass info and offset from rigid model'''
    if not rigidModel.mass is None and not rigidModel.com is None and not rigidModel.inertia is None:
        # sml model has all the info needed
        if scale != 1: log_info('scale is not supported in that case')
        return model_mass_info(rigidModel), rigidModel.position
    
    elif rigidModel.mesh:
        # there is a mesh: compute from it
        mi = rigidModel.getRigidMassInfo(density, scale)
        offset = StructuralAPI.scaleOffset(scale, rigidModel.position)
        return mi, offset
    else:
        # default
        log_warning("using default rigid mass distribution")        
        mi = SofaPython.mass.RigidMassInfo()
        mi.mass = rigidModel.mass if rigidModel.mass is not None else SofaPython.units.mass_from_SI(1.)
        mi.inertia = [scale * scale] * 3
        mi.com = scale * rigidModel.position # TODO does this even work?

        return mi, None
        
    
def insertRigid(parentNode, rigidModel, density, scale=1, param=None, color=[1,1,1,1]):
    """ create a StructuralAPI.RigidBody from the rigidModel. The model geometry is scaled with scale.
    The rigidMass is computed from:
        1) mass, com and inertia if present
        2) mesh if possible
        3) default to a unit sphere TODO: is it relevant to do so ?
    """

    if printLog:
        log_info("insertRigid", rigidModel.name)

        for mesh in rigidModel.mesh:
            if rigidModel.meshAttributes[mesh.id].collision:
                log_info("\tcollision mesh:", mesh.name)
            
    rigid = StructuralAPI.RigidBody(parentNode, rigidModel.name)

    # mass
    mi, offset = rigid_mass_info_offset(rigidModel, density, scale)
    if offset is None:
        rigid.setManually(mi.com, mi.mass, mi.inertia)        
    else:
        rigid.setFromRigidInfo(mi, offset = offset, inertia_forces = 0)

    # drawing
    if not param is None:
        rigid.dofs.showObject = param.showRigid
        rigid.dofs.showObjectScale = SofaPython.units.length_from_SI(param.showRigidScale)


    # add collision/visual meshes (visual points to collision when
    # possible)
    rigid.collisions = {}
    rigid.visuals = {}
    
    for mesh in rigidModel.mesh :
        if rigidModel.meshAttributes[mesh.id].collision:
            rigid.collisions[mesh.id] = rigid.addCollisionMesh(mesh.source,name_suffix='_'+mesh.name, 
                                                               scale3d=[scale] * 3)
            if rigidModel.meshAttributes[mesh.id].visual:
                rigid.visuals[mesh.id] = rigid.collisions[mesh.id].addVisualModel(color=color)
                
        elif rigidModel.meshAttributes[mesh.id].visual:
            rigid.visuals[mesh.id] = rigid.addVisualModel(mesh.source,name_suffix='_'+mesh.name,
                                                          scale3d=[scale]*3, color=color)
            
    return rigid



def insertJoint(jointModel, rigids, scale=1, param=None):
    """ create a StructuralAPI.GenericRigidJoint from the jointModel """

    frames=list()
    for i,offset in enumerate(jointModel.offsets):
        if not jointModel.solids[i].id in rigids:
            log_warning("insertJoint "+jointModel.name+" failed: "+jointModel.solids[i].id+" is not a rigid body")
            return None
        rigid = rigids[jointModel.solids[i].id] # shortcut
        if rigid is None:
            log_warning("in joint {0}, solid {1} is missing, ignored".format(jointModel.name, jointModel.solids[i].id))
            return
        if not offset is None:
            if offset.isAbsolute():
                frames.append( rigid.addAbsoluteOffset(offset.name, StructuralAPI.scaleOffset(scale,offset.value)) )
            else:
                frames.append( rigid.addOffset(offset.name, StructuralAPI.scaleOffset(scale,offset.value)) )
            if not param is None:
                frames[-1].dofs.showObject = param.showOffset
                frames[-1].dofs.showObjectScale = SofaPython.units.length_from_SI(param.showOffsetScale)
        else:
            frames.append(rigid)

    if printLog:
        log_info("insertJoint "+jointModel.name)

    mask = [1]*6
    limits=[] # mask for limited dofs
    isLimited = True # does the joint have valid limits?
    for d in jointModel.dofs:
        if isLimited:
            if d.min==None or d.max==None:
                isLimited = False # as soon as a limit is not defined, the limits cannot work
            else:
                if d.index < 3: # translation dofs have to be scaled based on units
                    limits.append(d.min*scale)
                    limits.append(d.max*scale)
                else: # rotation dofs do not need to be scaled
                    limits.append(d.min)
                    limits.append(d.max)

        mask[d.index] = 0

    joint = StructuralAPI.GenericRigidJoint(jointModel.name, frames[0].node, frames[1].node, mask,
                 compliance=SofaPython.sml.getValueByTag(param.jointComplianceByTag, jointModel.tags),
                 isCompliance=SofaPython.sml.getValueByTag(param.jointIsComplianceByTag, jointModel.tags))
    if isLimited:
        joint.addLimits(limits)
    return joint

class SceneArticulatedRigid(SofaPython.sml.BaseScene):
    """ Builds a (sub)scene from a model using compliant formulation
    [tag] rigid are simulated as RigidBody, more tags can be added to param.rigidTags
    Compliant joints are setup between the rigids """
    
    def __init__(self, parentNode, model):
        SofaPython.sml.BaseScene.__init__(self, parentNode, model)
        
        self.rigids = dict()
        self.joints = dict()

        # the set of tags simulated as rigids
        self.param.rigidTags={"rigid"}

        self.param.geometricStiffness=0 # TODO doc on the possible values !
        # for tagged joints, values come from these dictionnaries if they contain one of the tag
        self.param.jointIsComplianceByTag=dict()
        self.param.jointComplianceByTag=dict()

        # default joint is set up using isCompliance=True and self.param.jointCompliance value
        self.param.jointIsComplianceByTag["default"]=True
        self.param.jointComplianceByTag["default"]=0 # TODO add 2 default values: for translation and rotation dofs ?

        # specify the length unit into with the simulated model should be converted
        # hm, dam, m, dm, cm, mm
        # if None, no conversion is applied
        self.param.simuLengthUnit=None
        # internal variable to store the geometric scale factor to be applied on the model
        # to obtain the specified simuLengthUnit
        self._geometricScale=1

        self.param.showRigid=False
        self.param.showRigidScale=0.5 # SI unit (m)
        self.param.showOffset=False
        self.param.showOffsetScale=0.1 # SI unit (m)    

        self.param.color={"default": [1,1,1,1]}

    def insertMergeRigid(self, mergeNodeName="dofRigid", tags=None, rigidIndexById=None ):
        """ Merge all the rigids in a single MechanicalObject using a SubsetMultiMapping
        optionnaly give a list of tags to select the rigids which are merged
        return the created node"""
        mergeNode = None
        currentRigidIndex=0
        input=""
        indexPairs=""
        if tags is None:
            _tags = self.param.rigidTags
        else:
            _tags = tags

        for solid in self.model.getSolidsByTags(_tags):
            if not solid.id in self.rigids:
                log_warning("SceneArticulatedRigid.insertMergeRigid: "+solid.name+" is not a rigid")
                continue
            rigid = self.rigids[solid.id]
            if mergeNode is None:
                mergeNode = rigid.node.createChild(mergeNodeName)
            else:
                rigid.node.addChild(mergeNode)
            input += '@'+rigid.node.getPathName()+" "
            indexPairs += str(currentRigidIndex) + " 0 "
            if not rigidIndexById is None:
                rigidIndexById[solid.id]=currentRigidIndex
            currentRigidIndex+=1
        if input:
            mergeNode.createObject("MechanicalObject", template = "Rigid3", name="dofs")
            mergeNode.createObject('SubsetMultiMapping', template = "Rigid3,Rigid3", name="mapping", input = input , output = '@./', indexPairs=indexPairs, applyRestPosition=True )
        else:
            log_warning("insertMergeRigid: no rigid merged")
        return mergeNode

    def createScene(self):
        self.node.createObject('RequiredPlugin', name = 'Flexible' )
        self.node.createObject('RequiredPlugin', name = 'Compliant' )

        Compliant.StructuralAPI.geometric_stiffness = self.param.geometricStiffness

        # if there is no 'Rigids' node, just use our node as a parent for all rigids
        # self.nodes["Rigids"] can be set in sub-moulinette to set rigids parent node
        if not "Rigids" in self.nodes:
            self.nodes["Rigids"] = self.node

        if not self.param.simuLengthUnit is None:
            self._geometricScale = eval("SofaPython.units.length_"+self.model.units["length"]) / eval("SofaPython.units.length_"+self.param.simuLengthUnit)
            SofaPython.units.local_length = eval("SofaPython.units.length_"+self.param.simuLengthUnit)

        # rigids
        for rigidModel in self.model.getSolidsByTags(self.param.rigidTags):
            rigid =  insertRigid(self.nodes["Rigids"], rigidModel, self.material.density(self.getMaterial(rigidModel.id)), self._geometricScale, self.param, rigidModel.getValueByTag(self.param.color))
            self.rigids[rigidModel.id] = rigid
            self.visuals[rigidModel.id] = rigid.visuals
            self.collisions[rigidModel.id] = rigid.collisions
        
        # joints
        for jointModel in self.model.genericJoints.values():
            self.joints[jointModel.id] = insertJoint(jointModel, self.rigids, self._geometricScale, self.param)


class SceneSkinning(SceneArticulatedRigid) :
    """ Build a (sub-)scene based on SceneArticulatedRigid, add solids with skinning using the defined armature
    [tag] armature are simulated as RigidBody and used as bones for skinning
    """
    
    def __init__(self, parentNode, model):
        SceneArticulatedRigid.__init__(self, parentNode, model)
        self.deformables = dict()
        self.skinningArmatureBoneIndexById = dict() # keep track of bone armature index in the armature merge node

    def createScene(self):

        self.param.rigidTags.add("armature")

        SceneArticulatedRigid.createScene(self)
        
        # insert node containing all bones of the armature
        self.nodes["armature"] = self.insertMergeRigid(mergeNodeName="armature", tags={"armature"}, rigidIndexById=self.skinningArmatureBoneIndexById)
        for solidModel in self.model.solids.values():
            print solidModel.name, len(solidModel.skinnings)
            if len(solidModel.skinnings)>0: # ignore solid if it has no skinning
                # for each mesh create a Flexible.API.Deformable
                for mesh in solidModel.mesh:
                    # take care only of visual meshes with skinning
                    if solidModel.meshAttributes[mesh.id].visual:
                        deformable = Flexible.API.Deformable(self.nodes["armature"], solidModel.name+"_"+mesh.name)
                        deformable.loadMesh(mesh.source)
                        deformable.addMechanicalObject()
                        (indices, weights) = Flexible.sml.getSolidSkinningIndicesAndWeights(solidModel, self.skinningArmatureBoneIndexById)
                        deformable.addSkinning(self.nodes["armature"], indices.values(), weights.values())
                        deformable.addVisual()
                        self.deformables[mesh.id] = deformable

#                        self.deformables[solidModel.id]=Flexible.API.Deformable(self.nodes["armature"], solidModel.name)
#                    self.deformables[solidModel.id]=Flexible.sml.insertDeformableWithSkinning(self.node, solidModel, self.nodes["armature"].getPathName(), bonesId)
        
        
