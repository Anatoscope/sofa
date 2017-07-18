import os.path

import Sofa
import numpy
import SofaPython.sml
import SofaPython.script
import Compliant.sml
import RigidScale.sml
import RigidScale.tools


def createScene(node):
    
    node.dt = 0.01
    node.gravity = '0 0 0'
    
    node.createObject('RequiredPlugin', pluginName='image')
    node.createObject('RequiredPlugin', pluginName='Flexible')
    node.createObject('RequiredPlugin', pluginName='Compliant')
    node.createObject('RequiredPlugin', pluginName='RigidScale')
    
    model = SofaPython.sml.Model(os.path.join(os.path.dirname(__file__), "armature.sml"))
    
    scene = RigidScale.sml.SceneSkinningRigidScale(node, model)
    scene.param.showAffine = True
    scene.param.showAffineScale = 1
    scene.param.showOffset = True
    scene.createScene()

    controller = MoveController(node, "moveController")
    controller.armature = RigidScale.tools.Armature(model,scene)

    return node

class MoveController(SofaPython.script.Controller) :
    node = None
    armature = None
            
    def initGraph(self, node):
        self.node = node

    def setTranslation(self, translation):
        self.armature.translate("Bone", translation)
        self.node.propagatePositionAndVelocity()
        return

    def setRotation(self, rotation):
        radian = numpy.array(rotation)*3.14/180
        for bone in self.armature.scene.rigidScales:
            self.armature.rotate(bone, radian.tolist())
        self.node.propagatePositionAndVelocity()
        return

    def setScale(self, scale):
        self.armature.scale("Bone", scale)
        self.node.propagatePositionAndVelocity()
        return
        


