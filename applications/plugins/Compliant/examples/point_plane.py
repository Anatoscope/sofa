from Compliant import StructuralAPI as api

class Script(api.Script):

    def onEndAnimationStep(self, dt):
        print(self.dofs.force)




def createScene(node):

    node.createObject('RequiredPlugin', pluginName = 'Compliant')

    ode = node.createObject('CompliantImplicitSolver') # , constraint_forces = 'propagate')
    ode.debug = True
    
    num = node.createObject('SequentialSolver', iterations = 5, precision = 0)
    num.debug = True
    
    # a point mass
    point = node.createChild('point')
    
    dofs =point.createObject('MechanicalObject', template = 'Vec3', showObject = True, drawMode = 1)
    point.createObject('UniformMass', template = 'Vec3', totalMass = 1)
    

    # compute distance to a plane
    dist = point.createChild('plane_distance')
    dist.createObject('MechanicalObject', template = 'Vec1')

    # compute x^T u - (-1)
    dist.createObject('ProjectionMapping', template = 'Vec3,Vec1',
                      set = '0   0 1 0', # source dof index, then projection vector
                      offset = '-1')     # offset substracted from the projection
    
    # we want x^T u + 1 > 0
    dist.createObject('UnilateralConstraint')
    
    # how stiff is the constraint?
    dist.createObject('UniformCompliance', template = 'Vec1', compliance = 0)

    script = Script(node.createChild('post'))
    script.dofs = dofs
