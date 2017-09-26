

def vec3_dofs(node, **kwargs):
    show = kwargs.pop('show', False)
    
    res =  node.createObject('MechanicalObject', size = 1, template = 'Vec3d',  **kwargs)

    if show:
        res.drawMode = 2
        res.showObject = True
    
    return res

def vec3_mass(node, **kwargs):
    return node.createObject('UniformMass', **kwargs)


import Sofa
Sofa.loadPlugin('Compliant')

def createScene(node):
    node.createObject('CompliantAttachButtonSetting')
    
    node.createObject('simple_solver')
    # node.createObject('CompliantImplicitSolver')
    
    node.createObject('LDLTSolver', schur = False)

    p1 = node.createChild('p1')
    n1 = vec3_dofs(p1, position = [[-1, 0, 0]], show = True)
    m1 = vec3_mass(p1)
    p1.createObject('FixedConstraint', indices = [0, 1])
    
    p2 = node.createChild('p2')
    n2 = vec3_dofs(p2, position = [[1, 0, 0]], show = True)
    m2 = vec3_mass(p2, mass = 2)
    

    d1 = node.createChild('d1')
    n = vec3_dofs(d1)

    p1.addChild(d1)
    p2.addChild(d1)
    
    c = d1.createObject('UniformCompliance', compliance = 1e-2, isCompliance = True)

    m = d1.createObject('DifferenceMultiMapping',
                        input = '{0} {1}'.format(n1.getLinkPath(),
                                                 n2.getLinkPath()),
                        output = n.getLinkPath())
    
    
    from SofaPython import console
    console.start( locals() )
    
    node.dt = 1e-5
    node.animate = False
    
