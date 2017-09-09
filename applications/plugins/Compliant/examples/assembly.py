

def vec3_dofs(node, **kwargs):
    return node.createObject('MechanicalObject', **kwargs)

def vec3_mass(node, **kwargs):
    return node.createObject('UniformMass', **kwargs)



def createScene(node):

    node.createObject('simple_solver')
    node.createObject('LDLTSolver')

    
    p1 = node.createChild('p1')
    n1 = vec3_dofs(p1)
    m1 = vec3_mass(p1)

    p2 = node.createChild('p2')
    n2 = vec3_dofs(p2)
    m2 = vec3_mass(p2, mass = 2)
    

    d1 = node.createChild('d1')
    n = vec3_dofs(d1)

    p1.addChild(d1)
    p2.addChild(d1)
    
    c = d1.createObject('UniformCompliance', compliance = 1, isCompliance = True)

    m = d1.createObject('DifferenceMultiMapping',
                        input = '{0} {1}'.format(n1.getLinkPath(),
                                                 n2.getLinkPath()),
                        output = n.getLinkPath())
    
    
    from SofaPython import console
    console.start( locals() )
    
    
