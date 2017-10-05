
from types import *

from SofaPython.SofaNumpy import as_numpy, edit_data

def numpify(obj, name):
    return as_numpy( obj.findData(name), False )


def numpy_property(attr_name, attr_data, index = 0):
    '''easily bind data properties as attributes'''

    def getter(self):
        obj = getattr(self, attr_name)
        return numpify(obj, attr_data)[index]

    def setter(self, value):
        obj = getattr(self, attr_name)
        numpify(obj, attr_data)[index] = value
        
    return property(getter, setter)



class DOFs(object):
    '''state vector view for a mechanical object'''
    
    __slots__ = ('dofs', )
    
    def __init__(self, dofs):
        self.dofs = dofs
        
    @property
    def position(self):
        return numpify(self.dofs, 'position')
    
    @position.setter
    def position(self, value):
        self.position[:] = value


    @property
    def velocity(self):
        return numpify(self.dofs, 'velocity')
    
    @velocity.setter
    def velocity(self, value):
        self.velocity[:] = value


    @property
    def force(self):
        return numpify(self.dofs, 'force')
    
    @force.setter
    def force(self, value):
        self.force[:] = value


    @property
    def external_force(self):
        return numpify(self.dofs, 'externalForce')
    
    @external_force.setter
    def external_force(self, value):
        self.external_force[:] = value

        
    @property
    def template(self):
        return self.dofs.getTemplateName()


    @property
    def show(self):
        return self.dofs.showObject

    @show.setter
    def show(self, value):
        self.dofs.showObject = value


    @property
    def context(self):
        return self.dofs.getContext()


def single_dof(cls):
    '''decorator for single dof views'''

    class Result(cls):
        __slots__ = ()

        @cls.position.getter
        def position(self):
            return super(Result, self).position[0]


        @cls.velocity.getter
        def velocity(self):
            return super(Result, self).velocity[0]        


        @cls.force.getter
        def force(self):
            return super(Result, self).force[0]        


        @cls.external_force.getter
        def external_force(self):
            return super(Result, self).external_force[0]            
        
    return Result


@single_dof
class DOF(DOFs):
    '''single dof view'''
    pass



def dofs_type(coord_type, deriv_type = None):
    '''gives properly typed access to decorated class'''
    
    deriv_type = deriv_type or coord_type.Deriv
    
    def decorator(cls):

        # TODO maybe just add these to the class instead
        class Result(cls):
            __slots__ = ()

            @cls.position.getter
            def position(self):
                return super(Result, self).position.view(coord_type)


            @cls.velocity.getter
            def velocity(self):
                return super(Result, self).velocity.view(deriv_type)  


            @cls.force.getter
            def force(self):
                return super(Result, self).force.view(deriv_type)       


            @cls.external_force.getter
            def external_force(self):
                return super(Result, self).external_force.view(deriv_type)
        
        return Result

    return decorator





@dofs_type(Rigid3)
@single_dof
class MappedRigid(DOFs):
    '''rigid dof mapped from another rigid'''
    
    __slots__ = 'mapping', 'parent'
    
    def __init__(self, dofs, mapping):
        DOFs.__init__(self, dofs)
        self.mapping = mapping
        assert mapping.getClassName() == 'AssembledRigidRigidMapping'

    @property
    def offset(self):
        import ctypes
        ptr, shape, typename = self.mapping.findData('source').getValueVoidPtr()
        array = (ctypes.c_double * 7).from_address(ptr)

        return np.ctypeslib.as_array( array ).view(Rigid3)

    @offset.setter
    def offset(self, value):
        self.offset[:] = value
        
    @property
    def parent_context(self):
        return self.context.getParents()[0]
    

@dofs_type(Rigid3)
@single_dof
class RigidDOF(DOFs):
    '''rigid single dof view'''
    
    def map_rigid(self, name):
        node = self.context.createChild(name)

        dofs = node.createObject('MechanicalObject', template = 'Rigid3', size = 1)
        mapping = node.createObject('AssembledRigidRigidMapping', template = 'Rigid3,Rigid3',
                                    input = self.dofs.getLinkPath(),
                                    output = dofs.getLinkPath(),
                                    source = '0 0 0 0 0 0 0 1')
        
        return MappedRigid(dofs, mapping)

    # TODO map_points
    



class RigidBody(RigidDOF):
    '''a rigid body with single dof'''

    @property
    def total_mass(self):
        return self.mass.mass[0]

    @total_mass.setter
    def total_mass(self, value):
        self.mass.mass = value


    @property
    def inertia(self):
        return self.mass.inertia[0]

    @inertia.setter
    def inertia(self, value):
        self.mass.inertia = value

    @property
    def inertia_forces(self):
        return self.mass.inertia.inertia_forces

    @inertia_forces.setter
    def inertia_forces(self, value):
        self.mass.inertia_forces = value


    
    
