import Sofa
import SofaPython.SofaNumpy
import sys


def createSceneAndController(node):

    mo = node.createObject("MechanicalObject",position="1 2 3 4 5 6")

    # regular Data accessor performs a copy in a list
    pos_copy_in_list = mo.position
    print type(pos_copy_in_list)

    # since it is a copy, it is not possible to directly change a value :(
    print( "before:",mo.position[0][0] )
    mo.position[0][0] = 123
    print( "after:",mo.position[0][0] )


    # accessing the Data as a numpy array is sharing c++ memory
    pos = SofaPython.SofaNumpy.beginEdit( mo, "position" )
    print type(pos)


    # since memory is shared, it is possible to directly change a value :)
    print( "before:",mo.position[0][0] )
    pos[0][0] = 123
    print( "after:",mo.position[0][0] )

    # do not forget to unlock the Data and set it dirty
    SofaPython.SofaNumpy.endEdit( mo, "position" )



    fc = node.createObject("FixedConstraint", fixAll=False, indices="0")
    # print "a simple bool:", fc.fixAll, SofaPython.SofaNumpy.numpy_data( fc, "fixAll" )
    print "a simple scalar:", fc.drawSize, SofaPython.SofaNumpy.read_only( fc, "drawSize" )
    print "an array:", node.gravity, SofaPython.SofaNumpy.read_only( node, "gravity" )
    print "a 1D array:", fc.indices, SofaPython.SofaNumpy.read_only( fc, "indices" )
    print "a 2D array:", mo.position, SofaPython.SofaNumpy.read_only( mo, "position" )


    # WriteAccessor as Context
    print( "before:",mo.position[0][0] )
    with SofaPython.SofaNumpy.edit_data( mo, "position" ) as pos:
        pos[0][0] = 456
    print( "after:",mo.position[0][0] )




    sys.stdout.flush()
