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
    pos = SofaPython.SofaNumpy.numpy_data( mo, "position" )
    print type(pos)


    # since memory is shared, it is possible to directly change a value :)
    print( "before:",mo.position[0][0] )
    pos[0][0] = 123
    print( "after:",mo.position[0][0] )



    fc = node.createObject("FixedConstraint", fixAll=False, indices="0 1")
    # print "a simple bool:", fc.fixAll, SofaPython.SofaNumpy.numpy_data( fc, "fixAll" )
    print "a simple scalar:", fc.drawSize, SofaPython.SofaNumpy.numpy_data( fc, "drawSize" )
    print "an array:", node.gravity, SofaPython.SofaNumpy.numpy_data( node, "gravity" )
    print "a 1D array:", fc.indices, SofaPython.SofaNumpy.numpy_data( fc, "indices" )
    print "a 1D array (Vec3):", mo.translation, SofaPython.SofaNumpy.numpy_data( mo, "translation" )
    print "a 2D array (vector of Vec3):", mo.position, SofaPython.SofaNumpy.numpy_data( mo, "position" )

    mo1d = node.createChild("1d").createObject("MechanicalObject",template="Vec1d", position="9 10 11")
    print "a 2D array (vector of Vec1):", mo1d.position, SofaPython.SofaNumpy.numpy_data( mo1d, "position" )

    te = node.createObject("TranslateTransformMatrixEngine",inT="[0 1 2 3, 4 5 6 7, 8 9 10 11, 12 13 14 15]")
    print "a 2D array (Mat4x4):", te.inT, SofaPython.SofaNumpy.numpy_data( te, "inT" )






    m = node.createObject("MatrixMass",massMatrices="[1 2 3, 4 5 6, 7 8 9] [4 5 6, 3 2 1, -8 -9 -6]")
    print "a 3D vector (vector of matrices)",m.massMatrices,SofaPython.SofaNumpy.numpy_data( m, "massMatrices" )


    sys.stdout.flush()
