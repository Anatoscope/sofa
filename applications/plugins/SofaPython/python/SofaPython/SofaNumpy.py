## @author Maxime Tournier & Matthieu Nesme

import Sofa
import ctypes
import numpy

# TODO add more basic types
# check that type sizes are equivalent with c++ sizes
# or even better, find a way to get the type without any string lookup
ctypeFromName = {
    'double': ctypes.c_double,
    'float': ctypes.c_float,
    'bool': ctypes.c_bool,
    'char': ctypes.c_char,
    'unsigned char': ctypes.c_ubyte,
    'short': ctypes.c_short,
    'unsigned short': ctypes.c_ushort,
    'int': ctypes.c_int,
    'unsigned int': ctypes.c_uint,
    'long': ctypes.c_long,
    'unsigned long': ctypes.c_ulong,
}


def as_numpy( data, readOnly=-1 ):
    """ maps data content as a numpy array
    :param data: the Data
    :param readOnly: (boolean) if true you should not modify the numpy array content
                     if false (read/write), you have to call endEdit to unlock the Data and set it as dirty
    :return: numpy array
    """

    if not isinstance(readOnly, bool): # backward compatibility
        Sofa.msg_deprecated("SofaNumpy","as_numpy: Data must be explicitly accessed as read-only or read-write. "
                                        "With a read-only access, the numpy array should not be modified. "
                                        "With a read-write access, the Data must be explicitly unlocked and set as dirty with a call to 'Data.endEditVoidPtr'")
        readOnly = True # similar to the first behavior

    if readOnly:
        ptr, shape, typename = data.getValueVoidPtr()
    else:
        ptr, shape, typename = data.beginEditVoidPtr()

    type = ctypeFromName.get(typename,None)
    if not type: raise Exception("can't map data of type " + typename)

    # print (shape)

    # fold
    array_type = reduce(lambda x, y: x * y, reversed(shape), type)
    array = array_type.from_address(ptr)
    return numpy.ctypeslib.as_array(array)

    # https://github.com/numpy/numpy/issues/6511
    # array = ctypes.cast( ctypes.c_void_p(ptr), ctypes.POINTER(type))
    # return numpy.ctypeslib.as_array(array, shape)


# convenience
def numpy_data(obj, name):
    data = obj.findData(name)
    return as_numpy(data,-1)

# read-only
def numpy_data_ro(obj, name):
    data = obj.findData(name)
    return as_numpy(data,True)

# read-write
def numpy_data_beginEdit(obj, name):
    data = obj.findData(name)
    return as_numpy(data,False)

# convenience
def numpy_data_endEdit(obj, name):
    data = obj.findData(name)
    data.endEditVoidPtr()


# convenience WriteAccessor as context
class numpy_data_WriteAccessor(object):

    def __init__(self, obj, dataname):
        self.data = obj.findData(dataname)

    def __enter__(self):
        return as_numpy( self.data, False )

    def __exit__(self, type, value, traceback):
        self.data.endEditVoidPtr()



def vec_as_numpy( (ptr, size, typename) ):
    '''maps vec as a numpy array'''

    type = ctypeFromName.get(typename,None)
    if not type: raise Exception("can't map data of type " + typename)

    array_type = type * size
    array = array_type.from_address(ptr)
    return numpy.ctypeslib.as_array(array)
