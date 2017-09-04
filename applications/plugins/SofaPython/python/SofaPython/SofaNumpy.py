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



def wrap_as_numpy_array(ptr, shape, typename):
    ctype = ctypeFromName.get(typename, None)
    if not ctype: raise Exception("can't map data of type " + typename)

    # print (shape)

    # fold
    array_type = reduce(lambda x, y: x * y, reversed(shape), ctype)
    array = array_type.from_address(ptr)
    return numpy.ctypeslib.as_array(array)

    # https://github.com/numpy/numpy/issues/6511
    # array = ctypes.cast( ctypes.c_void_p(ptr), ctypes.POINTER(type))
    # return numpy.ctypeslib.as_array(array, shape)
    


# TODO fix broken semantics (sometimes you need to call endEdit, sometimes not)
def as_numpy( data, read_only = -1 ):
    """ maps data content as a numpy array
    :param data: the Data
    :param readOnly: (boolean) if true you should not modify the numpy array content
                     if false (read/write),  ***you have to call endEdit*** to unlock the Data and set it as dirty
    :return: numpy array
    """

    if not isinstance(read_only, bool): # backward compatibility
        Sofa.msg_deprecated("SofaNumpy","as_numpy: Data must be explicitly accessed as read-only or read-write. "
                                        "With a read-only access, the numpy array should not be modified. "
                                        "With a read-write access, the Data must be explicitly unlocked and set as dirty with a call to 'Data.endEditVoidPtr'")
        read_only = True # similar to the first behavior

    if read_only:
        ptr, shape, typename = data.getValueVoidPtr()
        # TODO voir comment bloquer le numpy array en ecriture avec numpy.ctypeslib.ndpointer flags WRITEABLE
        # https://docs.scipy.org/doc/numpy-1.10.0/reference/routines.ctypeslib.html
    else:
        ptr, shape, typename = data.beginEditVoidPtr()

    return wrap_as_numpy_array(ptr, shape, typename)



# convenience
def numpy_data(obj, name):
    Sofa.msg_deprecated("SofaNumpy","numpy_data: Data must be explicitly accessed as read-only or read-write. "
                        " Use numpy_data_ro() or numpy_data_WriteAccessor instead" )
    data = obj.findData(name)
    return as_numpy(data,-1)

# read-only
def numpy_data_ro(obj, name):
    data = obj.findData(name)
    return as_numpy(data, True)

# read-write
def numpy_data_beginEdit(obj, name):
    data = obj.findData(name)
    return as_numpy(data, False)

# convenience
def numpy_data_endEdit(obj, name):
    data = obj.findData(name)
    data.endEditVoidPtr()

from contextlib import contextmanager

@contextmanager
def edit_data(obj, name):
    data = obj.findData(name)
    view = as_numpy( data, read_only = False )
    
    try:
        yield view
    finally:
        data.endEditVoidPtr()
        

def vec_as_numpy( (ptr, size, typename) ):
    '''maps vec as a numpy array'''
    Sofa.msg_deprecated("SofaNumpy","vec_as_numpy: use wrap_as_numpy_array instead")
    
    type = ctypeFromName.get(typename,None)
    if not type: raise Exception("can't map data of type " + typename)

    array_type = type * size
    array = array_type.from_address(ptr)
    return numpy.ctypeslib.as_array(array)
