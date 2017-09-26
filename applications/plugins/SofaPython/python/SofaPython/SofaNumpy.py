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
    else:
        ptr, shape, typename = data.beginEditVoidPtr()

    nparray = wrap_as_numpy_array(ptr, shape, typename)

    if read_only: nparray.flags['WRITEABLE'] = False

    return nparray




def numpy_data(obj, name):
    ## @deprecated
    Sofa.msg_deprecated("SofaNumpy","numpy_data: Data must be explicitly accessed as read-only or read-write. "
                        " Use 'read_only()' or 'edit_data' context instead" )
    data = obj.findData(name)
    return as_numpy(data,-1)

def read_only(obj, name):
    ## read-only access to the data as a numpy array
    ## @warning the numpy array should NOT be modified
    data = obj.findData(name)
    return as_numpy(data, True)

def beginEdit(obj, name):
    ## read-write access to the data as a numpy array
    ## @warning do not forget to call 'endEdit' to unlock the data and set it as dirty
    data = obj.findData(name)
    return as_numpy(data, False)

def endEdit(obj, name):
    ## to be called after 'beginEdit'
    ## to unlock the data and set it as dirty
    data = obj.findData(name)
    data.endEditVoidPtr()
    # @todo: the nparray returned by beginEdit is still writeable...

from contextlib import contextmanager

@contextmanager
def edit_data(obj, name):
    ## equivalent to a WriteAccessor
    data = obj.findData(name)
    view = as_numpy( data, read_only = False )
    
    try:
        yield view
    finally:
        data.endEditVoidPtr()

        # make sure that leaked handles are not writable
        view.flags['WRITEABLE'] = False
        

def vec_as_numpy( (ptr, size, typename) ):
    '''maps vec as a numpy array'''
    Sofa.msg_deprecated("SofaNumpy","vec_as_numpy: use wrap_as_numpy_array instead")
    
    type = ctypeFromName.get(typename,None)
    if not type: raise Exception("can't map data of type " + typename)

    array_type = type * size
    array = array_type.from_address(ptr)
    return numpy.ctypeslib.as_array(array)







# mtournier: don't use if you're not me :D
def numpify(obj, name):
    # i very much want to alias data buffers in a read-write way without
    # triggering anything data-related, so please leave this be
    return as_numpy( obj.findData(name), True )

