'''Autogenerated by get_gl_extensions script, do not edit!'''
from OpenGL import platform as _p
from OpenGL.GL import glget
EXTENSION_NAME = 'GL_APPLE_row_bytes'
_p.unpack_constants( """GL_PACK_ROW_BYTES_APPLE 0x8A15
GL_UNPACK_ROW_BYTES_APPLE 0x8A16""", globals())


def glInitRowBytesAPPLE():
    '''Return boolean indicating whether this extension is available'''
    from OpenGL import extensions
    return extensions.hasGLExtension( EXTENSION_NAME )
