'''Autogenerated by get_gl_extensions script, do not edit!'''
from OpenGL import platform as _p
from OpenGL.GL import glget
EXTENSION_NAME = 'GL_APPLE_transform_hint'
_p.unpack_constants( """GL_TRANSFORM_HINT_APPLE 0x85B1""", globals())


def glInitTransformHintAPPLE():
    '''Return boolean indicating whether this extension is available'''
    from OpenGL import extensions
    return extensions.hasGLExtension( EXTENSION_NAME )
