# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------

""" A MeshVisual Visual that uses the new shader Function.
"""
from __future__ import division

import numpy as np

from vispy.visuals.visual import Visual
from vispy.visuals.mesh import MeshVisual
from vispy.visuals.shaders import Function, FunctionChain
from vispy.gloo import VertexBuffer, IndexBuffer
from vispy.gloo.buffer import DataBuffer, Buffer
from vispy.geometry import MeshData
from vispy.color import Color, get_colormap
from vispy.visuals.shaders.variable import Variable, Varying

# Shader code for non lighted rendering with variable visibility
vertex_template_vis = """
varying vec4 v_base_color;
varying float draw_yes_no;
void main() {
    v_base_color = $color_transform($base_color);
    gl_Position = $transform($to_vec4($position));
    draw_yes_no = $vis_vert;
}
"""

fragment_template_vis = """
varying vec4 v_base_color;
varying float draw_yes_no;
void main() {
    if (draw_yes_no > 0.5){
        gl_FragColor = v_base_color;
    }
    if (draw_yes_no < 0.5){
        discard;
    }
}
"""
# Shader code for non lighted rendering with constant visbility
vertex_template = """
varying vec4 v_base_color;

void main() {
    v_base_color = $color_transform($base_color);
    gl_Position = $transform($to_vec4($position));
}
"""

fragment_template = """
varying vec4 v_base_color;
void main() {
    gl_FragColor = v_base_color;
}
"""



# Functions that can be used as is (don't have template variables)
# Consider these stored in a central location in vispy ...

vec3to4 = Function("""
vec4 vec3to4(vec3 xyz) {
    return vec4(xyz, 1.0);
}
""")

vec2to4 = Function("""
vec4 vec2to4(vec2 xyz) {
    return vec4(xyz, 0.0, 1.0);
}
""")


_null_color_transform = 'vec4 pass(vec4 color) { return color; }'
_clim = 'float cmap(float val) { return (val - $cmin) / ($cmax - $cmin); }'


# Eventually this could be de-duplicated with visuals/image.py, which does
# something similar (but takes a ``color`` instead of ``float``)
def _build_color_transform(data, cmap, clim=(0., 1.)):
    if data.ndim == 2 and data.shape[1] == 1:
        fun = Function(_clim)
        fun['cmin'] = clim[0]
        fun['cmax'] = clim[1]
        fun = FunctionChain(None, [fun, Function(cmap.glsl_map)])
    else:
        fun = Function(_null_color_transform)
    return fun


class UpdatableMeshVisual(MeshVisual):
    """Updatable Mesh visual

    Parameters
    ----------
    vertices : array-like | None
        The vertices.
    faces : array-like | None
        The faces.
    vertex_colors : array-like | None
        Colors to use for each vertex.
    face_colors : array-like | None
        Colors to use for each face.
    color : instance of Color
        The color to use.
    vertex_values : array-like | None
        The values to use for each vertex (for colormapping).
    meshdata : instance of MeshData | None
        The meshdata.
    shading : str | None
        Shading to use.
    mode : str
        The drawing mode.
    variable_vis : bool
        If instance of UpdatableMeshVisual has variable visibility
    **kwargs : dict
        Keyword arguments to pass to `Visual`.
    """
    def __init__(self, nb_boxes, vertices=None, faces=None, vertex_colors=None,
                 face_colors=None, color=(0.5, 0.5, 1, 1), vertex_values=None,
                 meshdata=None, shading=None, mode='triangles', variable_vis=False, **kwargs):

        # Visual.__init__ -> prepare_transforms() -> uses shading

        if shading is not None:
            raise ValueError('"shading" must be "None"')
        
        self.shading = shading
        self._variable_vis = variable_vis

        if variable_vis:
            Visual.__init__(self, vcode=vertex_template_vis,
                            fcode=fragment_template_vis,
                            **kwargs)
        else:
            Visual.__init__(self, vcode=vertex_template,
                            fcode=fragment_template,
                            **kwargs)

        self.set_gl_state('translucent', depth_test=True,
                          cull_face=False)

        # Define buffers
        self._vertices = VertexBuffer(np.zeros((0, 3), dtype=np.float32))
        self._normals = None
        self._faces = IndexBuffer()
        self._normals = VertexBuffer(np.zeros((0, 3), dtype=np.float32))
        self._ambient_light_color = Color((0.3, 0.3, 0.3, 1.0))
        self._light_dir = (10, 5, -5)
        self._shininess = 1. / 200.
        self._cmap = get_colormap('cubehelix')
        self._clim = 'auto'
        self._mode = mode

        # Uniform color
        self._color = Color(color)

        # primitive mode
        self._draw_mode = mode

        # Init
        self._bounds = None
        # Note we do not call subclass set_data -- often the signatures
        # do no match.
        UpdatableMeshVisual.set_data(
            self, vertices=vertices, faces=faces, vertex_colors=vertex_colors,
            face_colors=face_colors, color=color, vertex_values=vertex_values,
            meshdata=meshdata)

        self.freeze()
    
    def set_data(self, vertices=None, faces=None, vertex_colors=None,
                 face_colors=None, color=None, vertex_values=None,
                 meshdata=None):
        """Set the mesh data

        Parameters
        ----------
        vertices : array-like | None
            The vertices.
        faces : array-like | None
            The faces.
        vertex_colors : array-like | None
            Colors to use for each vertex.
        face_colors : array-like | None
            Colors to use for each face.
        color : instance of Color
            The color to use.
        vertex_values : array-like | None
            Values for each vertex.
        meshdata : instance of MeshData | None
            The meshdata.
        """
        if meshdata is not None:
            self._meshdata = meshdata
        else:
            self._meshdata = MeshData(vertices=vertices, faces=faces,
                                      vertex_colors=vertex_colors,
                                      face_colors=face_colors,
                                      vertex_values=vertex_values)
        self._bounds = self._meshdata.get_bounds()
        if color is not None:
            self._color = Color(color)
        self.mesh_data_changed()

        if self.variable_vis:
            # Initialize all faces as visible
            if self.mode is 'lines':
                self._visible_verts = np.ones((faces.shape[0],2,1), dtype=np.float32)
            else:
                self._visible_verts = np.ones((faces.shape[0],3,1), dtype=np.float32)
            # Create visibility VertexBuffer
            self.vis_buffer = VertexBuffer()
            self.vis_buffer.set_data(self._visible_verts, convert=True)
            self.shared_program.vert['vis_vert'] = self.vis_buffer


    def set_visible_faces(self, idx_vis):
        """Set idx_vis indexes of visible_verts to "visible" (1).
        """
        self.visible_verts[idx_vis,:,:] = 1


    def set_invisible_faces(self, idx_vis):
        """Set idx_vis indexes of visible_verts to "invisible" (0).
        """
        self.visible_verts[idx_vis,:,:] = 0
 
    
    def update_vis_buffer(self):
        """Update the visibility VertexBuffer.
        """
        self.vis_buffer.set_data(self.visible_verts)
        self.shared_program.vert['vis_vert'] = self.vis_buffer
        
    
    @property
    def visible_verts(self):
        """Bool (float) array indicating which vertices are visible (1) and which are invisible (0).
        """
        return self._visible_verts

    @visible_verts.setter
    def visible_verts(self, visible_verts):
        self._visible_verts = visible_verts

    @property
    def variable_vis(self):
        """Bool if instance of UpdatableMeshVisual posseses variable visibility.
        """
        return self._variable_vis

    @variable_vis.setter
    def variable_vis(self, variable_vis):
        raise ValueError('Not allowed to change "variable_vis" after initialization.')