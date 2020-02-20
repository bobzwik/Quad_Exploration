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

# Shaders for lit rendering (using phong shading)
shading_vertex_template = """
varying vec3 v_normal_vec;
varying vec3 v_light_vec;
varying vec3 v_eye_vec;

varying vec4 v_ambientk;
varying vec4 v_light_color;
varying vec4 v_base_color;

void main() {
    v_ambientk = $ambientk;
    v_light_color = $light_color;
    v_base_color = $color_transform($base_color);


    vec4 pos_scene = $visual2scene($to_vec4($position));
    vec4 normal_scene = $visual2scene(vec4($normal, 1.0));
    vec4 origin_scene = $visual2scene(vec4(0.0, 0.0, 0.0, 1.0));

    normal_scene /= normal_scene.w;
    origin_scene /= origin_scene.w;

    vec3 normal = normalize(normal_scene.xyz - origin_scene.xyz);
    v_normal_vec = normal; //VARYING COPY

    vec4 pos_front = $scene2doc(pos_scene);
    pos_front.z += 0.01;
    pos_front = $doc2scene(pos_front);
    pos_front /= pos_front.w;

    vec4 pos_back = $scene2doc(pos_scene);
    pos_back.z -= 0.01;
    pos_back = $doc2scene(pos_back);
    pos_back /= pos_back.w;

    vec3 eye = normalize(pos_front.xyz - pos_back.xyz);
    v_eye_vec = eye; //VARYING COPY

    vec3 light = normalize($light_dir.xyz);
    v_light_vec = light; //VARYING COPY

    gl_Position = $transform($to_vec4($position));
}
"""

shading_fragment_template = """
varying vec3 v_normal_vec;
varying vec3 v_light_vec;
varying vec3 v_eye_vec;

varying vec4 v_ambientk;
varying vec4 v_light_color;
varying vec4 v_base_color;

void main() {
    //DIFFUSE
    float diffusek = dot(v_light_vec, v_normal_vec);
    // clamp, because 0 < theta < pi/2
    diffusek  = clamp(diffusek, 0.0, 1.0);
    vec4 diffuse_color = v_light_color * diffusek;

    //SPECULAR
    //reflect light wrt normal for the reflected ray, then
    //find the angle made with the eye
    float speculark = 0.0;
    if ($shininess > 0.) {
        speculark = dot(reflect(v_light_vec, v_normal_vec), v_eye_vec);
        speculark = clamp(speculark, 0.0, 1.0);
        //raise to the material's shininess, multiply with a
        //small factor for spread
        speculark = 20.0 * pow(speculark, 1.0 / $shininess);
    }
    vec4 specular_color = v_light_color * speculark;
    gl_FragColor = v_base_color * (v_ambientk + diffuse_color) + specular_color;
}
"""  # noqa

# Shader code for non lighted rendering
vertex_template = """
varying vec4 v_base_color;
varying float draw_yes_no;
void main() {
    v_base_color = $color_transform($base_color);
    gl_Position = $transform($to_vec4($position));
    draw_yes_no = $vis_vert;
}
"""

fragment_template = """
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
    """Mesh visual

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
    **kwargs : dict
        Keyword arguments to pass to `Visual`.
    """
    def __init__(self, nb_boxes, vertices=None, faces=None, vertex_colors=None,
                 face_colors=None, color=(0.5, 0.5, 1, 1), vertex_values=None,
                 meshdata=None, shading=None, mode='triangles', **kwargs):

        # Function for computing phong shading
        # self._phong = Function(phong_template)

        # Visual.__init__ -> prepare_transforms() -> uses shading
        self.shading = shading

        if shading is not None:
            Visual.__init__(self, vcode=shading_vertex_template,
                            fcode=shading_fragment_template,
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

        # Init
        self._bounds = None
        # Note we do not call subclass set_data -- often the signatures
        # do no match.
        MeshVisual.set_data(
            self, vertices=vertices, faces=faces, vertex_colors=vertex_colors,
            face_colors=face_colors, vertex_values=vertex_values,
            meshdata=meshdata, color=color)

        # Initialize all faces as visible
        if mode is 'lines':
            self._visible_verts = np.ones((faces.shape[0],2,1), dtype=np.float32)
        else:
            self._visible_verts = np.ones((faces.shape[0],3,1), dtype=np.float32)
        
        self.vis_buffer = VertexBuffer()
        self.vis_buffer.set_data(self._visible_verts, convert=True)
        self.shared_program.vert['vis_vert'] = self.vis_buffer

        # primitive mode
        self._draw_mode = mode
        self.freeze()
    

    def set_visible_faces(self, idx_vis):
        self._visible_verts[idx_vis,:,:] = 1


    def set_invisible_faces(self, idx_vis):
        self._visible_verts[idx_vis,:,:] = 0
    
    
    def update_vis_buffer(self):
        self.vis_buffer.set_data(self._visible_verts)
        self.shared_program.vert['vis_vert'] = self.vis_buffer
        

    
    # def set_visible_faces(self, idx_vis):
    #     # nfaces = self.mesh_data.n_faces
    #     # if self.mode is 'lines':
    #         # self._visible_verts = np.zeros((nfaces,2,1), dtype=np.float32)
    #         # self._visible_verts[idx_vis,:,:] = 1
    #     # else:
    #         # self._visible_verts = np.zeros((nfaces,3,1), dtype=np.float32)
    #         # self._visible_verts[idx_vis,:,:] = 1
    #     self._visible_verts[idx_vis,:,:] = 1
    #     self.vis_buffer.set_data(self._visible_verts)
    #     self.shared_program.vert['vis_vert'] = self.vis_buffer

    # def set_invisible_faces(self, idx_vis):
    #     # nfaces = self.mesh_data.n_faces
    #     # if self.mode is 'lines':
    #         # self._visible_verts = np.zeros((nfaces,2,1), dtype=np.float32)
    #         # self._visible_verts[idx_vis,:,:] = 0
    #     # else:
    #         # self._visible_verts = np.zeros((nfaces,3,1), dtype=np.float32)
    #         # self._visible_verts[idx_vis,:,:] = 0
    #     self._visible_verts[idx_vis,:,:] = 0
    #     self.vis_buffer.set_data(self._visible_verts)
    #     self.shared_program.vert['vis_vert'] = self.vis_buffer

