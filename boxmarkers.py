# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
import numpy as np
from vispy.geometry import create_box
from vispy.visuals.mesh import MeshVisual
from vispy.visuals.visual import CompoundVisual
from vispy.scene.visuals import create_visual_node

class BoxMarkersVisual(CompoundVisual):
    """Visual that displays a box.

    Parameters
    ----------
    width : float
        Box width.
    height : float
        Box height.
    depth : float
        Box depth.
    width_segments : int
        Box segments count along the width.
    height_segments : float
        Box segments count along the height.
    depth_segments : float
        Box segments count along the depth.
    planes: array_like
        Any combination of ``{'-x', '+x', '-y', '+y', '-z', '+z'}``
        Included planes in the box construction.
    vertex_colors : ndarray
        Same as for `MeshVisual` class. See `create_plane` for vertex ordering.
    face_colors : ndarray
        Same as for `MeshVisual` class. See `create_plane` for vertex ordering.
    color : Color
        The `Color` to use when drawing the cube faces.
    edge_color : tuple or Color
        The `Color` to use when drawing the cube edges. If `None`, then no
        cube edges are drawn.
    """

    def __init__(self, points=np.array([0,0,0]), width=1, height=1, depth=1, width_segments=1,
                 height_segments=1, depth_segments=1, planes=None,
                 vertex_colors=None, face_colors=None,
                 color=(0.5, 0.5, 1, 1), edge_color=None, **kwargs):
        vertices = np.zeros(24*points.shape[0],
                        [('position', np.float32, 3),
                         ('texcoord', np.float32, 2),
                         ('normal', np.float32, 3),
                         ('color', np.float32, 4)])
        filled_indices = np.zeros([12*points.shape[0], 3], np.uint32)
        outline_indices = np.zeros([24*points.shape[0], 2], np.uint32)

        vertices_box, filled_indices_box, outline_indices_box = create_box(
                width, height, depth, width_segments, height_segments,
                depth_segments, planes)

        for i in range( points.shape[0]):
            start_24_idx = 24*i
            end_24_idx = 24*(i+1)
            start_12_idx = 12*i
            end_12_idx = 12*(i+1)

            vertices[start_24_idx:end_24_idx]['position'] = vertices_box['position'] + points[i]
            filled_indices[start_12_idx:end_12_idx] = filled_indices_box + start_24_idx
            outline_indices[start_24_idx:end_24_idx] = outline_indices_box + start_24_idx

        self._mesh = MeshVisual(vertices['position'], filled_indices,
                                vertex_colors, face_colors, color)
        if edge_color:
            self._border = MeshVisual(vertices['position'], outline_indices,
                                      color=edge_color, mode='lines')
        else:
            self._border = MeshVisual()

        CompoundVisual.__init__(self, [self._mesh, self._border], **kwargs)
        self.mesh.set_gl_state(polygon_offset_fill=True,
                               polygon_offset=(1, 1), depth_test=True)

    @property
    def mesh(self):
        """The vispy.visuals.MeshVisual that used to fill in.
        """
        return self._mesh

    @mesh.setter
    def mesh(self, mesh):
        self._mesh = mesh

    @property
    def border(self):
        """The vispy.visuals.MeshVisual that used to draw the border.
        """
        return self._border

    @border.setter
    def border(self, border):
        self._border = border


BoxMarkers = create_visual_node(BoxMarkersVisual)
