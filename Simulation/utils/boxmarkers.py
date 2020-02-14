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
    point_coords : array_like
        Marker coordinates
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

    def __init__(self, point_coords=np.array([0,0,0]), width=1, height=1, depth=1, width_segments=1,
                 height_segments=1, depth_segments=1, planes=None,
                 vertex_colors=None, face_colors=None,
                 color=(0.5, 0.5, 1, 1), edge_color=None, **kwargs):
        
        self.point_coords = point_coords
        
        # Create a unit box
        width_box  = 1
        height_box = 1
        depth_box  = 1

        scale = np.array([width/width_box, height/height_box, depth/depth_box])

        vertices_box, filled_indices_box, outline_indices_box = create_box(
                width_box, height_box, depth_box, width_segments, height_segments,
                depth_segments, planes)

        # Store number of vertices, filled_indices and outline_indices per box
        self.nb_v  = vertices_box.shape[0]
        self.nb_fi = filled_indices_box.shape[0]
        self.nb_oi = outline_indices_box.shape[0]

        # Create empty arrays for vertices, filled_indices and outline_indices
        vertices = np.zeros(24*point_coords.shape[0],
                        [('position', np.float32, 3),
                         ('texcoord', np.float32, 2),
                         ('normal', np.float32, 3),
                         ('color', np.float32, 4)])
        filled_indices = np.zeros([12*point_coords.shape[0], 3], np.uint32)
        outline_indices = np.zeros([24*point_coords.shape[0], 2], np.uint32)

        # Iterate for every marker
        for i in range(point_coords.shape[0]):
            idx_v_start  = self.nb_v*i
            idx_v_end    = self.nb_v*(i+1)
            idx_fi_start = self.nb_fi*i
            idx_fi_end   = self.nb_fi*(i+1)
            idx_oi_start = self.nb_oi*i
            idx_oi_end   = self.nb_oi*(i+1)

            # Scale and translate unit box
            vertices[idx_v_start:idx_v_end]['position'] = vertices_box['position']*scale + point_coords[i]
            filled_indices[idx_fi_start:idx_fi_end] = filled_indices_box + idx_v_start
            outline_indices[idx_oi_start:idx_oi_end] = outline_indices_box + idx_v_start

        # Create MeshVisual for faces and borders
        self._mesh = MeshVisual(vertices['position'], filled_indices,
                                vertex_colors, face_colors, color, shading=None)
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
