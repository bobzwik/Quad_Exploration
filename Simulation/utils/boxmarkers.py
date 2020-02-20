# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
import numpy as np
from vispy.geometry import create_box
from vispy.visuals.mesh import MeshVisual
from vispy.visuals.visual import CompoundVisual
from vispy.scene.visuals import create_visual_node
from utils.updatableMesh import UpdatableMeshVisual

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
                 height_segments=1, depth_segments=1, planes=None, vertex_colors=None, face_colors=None,
                 color=(0.5, 0.5, 1, 1), edge_color=None, variable_vis=False, **kwargs):
        
        self.point_coords = point_coords
        self.nb_points = point_coords.shape[0]
        self.width = width
        self.height = height
        self.depth = depth
        self.color = color
        self.variable_vis = variable_vis

        # Create a unit box
        width_box  = 1
        height_box = 1
        depth_box  = 1

        scale = np.array([width, height, depth])

        self.vertices_box, self.filled_indices_box, self.outline_indices_box = create_box(
                width_box, height_box, depth_box, width_segments, height_segments,
                depth_segments, planes)

        # Store number of vertices, filled_indices and outline_indices per box
        self.nb_v  = self.vertices_box.shape[0]
        self.nb_fi = self.filled_indices_box.shape[0]
        self.nb_oi = self.outline_indices_box.shape[0]

        # Create empty arrays for vertices, filled_indices and outline_indices
        vertices = np.zeros(self.nb_v*point_coords.shape[0],
                        [('position', np.float32, 3),
                         ('texcoord', np.float32, 2),
                         ('normal', np.float32, 3),
                         ('color', np.float32, 4)])
        filled_indices = np.zeros([self.nb_fi*point_coords.shape[0], 3], np.uint32)
        outline_indices = np.zeros([self.nb_oi*point_coords.shape[0], 2], np.uint32)
        if self.variable_vis:
            box_to_face = np.zeros([point_coords.shape[0], self.nb_fi], np.uint32)
            box_to_outl = np.zeros([point_coords.shape[0], self.nb_oi], np.uint32)

        # Iterate for every marker
        for i in range(self.nb_points):
            idx_v_start  = self.nb_v*i
            idx_v_end    = self.nb_v*(i+1)
            idx_fi_start = self.nb_fi*i
            idx_fi_end   = self.nb_fi*(i+1)
            idx_oi_start = self.nb_oi*i
            idx_oi_end   = self.nb_oi*(i+1)

            # Scale and translate unit box
            vertices[idx_v_start:idx_v_end]['position'] = self.vertices_box['position']*scale + point_coords[i]
            filled_indices[idx_fi_start:idx_fi_end] = self.filled_indices_box + idx_v_start
            outline_indices[idx_oi_start:idx_oi_end] = self.outline_indices_box + idx_v_start
            if self.variable_vis:
                box_to_face[i,:] = np.arange(idx_fi_start,idx_fi_end)
                box_to_outl[i,:] = np.arange(idx_oi_start,idx_oi_end)
        
        if self.variable_vis:
            self.box_to_face = box_to_face
            self.box_to_outl = box_to_outl
            self._visible_boxes = np.arange(self.nb_points)

        # Create MeshVisual for faces and borders
        self._mesh = UpdatableMeshVisual(self.nb_points, vertices['position'], filled_indices,
                                vertex_colors, face_colors, color, shading=None, variable_vis=variable_vis)
        if edge_color:
            self._border = UpdatableMeshVisual(self.nb_points, vertices['position'], outline_indices,
                                      color=edge_color, mode='lines', variable_vis=variable_vis)
        else:
            self._border = UpdatableMeshVisual(self.nb_points)

        CompoundVisual.__init__(self, [self._mesh, self._border], **kwargs)
        self.mesh.set_gl_state(polygon_offset_fill=True,
                               polygon_offset=(1, 1), depth_test=True)

        self.freeze()

    def set_visible_boxes(self, idx_box_vis):
        if not self.variable_vis:
            raise ValueError('Variable visibility must be enabled via "variable_vis"')
        newbox_vis = np.setdiff1d(idx_box_vis, self.visible_boxes)
        oldbox_vis = np.setdiff1d(self.visible_boxes, idx_box_vis)
        
        idx_face_vis = np.ravel(self.box_to_face[newbox_vis])
        idx_outl_vis = np.ravel(self.box_to_outl[newbox_vis])
        idx_face_invis = np.ravel(self.box_to_face[oldbox_vis])
        idx_outl_invis = np.ravel(self.box_to_outl[oldbox_vis])

        self.mesh.set_visible_faces(idx_face_vis)
        self.mesh.set_invisible_faces(idx_face_invis)
        self.mesh.update_vis_buffer()
        
        self.border.set_visible_faces(idx_outl_vis)
        self.border.set_invisible_faces(idx_outl_invis)
        self.border.update_vis_buffer()

        self.visible_boxes = idx_box_vis


    @property
    def visible_boxes(self):
        if not self.variable_vis:
            raise ValueError('Variable visibility must be enabled via "variable_vis"')
        return self._visible_boxes


    @visible_boxes.setter
    def visible_boxes(self, visible_boxes):
        if not self.variable_vis:
            raise ValueError('Variable visibility must be enabled via "variable_vis"')
        self._visible_boxes = visible_boxes


    def set_data(self, point_coords=None, width=None, height=None, depth=None, vertex_colors=None, face_colors=None, color=None, edge_color=None):

        if point_coords is None:
            point_coords = self.point_coords
        else:
            self.point_coords = point_coords
        
        if width is None:
            width = self.width
        else:
            self.width = width
        
        if height is None:
            height = self.height
        else:
            self.height = height
        
        if depth is None:
            depth = self.depth
        else:
            self.depth = depth
        
        if color is None:
            color = self.color
        else:
            self.color = color

        self.nb_points = point_coords.shape[0]

        # Create empty arrays for vertices, filled_indices and outline_indices
        vertices = np.zeros(self.nb_v*point_coords.shape[0],
                        [('position', np.float32, 3),
                         ('texcoord', np.float32, 2),
                         ('normal', np.float32, 3),
                         ('color', np.float32, 4)])
        filled_indices = np.zeros([self.nb_fi*point_coords.shape[0], 3], np.uint32)
        outline_indices = np.zeros([self.nb_oi*point_coords.shape[0], 2], np.uint32)
        if self.variable_vis:
            box_to_face = np.zeros([point_coords.shape[0], self.nb_fi], np.uint32)
            box_to_outl = np.zeros([point_coords.shape[0], self.nb_oi], np.uint32)


        scale = np.array([width, height, depth])

        # Iterate for every marker
        for i in range(point_coords.shape[0]):
            idx_v_start  = self.nb_v*i
            idx_v_end    = self.nb_v*(i+1)
            idx_fi_start = self.nb_fi*i
            idx_fi_end   = self.nb_fi*(i+1)
            idx_oi_start = self.nb_oi*i
            idx_oi_end   = self.nb_oi*(i+1)

            # Scale and translate unit box
            vertices[idx_v_start:idx_v_end]['position'] = self.vertices_box['position']*scale + point_coords[i]
            filled_indices[idx_fi_start:idx_fi_end] = self.filled_indices_box + idx_v_start
            outline_indices[idx_oi_start:idx_oi_end] = self.outline_indices_box + idx_v_start
            if self.variable_vis:
                box_to_face[i,:] = np.arange(idx_fi_start,idx_fi_end)
                box_to_outl[i,:] = np.arange(idx_oi_start,idx_oi_end)


        if self.variable_vis:
            self.box_to_face = box_to_face
            self.box_to_outl = box_to_outl
            self._visible_boxes = np.arange(self.nb_points)

        # Create MeshVisual for faces and borders
        self.mesh.set_data(vertices['position'], filled_indices, vertex_colors, face_colors, color)
        self.border.set_data(vertices['position'], outline_indices, color=edge_color)

    
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
