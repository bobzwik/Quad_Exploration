# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
# Code heavily inspired by:
# Copyright (c) Vispy Development Team. All Rights Reserved.
# Distributed under the (new) BSD License. See LICENSE.txt for more info.
#
# Modified by:
# author: John Bass
# email: john.bobzwik@gmail.com
# license: MIT
# Please feel free to use and modify this, but keep the above information. Thanks!
# -----------------------------------------------------------------------------

""" A MeshVisual Visual that uses the new shader Function.
"""
import numpy as np
import vispy
from vispy import app
from vispy.scene import visuals
from vispy.color import ColorArray
from vispy.util import keys
from vispy.scene.cameras.perspective import PerspectiveCamera

import config

###################################
# Create a Markers subclass that can easily have it's color array changed

class ColorMarkers(visuals.Markers):
    def __init__(self, **kwargs):
        super(ColorMarkers, self).__init__(**kwargs)
    
    def set_color(self, face_color='white'):
        face_color = ColorArray(face_color).rgba
        if len(face_color) == 1:
            face_color = face_color[0]

        self._data['a_bg_color'] = face_color
        self._vbo.set_data(self._data)
        self.update()


###################################
# Create a turntable camera that doesn't automatically 
# update when setting the azimuth and center
# Also allow transate operation from mouse to update visual

class NonUpdatingTurntable(vispy.scene.cameras.TurntableCamera):
    def __init__(self, **kwargs):
        super(NonUpdatingTurntable, self).__init__(**kwargs)
    
    @property
    def azimuth(self):
        """ The angle of the camera in degrees around the y axis. An angle of
        0 places the camera within the (y, z) plane.
        """
        return self._azimuth

    @azimuth.setter
    def azimuth(self, azim):
        azim = float(azim)
        while azim < -180:
            azim += 360
        while azim > 180:
            azim -= 360
        self._azimuth = azim
        # self.view_changed()       # Don't update view right now
    
    @property
    def center(self):
        """ The center location for this camera

        The exact meaning of this value differs per type of camera, but
        generally means the point of interest or the rotation point.
        """
        return self._center or (0, 0, 0)
    
    @center.setter
    def center(self, val):
        if len(val) == 2:
            self._center = float(val[0]), float(val[1]), 0.0
        elif len(val) == 3:
            self._center = float(val[0]), float(val[1]), float(val[2])
        else:
            raise ValueError('Center must be a 2 or 3 element tuple')
        # self.view_changed()       # Don't update view right now
    
    def viewbox_mouse_event(self, event):
        """
        The viewbox received a mouse event; update transform
        accordingly.

        Parameters
        ----------
        event : instance of Event
            The event.
        """
        if event.handled or not self.interactive:
            return

        PerspectiveCamera.viewbox_mouse_event(self, event)

        if event.type == 'mouse_release':
            self._event_value = None  # Reset
        elif event.type == 'mouse_press':
            event.handled = True
        elif event.type == 'mouse_move':
            if event.press_event is None:
                return

            modifiers = event.mouse_event.modifiers
            p1 = event.mouse_event.press_event.pos
            p2 = event.mouse_event.pos
            d = p2 - p1

            if 1 in event.buttons and not modifiers:
                # Rotate
                self._update_rotation(event)

            elif 2 in event.buttons and not modifiers:
                # Zoom
                if self._event_value is None:
                    self._event_value = (self._scale_factor, self._distance)
                zoomy = (1 + self.zoom_factor) ** d[1]
                
                self.scale_factor = self._event_value[0] * zoomy
                # Modify distance if its given
                if self._distance is not None:
                    self._distance = self._event_value[1] * zoomy
                self.view_changed()

            elif 1 in event.buttons and keys.SHIFT in modifiers:
                # Translate
                norm = np.mean(self._viewbox.size)
                if self._event_value is None or len(self._event_value) == 2:
                    self._event_value = self.center
                dist = (p1 - p2) / norm * self._scale_factor
                dist[1] *= -1
                # Black magic part 1: turn 2D into 3D translations
                dx, dy, dz = self._dist_to_trans(dist)
                # Black magic part 2: take up-vector and flipping into account
                ff = self._flip_factors
                up, forward, right = self._get_dim_vectors()
                dx, dy, dz = right * dx + forward * dy + up * dz
                dx, dy, dz = ff[0] * dx, ff[1] * dy, dz * ff[2]
                c = self._event_value
                self.center = c[0] + dx, c[1] + dy, c[2] + dz
                self.view_changed()                                     # Update view right now

            elif 2 in event.buttons and keys.SHIFT in modifiers:
                # Change fov
                if self._event_value is None:
                    self._event_value = self._fov
                fov = self._event_value - d[1] / 5.0
                self.fov = min(180.0, max(0.0, fov))

# PerspectiveCamera.viewbox_mouse_event = new_viewbox_mouse_event