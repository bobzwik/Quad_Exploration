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

import config

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
