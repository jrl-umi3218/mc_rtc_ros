import rospy

from geometry_msgs.msg import Point, Quaternion
from interactive_markers.interactive_marker_server import InteractiveMarkerServer,\
                                                          InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl, Marker

from eigen import Quaterniond


class ContactControlMarker(object):
  def __init__(self, name, callback):
    self.name = name
    self.callback = callback
    self.server = InteractiveMarkerServer('%s_marker' % name)
    self.server.applyChanges()

  def init(self, position, orientation=None, dof=None):
    self.dof = dof
    if dof is not None:
      dim = len([d for d in self.dof if d == 0])
    else:
      dim = 6

    self.intMarker = InteractiveMarker()
    self.intMarker.header.frame_id = '/robot_map'
    self.intMarker.name = self.name
    self.intMarker.scale = 1.
    #FIXME : This only works for 3D and 6D
    self.intMarker.description = '{} {}D control'.format(self.name, dim)
    self.intMarker.pose.position = Point(*position)
    if orientation is not None:
      quat = list(Quaterniond(orientation).inverse().coeffs())
      self.intMarker.pose.orientation = Quaternion(*quat)

    self.marker = Marker()
    self.marker.type = Marker.CUBE
    self.marker.scale.x = 0.1
    self.marker.scale.y = 0.1
    self.marker.scale.z = 0.1
    self.marker.color.r = 1.
    self.marker.color.a = 1.

    self.visControl = InteractiveMarkerControl()
    self.visControl.always_visible = True
    self.visControl.markers.append(self.marker)

    def mkTransControl(name, axis):
      transControl = InteractiveMarkerControl()
      transControl.name = name
      transControl.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
      transControl.orientation = Quaternion(1., axis[0], axis[1], axis[2])
      return transControl

    def mkRotControl(name, axis):
      transControl = InteractiveMarkerControl()
      transControl.name = name
      transControl.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
      transControl.orientation = Quaternion(1., axis[0], axis[1], axis[2])
      return transControl

    self.transXControl = mkTransControl('trans_x', (1., 0., 0.))
    self.transYControl = mkTransControl('trans_y', (0., 1., 0.))
    self.transZControl = mkTransControl('trans_z', (0., 0., 1.))

    self.rotXControl = mkRotControl('rot_x', (1., 0., 0.))
    self.rotYControl = mkRotControl('rot_y', (0., 1., 0.))
    self.rotZControl = mkRotControl('rot_z', (0., 0., 1.))

    #FIXME : This maps dof to controls, but it is weird.
    #Works for drc_stairs and drc_egress
    self.controls = [self.rotZControl,
                     self.rotXControl,
                     self.rotYControl,
                     self.transZControl,
                     self.transXControl,
                     self.transYControl]

    self.intMarker.controls.append(self.visControl)

    if self.dof is None:
      self.intMarker.controls.extend(self.controls)
    else:
      print self.name, self.dof
      ctrl = [c for c, f in zip(self.controls, self.dof) if f == 0]
      self.intMarker.controls.extend(ctrl)

    self.server.insert(self.intMarker, self.callback)
    self.server.applyChanges()


  def clear(self):
    self.server.erase(self.intMarker.name)
    self.server.applyChanges()

