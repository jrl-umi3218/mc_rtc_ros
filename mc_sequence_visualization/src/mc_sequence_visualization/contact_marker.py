import rospy

from geometry_msgs.msg import Point, Quaternion
from interactive_markers.interactive_marker_server import InteractiveMarkerServer,\
                                                          InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl, Marker


class ContactControlMarker(object):
  def __init__(self, name, callback):
    self.name = name
    self.callback = callback
    self.server = InteractiveMarkerServer('%s_marker' % name)
    self.server.applyChanges()

  def init(self, position):
    self.intMarker = InteractiveMarker()
    self.intMarker.header.frame_id = '/robot_map'
    self.intMarker.name = self.name
    self.intMarker.description = 'Contact 3d control'
    self.intMarker.pose.position = Point(*position)
    self.intMarker.scale = 1.

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

    self.transXControl = mkTransControl('trans_x', (1., 0., 0.))
    self.transYControl = mkTransControl('trans_y', (0., 1., 0.))
    self.transZControl = mkTransControl('trans_z', (0., 0., 1.))

    self.intMarker.controls.append(self.visControl)
    self.intMarker.controls.append(self.transXControl)
    self.intMarker.controls.append(self.transYControl)
    self.intMarker.controls.append(self.transZControl)

    self.server.insert(self.intMarker, self.callback)
    self.server.applyChanges()

  def clear(self):
    self.server.erase(self.intMarker.name)
    self.server.applyChanges()

