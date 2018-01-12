#pragma onceo

#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>

#include <SpaceVecAlg/SpaceVecAlg>

namespace vm = visualization_msgs;

vm::Marker makeVisual(int t, const vm::InteractiveMarker & marker);

vm::InteractiveMarkerControl & makeVisualControl(int t,
                                                 vm::InteractiveMarker & marker);

vm::InteractiveMarker make6DMarker(const std::string & name,
                                   bool control_position,
                                   bool control_orientation);
