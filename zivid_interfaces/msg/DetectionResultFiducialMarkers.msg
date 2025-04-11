# The detected fiducial markers.
# Only markers belonging to the dictionary that was used at detection are reported. Markers are filtered to only include
# the ones with allowed ids, specified at detection. Markers can be detected multiple times if they are present on the
# scene multiple times.
MarkerShape[] detected_markers

# The allowed marker ids this detection result was made with.
int32[] allowed_marker_ids
