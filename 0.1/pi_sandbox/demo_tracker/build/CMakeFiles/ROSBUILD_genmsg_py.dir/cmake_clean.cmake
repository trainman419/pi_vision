FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/demo_tracker/msg"
  "../src/demo_tracker/srv"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/demo_tracker/msg/__init__.py"
  "../src/demo_tracker/msg/_Skeleton.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
