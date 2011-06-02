FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/demo_tracker/msg"
  "../src/demo_tracker/srv"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/Skeleton.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_Skeleton.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
