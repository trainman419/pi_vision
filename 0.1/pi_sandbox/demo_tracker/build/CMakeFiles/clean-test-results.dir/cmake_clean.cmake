FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/demo_tracker/msg"
  "../src/demo_tracker/srv"
  "CMakeFiles/clean-test-results"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/clean-test-results.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
