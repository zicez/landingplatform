FILE(REMOVE_RECURSE
  "src/ar_drone2/msg"
  "msg_gen"
  "msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/ar_drone2/msg/__init__.py"
  "src/ar_drone2/msg/_Navdata.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
