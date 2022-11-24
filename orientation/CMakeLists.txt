add_library(madgwick_filter STATIC src/madgwick.cpp)
add_library(efaroe_filter STATIC src/efaroe.cpp)
# add_library(android_fusion STATIC android_fusion.cpp)

set(FUSION_C_FILES src/new_madgwick_FusionAhrs.cpp src/new_madgwick_FusionBias.cpp src/new_madgwick_FusionCompass.cpp)

set_source_files_properties(${FUSION_C_FILES} PROPERTIES LANGUAGE CXX)
add_library(fusion_filter STATIC ${FUSION_C_FILES})
