add_library(madgwick_filter STATIC madgwick.cpp)
add_library(efaroe_filter STATIC efaroe.cpp)
# add_library(android_fusion STATIC android_fusion.cpp)

set(FUSION_C_FILES new_madgwick_FusionAhrs.cpp new_madgwick_FusionBias.cpp new_madgwick_FusionCompass.cpp)

set_source_files_properties(${FUSION_C_FILES} PROPERTIES LANGUAGE CXX)
add_library(fusion_filter STATIC ${FUSION_C_FILES})
