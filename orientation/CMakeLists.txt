add_library(madgwick_filter STATIC madgwick.cpp)
add_library(efaroe_filter STATIC efaroe.cpp)

set(FUSION_C_FILES new_madgwick_FusionAhrs.c new_madgwick_FusionBias.c new_madgwick_FusionCompass.c)

set_source_files_properties(${FUSION_C_FILES} PROPERTIES LANGUAGE CXX)
add_library(fusion_filter STATIC ${FUSION_C_FILES})
