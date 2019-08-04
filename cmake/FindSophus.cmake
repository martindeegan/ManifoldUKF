find_path(SOPHUS_INCLUDE_DIR sophus/so3.hpp
    PATHS ${PROJECT_SOURCE_DIR}/thirdparty/Sophus
    NO_DEFAULT_PATH
)

set(SOPHUS_INCLUDE_DIRS ${SOPHUS_INCLUDE_DIR})

mark_as_advanced(SOPHUS_INCLUDE_DIR)
