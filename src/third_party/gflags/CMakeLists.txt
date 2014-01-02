IF(WIN32)
  INCLUDE_DIRECTORIES(
    ../msinttypes
    ../pthreads-w32/include
  )
ENDIF(WIN32)

IF(CMAKE_COMPILER_IS_GNUCXX)
  SET(CMAKE_CXX_FLAGS "-fPIC")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

ADD_LIBRARY(gflags gflags.cc gflags_reporting.cc gflags_completions.cc)
TARGET_LINK_LIBRARIES(gflags pthread)

INSTALL(TARGETS gflags
        EXPORT  CeresExport
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib${LIB_SUFFIX}
        ARCHIVE DESTINATION lib${LIB_SUFFIX})

LIBMV_INSTALL_THIRD_PARTY_LIB(gflags)
