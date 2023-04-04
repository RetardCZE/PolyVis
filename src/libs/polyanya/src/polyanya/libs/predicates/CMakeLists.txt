#------------------------------------------------------------------------------
# Macro set set the library properties
macro(LibraryProperties targetName DEBUG_EXTENSION)
  if( NOT BUILD_SHARED_LIBS AND MSVC)
    SET_TARGET_PROPERTIES( ${targetName}
      PROPERTIES
      DEBUG_OUTPUT_NAME lib${targetName}
      RELEASE_OUTPUT_NAME lib${targetName}  )
  endif()

  set_target_properties( ${targetName} PROPERTIES FOLDER ${targetName}Proj)


  #-- Set the Debug and Release names for the libraries
  SET_TARGET_PROPERTIES( ${targetName}
      PROPERTIES
      DEBUG_POSTFIX ${DEBUG_EXTENSION} )

  if(BUILD_SHARED_LIBS)
    if(APPLE)
      # use, i.e. don't skip the full RPATH for the build tree
      SET(CMAKE_SKIP_BUILD_RPATH  FALSE)

      # when building, don't use the install RPATH already
      # (but later on when installing)
      SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

      SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

      # add the automatically determined parts of the RPATH
      # which point to directories outside the build tree to the install RPATH
      SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

    endif(APPLE)

    if(CMAKE_SYSTEM_NAME MATCHES "Linux")
      set(CMAKE_INSTALL_RPATH "\$ORIGIN/../lib")
      SET_TARGET_PROPERTIES( ${targetName}
                PROPERTIES
                INSTALL_RPATH \$ORIGIN/../lib)
    endif()

  endif( BUILD_SHARED_LIBS)

endmacro(LibraryProperties DEBUG_EXTENSION)

set (predicates_SRCS
  constants.c
  predicates.c
  random.c
  printing.c
  )

add_library (Predicates ${LIB_TYPE} ${predicates_SRCS})

# target_compile_options(predicates PRIVATE
#         -std=c17
#         )
# target_compile_options(predicates PUBLIC
#         -D__cplusplus
#         )
