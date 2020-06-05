function(EXPORT_CARTESIAN_INTERFACE)

    if(NOT DEFINED ${PROJECT_NAME}_INSTALL_INCLUDE_DIR)
        set(${PROJECT_NAME}_INSTALL_INCLUDE_DIR include)
    endif()

    # ...executables...
    if(NOT DEFINED ${PROJECT_NAME}_INSTALL_BIN_DIR)
        set(${PROJECT_NAME}_INSTALL_BIN_DIR bin)
    endif()

    # ...libraries...
    if(NOT DEFINED ${PROJECT_NAME}_INSTALL_LIBRARY_DIR)
        set(${PROJECT_NAME}_INSTALL_LIBRARY_DIR lib)
    endif()

    # .. and cmake files
    if(NOT DEFINED ${PROJECT_NAME}_INSTALL_CONFIG_DIR)
        set(${PROJECT_NAME}_INSTALL_CONFIG_DIR
            ${${PROJECT_NAME}_INSTALL_LIBRARY_DIR}/cmake/${PROJECT_NAME})
    endif()

    # write the package version file
    include(CMakePackageConfigHelpers)
    set(config_version_file ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake)
    write_basic_package_version_file(
        ${config_version_file}
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY AnyNewerVersion
        )

    # create export for build tree
    export(EXPORT ${PROJECT_NAME}Targets
        NAMESPACE ${PROJECT_NAME}::
        FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake"
        )

    # Configure 'xxxConfig.cmake' for a build tree
    set(build_config ${CMAKE_BINARY_DIR}/${PROJECT_NAME}Config.cmake)
    configure_package_config_file(
        cmake/${PROJECT_NAME}Config.cmake.in
        ${build_config}
        INSTALL_DESTINATION "${PROJECT_BINARY_DIR}"
        )

    install(
        EXPORT ${PROJECT_NAME}Targets
        NAMESPACE ${PROJECT_NAME}::
        FILE ${PROJECT_NAME}Targets.cmake
        DESTINATION ${${PROJECT_NAME}_INSTALL_CONFIG_DIR}
        )

    set(install_config ${PROJECT_BINARY_DIR}/CMakeFiles/${PROJECT_NAME}Config.cmake)
    configure_package_config_file(
        cmake/${PROJECT_NAME}Config.cmake.in
        ${install_config}
        INSTALL_DESTINATION ${${PROJECT_NAME}_INSTALL_CONFIG_DIR}
        )

    # Install config files
    install(
        FILES
        ${config_version_file}
        ${install_config}
        DESTINATION "${${PROJECT_NAME}_INSTALL_CONFIG_DIR}"
        )

endfunction()
