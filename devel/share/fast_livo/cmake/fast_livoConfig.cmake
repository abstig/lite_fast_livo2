# generated from catkin/cmake/template/pkgConfig.cmake.in

# append elements to a list and remove existing duplicates from the list
# copied from catkin/cmake/list_append_deduplicate.cmake to keep pkgConfig
# self contained
macro(_list_append_deduplicate listname)
  if(NOT "${ARGN}" STREQUAL "")
    if(${listname})
      list(REMOVE_ITEM ${listname} ${ARGN})
    endif()
    list(APPEND ${listname} ${ARGN})
  endif()
endmacro()

# append elements to a list if they are not already in the list
# copied from catkin/cmake/list_append_unique.cmake to keep pkgConfig
# self contained
macro(_list_append_unique listname)
  foreach(_item ${ARGN})
    list(FIND ${listname} ${_item} _index)
    if(_index EQUAL -1)
      list(APPEND ${listname} ${_item})
    endif()
  endforeach()
endmacro()

# pack a list of libraries with optional build configuration keywords
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_pack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  set(_argn ${ARGN})
  list(LENGTH _argn _count)
  set(_index 0)
  while(${_index} LESS ${_count})
    list(GET _argn ${_index} lib)
    if("${lib}" MATCHES "^(debug|optimized|general)$")
      math(EXPR _index "${_index} + 1")
      if(${_index} EQUAL ${_count})
        message(FATAL_ERROR "_pack_libraries_with_build_configuration() the list of libraries '${ARGN}' ends with '${lib}' which is a build configuration keyword and must be followed by a library")
      endif()
      list(GET _argn ${_index} library)
      list(APPEND ${VAR} "${lib}${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}${library}")
    else()
      list(APPEND ${VAR} "${lib}")
    endif()
    math(EXPR _index "${_index} + 1")
  endwhile()
endmacro()

# unpack a list of libraries with optional build configuration keyword prefixes
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_unpack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  foreach(lib ${ARGN})
    string(REGEX REPLACE "^(debug|optimized|general)${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}(.+)$" "\\1;\\2" lib "${lib}")
    list(APPEND ${VAR} "${lib}")
  endforeach()
endmacro()


if(fast_livo_CONFIG_INCLUDED)
  return()
endif()
set(fast_livo_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("TRUE" STREQUAL "TRUE")
  set(fast_livo_SOURCE_PREFIX /home/andi/lite_fast_livo2/src/FAST-LIVO2)
  set(fast_livo_DEVEL_PREFIX /home/andi/lite_fast_livo2/devel)
  set(fast_livo_INSTALL_PREFIX "")
  set(fast_livo_PREFIX ${fast_livo_DEVEL_PREFIX})
else()
  set(fast_livo_SOURCE_PREFIX "")
  set(fast_livo_DEVEL_PREFIX "")
  set(fast_livo_INSTALL_PREFIX /home/andi/lite_fast_livo2/install)
  set(fast_livo_PREFIX ${fast_livo_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'fast_livo' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(fast_livo_FOUND_CATKIN_PROJECT TRUE)

if(NOT "/usr/include/eigen3;/usr/local/pcl_1.14.1/include/pcl-1.14;/usr/include;/usr/include/ni;/usr/include/openni2;/usr/include/opencv4 " STREQUAL " ")
  set(fast_livo_INCLUDE_DIRS "")
  set(_include_dirs "/usr/include/eigen3;/usr/local/pcl_1.14.1/include/pcl-1.14;/usr/include;/usr/include/ni;/usr/include/openni2;/usr/include/opencv4")
  if(NOT " " STREQUAL " ")
    set(_report "Check the issue tracker '' and consider creating a ticket if the problem has not been reported yet.")
  elseif(NOT " " STREQUAL " ")
    set(_report "Check the website '' for information and consider reporting the problem.")
  else()
    set(_report "Report the problem to the maintainer 'claydergc <dev@livoxtech.com>' and request to fix the problem.")
  endif()
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir} " STREQUAL "include ")
      get_filename_component(include "${fast_livo_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'fast_livo' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  ${_report}")
      endif()
    else()
      message(FATAL_ERROR "Project 'fast_livo' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '/home/andi/lite_fast_livo2/src/FAST-LIVO2/${idir}'.  ${_report}")
    endif()
    _list_append_unique(fast_livo_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "/usr/local/pcl_1.14.1/lib/libpcl_common.so;/usr/local/pcl_1.14.1/lib/libpcl_kdtree.so;/usr/local/pcl_1.14.1/lib/libpcl_octree.so;/usr/local/pcl_1.14.1/lib/libpcl_search.so;/usr/local/pcl_1.14.1/lib/libpcl_sample_consensus.so;/usr/local/pcl_1.14.1/lib/libpcl_filters.so;/usr/local/pcl_1.14.1/lib/libpcl_io.so;/usr/local/pcl_1.14.1/lib/libpcl_features.so;/usr/local/pcl_1.14.1/lib/libpcl_ml.so;/usr/local/pcl_1.14.1/lib/libpcl_segmentation.so;/usr/local/pcl_1.14.1/lib/libpcl_visualization.so;/usr/local/pcl_1.14.1/lib/libpcl_surface.so;/usr/local/pcl_1.14.1/lib/libpcl_registration.so;/usr/local/pcl_1.14.1/lib/libpcl_keypoints.so;/usr/local/pcl_1.14.1/lib/libpcl_tracking.so;/usr/local/pcl_1.14.1/lib/libpcl_recognition.so;/usr/local/pcl_1.14.1/lib/libpcl_stereo.so;/usr/local/pcl_1.14.1/lib/libpcl_cuda_features.so;/usr/local/pcl_1.14.1/lib/libpcl_cuda_segmentation.so;/usr/local/pcl_1.14.1/lib/libpcl_cuda_sample_consensus.so;/usr/local/pcl_1.14.1/lib/libpcl_outofcore.so;/usr/local/pcl_1.14.1/lib/libpcl_gpu_containers.so;/usr/local/pcl_1.14.1/lib/libpcl_gpu_utils.so;/usr/local/pcl_1.14.1/lib/libpcl_gpu_octree.so;/usr/local/pcl_1.14.1/lib/libpcl_gpu_features.so;/usr/local/pcl_1.14.1/lib/libpcl_gpu_segmentation.so;/usr/local/pcl_1.14.1/lib/libpcl_people.so;/usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0;/usr/lib/x86_64-linux-gnu/libboost_iostreams.so.1.71.0;/usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0;/usr/lib/x86_64-linux-gnu/libboost_serialization.so.1.71.0;/usr/lib/libOpenNI.so;/usr/lib/libOpenNI2.so;/usr/lib/x86_64-linux-gnu/libusb-1.0.so;/usr/local/lib/libvtkChartsCore-9.4.so.9.4;/usr/local/lib/libvtkCommonColor-9.4.so.9.4;/usr/local/lib/libvtkCommonComputationalGeometry-9.4.so.9.4;/usr/local/lib/libvtkCommonCore-9.4.so.9.4;/usr/local/lib/libvtkCommonDataModel-9.4.so.9.4;/usr/local/lib/libvtkCommonExecutionModel-9.4.so.9.4;/usr/local/lib/libvtkCommonMath-9.4.so.9.4;/usr/local/lib/libvtkCommonMisc-9.4.so.9.4;/usr/local/lib/libvtkCommonTransforms-9.4.so.9.4;/usr/local/lib/libvtkFiltersCore-9.4.so.9.4;/usr/local/lib/libvtkFiltersExtraction-9.4.so.9.4;/usr/local/lib/libvtkFiltersGeneral-9.4.so.9.4;/usr/local/lib/libvtkFiltersGeometry-9.4.so.9.4;/usr/local/lib/libvtkFiltersModeling-9.4.so.9.4;/usr/local/lib/libvtkFiltersSources-9.4.so.9.4;/usr/local/lib/libvtkImagingCore-9.4.so.9.4;/usr/local/lib/libvtkImagingSources-9.4.so.9.4;/usr/local/lib/libvtkInteractionImage-9.4.so.9.4;/usr/local/lib/libvtkInteractionStyle-9.4.so.9.4;/usr/local/lib/libvtkInteractionWidgets-9.4.so.9.4;/usr/local/lib/libvtkIOCore-9.4.so.9.4;/usr/local/lib/libvtkIOGeometry-9.4.so.9.4;/usr/local/lib/libvtkIOImage-9.4.so.9.4;/usr/local/lib/libvtkIOLegacy-9.4.so.9.4;/usr/local/lib/libvtkIOPLY-9.4.so.9.4;/usr/local/lib/libvtkRenderingAnnotation-9.4.so.9.4;/usr/local/lib/libvtkRenderingCore-9.4.so.9.4;/usr/local/lib/libvtkRenderingContext2D-9.4.so.9.4;/usr/local/lib/libvtkRenderingLOD-9.4.so.9.4;/usr/local/lib/libvtkRenderingFreeType-9.4.so.9.4;/usr/local/lib/libvtkViewsCore-9.4.so.9.4;/usr/local/lib/libvtkViewsContext2D-9.4.so.9.4;/usr/local/lib/libvtkRenderingOpenGL2-9.4.so.9.4;/usr/local/lib/libvtkRenderingContextOpenGL2-9.4.so.9.4;/usr/lib/x86_64-linux-gnu/libflann_cpp.so;/usr/lib/x86_64-linux-gnu/libqhull_r.so;/usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_core.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_video.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_face.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_text.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.2.0;libSophus.so")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^(debug|optimized|general)$")
    list(APPEND fast_livo_LIBRARIES ${library})
  elseif(${library} MATCHES "^-l")
    list(APPEND fast_livo_LIBRARIES ${library})
  elseif(${library} MATCHES "^-")
    # This is a linker flag/option (like -pthread)
    # There's no standard variable for these, so create an interface library to hold it
    if(NOT fast_livo_NUM_DUMMY_TARGETS)
      set(fast_livo_NUM_DUMMY_TARGETS 0)
    endif()
    # Make sure the target name is unique
    set(interface_target_name "catkin::fast_livo::wrapped-linker-option${fast_livo_NUM_DUMMY_TARGETS}")
    while(TARGET "${interface_target_name}")
      math(EXPR fast_livo_NUM_DUMMY_TARGETS "${fast_livo_NUM_DUMMY_TARGETS}+1")
      set(interface_target_name "catkin::fast_livo::wrapped-linker-option${fast_livo_NUM_DUMMY_TARGETS}")
    endwhile()
    add_library("${interface_target_name}" INTERFACE IMPORTED)
    if("${CMAKE_VERSION}" VERSION_LESS "3.13.0")
      set_property(
        TARGET
        "${interface_target_name}"
        APPEND PROPERTY
        INTERFACE_LINK_LIBRARIES "${library}")
    else()
      target_link_options("${interface_target_name}" INTERFACE "${library}")
    endif()
    list(APPEND fast_livo_LIBRARIES "${interface_target_name}")
  elseif(TARGET ${library})
    list(APPEND fast_livo_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND fast_livo_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /home/andi/lite_fast_livo2/devel/lib;/opt/ros/noetic/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(fast_livo_LIBRARY_DIRS ${lib_path})
      list(APPEND fast_livo_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'fast_livo'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND fast_livo_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(fast_livo_EXPORTED_TARGETS "")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${fast_livo_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "geometry_msgs;nav_msgs;roscpp;rospy;std_msgs;message_runtime;cv_bridge;vikit_common;vikit_ros;image_transport")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 fast_livo_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${fast_livo_dep}_FOUND)
      find_package(${fast_livo_dep} REQUIRED NO_MODULE)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${fast_livo_dep} REQUIRED NO_MODULE ${depend_list})
  endif()
  _list_append_unique(fast_livo_INCLUDE_DIRS ${${fast_livo_dep}_INCLUDE_DIRS})

  # merge build configuration keywords with library names to correctly deduplicate
  _pack_libraries_with_build_configuration(fast_livo_LIBRARIES ${fast_livo_LIBRARIES})
  _pack_libraries_with_build_configuration(_libraries ${${fast_livo_dep}_LIBRARIES})
  _list_append_deduplicate(fast_livo_LIBRARIES ${_libraries})
  # undo build configuration keyword merging after deduplication
  _unpack_libraries_with_build_configuration(fast_livo_LIBRARIES ${fast_livo_LIBRARIES})

  _list_append_unique(fast_livo_LIBRARY_DIRS ${${fast_livo_dep}_LIBRARY_DIRS})
  _list_append_deduplicate(fast_livo_EXPORTED_TARGETS ${${fast_livo_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${fast_livo_DIR}/${extra})
  endif()
  include(${extra})
endforeach()
