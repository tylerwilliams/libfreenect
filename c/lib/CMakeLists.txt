include_directories (${CMAKE_SOURCE_DIR}/include)
IF(WIN32)
include_directories (${CMAKE_SOURCE_DIR}/include/win32)
ENDIF()
include_directories (${CMAKE_SOURCE_DIR}/lib)

find_package(Threads REQUIRED)
find_package(USB REQUIRED)
include_directories(${USB_INCLUDE_DIRS})

IF(WIN32)
  add_library (freenect cameras-win32.c ${CMAKE_CURRENT_BINARY_DIR}/inits.c)
ELSE()
  add_library (freenect cameras-libusb1.c ${CMAKE_CURRENT_BINARY_DIR}/inits.c)
ENDIF()

find_library (PTHREAD pthread)
target_link_libraries (freenect ${CMAKE_THREAD_LIBS_INIT} ${USB_LIBRARIES})

add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/inits.c
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/geninits.py
    MAIN_DEPENDENCY inits.txt
    COMMAND python ${CMAKE_CURRENT_SOURCE_DIR}/geninits.py ${CMAKE_CURRENT_SOURCE_DIR}/inits.txt ${CMAKE_CURRENT_BINARY_DIR}/inits.c)
