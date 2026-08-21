# ---------------------------------------------------------------------------
# FashionStart UART-Servo C++ driver build.
#
# Follows the FashionStart C++ SDK cmake instructions: it builds two shared
# libraries directly from the checked-out git submodule sources
# (motor_driver/fashionstart-uart-servo):
#
#   fsuartservo   - FashionStar servos control layer   (src/*.cpp)
#   cserialport   - itas109 CSerialPort transport      (dependency/CSerialPort)
#
# The driver ROS node links both. Because the sources are compiled in-tree the
# package is self-contained and no manual "make install" to /usr/local is needed.
# ---------------------------------------------------------------------------
set(FSUS_ROOT ${CMAKE_CURRENT_LIST_DIR}/fashionstart-uart-servo)
set(CSP_ROOT  ${FSUS_ROOT}/dependency/CSerialPort)

# ---------------------------------------------------------------------------
# libcserialport - serial transport (platform-specific sources)
# ---------------------------------------------------------------------------
add_library(cserialport SHARED
  ${CSP_ROOT}/src/SerialPort.cpp
  ${CSP_ROOT}/src/SerialPortBase.cpp
  ${CSP_ROOT}/src/SerialPortInfo.cpp
  ${CSP_ROOT}/src/SerialPortInfoBase.cpp
)

if(CMAKE_HOST_WIN32)
  target_sources(cserialport PRIVATE
    ${CSP_ROOT}/src/SerialPortInfoWinBase.cpp
    ${CSP_ROOT}/src/SerialPortWinBase.cpp
  )
  target_link_libraries(cserialport setupapi)
elseif(CMAKE_HOST_UNIX)
  target_sources(cserialport PRIVATE
    ${CSP_ROOT}/src/SerialPortInfoUnixBase.cpp
    ${CSP_ROOT}/src/SerialPortUnixBase.cpp
  )
  target_link_libraries(cserialport pthread)
elseif(APPLE)
  find_library(IOKIT_LIBRARY IOKit)
  find_library(FOUNDATION_LIBRARY Foundation)
  target_sources(cserialport PRIVATE
    ${CSP_ROOT}/src/SerialPortInfoUnixBase.cpp
    ${CSP_ROOT}/src/SerialPortUnixBase.cpp
  )
  target_link_libraries(cserialport ${FOUNDATION_LIBRARY} ${IOKIT_LIBRARY})
endif()

target_include_directories(cserialport
  PUBLIC
    ${CSP_ROOT}/include
)

# ---------------------------------------------------------------------------
# fsuartservo - FashionStar bus-servo logic on top of cserialport
# ---------------------------------------------------------------------------
add_library(fsuartservo SHARED
  ${FSUS_ROOT}/src/FashionStar_UartServo.cpp
  ${FSUS_ROOT}/src/FashionStar_UartServoProtocol.cpp
)

target_include_directories(fsuartservo
  PUBLIC
    ${FSUS_ROOT}/include
    ${CSP_ROOT}/include
)

target_link_libraries(fsuartservo cserialport)