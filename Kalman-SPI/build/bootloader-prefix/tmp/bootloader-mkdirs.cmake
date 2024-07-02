# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/sanch/esp/v5.2.2/esp-idf/components/bootloader/subproject"
  "/home/sanch/prog/esp32/esp-idf-bmi160/Kalman-SPI/build/bootloader"
  "/home/sanch/prog/esp32/esp-idf-bmi160/Kalman-SPI/build/bootloader-prefix"
  "/home/sanch/prog/esp32/esp-idf-bmi160/Kalman-SPI/build/bootloader-prefix/tmp"
  "/home/sanch/prog/esp32/esp-idf-bmi160/Kalman-SPI/build/bootloader-prefix/src/bootloader-stamp"
  "/home/sanch/prog/esp32/esp-idf-bmi160/Kalman-SPI/build/bootloader-prefix/src"
  "/home/sanch/prog/esp32/esp-idf-bmi160/Kalman-SPI/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/sanch/prog/esp32/esp-idf-bmi160/Kalman-SPI/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/sanch/prog/esp32/esp-idf-bmi160/Kalman-SPI/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
