#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := Gateway

COMPONENT_ADD_INCLUDEDIRS := components/include

include $(IDF_PATH)/make/project.mk

target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "html/webserver.html" TEXT)
