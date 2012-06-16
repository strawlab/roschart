include $(shell rospack find mk)/cmake.mk

EXTRA_CMAKE_FLAGS=$(shell pkg-config --cflags gtk+-2.0 gthread-2.0)
