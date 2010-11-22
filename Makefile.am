AUTOMAKE_OPTIONS = subdir-objects

## Additional flags to pass to aclocal when it is invoked automatically at
## make time. The ${ACLOCAL_FLAGS} variable is picked up from the environment
## to provide a way for the user to supply additional arguments.
ACLOCAL_AMFLAGS = ${ACLOCAL_FLAGS}

## Define a libtool archive target "libexample-@EXAMPLE_API_VERSION@.la", with
## @EXAMPLE_API_VERSION@ substituted into the generated Makefile at configure
## time.
## The libtool archive file (.la) will be installed into the directory named
## by the predefined variable $(bindir), along with the actual shared library
## file (.so).
lib_LTLIBRARIES = libfreenect-@LIBFREENECT_API_VERSION@.la

## Define the source file list for the "libexample-@EXAMPLE_API_VERSION@.la"
## target.  Note that @EXAMPLE_API_VERSION@ is not interpreted by Automake and
## will therefore be treated as if it were literally part of the target name,
## and the variable name derived from that.
## The file extension .cc is recognized by Automake, and makes it produce
## rules which invoke the C++ compiler to produce a libtool object file (.lo)
## from each source file.  Note that it is not necessary to list header files
## which are already listed elsewhere in a _HEADERS variable assignment.
libfreenect_@LIBFREENECT_API_VERSION@_la_SOURCES = libfreenect/inits.c \
	libfreenect/core.c libfreenect/cameras.c \
	libfreenect/tilt.c libfreenect/usb_libusb10.c

BUILT_SOURCES = libfreenect/inits.c

libfreenect/inits.c:
	python libfreenect/geninits.py libfreenect/inits.txt libfreenect/inits.c

## Instruct libtool to include ABI version information in the generated shared
## library file (.so).  The library ABI version is defined in configure.ac, so
## that all version information is kept in one place.
libfreenect_@LIBFREENECT_API_VERSION@_la_LDFLAGS = $(all_libraries) -version-info $(LIBFREENECT_SO_VERSION)
# libfreenect_@LIBFREENECT_API_VERSION@_la_LDFLAGS = $(all_libraries) -dynamiclib -version-info $(LIBFREENECT_SO_VERSION)


libfreenect_@LIBFREENECT_API_VERSION@_la_CPPFLAGS = $(LIBUSB_CFLAGS)
libfreenect_@LIBFREENECT_API_VERSION@_la_LIBADD = $(LIBUSB_LIBS)

libfreenect_includedir = $(includedir)/libfreenect-$(LIBFREENECT_API_VERSION)
nobase_libfreenect_include_HEADERS = libfreenect/usb_libusb10.h 
libfreenect_include_HEADERS = libfreenect/libfreenect.h

libfreenect_libincludedir = $(libdir)/libfreenect-$(LIBFREENECT_API_VERSION)/include

## Install the generated pkg-config file (.pc) into the expected location for
## architecture-dependent package configuration information.  Occasionally,
## pkg-config files are also used for architecture-independent data packages,
## in which case the correct install location would be $(datadir)/pkgconfig.
pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = libfreenect-$(LIBFREENECT_API_VERSION).pc

## Define an independent executable script for inclusion in the distribution
## archive.  However, it will not be installed on an end user's system due to
## the noinst_ prefix.
dist_noinst_SCRIPTS = autogen.sh
