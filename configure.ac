AC_INIT([libfreenect], [0.5], [openkinect@googlegroups.com], [openkinect],
        [http://groups.google.com/group/openkinect])
AC_PREREQ([2.59])
AC_CONFIG_MACRO_DIR([m4])
AM_INIT_AUTOMAKE([1.10 -Wall no-define])
m4_include([pkg.m4])

PKG_CHECK_MODULES([LIBUSB], [libusb-1.0 >= 1.0.0])

AC_CONFIG_HEADERS([config.h])
AC_PROG_CC
AC_PROG_LIBTOOL
AC_C_INLINE
AM_PROG_CC_C_O

# LT_INIT([disable-static])

# Define these substitions here to keep all version information in one place.
# For information on how to properly maintain the library version information,
# refer to the libtool manual, section "Updating library version information":
# http://www.gnu.org/software/libtool/manual/html_node/Updating-version-info.html
AC_SUBST([LIBFREENECT_SO_VERSION], [0:1])
AC_SUBST([LIBFREENECT_API_VERSION], [1.0])

# Override the template file name of the generated .pc file, so that there
# is no need to rename the template file when the API version changes.
AC_CONFIG_FILES([Makefile
                 libfreenect-${LIBFREENECT_API_VERSION}.pc:libfreenect.pc.in])
AC_OUTPUT
