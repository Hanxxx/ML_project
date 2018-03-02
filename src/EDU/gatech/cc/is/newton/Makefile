#
# $RCSfile: Makefile,v $
# $Revision: 1.3 $
# $Date: 2000/03/07 20:28:23 $
# $Locker:  $
#  author: Tucker Balch
#

THISDIR = src/EDU/gatech/cc/is/newton
SUBDIRS =
JAVAFILES = $(shell echo *.java)

include ../../../../../../Makefile.src

#
# C
#
#LOCAL_CFLAGS = ${INCLUDES} ${CFLAGS} -g
LOCAL_CFLAGS = ${INCLUDES} ${CFLAGS} 
#LOCAL_CFLAGS = ${INCLUDES} ${CFLAGS} -DNEWTONDEBUG -g
INCLUDES = -I. -I/usr/include
LINKLIBS = -L. -lnewton -L/usr/lib -lc
AR = ar

SRCS = init_newton.c newton_parse.c \
       newton_serial.c Newton_interface.c

NEWTONOBJS = init_newton.o newton_parse.o \
             newton_serial.o Newton_interface.o

NEWTON_LIBRARY = libnewton.so

HEADERS = newton.h

#- ------------------------------------------------------------------------

native:: $(NEWTON_LIBRARY) newton_test parse_test

clean::
	$(RM) -f newton_test parse_test EDU_gatech_cc_newton_Newton.h

$(NEWTON_LIBRARY): $(NEWTONOBJS)
	ld ${LDFLAGS} $(NEWTONOBJS) -o libnewton.so

EDU_gatech_cc_is_newton_Newton.h: Newton.java
	javah -jni EDU.gatech.cc.is.newton.Newton

init_newton.o: init_newton.c newton.h
	$(CC) $(LOCAL_CFLAGS) -c init_newton.c

locate_objects.o: locate_objects.c newton.h
	$(CC) $(LOCAL_CFLAGS) -c locate_objects.c

newton_parse.o: newton_parse.c newton.h
	$(CC) $(LOCAL_CFLAGS) -c newton_parse.c

newton_serial.o: newton_serial.c newton.h
	$(CC) $(LOCAL_CFLAGS) -c newton_serial.c

Newton_interface.o : Newton_interface.c EDU_gatech_cc_is_newton_Newton.h
	$(CC) $(LOCAL_CFLAGS) -c Newton_interface.c
	
#newton_serial.o: newton_serial.c newton.h
#	$(CC)  -c newton_serial.c

newton_test: newton_test.c $(NEWTON_LIBRARY)
	$(CC) -o $@ $< $(LINKLIBS)

parse_test: parse_test.c $(NEWTON_LIBRARY)
	$(CC) -o $@ $< $(LINKLIBS)

#- ~robot/newton/Makefile -#
