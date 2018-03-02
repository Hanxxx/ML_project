#
# $RCSfile: Makefile,v $
# $Revision: 1.3 $
# $Date: 2000/03/07 20:28:25 $
# $Locker:  $
#  author: Tucker Balch
#

THISDIR = src/EDU/gatech/cc/is/nomad150
SUBDIRS = 
JAVAFILES = $(shell echo *.java)

include ../../../../../../Makefile.src

LOCAL_CFLAGS = ${CFLAGS} -DNOMAD150

native:: libNdirect.so

clean:: 
	$(RM) EDU_gatech_cc_is_nomad150_Ndirect.h

EDU_gatech_cc_is_nomad150_Ndirect.h: Ndirect.class
	javah -jni EDU.gatech.cc.is.nomad150.Ndirect

Ndirect.o: Ndirect.c
	${CC} ${LOCAL_CFLAGS} -c Ndirect.c -o Ndirect.o

Ndirect_interface.o: Ndirect_interface.c EDU_gatech_cc_is_nomad150_Ndirect.h
	${CC} ${LOCAL_CFLAGS} -c Ndirect_interface.c -o Ndirect_interface.o

libNdirect.so: Ndirect.o Ndirect_interface.o 
	ld ${LDFLAGS} Ndirect_interface.o Ndirect.o -o libNdirect.so
