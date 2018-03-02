/*=========================================================================
    capture.cc
  -------------------------------------------------------------------------
    Example code for video capture under Video4Linux II
  -------------------------------------------------------------------------
    Copyright 1999, 2000
    Anna Helena Reali Costa, James R. Bruce
    School of Computer Science
    Carnegie Mellon University
  -------------------------------------------------------------------------
    This source code is distributed "as is" with absolutely no warranty.
    See LICENSE, which should be included with this distribution.
  -------------------------------------------------------------------------
    Revision History:
      2000-02-05:  Ported to work with V4L2 API
      1999-11-23:  Quick C++ port to simplify & wrap in an object (jbruce) 
      1999-05-01:  Initial version (annar)
  =========================================================================*/

#include "capture.h"


//==== Capture Class Implementation =======================================//

bool capture::initialize(char *device,int nwidth,int nheight,int nfmt)
{
  struct v4l2_requestbuffers req;
  int err;
  int i;

  // Set defaults if not given
  if(!device) device = DEFAULT_VIDEO_DEVICE;
  if(!nfmt) nfmt = DEFAULT_VIDEO_FORMAT;
  if(!nwidth || !nheight){
    nwidth  = DEFAULT_IMAGE_WIDTH;
    nheight = DEFAULT_IMAGE_HEIGHT;
  }

  // Open the video device
  vid_fd = open(device, O_RDONLY);
  if(vid_fd == -1){
    printf("Could not open video device [%s]\n",device);
    return(false);
  }

  fmt.type = V4L2_BUF_TYPE_CAPTURE;
  err = ioctl(vid_fd, VIDIOC_G_FMT, &fmt);
  if(err){
    printf("G_FMT returned error %d\n",errno);
    return(false);
  }

  fmt.fmt.pix.width = nwidth;
  fmt.fmt.pix.height = nheight;
  fmt.fmt.pix.pixelformat = nfmt;
  // ioctl(vid, VIDIOC_S_FMT, &fmt);
  // Set video format
  // fmt.fmt.pix.pixelformat = nfmt;

  // need to repeat following twice?
  err = ioctl(vid_fd, VIDIOC_S_FMT, &fmt);
  if(err){
    printf("S_FMT returned error %d\n",errno);
    return(false);
  }

  // Request mmap-able capture buffers
  req.count = STREAMBUFS;
  req.type  = V4L2_BUF_TYPE_CAPTURE;
  err = ioctl(vid_fd, VIDIOC_REQBUFS, &req);
  if(err < 0 || req.count < 1){
    printf("REQBUFS returned error %d, count %d\n",
	   errno,req.count);
    return(false);
  }

  for(i=0; i<req.count; i++){
    vimage[i].vidbuf.index = i;
    vimage[i].vidbuf.type = V4L2_BUF_TYPE_CAPTURE;
    err = ioctl(vid_fd, VIDIOC_QUERYBUF, &vimage[i].vidbuf);
    if(err < 0){
      printf("QUERYBUF returned error %d\n",errno);
      return(false);
    }

    vimage[i].data = (char*)mmap(0, vimage[i].vidbuf.length, PROT_READ,
			  MAP_SHARED, vid_fd, 
			  vimage[i].vidbuf.offset);
    if((int)vimage[i].data == -1){
      printf("mmap() returned error %d\n", errno);
      return(false);
    }
  }

  for(i=0; i<req.count; i++){
    if((err = ioctl(vid_fd, VIDIOC_QBUF, &vimage[i].vidbuf))){
      printf("QBUF returned error %d\n",errno);
      return(false);
    }
  }

  // Turn on streaming capture
  err = ioctl(vid_fd, VIDIOC_STREAMON, &vimage[0].vidbuf.type);
  if(err){
    printf("STREAMON returned error %d\n",errno);
    return(false);
  }

  width   = nwidth;
  height  = nheight;
  current = NULL;

  return(true);
}

void capture::close()
{
  int i,t;

  if(vid_fd >= 0){
    t = V4L2_BUF_TYPE_CAPTURE;
    ioctl(vid_fd, VIDIOC_STREAMOFF, &t);

    for(i=0; i<STREAMBUFS; i++){
      if(vimage[i].data){
        munmap(vimage[i].data,vimage[i].vidbuf.length);
      }
    }
  }
}

struct v4l2_buffer tempbuf;

unsigned char *capture::captureFrame()
{
  // struct v4l2_buffer tempbuf;
  int err;

  fd_set          rdset;
  struct timeval  timeout;
  int		  n;

  FD_ZERO(&rdset);
  FD_SET(vid_fd, &rdset);
  timeout.tv_sec = 1;
  timeout.tv_usec = 0;
  n = select(vid_fd + 1, &rdset, NULL, NULL, &timeout);
  err = -1;
  if (n == -1)
    fprintf(stderr, "select error.\n");
  else if (n == 0)
    fprintf(stderr, "select timeout\n");
  else if (FD_ISSET(vid_fd, &rdset))
    err = 0;
  if(err) return(NULL);

  // Grab last frame
  tempbuf.type = vimage[0].vidbuf.type;
  err = ioctl(vid_fd, VIDIOC_DQBUF, &tempbuf);
  if(err) printf("DQBUF returned error %d\n",errno);

  // Set current to point to captured frame data
  current = (unsigned char *)vimage[tempbuf.index].data;

  // Initiate the next capture
  err = ioctl(vid_fd, VIDIOC_QBUF, &tempbuf);
  if(err) printf("QBUF returned error %d\n",errno);

  return(current);
}
