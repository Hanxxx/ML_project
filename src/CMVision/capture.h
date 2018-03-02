#ifndef __CAPTURE_H__
#define __CAPTURE_H__

#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <fcntl.h>

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>

#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/videodev.h>

#define DEFAULT_VIDEO_DEVICE  "/dev/video"
#define VIDEO_STANDARD        "NTSC"
#define DEFAULT_VIDEO_FORMAT  V4L2_PIX_FMT_YUYV
#define DEFAULT_IMAGE_WIDTH   160
#define DEFAULT_IMAGE_HEIGHT  120
#define STREAMBUFS            4

class capture{
  struct vimage_t{
    v4l2_buffer vidbuf;
    char *data;
  };

  int vid_fd;                    // video device
  vimage_t vimage[STREAMBUFS];      // buffers for images
  struct v4l2_format fmt;        // video format request

  unsigned char *current; // most recently captured frame
  timeval tv;             // best estimate frame time stamp
  int width,height;       // dimensions of video frame
public:
  capture() {vid_fd = 0; current=NULL;}
  ~capture() {close();}

  bool initialize(char *device,int nwidth,int nheight,int nfmt);
  bool initialize(int nwidth,int nheight)
    {return(initialize(NULL,nwidth,nheight,0));}
  bool initialize()
    {return(initialize(NULL,0,0,0));}

  void close();

  unsigned char *captureFrame();

  unsigned char *getFrame() {return(current);}
  timeval getFrameTime() {return(tv);}
  int getWidth() {return(width);}
  int getHeight() {return(height);}
};

#endif // __CAPTURE_H__
