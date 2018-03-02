#ifndef __CAPTUREWINDOW__H__
#define __CAPTUREWINDOW__H__

#include "XWindow.h"
#include "capture.h"
#include "cmvision.h"

class CMVWindow : public XWindow{
protected:  
  capture c;
  CMVision cmv;

  const int defaultWidth;
  const int defaultHeight;
  const int logoWidth;
  const int logoHeight;
  const int antWidth;
  const int antHeight;


  unsigned char *cmvLogo;
  unsigned char *antLogo;
  unsigned char *rgbData;
  
  bool logoVisible;

  // used to bound r,g,b values based 
  int bound(int low,int high,int n){
    if(n < low ) n = low;
    if(n > high) n = high;
    return(n);
  }
  
  // convert a YUV to RGB to display (thanks Kevin, Ashley, and Rosemary)
  void YUVToRGB(rgb *dest, yuv422 *src,int w,int h) {
    int i,s;
    int y,u,v;
    rgb c;
    yuv422 p;

    s = w * h;

    for(i=0; i<s; i++){
      p = src[i / 2];
      u = 2*p.v - 256;
      v = 2*p.u - 256;

      y = p.y1;
      c.red   = bound(0,255,y + u);
      c.green = bound(0,255,(int)(y - 0.51*u - 0.19*v));
      c.blue  = bound(0,255,y + v);
      dest[i] = c;
      
      y = p.y2;
      c.red   = bound(0,255,y + u);
      c.green = bound(0,255,(int)(y - 0.51*u - 0.19*v));
      c.blue  = bound(0,255,y + v);
      dest[i + 1] = c;
    }
  }

  unsigned int colors[CMV_MAX_COLORS];
public:
  CMVWindow(Display *d, int s);
  ~CMVWindow();
  void update();
  void paint();
  void showWindow();
  bool handleEvent(XEvent &report);
};

#endif
