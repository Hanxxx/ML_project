#ifndef __XWINDOW__H__
#define __XWINDOW__H__

#include <X11/Xlib.h>

class XWindow{
 protected:
  Display *display;
  int screen_num;
  Window win;
  GC gc;
  unsigned int width, height;	 // initial window size
  int x, y;    // initial window posiion
  XSizeHints *size_hints;
  int x_depth;
  
 public:
  XWindow(Display *d, int s, char* name, int xp, int yp, int w, int h);
  ~XWindow();
  bool handleEvent(XEvent &report);
  void update();
  void paint();
  void showWindow();
  int getXDepth();
  XImage *RGBToXImage(int w, int h, unsigned char *data);
  void putImage(XImage *ximg, int x, int y, int w, int h);
  void drawRect(int x, int y, int w, int h, int c);
  void drawPoint(int x, int y, int c);
};

#endif /* __XWINDOW__H__ */
