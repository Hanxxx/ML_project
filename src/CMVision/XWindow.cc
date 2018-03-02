/*
  XWindow.cc

  Some xlib stuff for the CMVision demo.
  Author: Ziauddin Khan <zkhan@cs.cmu.edu>
      ,_      _,
        '.__.'
   '-,   (__)   ,-'
     '._ .::. _.'
       _'(^^)'_
    _,` `>\/<` `,_
   `  ,-` )( `-,  `
      |  /==\  |
    ,-'  |=-|  '-,
         )-=(
jgs      \__/

  Revision History:
  3-30-00, Initial release.

 */ 
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/Xos.h>
#include <X11/Xatom.h>

#include "XWindow.h"
#include <iostream>

// default event hanlder
bool XWindow::handleEvent(XEvent &report){
  // ignore any event
  return false;
}


// default window update function
void XWindow::update(){
  // do nothing
}

// default paint funciton
void XWindow::paint(){
}

// call this to display the window
void XWindow::showWindow(){
  XMapWindow(display, win); // display window
}

// return the color depth XWindows is currently working at
int XWindow::getXDepth(){
  return x_depth;
}

// default constructor
XWindow::XWindow(Display *d, int s, char* name, int xp, int yp, int w, int h){
  display = d;
  screen_num = s;
  x = xp;
  y = yp;
  width = w;
  height = h;

  // make our window
  win = XCreateSimpleWindow(display, 
			    RootWindow(display,screen_num), 
			    x, y, width, height, 0, 
			    BlackPixel(display, screen_num), 
			    BlackPixel(display, screen_num));

  size_hints = XAllocSizeHints();
  size_hints->min_width = 300;
  size_hints->min_height = 200;

  XmbSetWMProperties(display, win, name, name, NULL, 0, size_hints, NULL, NULL);
  XSelectInput(display, win, ExposureMask | KeyPressMask | 
	       ButtonPressMask | StructureNotifyMask);

  // make the pesky, GC
  XGCValues values;
  unsigned long valuemask = 0; // ignore XGCvalues and use defaults 
  gc = XCreateGC(display, win, valuemask, &values);

  // get the screen depth
  XWindowAttributes wa;
  XGetWindowAttributes(display, win, &wa);
  x_depth = wa.depth;
  if(x_depth < 24){
    cerr << "24-bit (or better) display only, my friend" << endl;
    exit(-1);
  }
}

// destructor... get rid of that pesky GC
XWindow::~XWindow(){
  XFreeGC(display, gc);
}


// use this to generate an XImage from an array of byte-size RGB values
XImage *XWindow::RGBToXImage(int w, int h, unsigned char *data){
  XImage *ximg = new XImage;
  
  ximg->width = w;
  ximg->height = h;
  ximg->xoffset = 0;
  ximg->format = ZPixmap;
  ximg->data = (char*)data;
  

  ximg->byte_order = MSBFirst;
  ximg->bitmap_unit = 8;
  ximg->bitmap_bit_order = LSBFirst;

  ximg->bitmap_pad = 8;

  ximg->depth = 24;
  ximg->bytes_per_line = w * (24/8);
  ximg->bits_per_pixel = 24;

  //ximg->red_mask   = 0xFF << 16; magic markers
  //ximg->green_mask = 0xFF << 8;
  //ximg->blue_mask  = 0xFF << 0;  

  XInitImage(ximg);

  return ximg;
}


// draw an image on this window, at a specific X,Y location
void XWindow::putImage(XImage *ximg, int x, int y, int w, int h){
  XPutImage(display, win, gc, ximg, 0,0,x,y,w,h);
}

// what do you know? draw a rectangle
void XWindow::drawRect(int x, int y, int w, int h, int c){
  XSetForeground(display, gc, c);
  XDrawRectangle(display, win, gc, x, y, w, h);  
}

// what do you know? draw a point
void XWindow::drawPoint(int x, int y, int c){
  XSetForeground(display, gc, c);
  XDrawPoint(display, win, gc, x, y);
}
