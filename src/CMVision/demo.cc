/*
  demo.cc

  Demo code for CMVision. Use this to learn stuff.
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

#include <stdio.h>
#include <iostream>

#include "CMVWindow.h"

// check if there are X-Windows events waiting 
bool eventsPending(Display *display){
  if(XPending(display) == 0)
    false;
  else 
    true;
}


// main, main, lalala 
int main(int argc, char**argv){
  char *display_name = NULL;
  Display *display;
  int screen_num; 

  // connect to X server
  if ((display=XOpenDisplay(display_name)) == NULL){
    cerr <<  "CMVision demo cannot connect to X server, doh! xserver is -->  " << XDisplayName(display_name) << endl; 
    exit(-1);
  }
  screen_num = DefaultScreen(display);

  CMVWindow cmvWindow(display, screen_num);
  cmvWindow.showWindow();

  // main event loop
  XEvent report;
  while(1){
    if(eventsPending(display)){ 
      XNextEvent(display, &report);
      // let the window(s) deal w/ events first
      if(!cmvWindow.handleEvent(report)){ 
	switch(report.type){ // then, we'll deal with it
	case ButtonPress: // exit on key or button press
	case KeyPress:
	  XCloseDisplay(display);      
	  exit(1);
	}
      }
    }
    cmvWindow.update();
    // update window(s)
  }
  return 0;
}
