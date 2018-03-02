/*
  CMVWindow.cc

  Demonstration of some features of CMVision, initiail release version.
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
#include <string.h>
#include <iostream>
#include <fstream>
#include <strstream>

#include "CMVWindow.h"

// default constructor 
CMVWindow::CMVWindow(Display *d, int s) : 
  XWindow(d, s, "CMVision Demo ", 0,0, 300, 640), 
  defaultWidth(DEFAULT_IMAGE_WIDTH),
  defaultHeight(DEFAULT_IMAGE_HEIGHT),
  logoWidth(300),
  logoHeight(30),
  antWidth(300),
  antHeight(214)
{
  c.initialize(DEFAULT_VIDEO_DEVICE, defaultWidth , defaultHeight, DEFAULT_VIDEO_FORMAT);

  // read in CMVision  logo
  ifstream cmvLogoRGB("cmvision.rgb");
  cmvLogo = new unsigned char[logoWidth*logoHeight * 3];
  cmvLogoRGB.read(cmvLogo, logoWidth * logoHeight * 3);

  // no program is complete w/ some ants
  ifstream antLogoRGB("ants.rgb");
  antLogo = new unsigned char[antWidth*antHeight * 3];
  antLogoRGB.read(antLogo, antWidth * antHeight * 3);

  // initialize CMVisoin
  cmv.initialize(defaultWidth, defaultHeight);

  cmv.loadOptions("colors.txt"); // default options file

  // get colors of bounding boxes, parse the options file ourselves, pretty demo
  ifstream options("colors.txt");
  char input[160];
  bool foundColors = false;
  int i = 0;
  for (i = 0; i < CMV_MAX_COLORS; i++){
    colors[i] = 0;
  }
  i = 0;
  while(!options.eof()){
    options.getline(input, 160);
    if(strcmp(input, "") == 0)
      continue;
    if(!foundColors){
      if(input[0] == '['){
	if(strstr(input, "[Colors]") != NULL){
	  foundColors = true;
	}
      }
    }
    else{
      if(input[0] == '['){
	break;
      }
      istrstream is(input);
      unsigned int r, g, b;
      float m;
      char name[80];
      is >> '(' >> r >> ',' >>  g >> ',' >> b >> ')' >> m >> name;
      //cout << " r=" << r << " g=" << g << " b=" << b << " name = " << name <<  endl;
      int c = b | (g << 8) | (r << 16);
      //cout << c << endl;
      colors[i++] = c;
    }
  }

  rgbData = new unsigned char[defaultWidth * defaultHeight * 3];
  logoVisible = false;
}

CMVWindow::~CMVWindow(){
  delete[] cmvLogo;
  delete[] rgbData;
  delete[] antLogo;
}

// draw static images here, like the CMVision logo
void CMVWindow::paint(){
  XImage *ximg;
  ximg = RGBToXImage(logoWidth, logoHeight, cmvLogo);
  putImage(ximg, 0, 0, logoWidth, logoHeight);
  ximg = RGBToXImage(antWidth, antHeight, antLogo);
  putImage(ximg, 0, height - antHeight, antWidth, antHeight);
}

void CMVWindow::update(){
  XImage *ximg;
  unsigned char *cameraYUV;


  // capture a frame in YUV
  cameraYUV =  c.captureFrame(); 
  // convert it to RGB so we can display it
  YUVToRGB((rgb*)rgbData, (yuv422*)cameraYUV,defaultWidth, defaultHeight); 

  // create an XImage that can be thrown on the screen
  ximg = RGBToXImage(defaultWidth, defaultHeight, rgbData);

  // center the image and display
  putImage(ximg, (width-defaultWidth)/2, logoHeight + 20, defaultWidth, defaultHeight);

  // center the image and display, again for drawing magic bounding boxes, and bLOBs
  putImage(ximg, (width-defaultWidth)/2, 20 + logoHeight + 20*2 + defaultHeight * 2, defaultWidth, defaultHeight);


  // Jim's nifty classifier test code, wooo hoo!
  cmv.testClassify((rgb*)rgbData, (yuv422*)cameraYUV);

  // put clasified image on the screen
  ximg = RGBToXImage(defaultWidth, defaultHeight, rgbData);  
  putImage(ximg, (width-defaultWidth)/2 , 20 + logoHeight + 20 + defaultHeight, defaultWidth, defaultHeight);
  
  cmv.processFrame((yuv422*)cameraYUV);
  // drawing magic bounding boxes and centroids for the,  bLOBs  
  int x_adj = (width-defaultWidth)/2;
  int y_adj = 20 + logoHeight + 20 *2 + defaultHeight * 2;
  for(int color_id = 0; color_id < CMV_MAX_COLORS; color_id++){ 
    int n =  cmv.numRegions(color_id); // see if there's a region for a particular color
    if(n > 0){
      CMVision::region *reg_i = cmv.getRegions(color_id);
      while(reg_i != NULL){
	drawPoint((int)reg_i->cen_x + x_adj, (int)reg_i->cen_y + y_adj, colors[color_id]);
	drawRect((int)reg_i->x1 + x_adj, (int)reg_i->y1 + y_adj,  (int)reg_i->x2 - (int)reg_i->x1,(int)reg_i->y2 - (int)reg_i->y1, colors[color_id]); 
	reg_i = reg_i->next;
      }
    }
  }
}


// draw the static stuff before we display the window
void CMVWindow::showWindow(){
  XWindow::showWindow();
}


// handle expose event so static stuff can be redrawn
bool CMVWindow::handleEvent(XEvent &report){
  switch(report.type){
  case Expose:
    paint();
    return true;
  default:
    return false;
  }
}

