/***********************************************************************
 ** Copyright (C) 2010 Movid Authors.  All rights reserved.
 **
 ** This file is part of the Movid Software.
 **
 ** This file may be distributed under the terms of the Q Public License
 ** as defined by Trolltech AS of Norway and appearing in the file
 ** LICENSE included in the packaging of this file.
 **
 ** This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 ** WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 **
 ** Contact info@movid.org if any conditions of this licensing are
 ** not clear to you.
 **
 **********************************************************************/


#include <stdio.h>
#include <assert.h>

#include "../moLog.h"
#include "../moModule.h"
#include "../moDataStream.h"
#include <zmq.hpp>
#include "moKinectZMQModule.h"
#include "highgui.h"

#include <math.h>


#include <string>
#include <strings.h>
#include <stdint.h>

MODULE_DECLARE(KinectZMQ, "native", "Fetch Kinect stream from zeromq");

#define DEPTH_LEN		(640 * 480)
#define DEPTH_SZ		DEPTH_LEN * sizeof(uint16_t)
#define RGB_LEN			(640 * 480 * 3)
#define RGB_SZ			RGB_LEN * sizeof(uint8_t)

uint16_t _t_gamma[2048];

zmq::context_t context(1);


const char moKinectZMQModule::depth_sub_str[] = "depth";

const char moKinectZMQModule::rgb_sub_str[] = "rgb";


std::string hexDump( zmq::message_t & aMessage, int num = 0 ) {
  // I'm going to build a hex/ascii dump like you've seen so many times before
  std::string  msg;
  std::string  ascii;
  // get the pointer to the start of the message's payload - and it's size
  const char *buff = (const char *)aMessage.data();
  int         size = aMessage.size();
  const char *end;
  if (num){
      char *_end = const_cast<char*>(buff) + num;
      if (_end < (buff + size - 1)){
        const char *end = _end;
      } else {
        const char *end = buff + size - 1;
      }

  } else {
    const char *end  = buff + size - 1;
  }

  // see if it's the trivial case
  if (buff == NULL) {
    msg.append("NULL");
  } else {
    // get a place to hold the conversion of each byte
    char   hex[3];
    bzero(hex, 3);
    // run through the valid data in the buffer
    for (const char *p = buff; p <= end; ++p) {
      // generate the hex code for the byte and add it
      snprintf(hex, 3, "%02x", (uint8_t)(*p));
      msg.append(hex).append(" ");
      // if it's printable, add it to the ascii part, otherwise, put a '.'
      if (isprint(*p)) {
        ascii.append(p, 1);
      } else {
        ascii.append(".");
      }
      // see if we have a complete line
      if (ascii.size() >= 19) {
        msg.append(" ").append(ascii).append("\n");
        ascii.clear();
      }
    }
    // if we have anything left, put it on the line as well
    if (ascii.size() > 0) {
      msg.append((19 - ascii.length())*3 + 1, ' ').append(ascii);
    }
  }

  return msg;
}


moKinectZMQModule::~moKinectZMQModule()
{
    //dtor
}

typedef struct {
    char address[6];
    uint16_t frame[DEPTH_LEN];
} DepthMsg;

moKinectZMQModule::moKinectZMQModule() : moModule(MO_MODULE_OUTPUT),
                                         depth_socket(context, ZMQ_SUB),
                                         rgb_socket(context, ZMQ_SUB)
{

	MODULE_INIT();

	//this->depth_buffer = NULL;
	this->depth = new moDataStream("IplImage");
	//this->depth_sub_str = "depth";
	this->rgb = new moDataStream("IplImage8");

	// declare outputs
	this->declareOutput(0, &this->depth, new moDataStreamInfo(
			"camera", "IplImage", "Image stream of the depth information"));
	this->declareOutput(1, &this->rgb, new moDataStreamInfo(
			"camera", "IplImage8", "Image stream of the camera"));

	// declare properties
	this->properties["port"] = new moProperty("tcp://127.0.0.1:6666");

	// reinit t_gamma
	int i;
	for (i=0; i<2048; i++) {
		float v = i/2048.0;
		v = powf(v, 3)* 6;
		_t_gamma[i] = v*6*256;
	}
}


void moKinectZMQModule::start() {


    const char *port = this->properties["port"]->asString().c_str();

    LOGM(MO_DEBUG, "connect to zmq socket");
    depth_socket.connect(port);
    depth_socket.setsockopt(ZMQ_SUBSCRIBE, depth_sub_str, strlen(depth_sub_str));
	LOGM(MO_DEBUG, "connected successfully to zmq socket");


	// Allocate image
	CvSize size;
	size.width = 640;
	size.height = 480;
	this->depth_buffer = cvCreateImage(size, IPL_DEPTH_8U, 1);

	moModule::start();
}

void moKinectZMQModule::stop() {
	moModule::stop();
	if ( this->depth_buffer != NULL )
		cvReleaseImage(&this->depth_buffer);
    depth_socket.setsockopt(ZMQ_UNSUBSCRIBE, depth_sub_str, strlen(depth_sub_str));
}

void moKinectZMQModule::update() {

    LOGM(MO_DEBUG, "update");
    zmq::message_t depth_msg;
    depth_socket.recv(&depth_msg, ZMQ_NOBLOCK);
    size_t msg_size = depth_msg.size();
    if (msg_size) {
        LOGM(MO_DEBUG, "got message" << msg_size);
    } else {
        LOGM(MO_DEBUG, "no message");
        return;
    }
    DepthMsg  *depth_payload = (DepthMsg *)depth_msg.data();
    std::string addr(depth_payload->address);
    LOGM(MO_DEBUG, "message" << hexDump(depth_msg, 20));
    //LOGM(MO_DEBUG, "depth msg address: " << (depth_payload->frame) << "end");


//	int n, sz;
//	uint8_t buf8[DEPTH_SZ];
//
//	if ( this->s == -1 )
//		return;
//
//	sz = DEPTH_SZ;
//	while ( sz ) {
//		n = read(this->s, &buf8[DEPTH_SZ - sz], sz);
//		if ( n == -1 ) {
//			LOGM(MO_ERROR, "failed to read a full image");
//			return;
//		}
//		sz -= n;
//	}
//
//	// Our current module work on 8 bits depth, not 16. Convert.
//	uint16_t buf16[DEPTH_LEN];
//	uint8_t buf8n[DEPTH_LEN];
//	memset(buf8n, 0, DEPTH_LEN);
//	memcpy((char *)(void *)buf16, buf8, DEPTH_SZ);
//	for ( int i = 0; i < DEPTH_LEN; i++ ) {
//		/**
//		int pval = t_gamma[buf16[i]];
//		int lb = pval & 0xff;
//		if ( (pval >> 8) == 0 )
//			buf8n[i] = lb;
//        **/
//		buf8n[i] = (buf16[i] >> 1) & 0xff;
//	}
//
//
//	// Copy
//	memcpy(this->output_buffer->imageData, buf8n, DEPTH_LEN);
//
//	// Push
//	this->monochrome->push(this->output_buffer);
//
//	/** Import algo from glview
//	uint8_t gl_depth_back[640*480*4];
//	uint16_t buf16[DEPTH_LEN];
//	memcpy((char *)(void *)buf16, buf8, DEPTH_SZ);
//	for (int i=0; i<640*480; i++) {
//		int pval = t_gamma[buf16[i]];
//		int lb = pval & 0xff;
//		switch (pval>>8) {
//			case 0:
//				gl_depth_back[3*i+0] = 255;
//				gl_depth_back[3*i+1] = 255-lb;
//				gl_depth_back[3*i+2] = 255-lb;
//				break;
//			case 1:
//				gl_depth_back[3*i+0] = 255;
//				gl_depth_back[3*i+1] = lb;
//				gl_depth_back[3*i+2] = 0;
//				break;
//			case 2:
//				gl_depth_back[3*i+0] = 255-lb;
//				gl_depth_back[3*i+1] = 255;
//				gl_depth_back[3*i+2] = 0;
//				break;
//			case 3:
//				gl_depth_back[3*i+0] = 0;
//				gl_depth_back[3*i+1] = 255;
//				gl_depth_back[3*i+2] = lb;
//				break;
//			case 4:
//				gl_depth_back[3*i+0] = 0;
//				gl_depth_back[3*i+1] = 255-lb;
//				gl_depth_back[3*i+2] = 255;
//				break;
//			case 5:
//				gl_depth_back[3*i+0] = 0;
//				gl_depth_back[3*i+1] = 0;
//				gl_depth_back[3*i+2] = 255-lb;
//				break;
//			default:
//				gl_depth_back[3*i+0] = 0;
//				gl_depth_back[3*i+1] = 0;
//				gl_depth_back[3*i+2] = 0;
//				break;
//		}
//	}
//
//	// Convert to an Iplimage
//	IplImage *img;
//	CvSize size;
//	size.width = 640;
//	size.height = 480;
//	img = cvCreateImage(size, IPL_DEPTH_8U, 3);
//	memcpy(img->imageData, gl_depth_back, DEPTH_LEN * 3);
//
//	// Push
//	this->stream->push(img);
//	**/

	this->notifyUpdate();
}

void moKinectZMQModule::poll() {
	this->notifyUpdate();
	moModule::poll();
}



