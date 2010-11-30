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


#ifndef MO_KINECEZMQMODULE_H
#define MO_KINECTZMQMODULE_H

#include <zmq.hpp>
#include "../moModule.h"
#include "../moDataStream.h"
#include "cv.h"

class moDataStream;


class moKinectZMQModule : public moModule {
public:
    moKinectZMQModule();
    virtual ~moKinectZMQModule();

    virtual void start();
	virtual void stop();
	virtual void update();
	virtual void poll();

private:
	zmq::socket_t depth_socket;
	zmq::socket_t rgb_socket;
	IplImage *depth_buffer;
	IplImage *rgb_buffer;

	static const char depth_sub_str[];
	static const char rgb_sub_str[];
	moDataStream *depth;
	moDataStream *rgb;


	MODULE_INTERNALS();
};

#endif
