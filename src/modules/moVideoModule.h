#ifndef MO_VIDEO_MODULE_H
#define MO_VIDEO_MODULE_H

#include "../moModule.h"

class moVideoModule : public moModule {
public:
	moVideoModule();
	virtual ~moVideoModule();

	virtual void setInput(moDataStream* stream, int n=0);
	virtual moDataStream *getInput(int n=0);
	virtual moDataStream *getOutput(int n=0);

	void start();
	void stop();
	void update();

private:
	moDataStream *stream;
	int numframes;
	void *video;

	MODULE_INTERNALS();
};

#endif

