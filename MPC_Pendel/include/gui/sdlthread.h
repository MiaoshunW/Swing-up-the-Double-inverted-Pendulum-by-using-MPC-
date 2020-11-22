#ifndef sdlthread_h
#define sdlthread_h
#include "../base/defines.h"
#include "SDL2/SDL_thread.h"

#ifndef NOGRAPHICS

typedef void* (*threadf) (int*active);
class TransWorhp;
class TWfolder;

/** @ingroup gl
 *  @brief Starting jobs in different thread (e.g. optimizing).
 */
class SDLThread {
public:

	/** Constructor. */
	SDLThread();
	
	/** Destructor. */
	~SDLThread();
	
	/** Start job. */
	//int Run(threadf f);
	int Run(TWfolder *tw, TransWorhp *ph);
	
	/** Lock data. */
	int Lock();
	
	/** Unlock data. */
	int Unlock();

	int threadlive;
	threadf threadfunction;
	
	int Active();
private:
	SDL_Thread *sdl_t;
	SDL_mutex *lock;
	SDL_sem *sem;

};

extern SDLThread *thethread0;

#endif // sdllthread_h
#endif