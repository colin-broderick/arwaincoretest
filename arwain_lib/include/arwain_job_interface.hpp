#ifndef _ARWAIN_JOB_INTERFACE
#define _ARWAIN_JOB_INTERFACE

class ArwainJob
{
    public:
        virtual bool init() = 0;
        virtual bool join() = 0;
    protected:
        virtual void core_setup() = 0;
        virtual void run() = 0;
        virtual void run_idle() = 0;
        virtual void run_inference() = 0;
};

#endif
