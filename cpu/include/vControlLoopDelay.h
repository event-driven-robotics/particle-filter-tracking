/*
 *   Copyright (C) 2017 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef __VCONTROLLOOPDELAYPF__
#define __VCONTROLLOOPDELAYPF__

#include <yarp/os/all.h>
#include <yarp/sig/Vector.h>
#include <iCub/eventdriven/all.h>
#include "vParticle.h"

using namespace ev;
using namespace yarp::os;
using namespace yarp::sig;

//void drawEvents(yarp::sig::ImageOf< yarp::sig::PixelBgr> &image, deque<AE> &q, int offsetx = 0);

//void drawcircle(yarp::sig::ImageOf<yarp::sig::PixelBgr> &image, int cx, int cy, int cr, int id = 0);

//void drawDistribution(yarp::sig::ImageOf<yarp::sig::PixelBgr> &image, std::vector<vParticle> &indexedlist);

/*////////////////////////////////////////////////////////////////////////////*/
// ROIQ
/*////////////////////////////////////////////////////////////////////////////*/
class roiq
{
public:

    deque<AE> q;
    unsigned int n;
    yarp::sig::Vector roi;
    bool use_TW;

    roiq();
    void setSize(unsigned int value);
    void setROI(int xl, int xh, int yl, int yh);
    int add(const AE &v);

};

/*////////////////////////////////////////////////////////////////////////////*/
// DELAYCONTROL
/*////////////////////////////////////////////////////////////////////////////*/

class delayControl : public yarp::os::RFModule, public yarp::os::Thread
{
private:

    //data structures and ports
    Port rpcPort;
    vReadPort< vector<AE> > input_port;
    vWritePort event_output_port;
    BufferedPort<Bottle> raw_output_port;
    BufferedPort<Vector> scopePort;
    roiq qROI;
    vParticlefilter vpf;
    Mutex m;

    //variables
    resolution res;
    double avgx, avgy, avgr;
    int maxRawLikelihood;
    double gain;
    double detectionThreshold;
    double resetTimeout;
    double motionVariance;
    int batch_size;
    double output_sample_delta;

    //diagnostics
    double filterPeriod;
    unsigned int targetproc;
    double dx;
    double dy;
    double dr;
    double px, py, pr;
    ev::benchmark cpuusage;
    BufferedPort< ImageOf<PixelBgr> > debugPort;


public:

    delayControl() {}

    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool interruptModule();
    virtual double getPeriod();
    virtual bool updateModule();
    virtual bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);

    bool open(std::string name, unsigned int qlimit = 0);
    void performReset(int x = -1, int y = -1, int r = -1);
    void setFilterInitialState(int x, int y, int r);

    void setMinRawLikelihood(double value);
    void setMaxRawLikelihood(int value);
    void setNegativeBias(int value);
    void setInlierParameter(int value);
    void setMotionVariance(double value);
    void setTrueThreshold(double value);
    void setAdaptive(double value = true);
    void setOutputSampleDelta(double value);

    void setGain(double value);
    void setMinToProc(int value);
    void setResetTimeout(double value);

    yarp::sig::Vector getTrackingStats();

    //bool threadInit();
    void pause();
    void resume();
    void onStop();
    void run();
    //void threadRelease();


};





#endif
