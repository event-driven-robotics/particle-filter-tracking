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

#include "vControlLoopDelay.h"

int main(int argc, char * argv[])
{
    /* initialize yarp network */
    yarp::os::Network yarp;
    if(!yarp.checkNetwork()) {
        std::cout << "Could not connect to YARP" << std::endl;
        return false;
    }

    /* create the module */
    delayControl instance;

    /* prepare and configure the resource finder */
    yarp::os::ResourceFinder rf;
    rf.setVerbose( true );
    rf.setDefaultContext( "event-driven" );
    rf.setDefaultConfigFile( "vParticleFilterTracker.ini" );
    rf.configure( argc, argv );

    return instance.runModule(rf);
}

/*////////////////////////////////////////////////////////////////////////////*/
// ROIQ
/*////////////////////////////////////////////////////////////////////////////*/


roiq::roiq()
{
    roi.resize(4);
    n = 1000;
    roi[0] = 0; roi[1] = 1000;
    roi[2] = 0; roi[3] = 1000;
    use_TW = false;
}

void roiq::setSize(unsigned int value)
{
    //if TW n is in clock-ticks
    //otherwise n is in # events.
    n = value;
    while(q.size() > n)
        q.pop_back();
}

void roiq::setROI(int xl, int xh, int yl, int yh)
{
    roi[0] = xl; roi[1] = xh;
    roi[2] = yl; roi[3] = yh;
}

int roiq::add(const AE &v)
{

    if(v.x < roi[0] || v.x > roi[1] || v.y < roi[2] || v.y > roi[3])
        return 0;
    q.push_front(v);
    return 1;
}

/*////////////////////////////////////////////////////////////////////////////*/
// DELAYCONTROL
/*////////////////////////////////////////////////////////////////////////////*/

bool delayControl::configure(yarp::os::ResourceFinder &rf)
{
    //module name and control
    setName((rf.check("name", Value("/vpf")).asString()).c_str());
    if(!rpcPort.open(getName() + "/cmd")) {
        yError() << "Could not open rpc port for" << getName();
        return false;
    }
    attach(rpcPort);

    //options and parameters
    px = py = pr = 0;
    res.height = rf.check("height", Value(240)).asInt();
    res.width = rf.check("width", Value(304)).asInt();
    gain = rf.check("gain", Value(0.0005)).asDouble();
    batch_size = rf.check("batch", Value(0)).asInt();
    bool adaptivesampling = rf.check("adaptive") &&
            rf.check("adaptive", yarp::os::Value(true)).asBool();
    motionVariance = rf.check("variance", yarp::os::Value(0.7)).asDouble();
    output_sample_delta = rf.check("output_sample", Value(0)).asDouble();
    resetTimeout = rf.check("reset", yarp::os::Value(1.0)).asDouble();
    bool start = rf.check("start") &&
            rf.check("start", yarp::os::Value(true)).asBool();

    //maxRawLikelihood must be set before TrueThreshold
    maxRawLikelihood  = rf.check("bins", Value(64)).asInt();
    setTrueThreshold(rf.check("truethresh", yarp::os::Value(0.35)).asDouble());

    vpf.initialise(res.width, res.height,
                   rf.check("particles", yarp::os::Value(32)).asInt(),
                   maxRawLikelihood,
                   adaptivesampling,
                   rf.check("threads", Value(1)).asInt(),
                   rf.check("obsthresh", yarp::os::Value(0.2)).asDouble(),
                   rf.check("obsinlier", yarp::os::Value(1.5)).asDouble(),
                   rf.check("randoms", yarp::os::Value(0.0)).asDouble(),
                   rf.check("negbias", yarp::os::Value(10.0)).asDouble());

    yarp::os::Bottle * seed = rf.find("seed").asList();
    if(seed && seed->size() == 3) {
        yInfo() << "Setting initial seed state:" << seed->toString();
        vpf.setSeed(seed->get(0).asDouble(),
                    seed->get(1).asDouble(),
                    seed->get(2).asDouble());
        vpf.resetToSeed();
    }

    if(!scopePort.open(getName() + "/scope:o"))
        return false;

    event_output_port.setWriteType(GaussianAE::tag);
    if(!event_output_port.open(getName() + "/GAE:o"))
        return false;

    if(!raw_output_port.open(getName() + "/state:o"))
        return false;

    if(!debugPort.open(getName() + "/debug:o"))
        return false;

    input_port.setQLimit(rf.check("qlimit", Value(0)).asInt());
    if(!input_port.open(getName() + "/AE:i"))
        return false;

    if(!start)
        pause();

    return Thread::start();

}

bool delayControl::interruptModule()
{
    return Thread::stop();
}

bool delayControl::updateModule()
{
    if(scopePort.getOutputCount()) {
        scopePort.prepare() = getTrackingStats();
        scopePort.write();
    }

    return Thread::isRunning();
}

double delayControl::getPeriod()
{
    return 0.2;
}

void delayControl::setTrueThreshold(double value)
{
    detectionThreshold = value * maxRawLikelihood;
}

void delayControl::performReset(int x, int y, int r)
{
    if(x > 0)
        vpf.setSeed(x, y, r);
    vpf.resetToSeed();
    input_port.resume();
}

yarp::sig::Vector delayControl::getTrackingStats()
{
    yarp::sig::Vector stats(10);

    stats[0] = 1000*input_port.queryDelayT();
    stats[1] = 1.0/filterPeriod;
    stats[2] = targetproc;
    stats[3] = input_port.queryRate() / 1000.0;
    stats[4] = dx;
    stats[5] = dy;
    stats[6] = dr;
    stats[7] = vpf.maxlikelihood / (double)maxRawLikelihood;
    stats[8] = cpuusage.getProcessorUsage();
    stats[9] = qROI.n;

    return stats;
}

void delayControl::onStop()
{
    input_port.close();
    event_output_port.close();
    raw_output_port.close();
    debugPort.close();
    scopePort.close();
}

void delayControl::pause()
{
    input_port.interrupt();
}

void delayControl::resume()
{
    input_port.resume();
}

void delayControl::run()
{
    double Tresample = 0;
    double Tpredict = 0;
    double Tlikelihood = 0;
    double Tgetwindow = 0;

    targetproc = 0;
    unsigned int i = 0;
    yarp::os::Stamp ystamp;
    //double stagnantstart = 0;
    int channel;
    if(batch_size)
        qROI.setSize(batch_size);
    else
        qROI.setSize(50);

    //START HERE!!
    const vector<AE> *q = input_port.read(ystamp);
    if(!q || Thread::isStopping()) return;
    vpf.extractTargetPosition(avgx, avgy, avgr);

    channel = q->front().getChannel();

    while(true) {

        //calculate error
        double delay = input_port.queryDelayT();
        unsigned int unprocdqs = input_port.queryunprocessed();
        targetproc = M_PI * avgr;
        if(unprocdqs > 1 && delay > gain)
            targetproc *= (delay / gain);

        //update the ROI with enough events
        Tgetwindow = yarp::os::Time::now();
        unsigned int addEvents = 0;
        unsigned int testedEvents = 0;
        while(addEvents < targetproc) {

            //if we ran out of events get a new queue
            if(i >= q->size()) {
                //if(input_port.queryunprocessed() < 3) break;
                i = 0;
                q = input_port.read(ystamp);
                if(!q || Thread::isStopping()) return;
            }

            //auto v = is_event<AE>((*q)[i]);
            addEvents += qROI.add((*q)[i]);
            //if(breakOnAdded) testedEvents = addEvents;
            //else testedEvents++;
            testedEvents++;
            i++;
        }
        Tgetwindow = yarp::os::Time::now() - Tgetwindow;

        //get the current time
        int currentstamp = 0;
        if(i >= q->size())
            currentstamp = (*q)[i-1].stamp;
        else
            currentstamp = (*q)[i].stamp;

        if(batch_size)
            qROI.setSize(batch_size);

        //do our update!!
        //yarp::os::Time::delay(0.005);
        Tlikelihood = yarp::os::Time::now();
        vpf.performObservation(qROI.q);
        Tlikelihood = yarp::os::Time::now() - Tlikelihood;

        //set our new position
        dx = avgx, dy = avgy, dr = avgr;
        vpf.extractTargetPosition(avgx, avgy, avgr);
        dx = avgx - dx; dy = avgy - dy; dr = avgr - dr;
        double roisize = avgr + 10;
        qROI.setROI(avgx - roisize, avgx + roisize, avgy - roisize, avgy + roisize);

        //set our new window #events
        if(!batch_size) {
            double nw; vpf.extractTargetWindow(nw);
            if(qROI.q.size() - nw > 30)
                qROI.setSize(std::max(nw, 50.0));
            if(qROI.q.size() > 3000)
                qROI.setSize(3000);
        }

        //calculate the temporal window of the q
        double tw = qROI.q.front().stamp - qROI.q.back().stamp;
        if(tw < 0) tw += vtsHelper::max_stamp;

        Tresample = yarp::os::Time::now();
        vpf.performResample();
        Tresample = yarp::os::Time::now() - Tresample;

        Tpredict = yarp::os::Time::now();
        //vpf.performPrediction(std::max(addEvents / (5.0 * avgr), 0.7));
        vpf.performPrediction(motionVariance);
        Tpredict = yarp::os::Time::now() - Tpredict;

        int is_tracking = 1;
        if(vpf.maxlikelihood < detectionThreshold)
            is_tracking = 0;

//        //check for stagnancy
//        int is_tracking = 1;
//        if(vpf.maxlikelihood < detectionThreshold) {

//            if(!stagnantstart) {
//                stagnantstart = yarp::os::Time::now();
//            } else {
//                if(yarp::os::Time::now() - stagnantstart > resetTimeout) {
//                    //vpf.resetToSeed();
//                    stagnantstart = 0;
//                    is_tracking = 0;
//                }
//            }

//        } else {
//            stagnantstart = 0;
//        }

        double delta_x = avgx - px;
        double delta_y = avgy - py;
        double dpos = sqrt(delta_x * delta_x + delta_y * delta_y);
        if(dpos > output_sample_delta) {
            px = avgx;
            py = avgy;
            pr = avgr;
            //output our event
            if(event_output_port.getOutputCount()) {
                auto ceg = make_event<GaussianAE>();
                ceg->stamp = currentstamp;
                ceg->setChannel(channel);
                ceg->x = avgx;
                ceg->y = avgy;
                ceg->sigx = avgr;
                ceg->sigy = tw;
                ceg->sigxy = 1.0;
                if(vpf.maxlikelihood > detectionThreshold)
                    ceg->polarity = 1.0;
                else
                    ceg->polarity = 0.0;

                vQueue outq; outq.push_back(ceg);
                event_output_port.write(outq, ystamp);

            }
            //output the raw data
            if(raw_output_port.getOutputCount()) {
                Bottle &next_sample = raw_output_port.prepare();
                next_sample.clear();
                next_sample.addVocab(createVocab('T', '_', 'S', 'T'));
                next_sample.addInt(is_tracking);
                next_sample.addDouble(currentstamp);
                next_sample.addDouble(Time::now());
                next_sample.addDouble(avgx);
                next_sample.addDouble(avgy);
                next_sample.addDouble(avgr);
                next_sample.addDouble(channel);
                next_sample.addDouble(tw);
                next_sample.addDouble(vpf.maxlikelihood);

                raw_output_port.setEnvelope(ystamp);
                raw_output_port.write();
            }


        }





        static double prev_update_time = Tgetwindow;
        filterPeriod = Time::now() - prev_update_time;
        prev_update_time += filterPeriod;

        //output a debug image
        if(debugPort.getOutputCount()) {

            //static double prev_likelihood = vpf.maxlikelihood;
            static int NOFPANELS = 3;

            static yarp::sig::ImageOf< yarp::sig::PixelBgr> *image_ptr = 0;
            static int panelnumber = NOFPANELS;

            static double pimagetime = yarp::os::Time::now();

            //if we are in waiting state, check trigger condition
            bool trigger_capture = false;
            if(panelnumber >= NOFPANELS) {
                //trigger_capture = prev_likelihood > detectionThreshold &&
                 //       vpf.maxlikelihood <= detectionThreshold;
                trigger_capture = yarp::os::Time::now() - pimagetime > 0.1;
            }
            //prev_likelihood = vpf.maxlikelihood;

            //if we are in waiting state and
            if(trigger_capture) {
                //trigger the capture of the panels only if we aren't already
                pimagetime = yarp::os::Time::now();
                yarp::sig::ImageOf< yarp::sig::PixelBgr> &image_ref =
                        debugPort.prepare();
                image_ptr = &image_ref;
                image_ptr->resize(res.width * NOFPANELS, res.height);
                image_ptr->zero();
                panelnumber = 0;
            }

            if(panelnumber < NOFPANELS) {

                yarp::sig::ImageOf<yarp::sig::PixelBgr> &image = *image_ptr;
                int panoff = panelnumber * res.width;

                int px1 = avgx - roisize; if(px1 < 0) px1 = 0;
                int px2 = avgx + roisize; if(px2 >= res.width) px2 = res.width-1;
                int py1 = avgy - roisize; if(py1 < 0) py1 = 0;
                int py2 = avgy + roisize; if(py2 >= res.height) py2 = res.height-1;

                px1 += panoff; px2 += panoff;
                for(int x = px1; x <= px2; x+=2) {
                    image(x, py1) = yarp::sig::PixelBgr(255, 255, 120 * panelnumber);
                    image(x, py2) = yarp::sig::PixelBgr(255, 255, 120 * panelnumber);
                }
                for(int y = py1; y <= py2; y+=2) {
                    image(px1, y) = yarp::sig::PixelBgr(255, 255, 120 * panelnumber);
                    image(px2, y) = yarp::sig::PixelBgr(255, 255, 120 * panelnumber);
                }

                std::vector<vParticle> indexedlist = vpf.getps();

                for(unsigned int i = 0; i < indexedlist.size(); i++) {

                    int py = indexedlist[i].gety();
                    int px = indexedlist[i].getx();

                    if(py < 0 || py >= res.height || px < 0 || px >= res.width)
                        continue;
                    int pscale = 255 * indexedlist[i].getl() / maxRawLikelihood;
                    image(px+panoff, py) =
                            yarp::sig::PixelBgr(pscale, 255, pscale);

                }
                drawEvents(image, qROI.q, panoff);

                panelnumber++;
            }

            if(panelnumber == NOFPANELS) {
                panelnumber++;
                debugPort.write();
            }

        }

    }

}


#define CMD_HELP  createVocab('h', 'e', 'l', 'p')
#define CMD_SET   createVocab('s', 'e', 't')
#define CMD_START createVocab('S', 'T', 'A', 'R')
#define CMD_STOP createVocab('S', 'T', 'O', 'P')

bool delayControl::respond(const yarp::os::Bottle& command,
                                yarp::os::Bottle& reply) {

    //initialise for default response
    bool error = false;
    yInfo() << command.size();
    reply.clear();

    //switch on the command word
    switch(command.get(0).asVocab()) {

    case CMD_HELP:
    {
        reply.addString("<<Event-based Particle Filter with Delay Control>>");
        reply.addString("Set the following parameters with | set <param> "
                        "<value> |");
        reply.addString("trackThresh [0-1]");
        reply.addString("trueThresh [0-1]");
        reply.addString("gain [0-1]");
        reply.addString("resetTimeout [0 inf]");
        reply.addString("negativeBias [0 inf]");
        reply.addString("motionVar [0 inf]");
        reply.addString("inlierParam [0 inf]");
        reply.addString("adaptive [true false]");
        break;
    }
    case CMD_SET:
    {

        std::string param = command.get(1).asString();
        double value = command.get(2).asDouble();

        if(param == "trackThresh") {
            reply.addString("setting tracking parameter to ");
            reply.addDouble(value);
            vpf.setMinLikelihood(value);
        }
        else if(param == "gain") {
            reply.addString("gain changed from ");
            reply.addDouble(gain);
            gain = value;
            reply.addString(" to ");
            reply.addDouble(gain);
        }
        else if(param == "trueThresh") {
            reply.addString("setting true classification parameter to ");
            reply.addDouble(value);
            setTrueThreshold(value);
        }
        else if(param == "resetTimeout") {
            reply.addString("resetTimeout changed from ");
            reply.addDouble(resetTimeout);
            resetTimeout = value;
            reply.addString(" to ");
            reply.addDouble(resetTimeout);

        }
        else if(param == "negativeBias") {
            reply.addString("setting the observation negative bias to ");
            reply.addDouble(value);
            vpf.setNegativeBias(value);
        }
        else if(param == "motionVar") {
            reply.addString("motionVar changed from ");
            reply.addDouble(motionVariance);
            motionVariance = value;
            reply.addString(" to ");
            reply.addDouble(motionVariance);
        }
        else if(param == "inlierParam") {
            reply.addString("setting the inlier width to ");
            reply.addDouble(value);
            vpf.setInlierParameter(value);
        }
        else if(param == "adaptive") {
            if(value) {
                reply.addString("setting the resample method = adaptive");
                vpf.setAdaptive(true);;
            }
            else {
                reply.addString("setting the resample method = every update");
                vpf.setAdaptive(false);
            }
        }
        else {
            error = true;
            reply.addString("incorrect parameter");
        }
        break;
    }
    case CMD_STOP:
    {
        reply.addString("tracking paused");
        pause();
        break;
    }
    case CMD_START:
    {
        if(command.size() == 4) {
            int x = command.get(1).asInt();
            int y = command.get(2).asInt();
            int r = command.get(3).asInt();
            reply.addString("resetting particle positions to custom positions");
            performReset(x, y, r);
        } else {
            reply.addString("resetting particle positions to seed");
            performReset();
        }
        break;
    }
    default:
    {
        error = true;
        break;
    }

    } //switch

    //return the error - the reply is automatically sent
    return !error;

}
