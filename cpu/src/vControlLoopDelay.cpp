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
#include <cmath>
#include <algorithm>
#include <iostream>
#include <fstream>

using std::vector;
using std::deque;
using ev::AE;
using ev::packet;
using yarp::sig::ImageOf;
using yarp::sig::PixelFloat;
using yarp::sig::PixelBgr;
using yarp::os::Value;
using yarp::os::Bottle;
using yarp::os::createVocab;

int main(int argc, char * argv[])
{
    /* initialize yarp network */


    /* create the module */
    delayControl instance;

    /* prepare and configure the resource finder */
    yarp::os::ResourceFinder rf;
    //rf.setVerbose( false );
    //rf.setDefaultContext( "event-driven" );
    //rf.setDefaultConfigFile( "vParticleFilterTracker.ini" );
    rf.configure( argc, argv );

    return instance.runModule(rf);
}

void drawEvents(yarp::sig::ImageOf< yarp::sig::PixelBgr> &image, deque<AE> &q,
                int offsetx = 0) {

    if(q.empty()) return;

    //draw oldest first
    for(int i = (int)q.size()-1; i >= 0; i--) {
        double p = (double)i / (double)q.size();
        //image(q[i].x + offsetx, q[i].y).b =  255 * (1-p);
        image(q[i].x + offsetx, q[i].y).b =  255;
    }
}

void drawcircle(yarp::sig::ImageOf<yarp::sig::PixelBgr> &image, int cx, int cy,
                int cr, int id)
{

    for(int y = -cr; y <= cr; y++) {
        for(int x = -cr; x <= cr; x++) {
            if(fabs(sqrt(pow(x, 2.0) + pow(y, 2.0)) - (double)cr) > 0.8)
                continue;
            int px = cx + x; int py = cy + y;
            if(py<0 || py>(int)image.height()-1 || px<0 || px>(int)image.width()-1)
                continue;
            switch(id) {
            case(0): //green
                image(px, py) = yarp::sig::PixelBgr(0, 255, 0);
                break;
            case(1): //blue
                image(px, py) = yarp::sig::PixelBgr(0, 0, 255);
                break;
            case(2): //red
                image(px, py) = yarp::sig::PixelBgr(255, 0, 0);
                break;
            default:
                image(px, py) = yarp::sig::PixelBgr(255, 255, 0);
                break;

            }

        }
    }

}

void drawDistribution(yarp::sig::ImageOf<yarp::sig::PixelBgr> &image, std::vector<templatedParticle> &indexedlist)
{

    double sum = 0;
    std::vector<double> weights;
    for(unsigned int i = 0; i < indexedlist.size(); i++) {
        weights.push_back(indexedlist[i].weight);
        sum += weights.back();
    }

    std::sort(weights.begin(), weights.end());


    image.resize(indexedlist.size(), 100);
    image.zero();
    for(unsigned int i = 0; i < weights.size(); i++) {
        image(weights.size() - 1 -  i, 99 - weights[i]*100) = yarp::sig::PixelBgr(255, 255, 255);
    }
}

void drawTemplate(ImageOf<PixelBgr> &image, ImageOf<PixelFloat> &appearance,
                  double t_x, double t_y, double t_r)
{
    int mid_x = appearance.width() / 2;
    int mid_y = appearance.height() / 2;
    double t_s = t_r / mid_x;

    for(auto x = 0; x < appearance.width(); x++) {
        for(auto y = 0; y < appearance.height(); y++) {

            int i_x = t_x + ((x - mid_x) * t_s);
            i_x = std::max(std::min(i_x, (int)(image.width() - 1)), 0);
            int i_y = t_y + ((y - mid_y) * t_s);
            i_y = std::max(std::min(i_y, (int)(image.height() - 1)), 0);

            PixelBgr &p = image(i_x, i_y);
            if(appearance(x, y) > 0) {
                p.g = appearance(x, y) * 125;
            } else if(appearance(x, y) < 0) {
                p.r = -appearance(x, y) * 125;
            }


        }
    }





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

    if(rf.check("h") || rf.check("H")) 
    {
        yInfo() << "--name";
        yInfo() << "--height, --width";
        yInfo() << "--gain, --adaptive, --variance";
        yInfo() << "--particles (20), --threads (1)";
        yInfo() << "--seed \"(x, y, r)\", --start_time <s>, --file <full_path>";
        return false;
    }

    yarp::os::Network yarp;
    if(!yarp.checkNetwork(2.0)) {
        std::cout << "Could not connect to YARP" << std::endl;
        return false;
    }

    //module name and control
    setName((rf.check("name", Value("/vpf")).asString()).c_str());
    if(!rpcPort.open(getName() + "/cmd")) {
        yError() << "Could not open rpc port for" << getName();
        return false;
    }
    attach(rpcPort);

    //options and parameters
    px = py = pr = 0;
    res.height = rf.check("height", Value(480)).asInt();
    res.width = rf.check("width", Value(640)).asInt();
    gain = rf.check("gain", Value(0.01)).asDouble();
    batch_size = rf.check("batch", Value(0)).asInt();
    bool adaptivesampling = rf.check("adaptive") &&
            rf.check("adaptive", yarp::os::Value(true)).asBool();
    motionVariance = rf.check("variance", yarp::os::Value(3.0)).asDouble();
    output_sample_delta = rf.check("output_sample", Value(0)).asDouble();
    bool start = rf.check("start") &&
            rf.check("start", yarp::os::Value(true)).asBool();

    maxRawLikelihood = vpf.initialise(res.width, res.height,
                   rf.check("particles", yarp::os::Value(100)).asInt(),
                   adaptivesampling,
                   rf.check("threads", Value(1)).asInt());

    //maxRawLikelihood must be set before TrueThreshold
    setTrueThreshold(rf.check("truethresh", yarp::os::Value(0.35)).asDouble());

    yarp::os::Bottle * seed = rf.find("seed").asList();
    if(seed && seed->size() == 3) {
        yInfo() << "Setting initial seed state:" << seed->toString();
        vpf.setSeed(seed->get(0).asDouble(),
                    seed->get(1).asDouble(),
                    seed->get(2).asDouble());
        vpf.resetToSeed();
    }

    start_time = rf.check("start_time", Value(0.0)).asDouble();

    if(rf.check("file")) {
        std::string file_name = rf.find("file").asString();
        fs.open(file_name, std::ios_base::out | std::ios_base::trunc);
        if(!fs.is_open()) {
            yError() << "Could not open output file" << file_name;
            return false;
        } else {
            yInfo() << "Saving data to file" << file_name;
        }
    }


    // if(!scopePort.open(getName() + "/scope:o"))
    //     return false;

    //event_output_port.setWriteType(GaussianAE::tag);
    // if(!event_output_port.open(getName() + "/GAE:o"))
    //     return false;

    // if(!raw_output_port.open(getName() + "/state:o"))
    //     return false;

    if(!debug_port.open(getName("/debug:o")))
        return false;

    //input_port.setQLimit(rf.check("qlimit", Value(0)).asInt());
    if(!input_port.open(getName("/AE:i")))
        return false;

    yarp::os::Network::connect("/atis3/AE:o", getName("/AE:i"), "fast_tcp");
    yarp::os::Network::connect(getName("/debug:o"), "/ptView", "fast_tcp");

    //if(!start)
     //   pause();

    return Thread::start();

}

bool delayControl::interruptModule()
{
    return Thread::stop();
}

bool delayControl::updateModule()
{
    //output the scope if connected
    // if(scopePort.getOutputCount()) {
    //     scopePort.prepare() = getTrackingStats();
    //     scopePort.write();
    // }

    //output the debug image if connected
    if(debug_port.getOutputCount()) {
        ImageOf<PixelBgr> &image = debug_port.prepare();

        
        image.resize(res.width, res.height);
        image.zero();
        m.lock();
        drawTemplate(image, vpf.appearance, avgx, avgy, avgr);
        drawEvents(image, qROI.q);
        m.unlock();

        debug_port.write();
    }

    return Thread::isRunning();
}

double delayControl::getPeriod()
{
    return 0.05;
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

    stats[0] = 0;//1000*input_port.queryDelayT();
    stats[1] = 1.0/filterPeriod;
    stats[2] = targetproc;
    stats[3] = 0;//input_port.queryRate() / 1000.0;
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
    input_port.stop();
    debug_port.close();

    if(fs.is_open()) 
    {
        yInfo() << "Writing data";
        for(auto i : data_to_save)
            fs << i[0] << " " << i[1] << " " << i[2] << " " << i[3] << std::endl;
        fs.close();
        yInfo() << "Finished Writing data";
    }

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
    
    //initialise the position
    vpf.extractTargetPosition(avgx, avgy, avgr);
    double roisize = avgr + 20;
    qROI.setROI(avgx - roisize, avgx + roisize, avgy - roisize, avgy + roisize);

    //read some data to extract the channel
    if (fs.is_open())
        data_to_save.push_back({start_time, avgx, avgy, 0.0});

    double time_offset = -1.0;
    double time_now = -1.0;
    while(time_now-time_offset <= start_time) {
        ev::info read_stats = input_port.readChunkN(1);
        if(input_port.isStopping()) return;
        time_now = input_port.begin().packetTime();
        if(time_offset < 0) {
            time_offset = input_port.begin().packetTime();
        }

        m.lock();
        for(auto a = input_port.begin(); a != input_port.end(); a++) {
            qROI.add(*a);
        }
        m.unlock();
    }
    
    m.lock();
    if (batch_size)
        qROI.setSize(batch_size);
    else
        qROI.setSize(300);
    m.unlock();

    double nw = 0;
    unsigned int addEvents = 0;
    unsigned int testedEvents = 0;

    targetproc = M_PI * avgr;

    while(true) {

        if(input_port.isStopping())
            break;

        input_port.readChunkN(1);

        m.lock();
        for(auto a = input_port.begin(); a != input_port.end(); a++) {  
            addEvents += qROI.add(*a);
            if(addEvents > targetproc) 
            {
                addEvents = 0;
                if (batch_size) qROI.setSize(batch_size);
                m.unlock();
                vpf.performObservation(qROI.q);
                vpf.extractTargetWindow(nw);
                vpf.extractTargetPosition(avgx, avgy, avgr);
                vpf.performResample();
                vpf.performPrediction(motionVariance);

                roisize = avgr + 20;
                qROI.setROI(avgx - roisize, avgx + roisize, avgy - roisize, avgy + roisize);

                // set our new window #events
                m.lock();
                if (!batch_size) {
                    if (qROI.q.size() - nw > 30)
                        qROI.setSize(std::max(nw, 300.0));
                    if (qROI.q.size() > 2000)
                        qROI.setSize(2000);
                }

                if (fs.is_open())
                    data_to_save.push_back({a.packetTime() - time_offset, avgx, avgy, input_port.getUnprocessedDelay()});
            }
        }
        m.unlock();
        


        

        
        // while(addEvents < targetproc) {

        //     //if we ran out of events get a new queue
        //     if(i >= q->size()) {
        //         i = 0;
        //         q = input_port.read();
        //         if(!q || Thread::isStopping()) {
        //             m.unlock();
        //             return;
        //         }
        //         input_port.getEnvelope(ystamp);
        //     }

        //     addEvents += qROI.add((*q)[i]);
        //     testedEvents++;
        //     i++;
        // }
        // Tgetwindow = yarp::os::Time::now() - Tgetwindow;

        // //if using a batch fix the size to the batch size ALWAYS


        // m.unlock();

        // //update the particle weights
        // Tlikelihood = yarp::os::Time::now();
        // vpf.performObservation(qROI.q);
        // Tlikelihood = yarp::os::Time::now() - Tlikelihood;

        // //extract the state
        // vpf.extractTargetWindow(nw);
        // dx = avgx, dy = avgy, dr = avgr;
        // vpf.extractTargetPosition(avgx, avgy, avgr);
        // dx = avgx - dx; dy = avgy - dy; dr = avgr - dr;

        // // if (debug_port.getOutputCount()) {
        // //     ImageOf<PixelBgr> &image = debug_port.prepare();
        // //     image.resize(res.width, res.height);
        // //     image.zero();
        // //     drawTemplate(image, vpf.appearance, avgx, avgy, avgr);
        // //     drawEvents(image, qROI.q);
        // //     debug_port.write();
        // // }
        // //yarp::os::Time::delay(0.05);

        // Tresample = yarp::os::Time::now();
        // vpf.performResample();
        // Tresample = yarp::os::Time::now() - Tresample;

        // Tpredict = yarp::os::Time::now();
        // vpf.performPrediction(motionVariance);
        // Tpredict = yarp::os::Time::now() - Tpredict;

        // int is_tracking = 1;
        // if(vpf.maxlikelihood < detectionThreshold)
        //     is_tracking = 0;

        // //to compute the delay here, we should probably have the
        // double data_time_passed = ystamp.getTime() - time_offset;
        // if(fs.is_open())
        //     data_to_save.push_back({data_time_passed, avgx, avgy, input_port.getPendingReads() * 0.004});

        // double delta_x = avgx - px;
        // double delta_y = avgy - py;
        // double dpos = sqrt(delta_x * delta_x + delta_y * delta_y);
        // if(dpos > output_sample_delta) {
        //     px = avgx;
        //     py = avgy;
        //     pr = avgr;

        //     double tw = qROI.q.front().ts - qROI.q.back().ts;
        //     if(tw < 0) tw += ev::max_stamp;

            //output our event
            // if(event_output_port.getOutputCount()) {
            //     auto ceg = make_event<GaussianAE>();
            //     ceg->stamp = qROI.q.front().stamp;
            //     ceg->setChannel(channel);
            //     ceg->x = avgx;
            //     ceg->y = avgy;
            //     ceg->sigx = avgr;
            //     ceg->sigy = tw;
            //     ceg->sigxy = 1.0;
            //     if(is_tracking)
            //         ceg->polarity = 1.0;
            //     else
            //         ceg->polarity = 0.0;

            //     vQueue outq; outq.push_back(ceg);
            //     event_output_port.write(outq, ystamp);

            // }
            //output the raw data
            // if(raw_output_port.getOutputCount()) {
            //     Bottle &next_sample = raw_output_port.prepare();
            //     next_sample.clear();
            //     next_sample.addVocab(createVocab('T', '_', 'S', 'T'));
            //     next_sample.addInt(is_tracking);
            //     next_sample.addDouble(qROI.q.front().stamp);
            //     next_sample.addDouble(Time::now());
            //     next_sample.addDouble(avgx);
            //     next_sample.addDouble(avgy);
            //     next_sample.addDouble(avgr);
            //     next_sample.addDouble(channel);
            //     next_sample.addDouble(tw);
            //     next_sample.addDouble(vpf.maxlikelihood);

            //     raw_output_port.setEnvelope(ystamp);
            //     raw_output_port.write();
            // }


        //}

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
