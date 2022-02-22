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

#ifndef __VPARTICLE__
#define __VPARTICLE__

#include <event-driven/core.h>
#include <yarp/sig/all.h>
#include <vector>
#include <deque>

class vParticle;

/*////////////////////////////////////////////////////////////////////////////*/
// templatedParticle
/*////////////////////////////////////////////////////////////////////////////*/
class templatedParticle
{
protected:

    int offset_x, offset_y;

    bool constrain;
    std::vector<double> min_state;
    std::vector<double> max_state;

    //incremental counters
    double likelihood, score, min_likelihood, max_likelihood;
    int n;

    void checkConstraints();

public:

    enum { x = 0, y = 1, s = 2};

    int id;
    std::vector<double> state;
    double weight;
    yarp::sig::ImageOf<yarp::sig::PixelFloat> *appearance;

    templatedParticle();
    //templatedParticle(const templatedParticle &from);
    templatedParticle& operator=(const templatedParticle &rhs);

    void setAppearance(yarp::sig::ImageOf<yarp::sig::PixelFloat> *appearance, double max_likelihood);
    bool setConstraints(std::vector<double> mins, std::vector<double> maxs);
    void predict(double sigma);
    double getl() { return likelihood; }
    double getn() { return n; }
    double getAbsoluteSize() { return offset_x / state[s]; }

    void initLikelihood(int windowSize)
    {
        likelihood = min_likelihood;
        score = 0;
        n = 0;
    }

    inline void incrementalLikelihood(int vx, int vy, int n)
    {
        int index_x = (vx - state[x]) * state[s] + offset_x + 0.5;
        int index_y = (vy - state[y]) * state[s] + offset_y + 0.5;
        if(index_x < 0 ||
           index_y < 0 ||
           index_x >= (int)appearance->width() ||
           index_y >= (int)appearance->height())
        {
            return;
        }

        score += (*appearance)(index_x, index_y);
        if(score >= likelihood)
        {
            likelihood = score;
            this->n = n;
        }

    }

    void concludeLikelihood()
    {
        likelihood *= (state[s] * state[s]);
        if(likelihood < min_likelihood) {
            //yInfo() << "scaling up likelihood from minimum";
            likelihood = min_likelihood;
        }
        if(likelihood > max_likelihood) {
        //    yInfo() << "scaling down likelihood from maximum";
            likelihood = max_likelihood;
        }
        weight *= likelihood;
    }

    void normaliseWithinPopulation(double normval)
    {
        weight *= normval;
    }


};

/*////////////////////////////////////////////////////////////////////////////*/
// vParticleObserver
/*////////////////////////////////////////////////////////////////////////////*/
class vPartObsThread : public yarp::os::Thread
{
private:

    std::mutex processing;
    std::mutex done;
    int pStart;
    int pEnd;

    double normval;

    std::vector<templatedParticle> *particles;
    const std::deque<ev::AE> *stw;

public:

    vPartObsThread(int pStart, int pEnd);
    void setDataSources(std::vector<templatedParticle> *particles, const std::deque<ev::AE> *stw);
    void process();
    double waittilldone();

    void run();
};

/*////////////////////////////////////////////////////////////////////////////*/
//VPARTICLEFILTER
/*////////////////////////////////////////////////////////////////////////////*/

class vParticlefilter
{
private:

    //parameters
    int nparticles;
    int nthreads;
    ev::resolution res;
    bool adaptive;
    double seedx, seedy, seedr;

    //data
    std::vector<templatedParticle> ps;
    std::vector<templatedParticle> ps_snap;
    std::vector<double> accum_dist;
    std::vector<vPartObsThread *> computeThreads;

    //variables
    double pwsumsq;
    int rbound_min;
    int rbound_max;

    double initialiseAsCircle(int r);
    double initialiseAsEllipse(double r);

public:

    double maxlikelihood;
    yarp::sig::ImageOf< yarp::sig::PixelFloat > appearance;

    vParticlefilter() {}

    double initialise(int width, int height, int nparticles, bool adaptive,
                    int nthreads);

    void setSeed(double x, double y, double r = 0.0);
    void resetToSeed();
    void setMinLikelihood(double value);
    void setInlierParameter(double value);
    void setNegativeBias(double value);
    void setAdaptive(bool value = true);

    void setAppearance(const yarp::sig::ImageOf<yarp::sig::PixelFloat> &new_appearance, double max_likelihood);
    void performObservation(const std::deque<ev::AE> &q);
    void extractTargetPosition(double &x, double &y, double &r);
    void extractTargetWindow(double &tw);
    void performResample();
    void performPrediction(double sigma);

    std::vector<templatedParticle> getps();

};



#endif
