/*
 * Copyright (c) 2008-2018, Andrew Walker
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#ifdef WIN32
#define _USE_MATH_DEFINES
#endif
#include "dubins.hpp"

#include <limits>
#include <cmath>
#include <numbers>

#define EPSILON std::numeric_limits<real_t>::epsilon()
#define PI std::numbers::pi_v<real_t>
#define INF std::numeric_limits<real_t>::infinity()

typedef enum 
{
    L_SEG = 0,
    S_SEG = 1,
    R_SEG = 2
} SegmentType;

/* The segment types for each of the Path types */
constexpr SegmentType DIRDATA[][3] = {
    { L_SEG, S_SEG, L_SEG },
    { L_SEG, S_SEG, R_SEG },
    { R_SEG, S_SEG, L_SEG },
    { R_SEG, S_SEG, R_SEG },
    { R_SEG, L_SEG, R_SEG },
    { L_SEG, R_SEG, L_SEG }
};

typedef struct 
{
    real_t alpha;
    real_t beta;
    real_t d;
    real_t sa;
    real_t sb;
    real_t ca;
    real_t cb;
    real_t c_ab;
    real_t d_sq;
} DubinsIntermediateResults;


int dubins_word(DubinsIntermediateResults* in, DubinsPathType pathType, real_t out[3]);
int dubins_intermediate_results(DubinsIntermediateResults* in, real_t q0[3], real_t q1[3], real_t rho);

/**
 * Floating point modulus suitable for rings
 *
 * fmod doesn't behave correctly for angular quantities, this function does
 */
real_t fmodr( real_t x, real_t y)
{
    return x - y*std::floor(x/y);
}

real_t mod2pi( real_t theta )
{
    return fmodr( theta, 2 * PI );
}

int dubins_shortest_path(DubinsPath* path, real_t q0[3], real_t q1[3], real_t rho)
{
    int i, errcode;
    DubinsIntermediateResults in;
    real_t params[3];
    real_t cost;
    real_t best_cost = INF;
    int best_word = -1;
    errcode = dubins_intermediate_results(&in, q0, q1, rho);
    if(errcode != EDUBOK) {
        return errcode;
    }


    path->qi[0] = q0[0];
    path->qi[1] = q0[1];
    path->qi[2] = q0[2];
    path->rho = rho;
 
    for( i = 0; i < 6; i++ ) {
        DubinsPathType pathType = (DubinsPathType)i;
        errcode = dubins_word(&in, pathType, params);
        if(errcode == EDUBOK) {
            cost = params[0] + params[1] + params[2];
            if(cost < best_cost) {
                best_word = i;
                best_cost = cost;
                path->param[0] = params[0];
                path->param[1] = params[1];
                path->param[2] = params[2];
                path->type = pathType;
            }
        }
    }
    if(best_word == -1) {
        return EDUBNOPATH;
    }
    return EDUBOK;
}

int dubins_path(DubinsPath* path, real_t q0[3], real_t q1[3], real_t rho, DubinsPathType pathType)
{
    int errcode;
    DubinsIntermediateResults in;
    errcode = dubins_intermediate_results(&in, q0, q1, rho);
    if(errcode == EDUBOK) {
        real_t params[3];
        errcode = dubins_word(&in, pathType, params);
        if(errcode == EDUBOK) {
            path->param[0] = params[0];
            path->param[1] = params[1];
            path->param[2] = params[2];
            path->qi[0] = q0[0];
            path->qi[1] = q0[1];
            path->qi[2] = q0[2];
            path->rho = rho;
            path->type = pathType;
        }
    }
    return errcode;
}

real_t dubins_path_length( DubinsPath* path )
{
    real_t length = 0.;
    length += path->param[0];
    length += path->param[1];
    length += path->param[2];
    length = length * path->rho;
    return length;
}

real_t dubins_segment_length( DubinsPath* path, int i )
{
    if( (i < 0) || (i > 2) )
    {
        return INF;
    }
    return path->param[i] * path->rho;
}

real_t dubins_segment_length_normalized( DubinsPath* path, int i )
{
    if( (i < 0) || (i > 2) )
    {
        return INF;
    }
    return path->param[i];
} 

DubinsPathType dubins_path_type( DubinsPath* path ) 
{
    return path->type;
}

void dubins_segment( real_t t, real_t qi[3], real_t qt[3], SegmentType type)
{
    real_t st = std::sin(qi[2]);
    real_t ct = std::cos(qi[2]);
    if( type == L_SEG ) {
        qt[0] = +std::sin(qi[2]+t) - st;
        qt[1] = -std::cos(qi[2]+t) + ct;
        qt[2] = t;
    }
    else if( type == R_SEG ) {
        qt[0] = -std::sin(qi[2]-t) + st;
        qt[1] = +std::cos(qi[2]-t) - ct;
        qt[2] = -t;
    }
    else if( type == S_SEG ) {
        qt[0] = ct * t;
        qt[1] = st * t;
        qt[2] = 0.0;
    }
    qt[0] += qi[0];
    qt[1] += qi[1];
    qt[2] += qi[2];
}

int dubins_path_sample( DubinsPath* path, real_t t, real_t q[3] )
{
    /* tprime is the normalised variant of the parameter t */
    real_t tprime = t / path->rho;
    real_t qi[3]; /* The translated initial configuration */
    real_t q1[3]; /* end-of segment 1 */
    real_t q2[3]; /* end-of segment 2 */
    const SegmentType* types = DIRDATA[path->type];
    real_t p1, p2;

    if( t < 0 || t > dubins_path_length(path) ) {
        return EDUBPARAM;
    }

    /* initial configuration */
    qi[0] = 0.0;
    qi[1] = 0.0;
    qi[2] = path->qi[2];

    /* generate the target configuration */
    p1 = path->param[0];
    p2 = path->param[1];
    dubins_segment( p1,      qi,    q1, types[0] );
    dubins_segment( p2,      q1,    q2, types[1] );
    if( tprime < p1 ) {
        dubins_segment( tprime, qi, q, types[0] );
    }
    else if( tprime < (p1+p2) ) {
        dubins_segment( tprime-p1, q1, q,  types[1] );
    }
    else {
        dubins_segment( tprime-p1-p2, q2, q,  types[2] );
    }

    /* scale the target configuration, translate back to the original starting point */
    q[0] = q[0] * path->rho + path->qi[0];
    q[1] = q[1] * path->rho + path->qi[1];
    q[2] = mod2pi(q[2]);

    return EDUBOK;
}

int dubins_path_sample_many(DubinsPath* path, real_t stepSize, 
                            DubinsPathSamplingCallback cb, void* user_data)
{
    int retcode;
    real_t q[3];
    real_t x = 0.0;
    real_t length = dubins_path_length(path);
    while( x <  length ) {
        dubins_path_sample( path, x, q );
        retcode = cb(q, x, user_data);
        if( retcode != 0 ) {
            return retcode;
        }
        x += stepSize;
    }
    return 0;
}

int dubins_path_endpoint( DubinsPath* path, real_t q[3] )
{
    return dubins_path_sample( path, dubins_path_length(path) - EPSILON, q );
}

int dubins_extract_subpath( DubinsPath* path, real_t t, DubinsPath* newpath )
{
    /* calculate the true parameter */
    real_t tprime = t / path->rho;

    if((t < 0) || (t > dubins_path_length(path)))
    {
        return EDUBPARAM; 
    }

    /* copy most of the data */
    newpath->qi[0] = path->qi[0];
    newpath->qi[1] = path->qi[1];
    newpath->qi[2] = path->qi[2];
    newpath->rho   = path->rho;
    newpath->type  = path->type;

    /* fix the parameters */
    newpath->param[0] = std::fmin( path->param[0], tprime );
    newpath->param[1] = std::fmin( path->param[1], tprime - newpath->param[0]);
    newpath->param[2] = std::fmin( path->param[2], tprime - newpath->param[0] - newpath->param[1]);
    return 0;
}

int dubins_intermediate_results(DubinsIntermediateResults* in, real_t q0[3], real_t q1[3], real_t rho)
{
    real_t dx, dy, D, d, theta, alpha, beta;
    if( rho <= 0.0 ) {
        return EDUBBADRHO;
    }

    dx = q1[0] - q0[0];
    dy = q1[1] - q0[1];
    D = std::sqrt( dx * dx + dy * dy );
    d = D / rho;
    theta = 0;

    /* test required to prevent domain errors if dx=0 and dy=0 */
    if(d > 0) {
        theta = mod2pi(std::atan2( dy, dx ));
    }
    alpha = mod2pi(q0[2] - theta);
    beta  = mod2pi(q1[2] - theta);

    in->alpha = alpha;
    in->beta  = beta;
    in->d     = d;
    in->sa    = std::sin(alpha);
    in->sb    = std::sin(beta);
    in->ca    = std::cos(alpha);
    in->cb    = std::cos(beta);
    in->c_ab  = std::cos(alpha - beta);
    in->d_sq  = d * d;

    return EDUBOK;
}

int dubins_LSL(DubinsIntermediateResults* in, real_t out[3]) 
{
    real_t tmp0, tmp1, p_sq;
    
    tmp0 = in->d + in->sa - in->sb;
    p_sq = 2 + in->d_sq - (2*in->c_ab) + (2 * in->d * (in->sa - in->sb));

    if(p_sq >= 0) {
        tmp1 = std::atan2( (in->cb - in->ca), tmp0 );
        out[0] = mod2pi(tmp1 - in->alpha);
        out[1] = std::sqrt(p_sq);
        out[2] = mod2pi(in->beta - tmp1);
        return EDUBOK;
    }
    return EDUBNOPATH;
}


int dubins_RSR(DubinsIntermediateResults* in, real_t out[3]) 
{
    real_t tmp0 = in->d - in->sa + in->sb;
    real_t p_sq = 2 + in->d_sq - (2 * in->c_ab) + (2 * in->d * (in->sb - in->sa));
    if( p_sq >= 0 ) {
        real_t tmp1 = std::atan2( (in->ca - in->cb), tmp0 );
        out[0] = mod2pi(in->alpha - tmp1);
        out[1] = std::sqrt(p_sq);
        out[2] = mod2pi(tmp1 -in->beta);
        return EDUBOK;
    }
    return EDUBNOPATH;
}

int dubins_LSR(DubinsIntermediateResults* in, real_t out[3]) 
{
    real_t p_sq = -2 + (in->d_sq) + (2 * in->c_ab) + (2 * in->d * (in->sa + in->sb));
    if( p_sq >= 0 ) {
        real_t p    = std::sqrt(p_sq);
        real_t tmp0 = std::atan2( (-in->ca - in->cb), (in->d + in->sa + in->sb) ) - std::atan2(-2.0, p);
        out[0] = mod2pi(tmp0 - in->alpha);
        out[1] = p;
        out[2] = mod2pi(tmp0 - mod2pi(in->beta));
        return EDUBOK;
    }
    return EDUBNOPATH;
}

int dubins_RSL(DubinsIntermediateResults* in, real_t out[3]) 
{
    real_t p_sq = -2 + in->d_sq + (2 * in->c_ab) - (2 * in->d * (in->sa + in->sb));
    if( p_sq >= 0 ) {
        real_t p    = std::sqrt(p_sq);
        real_t tmp0 = std::atan2( (in->ca + in->cb), (in->d - in->sa - in->sb) ) - std::atan2(2.0, p);
        out[0] = mod2pi(in->alpha - tmp0);
        out[1] = p;
        out[2] = mod2pi(in->beta - tmp0);
        return EDUBOK;
    }
    return EDUBNOPATH;
}

int dubins_RLR(DubinsIntermediateResults* in, real_t out[3]) 
{
    real_t tmp0 = (6. - in->d_sq + 2*in->c_ab + 2*in->d*(in->sa - in->sb)) / 8.;
    real_t phi  = std::atan2( in->ca - in->cb, in->d - in->sa + in->sb );
    if( std::abs(tmp0) <= 1) {
        real_t p = mod2pi((2*PI) - std::acos(tmp0) );
        real_t t = mod2pi(in->alpha - phi + mod2pi(p/2.));
        out[0] = t;
        out[1] = p;
        out[2] = mod2pi(in->alpha - in->beta - t + mod2pi(p));
        return EDUBOK;
    }
    return EDUBNOPATH;
}

int dubins_LRL(DubinsIntermediateResults* in, real_t out[3])
{
    real_t tmp0 = (6. - in->d_sq + 2*in->c_ab + 2*in->d*(in->sb - in->sa)) / 8.;
    real_t phi = std::atan2( in->ca - in->cb, in->d + in->sa - in->sb );
    if( std::abs(tmp0) <= 1) {
        real_t p = mod2pi( 2*PI - std::acos( tmp0) );
        real_t t = mod2pi(-in->alpha - phi + p/2.);
        out[0] = t;
        out[1] = p;
        out[2] = mod2pi(mod2pi(in->beta) - in->alpha -t + mod2pi(p));
        return EDUBOK;
    }
    return EDUBNOPATH;
}

int dubins_word(DubinsIntermediateResults* in, DubinsPathType pathType, real_t out[3]) 
{
    int result;
    switch(pathType)
    {
    case LSL:
        result = dubins_LSL(in, out);
        break;
    case RSL:
        result = dubins_RSL(in, out);
        break;
    case LSR:
        result = dubins_LSR(in, out);
        break;
    case RSR:
        result = dubins_RSR(in, out);
        break;
    case LRL:
        result = dubins_LRL(in, out);
        break;
    case RLR:
        result = dubins_RLR(in, out);
        break;
    default:
        result = EDUBNOPATH;
    }
    return result;
}


