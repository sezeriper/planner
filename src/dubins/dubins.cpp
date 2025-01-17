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
#include <cmath>
#include "dubins.hpp"
#include <numbers>

constexpr float PI = std::numbers::pi_v<float>;
constexpr float TWO_PI = 2.0f * PI;
constexpr float INF = std::numeric_limits<float>::max();


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
    float alpha;
    float beta;
    float d;
    float sa;
    float sb;
    float ca;
    float cb;
    float c_ab;
    float d_sq;
} DubinsIntermediateResults;


int dubins_word(DubinsIntermediateResults* in, DubinsPathType pathType, float out[3]);
int dubins_intermediate_results(DubinsIntermediateResults* in, const float q0[3], const float q1[3], float rho);

/**
 * Floating vec2_t modulus suitable for rings
 *
 * fmod doesn't behave correctly for angular quantities, this function does
 */
constexpr float fmodr(float x, float y)
{
    return x - y * std::floor(x/y);
}

constexpr float mod2pi(float theta)
{
    return fmodr(theta, 2.0f * PI);
}

int dubins_shortest_path(DubinsPath& path, const float q0[3], const float q1[3], float rho)
{
    int i, errcode;
    DubinsIntermediateResults in;
    float params[3];
    float cost;
    float best_cost = INF;
    int best_word = -1;
    errcode = dubins_intermediate_results(&in, q0, q1, rho);
    if(errcode != EDUBOK) {
        return errcode;
    }

    path.qi[0] = q0[0];
    path.qi[1] = q0[1];
    path.qi[2] = q0[2];
    path.rho = rho;
 
    for( i = 0; i < 6; ++i ) {
        DubinsPathType pathType = (DubinsPathType)i;
        errcode = dubins_word(&in, pathType, params);
        if(errcode == EDUBOK) {
            cost = params[0] + params[1] + params[2];
            if(cost < best_cost) {
                best_word = i;
                best_cost = cost;
                path.param[0] = params[0];
                path.param[1] = params[1];
                path.param[2] = params[2];
                path.type = pathType;
            }
        }
    }
    if(best_word == -1) {
        return EDUBNOPATH;
    }
    return EDUBOK;
}

int dubins_path(DubinsPath& path, float q0[3], float q1[3], float rho, DubinsPathType pathType)
{
    int errcode;
    DubinsIntermediateResults in;
    errcode = dubins_intermediate_results(&in, q0, q1, rho);
    if(errcode == EDUBOK) {
        float params[3];
        errcode = dubins_word(&in, pathType, params);
        if(errcode == EDUBOK) {
            path.param[0] = params[0];
            path.param[1] = params[1];
            path.param[2] = params[2];
            path.qi[0] = q0[0];
            path.qi[1] = q0[1];
            path.qi[2] = q0[2];
            path.rho = rho;
            path.type = pathType;
        }
    }
    return errcode;
}

float dubins_path_length( const DubinsPath& path )
{
    float length = 0.0f;
    length += path.param[0];
    length += path.param[1];
    length += path.param[2];
    length = length * path.rho;
    return length;
}

float dubins_segment_length( const DubinsPath& path, int i )
{
    if( (i < 0) || (i > 2) )
    {
        return INF;
    }
    return path.param[i] * path.rho;
}

float dubins_segment_length_normalized( const DubinsPath& path, int i )
{
    if( (i < 0) || (i > 2) )
    {
        return INF;
    }
    return path.param[i];
} 

DubinsPathType dubins_path_type( const DubinsPath& path ) 
{
    return path.type;
}

void dubins_segment( float t, float qi[3], float qt[3], SegmentType type)
{
    float st = std::sin(qi[2]);
    float ct = std::cos(qi[2]);
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
        qt[2] = 0.0f;
    }
    qt[0] += qi[0];
    qt[1] += qi[1];
    qt[2] += qi[2];
}

int dubins_path_sample( const DubinsPath& path, float t, float q[3] )
{
    /* tprime is the normalised variant of the parameter t */
    float tprime = t / path.rho;
    float qi[3]; /* The translated initial configuration */
    float q1[3]; /* end-of segment 1 */
    float q2[3]; /* end-of segment 2 */
    const SegmentType* types = DIRDATA[static_cast<int>(path.type)];
    float p1, p2;

    if( t < 0.0f || t > dubins_path_length(path) ) {
        return EDUBPARAM;
    }

    /* initial configuration */
    qi[0] = 0.0f;
    qi[1] = 0.0f;
    qi[2] = path.qi[2];

    /* generate the target configuration */
    p1 = path.param[0];
    p2 = path.param[1];
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

    /* scale the target configuration, translate back to the original starting vec2_t */
    q[0] = q[0] * path.rho + path.qi[0];
    q[1] = q[1] * path.rho + path.qi[1];
    q[2] = mod2pi(q[2]);

    return EDUBOK;
}

int dubins_path_sample_many(const DubinsPath& path, float stepSize, 
                            DubinsPathSamplingCallback cb, void* user_data)
{
    int retcode;
    float q[3];
    float x = 0.0f;
    float length = dubins_path_length(path);
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

int dubins_path_endpoint( const DubinsPath& path, float q[3] )
{
    return dubins_path_sample( path, dubins_path_length(path) - EPSILON, q );
}

int dubins_extract_subpath( const DubinsPath& path, float t, DubinsPath* newpath )
{
    /* calculate the true parameter */
    float tprime = t / path.rho;

    if((t < 0.0f) || (t > dubins_path_length(path)))
    {
        return EDUBPARAM; 
    }

    /* copy most of the data */
    newpath->qi[0] = path.qi[0];
    newpath->qi[1] = path.qi[1];
    newpath->qi[2] = path.qi[2];
    newpath->rho   = path.rho;
    newpath->type  = path.type;

    /* fix the parameters */
    newpath->param[0] = std::fmin( path.param[0], tprime );
    newpath->param[1] = std::fmin( path.param[1], tprime - newpath->param[0]);
    newpath->param[2] = std::fmin( path.param[2], tprime - newpath->param[0] - newpath->param[1]);
    return 0;
}

int dubins_intermediate_results(DubinsIntermediateResults* in, const float q0[3], const float q1[3], float rho)
{
    float dx, dy, D, d, theta, alpha, beta;
    if(rho <= 0.0f) {
        return EDUBBADRHO;
    }

    dx = q1[0] - q0[0];
    dy = q1[1] - q0[1];
    D = std::sqrt( dx * dx + dy * dy );
    d = D / rho;
    theta = 0.0f;

    /* test required to prevent domain errors if dx=0 and dy=0 */
    if(d > 0.0f) {
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

int dubins_LSL(DubinsIntermediateResults* in, float out[3]) 
{
    float tmp0, tmp1, p_sq;
    
    tmp0 = in->d + in->sa - in->sb;
    p_sq = 2.0f + in->d_sq - (2.0f * in->c_ab) + (2.0f * in->d * (in->sa - in->sb));

    if(p_sq >= 0.0f) {
        tmp1 = std::atan2( (in->cb - in->ca), tmp0 );
        out[0] = mod2pi(tmp1 - in->alpha);
        out[1] = std::sqrt(p_sq);
        out[2] = mod2pi(in->beta - tmp1);
        return EDUBOK;
    }
    return EDUBNOPATH;
}


int dubins_RSR(DubinsIntermediateResults* in, float out[3]) 
{
    float tmp0 = in->d - in->sa + in->sb;
    float p_sq = 2.0f + in->d_sq - (2.0f * in->c_ab) + (2.0f * in->d * (in->sb - in->sa));
    if( p_sq >= 0.0f ) {
        float tmp1 = std::atan2( (in->ca - in->cb), tmp0 );
        out[0] = mod2pi(in->alpha - tmp1);
        out[1] = std::sqrt(p_sq);
        out[2] = mod2pi(tmp1 -in->beta);
        return EDUBOK;
    }
    return EDUBNOPATH;
}

int dubins_LSR(DubinsIntermediateResults* in, float out[3]) 
{
    float p_sq = -2.0f + (in->d_sq) + (2.0f * in->c_ab) + (2.0f * in->d * (in->sa + in->sb));
    if( p_sq >= 0.0f ) {
        float p    = std::sqrt(p_sq);
        float tmp0 = std::atan2( (-in->ca - in->cb), (in->d + in->sa + in->sb) ) - std::atan2(-2.0f, p);
        out[0] = mod2pi(tmp0 - in->alpha);
        out[1] = p;
        out[2] = mod2pi(tmp0 - mod2pi(in->beta));
        return EDUBOK;
    }
    return EDUBNOPATH;
}

int dubins_RSL(DubinsIntermediateResults* in, float out[3]) 
{
    float p_sq = -2.0f + in->d_sq + (2.0f * in->c_ab) - (2.0f * in->d * (in->sa + in->sb));
    if( p_sq >= 0.0f ) {
        float p    = std::sqrt(p_sq);
        float tmp0 = std::atan2( (in->ca + in->cb), (in->d - in->sa - in->sb) ) - std::atan2(2.0f, p);
        out[0] = mod2pi(in->alpha - tmp0);
        out[1] = p;
        out[2] = mod2pi(in->beta - tmp0);
        return EDUBOK;
    }
    return EDUBNOPATH;
}

int dubins_RLR(DubinsIntermediateResults* in, float out[3]) 
{
    float tmp0 = (6.0f - in->d_sq + 2.0f*in->c_ab + 2.0f*in->d*(in->sa - in->sb)) / 8.0f;
    float phi  = std::atan2( in->ca - in->cb, in->d - in->sa + in->sb );
    if( std::fabs(tmp0) <= 1.0f) {
        float p = mod2pi((2.0f*PI) - std::acos(tmp0) );
        float t = mod2pi(in->alpha - phi + mod2pi(p/2.0f));
        out[0] = t;
        out[1] = p;
        out[2] = mod2pi(in->alpha - in->beta - t + mod2pi(p));
        return EDUBOK;
    }
    return EDUBNOPATH;
}

int dubins_LRL(DubinsIntermediateResults* in, float out[3])
{
    float tmp0 = (6.0f - in->d_sq + 2.0f*in->c_ab + 2.0f*in->d*(in->sb - in->sa)) / 8.0f;
    float phi = std::atan2( in->ca - in->cb, in->d + in->sa - in->sb );
    if( std::fabs(tmp0) <= 1.0f) {
        float p = mod2pi( 2.0f*PI - std::acos( tmp0) );
        float t = mod2pi(-in->alpha - phi + p/2.0f);
        out[0] = t;
        out[1] = p;
        out[2] = mod2pi(mod2pi(in->beta) - in->alpha -t + mod2pi(p));
        return EDUBOK;
    }
    return EDUBNOPATH;
}

int dubins_word(DubinsIntermediateResults* in, DubinsPathType pathType, float out[3]) 
{
    int result;
    switch(pathType)
    {
    case DubinsPathType::LSL:
        result = dubins_LSL(in, out);
        break;
    case DubinsPathType::RSL:
        result = dubins_RSL(in, out);
        break;
    case DubinsPathType::LSR:
        result = dubins_LSR(in, out);
        break;
    case DubinsPathType::RSR:
        result = dubins_RSR(in, out);
        break;
    case DubinsPathType::LRL:
        result = dubins_LRL(in, out);
        break;
    case DubinsPathType::RLR:
        result = dubins_RLR(in, out);
        break;
    default:
        result = EDUBNOPATH;
    }
    return result;
}