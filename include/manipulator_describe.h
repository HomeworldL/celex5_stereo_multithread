#pragma once

#include <iostream>

typedef struct D_H
{
    double a;
    double alpha;
    double d;
    double theta;
} D_H;


typedef struct Link
{
    D_H DH;

} Link;

typedef struct Manipulator
{
    Link link1;
    Link link2;
    Link link3;
    Link link4;
    Link link5;
    Link link6;
    Link link7;
    Link tool;
} Manipulator;