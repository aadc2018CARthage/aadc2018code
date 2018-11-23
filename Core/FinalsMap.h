#pragma once

#include "stdafx.h"
#include "aadc_structs.h"
#include <a_utils/core/a_utils_core.h>

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;

class FinalsMap
{
private:

public:
    FinalsMap() = default;
};

class Sign
{
private:

public:
    tInt32 id;
};

class Crossing : public Sign
{
private:

public:
    tInt32 right;
    tInt32 straight;
    tInt32 left;
};

class Parking : public Sign
{
public:
    tInt32 right;
    tInt32 left;
};

class Zebra : public Sign
{
public:
    tInt32 straight;
};
