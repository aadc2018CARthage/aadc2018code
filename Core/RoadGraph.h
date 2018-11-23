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

class RoadGraph
{
private:
	enum class Direction { Left, Straight, Right };
	enum class RoadType { Straight, Curve, Junction };

    typedef tUInt32 RoadId;
    typedef tInt LaneId;

	struct Connection
	{
		RoadId road_id;
		LaneId lane_id;
		Direction direction = Direction::Straight;
	};

	struct Lane
	{
		LaneId id;
		tFloat64 length;
		std::vector<Connection> next;
	};

	struct RoadElement
	{
		RoadId id;
		RoadType type = RoadType::Straight;
        std::vector<Lane> lanes;
	};

	std::map<RoadId, RoadElement> roads;
	Connection current;

public:
    RoadGraph() = default;
    tResult load(const cString& opendrive);
};
