#define A_UTILS_NO_DEPRECATED_WARNING

#include "RoadGraph.h"

tResult RoadGraph::load(const cString& opendrive)
{
	cDOM oDOM;
	oDOM.FromString(opendrive);

	cDOMElementRefList oRoads;
	if (IS_OK(oDOM.FindNodes("road", oRoads)))
	{
		for (auto it = oRoads.begin(); it != oRoads.end(); it++)
		{
			RoadElement road;
			// id of road element
			road.id = (*it)->GetAttributeUInt32("id");
			// length of the geometry reference line as rough approximation for lane lengths
			tFloat64 length = (*it)->GetAttributeFloat64("length");
			// is this part of a junction ?
			cString junction = (*it)->GetAttribute("junction");

			// lateral offset between start and end as indicator for curves
			cDOMElement *paramPoly3;
            RETURN_IF_FAILED((*it)->FindNode("planView/geometry/paramPoly3", paramPoly3));
			tFloat64 bV, cV, dV;
			bV = paramPoly3->GetAttributeFloat64("bV");
			cV = paramPoly3->GetAttributeFloat64("cV");
			dV = paramPoly3->GetAttributeFloat64("dV");
			tFloat64 relTrans = (bV + cV + dV) / length;

			if (junction != "-1")
			{
				road.type = RoadType::Junction;
			}
            else if (std::abs(relTrans) > 0.5f)
			{
				road.type = RoadType::Curve;
			}
			else
			{
				road.type = RoadType::Straight;
			}

			// get successor
			cDOMElement *succ;
            RETURN_IF_FAILED((*it)->FindNode("link/successor", succ));
			tUInt32 succ_id = succ->GetAttributeUInt32("elementId");
			cString succ_type = succ->GetAttribute("elementType");

			// iterate left lanes
            cDOMElementRefList leftLanes;
            RETURN_IF_FAILED((*it)->FindNodes("lanes/laneSection/left/lane", leftLanes));
			for (auto left_it = leftLanes.begin(); left_it != leftLanes.end(); left_it++)
			{
				Lane lane;
				lane.id = (*left_it)->GetAttributeInt("id");
				lane.length = length;
				if (succ_type == "road")
				{
					// successor is road type, has single connection
					cDOMElement *lane_succ;
                    RETURN_IF_FAILED((*left_it)->FindNode("link/successor", lane_succ));
					
					Connection conn;
					conn.road_id = succ_id;
					conn.lane_id = lane_succ->GetAttributeUInt32("id");
					if (junction != "-1")
					{
                        if (relTrans > 0.5f)
						{
							conn.direction = Direction::Right;
						}
                        else if (relTrans < -0.5f)
						{
							conn.direction = Direction::Left;
						}
						else
						{
							conn.direction = Direction::Straight;
						}
					}
					else
					{
						conn.direction = Direction::Straight;
					}
					// add connection to lane
					lane.next.push_back(conn);
				}
				else if (succ_type == "junction")
				{
					// successor is junction type, may have multiple connections
                    cDOMElementRefList successors;
                    cString query = cString::Format("junction[\@id='%d']/connection[\@incomingRoad='%d']/laneLink[\@from='%d']", succ_id, road.id, lane.id);
                    RETURN_IF_FAILED(oDOM.FindNodes(query.GetPtr(), successors));
					for (auto succ_it = successors.begin(); succ_it != successors.end(); succ_it++)
					{
						Connection conn;
                        conn.road_id = (*succ_it)->GetParent().GetAttributeUInt32("connectingRoad");
						conn.lane_id = (*succ_it)->GetAttributeInt("to");
						conn.direction = Direction::Straight;
						// add connectio to lane
						lane.next.push_back(conn);
					}
				}
				road.lanes.push_back(lane);
			}

			// iterate rigth lanes
            cDOMElementRefList rightLanes;
            RETURN_IF_FAILED((*it)->FindNodes("lanes/laneSection/right/lane", rightLanes));
			for (auto right_it = rightLanes.begin(); right_it != rightLanes.end(); right_it++)
			{
				Lane lane;
				lane.id = (*right_it)->GetAttributeInt("id");
				lane.length = length;
				if (succ_type == "road")
				{
					// successor is road type, has single connection
					cDOMElement *lane_succ;
                    RETURN_IF_FAILED((*right_it)->FindNode("link/successor", lane_succ));

					Connection conn;
					conn.road_id = succ_id;
					conn.lane_id = lane_succ->GetAttributeUInt32("id");
					if (junction != "-1")
					{
                        if (relTrans > 0.5f)
						{
							conn.direction = Direction::Left;
						}
                        else if (relTrans < -0.5f)
						{
							conn.direction = Direction::Right;
						}
						else
						{
							conn.direction = Direction::Straight;
						}
					}
					else
					{
						conn.direction = Direction::Straight;
					}
					// add connection to lane
					lane.next.push_back(conn);
				}
				else if (succ_type == "junction")
				{
					// successor is junction type, may have multiple connections
                    cDOMElementRefList successors;
					cString query = cString::Format("junction[\@id='%d']/connection[\@incomingRoad='%d']/laneLink[\@from='%d']", succ_id, road.id, lane.id);
                    RETURN_IF_FAILED(oDOM.FindNodes(query.GetPtr(), successors));
					for (auto succ_it = successors.begin(); succ_it != successors.end(); succ_it++)
					{
						Connection conn;
                        conn.road_id = (*succ_it)->GetParent().GetAttributeUInt32("connectingRoad");
						conn.lane_id = (*succ_it)->GetAttributeInt("to");
						conn.direction = Direction::Straight;
						// add connection to lane
						lane.next.push_back(conn);
					}
				}

				// add lane to road
				road.lanes.push_back(lane);
			}

			// save road to map
			roads[road.id] = road;
		}
	}
    RETURN_NOERROR;
}
