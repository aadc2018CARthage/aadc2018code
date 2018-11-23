/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

**********************************************************************/

#pragma once

#include "stdafx.h"
#include "aadc_structs.h"
#include "aadc_jury.h"
#include "RoadGraph.h"
#include <a_utils/core/a_utils_core.h>

//*************************************************************************************************
#define CID_CORE_FILTER  "core_filter.filter.user.aadc.cid"
#define LABEL_CORE_FILTER  "Core Filter"

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;
using namespace aadc::jury;

#define inf 10000

class cCoreFilter : public cFilter
{
public:
    ADTF_CLASS_ID_NAME(cCoreFilter, CID_CORE_FILTER, LABEL_CORE_FILTER);
    ADTF_CLASS_DEPENDENCIES(REQUIRE_INTERFACE(adtf::services::IReferenceClock));

private:

    enum class ManeuverStage { Init = 0, Search, SearchCrossing, CheckTraffic, PullOutStraight, PullOutSteer, ParkingForwardStraight, ParkingForwardSteer, ParkingReverseSteer, ParkingReverseStraight, Run, End, BeforeResetRoi, ResetRoi, Complete };

    enum RoadSigns : tInt16 { Crossing = 0, Stop, ParkingSpace, Rocket, StraightOnly, Yield, ZebraCrossing, Roundabout, NoOvertake, NoEntry, Repostionioning, Oneway, ConstructionSite, SpeedFifty, SpeedHundred };

    enum class Lights { Left = 0, Right, Head, Hazard, Reverse, Brake };

    enum class CrossingSigns : tInt16{ None = 0, Crossing, Stop, Rocket, StraightOnly, Yield };

    enum class Objects : tInt32 { Car = 3, Person = 1, Child = 2, Other = 0}; //have to be assigned according to the trained neural network

    enum class Curve : tInt32 { Left = -1, None = 0, Right = 1};

    #pragma pack(push,1)
    typedef struct
    {
        tInt  situation; //1 = left, 2 = right, 3 = both
        tFloat32  distance;
    } tCrossingSituation;
    #pragma pack(pop)

    //Road Sign Map
    struct tRoadSign
    {
        RoadSigns type;
        tUInt32 id;
        tFloat32 x;
        tFloat32 y;
        tInt32 direction;
        tUInt32 initId;
    };

    struct tParkingSpc
    {
        tUInt32 id;
        tFloat32 x;
        tFloat32 y;
        tInt32 direction;
    };

    struct tRoadSignMap
    {
        vector<tRoadSign> signs;
        vector<tRoadSign> initSigns;
        vector<tParkingSpc> parkingSpaces;
    } m_roadSignMap;

    //Open Drive Map

    RoadGraph m_roadGraph;

    //Start Sign Initialize

    tBool m_searchForStartSign = tFalse;
    tTimeStamp m_searchForStartSignInitTime;
    tRoadSign m_startSign;
    tBool m_startSignSet = tFalse;

    //Route Initialize

    tBool m_routeInitialized = tFalse;

    //Maneuver
    struct
    {
        tManeuver maneuver;
        ManeuverStage stage;
    } m_currentManeuver;

    tDriverStruct m_currentState;

    std::mutex m_oStateManeuverMutex;

    tBool m_bJuryStop;

    tBool m_bIdleDriving = tFalse;

    tBool m_bEmergencyCarManeuver = tFalse;
    tBool m_bEmergencyCar = tFalse;
    tInt8 m_i8EmergencyStage = 0;
    tTimeStamp m_bEmergencyCarInitTime;

    tBool m_bOvertakeManeuver = tFalse;
    tBool m_bOvertake = tFalse;
    tInt8 m_i8OvertakeStage = 0;

    tBool m_bZebraCrossingManeuver = tFalse;
    tBool m_bZebraCrossing = tFalse;
    tInt8 m_i8ZebraStage = 0;

    tBool m_bChildManeuver = tFalse;
    tBool m_bChild = tFalse;
    tTimeStamp m_bChildInitTime;
    tTimeStamp m_bChildManeuverInitTime;
    tInt8 m_i8ChildStage = 0;

	tBool m_startupSent;
    tBool m_maneuverListReceived;
    tBool m_openDriveMapReceived;
    tBool m_roadSignMapReceived;

    tInt32 m_lastComID = -1;

    tBool m_crossingLeft = tFalse;
    tBool m_crossingRight = tFalse;
    tFloat32 m_crossingDistance;

    tBool m_vpStopActive = tFalse;

    tBool m_bCrossingBefore = tFalse;

    tBool m_bCurveParamsSet = tFalse;

    struct
    {
        cPinReader emergencyCar;
        cPinReader inertMeas;
        cPinReader roadSIgnExt;
        cPinReader crossingSituation;
        cPinReader objectDetection;
        cPinReader comID;
        cPinReader juryStruct;
        cPinReader maneuverList;
        cPinReader openDriveMap;
        cPinReader roadSignMap;
        cPinReader overtake;
        cPinReader rampDetector;
        cPinReader OCcom;
        cPinReader steering;
	} m_pinReaders;

    struct
    {
        cPinWriter turnLeft;
        cPinWriter turnRight;
        cPinWriter head;
        cPinWriter hazard;
        cPinWriter reverse;
        cPinWriter brake;
        cPinWriter crossingDetectorActivate;
        cPinWriter DriveDistanceCom;
        cPinWriter driverStruct;
        cPinWriter laneChange;
        cPinWriter VPStopBool;
        cPinWriter SetLaneROI;
        cPinWriter ramp;
        cPinWriter laneUseRoiLeft;
        cPinWriter laneUseRoiRight;
        cPinWriter stopLaneChange;
        cPinWriter searchForStartSign;
        cPinWriter OCswitch;
        cPinWriter OCactivateKI;
        cPinWriter OCcomOut;
        cPinWriter speedScaleToggle;
        cPinWriter DDspeedScalar;
	} m_pinWriters;

    struct
    {
        cSampleCodecFactory signalValue;
        cSampleCodecFactory boolSignalValue;
        cSampleCodecFactory inertMeas;
        cSampleCodecFactory roadSignExt;
        cSampleCodecFactory crossingSituation;
        cSampleCodecFactory objectDetectionData;
        cSampleCodecFactory intSignalValue;
        cSampleCodecFactory driveDistanceCom;
        cSampleCodecFactory wheelData;
        cSampleCodecFactory juryStruct;
        cSampleCodecFactory driverStruct;
	} m_sampleCodecFactories;

    struct
    {
        struct
        {
            tSize situation;
            tSize distance;
        } crossingSituation;

        struct
        {
            tSize identifier;
            tSize tVec;
            tSize rVec;
        } roadSignExt;

        struct
        {
            tSize f32Value;
        } signalValue;

        struct
        {
            tSize bValue;
        } boolSignalValue;

        struct
        {
            tSize ui32ArduinoTimestamp;
            tSize f32G_z;
            tSize f32A_x;
        } inertMeas;

        struct
        {
            tSize value;
        } intSignalValue;

        struct
        {
            tSize comID;
            tSize distanceToDrive;
            tSize speedToDrive;
            tSize stopAfter;
            tSize steeringToDrive;
        } driveDistanceCom;

        struct
        {
            tSize actionId;
            tSize maneuverEntry;
        } juryStruct;

        struct
        {
            tSize stateId;
            tSize maneuverEntry;
        } driverStruct;

        struct
        {
            tSize size;
            tSize objectArray;
        } objectDetectionData;

	} m_sampleIndices;

    struct
    {
        adtf::base::ant::runnable<adtf::base::ant::IRunnable::RUN_TRIGGER> juryStruct;
        adtf::base::ant::runnable<> core;
        adtf::base::ant::runnable<adtf::base::ant::IRunnable::RUN_TRIGGER> emergencyCar;
        adtf::base::ant::runnable<adtf::base::ant::IRunnable::RUN_TRIGGER> comID;

        adtf::base::ant::runnable<adtf::base::ant::IRunnable::RUN_TRIGGER> maneuverList;
        adtf::base::ant::runnable<adtf::base::ant::IRunnable::RUN_TRIGGER> openDriveMap;
        adtf::base::ant::runnable<adtf::base::ant::IRunnable::RUN_TRIGGER> roadSignMap;

        adtf::base::ant::runnable<adtf::base::ant::IRunnable::RUN_TRIGGER> crossingSituation;
        adtf::base::ant::runnable<adtf::base::ant::IRunnable::RUN_TRIGGER> roadSignExt;
        adtf::base::ant::runnable<adtf::base::ant::IRunnable::RUN_TRIGGER> objectDetection;

        adtf::base::ant::runnable<adtf::base::ant::IRunnable::RUN_TRIGGER> overtake;
        adtf::base::ant::runnable<adtf::base::ant::IRunnable::RUN_TRIGGER> rampDetector;
	} m_runners;

    struct
    {
        tTimeStamp ts;
        tFloat32 rotation;

        struct
        {
            CrossingSigns sign = CrossingSigns::None;
            CrossingSigns situation = CrossingSigns::None;
            tBool watchForOthers = tFalse;
            tBool giveRightOfWay = tFalse;
            tBool initTimeSet = tFalse;
            tTimeStamp initTime = 0;
            tInt32 step;
            tBool stopSent = tFalse;
            tTimeStamp initTimeWaitMax;
            property_variable<tInt32> waitMax = 6;
            tBool nextManSkipSign = tFalse;
            CrossingSigns nextManSign = CrossingSigns::None;
            tFloat32 nextManDist = 0.f;
            tBool skipSign = tFalse;
            tFloat32 maxDistanceToStopLine = 100.f;
            tFloat32 signDistanceToStop = 0.f;
        } crossing;

        struct
        {
            tBool wait = tFalse;
            tBool initTimeSet = tFalse;
            tTimeStamp initTime = 0;
            tInt32 step;
            tBool foundRamp = tFalse;
        } mergeLeft;

        struct
        {
            tBool skipBackOff = tFalse;
        } overtake;

        struct
        {
            tBool signFound = tFalse;
            tBool initTimeSet = tFalse;
            tTimeStamp initTime;
            tFloat32 distanceToSign = 0.f;
        } parking;

        struct
        {
            tBool initTimeSet = tFalse;
            tTimeStamp initTime;
            tBool stopSent = tFalse;
            tFloat32 distanceToZebra = 0.f;
        } zebra;

        struct
        {
            tBool carLeft = tFalse;
            tTimeStamp carLeftInitTime;

            tBool carRight = tFalse;
            tTimeStamp carRightInitTime;

            tBool carMiddle = tFalse;
            tTimeStamp carMiddleInitTime;

            tBool personRight = tFalse;
            tTimeStamp personRightInitTime;
        } objectDetection;

        struct
        {
            tTimeStamp initTimeWait;
        } emergency;

        //Open Challenge
        struct
        {
            tTimeStamp initTime;
            tBool parking = tFalse;
            tBool zebra = tTrue;
        } openChallenge;

        struct
        {
            tFloat32 straightSteeringOffset = 0.f;
            tFloat32 straightDist = 0.f;
            tFloat32 steerDist = 0.f;
            tFloat32 steeringOffset = 0.f;
        } curve;

    } m_maneuverHelpers;

    struct
    {
        struct
        {
            property_variable<tFloat32> defaultSpeed = 0.5f;
            property_variable<tFloat32> defaultSpeedSlower = 0.3f;
            property_variable<tInt64> stopWaitTime = 2;
            property_variable<tFloat32> yawOffset = 0.3f;
            property_variable<tFloat32> pullOutAngle = 80.f;
            property_variable<tFloat32> steeringOffset = -20.f;
            property_variable<tBool> useInit = tFalse;
            property_variable<tBool> useSignDistance = tFalse;
        } general;

        struct
        {
            property_variable<tInt> TCPPort = 1234;
            property_variable<tInt> TCPSleep = 1000000;
            property_variable<tBool> enableConsoleOutput = tFalse;
        } jury;

        struct
        {
            property_variable<tFloat32> distanceToIgnore = 0.8f;
            property_variable<tFloat32> distanceToIgnorePredOffset = 0.7f;
            property_variable<tFloat32> offsetToStopLine = 0.25f;
            property_variable<tFloat32> distanceSkipR = 0.1f;
            property_variable<tFloat32> distanceSkipL = 0.0f;
            property_variable<tFloat32> shiftStopLine = 0.1f;
        } signs;

        struct
        {
            property_variable<tFloat32> straightDist = .35f;
            property_variable<tFloat32> steering = -100.f;
            property_variable<tFloat32> speed = 12.f;
        } pullOutLeft;

        struct
        {
            property_variable<tFloat32> straightDist = .25f;
            property_variable<tFloat32> steering = 100.f;
            property_variable<tFloat32> speed = 12.f;
        } pullOutRight;

        struct
        {
            property_variable<tFloat32> dist1 = 0.5f;
            property_variable<tFloat32> dist2 = 0.25f;
            property_variable<tFloat32> forwardStraightDist = .3f;
            property_variable<tFloat32> reverseStraightDist = .5f;
            property_variable<tFloat32> steering = 99.f;
            property_variable<tFloat32> forwardAngle = 45.f;
            property_variable<tFloat32> reverseAngle = 45.f;
        } parking;

        struct
        {
            property_variable<tFloat32> straightDistance = 0.2f;
            property_variable<tFloat32> distance = 1.2f;            //magic number, to determine!
            property_variable<tFloat32> steering = 80.0f;           //magic number, to determine!
            property_variable<tFloat32> distanceToCrossing = 0.5f;  //magic number, to determine!
            property_variable<tFloat32> distanceToStopLine = 0.35f; //magic number, to determine!
            property_variable<tFloat32> signDistanceStraight = 0.1f;
            property_variable<tFloat32> setROI = 50.f;
        } turnRight;

        struct
        {

            property_variable<tFloat32> distance = 1.7f;            //magic number, to determine!
            property_variable<tFloat32> steering = -65.0f;          //magic number, to determine!
            property_variable<tFloat32> distanceToCrossing = 0.25f; //magic number, to determine!
            property_variable<tFloat32> distanceToStopLine = 0.1f;  //magic number, to determine!
            property_variable<tFloat32> signDistanceStraight = 0.2f;
            property_variable<tFloat32> setROI = 100.f;
        } turnLeft;

        struct
        {
            property_variable<tFloat32> distance = 1.0f;            //magic number, to determine!
            property_variable<tBool> useLanekeeping = tFalse;
            property_variable<tFloat32> signDistanceStraight = 0.6f;
            property_variable<tFloat32> setROI = 50.f;
            property_variable<tFloat32> nonCurveDist = 0.f;
        } straight;

        struct
        {
            property_variable<tFloat32> dist1 = .35f;               //magic number, to determine!
            property_variable<tFloat32> dist2 = .45f;               //magic number, to determine!
            property_variable<tFloat32> dist3 = .4f;                //magic number, to determine!
            property_variable<tFloat32> dist4 = .4f;                //magic number, to determine!
            property_variable<tInt32> resetTime = 2;
            property_variable<tInt32> waitTime = 3;
        } emergency;

        struct
        {
            property_variable<tFloat32> dist1 = 6.f;      //magic number, to determine!
            property_variable<tFloat32> distanceToMerge = 7.0f;     //magic number, to determine!
            property_variable<tFloat32> distanceLookLeft = 1.5f;    //magic number, to determine!
            property_variable<tInt64> waitTime = 5;                 //magic number, to determine!
            property_variable<tBool> useLaneChange = tFalse;
            property_variable<tFloat32> distResetRoi = 0.5;
            property_variable<tFloat32> distRamp = 2.5;
        } mergeLeft;

        struct
        {
            property_variable<tFloat32> distBackOff = 1.0f;         //magic number, to determine!
            property_variable<tFloat32> dist1 = 1.5f;               //magic number, to determine!
            property_variable<tFloat32> dist2 = 0.0f;               //magic number, to determine!
            property_variable<tFloat32> dist3 = 1.5f;               //magic number, to determine!
        } overtake;

        struct
        {
            property_variable<tFloat32> dist1 = 0.25f;              //magic number, to determine!
            property_variable<tFloat32> dist2 = 0.25f;              //magic number, to determine!
            property_variable<tFloat32> waitTime = 1.f;
            property_variable<tFloat32> distAfterR = 0.4f;
        } zebra;

        struct
        {
            property_variable<tFloat32> dist1 = 1.f;
            property_variable<tFloat32> resetTime = 1.f;              //magic number, to determine!
        } child;

        struct
        {
            property_variable<tFloat32> resetTime = 1.f;
            property_variable<tFloat32> resetTimePerson = 3.f;
        } objectDetection;

        //openChallenge
        struct
        {
            property_variable<tFloat32> distanceOvertake = 1.f;
            property_variable<tFloat32> distanceOvertakeEnd = 0.2f;
            property_variable<tFloat32> defaultSpeed = 0.4f;
            property_variable<tFloat32> defaultSpeedSlower = 0.2f;
            property_variable<tFloat32> parkingStraightDist = 0.f;
        } openChallenge;

        struct
        {
            property_variable<tFloat32> threshold = 25.f;

            property_variable<tFloat32> straightSteeringOffsetL = -25.f;
            property_variable<tFloat32> straightSteeringOffsetR = 25.f;
            property_variable<tFloat32> straightDistL = 0.f;
            property_variable<tFloat32> straightDistR = 0.f;

            property_variable<tFloat32> rightStraightDistL = 0.f;
            property_variable<tFloat32> rightSteerDistL = 0.f;
            property_variable<tFloat32> rightSteeringOffsetL = 0.f;
            property_variable<tFloat32> rightStraightDistR = 0.f;
            property_variable<tFloat32> rightSteerDistR = 0.f;
            property_variable<tFloat32> rightSteeringOffsetR = 0.f;

            property_variable<tFloat32> leftStraightDistL = 0.f;
            property_variable<tFloat32> leftSteerDistL = 0.f;
            property_variable<tFloat32> leftSteeringOffsetL = 0.f;
            property_variable<tFloat32> leftStraightDistR = 0.f;
            property_variable<tFloat32> leftSteerDistR = 0.f;
            property_variable<tFloat32> leftSteeringOffsetR = 0.f;
        } curve;
    } m_properties;

    std::mutex m_oManeuverListMutex;

	/*! this is the list with all the loaded sections from the maneuver list*/
	aadc::jury::maneuverList m_sectorList;
    object_ptr<adtf::services::IReferenceClock> m_pClock;
    /*! The maneuver file string */
    cString m_strManeuverFileString;
    cString m_roadSignString;
    cString m_openDriveMapString;

    tBool initManeuver(tInt id);
    tVoid nextManeuver();

    tVoid performManeuver();
    tVoid performManeuverEmergencyCar();
    tVoid performManeuverOvertake();
    tVoid performManeuverZebra();
    tVoid performManeuverChild();
    tVoid performManeuverIdle();
    tVoid performManeuverLeft();
    tVoid performManeuverRight();
    tVoid performManeuverStraight();

    tVoid performManeuverLeftSign();
    tVoid performManeuverRightSign();
    tVoid performManeuverStraightSign();

    tVoid performManeuverCrossParking();
    tVoid performManeuverPullOutLeft();
    tVoid performManeuverPullOutRight();
    tVoid performManeuverPullOut(Lights indicator, tFloat32 straightDistance, tFloat32 steering);
    tVoid performManeuverMergeLeft();

    tResult SetLaneROI(tFloat32 offset);
    tResult SwitchCrossingDetector(tInt32 activate);

    tInt32 getSteering();

    tVoid updateState(stateCar stateID, tInt16 i16ManeuverEntry);
    tVoid updateState(stateCar stateID);
    tVoid setState(stateCar stateID, tInt16 i16ManeuverEntry);
    tVoid setState(stateCar stateID);
    tResult transmitState();
    tVoid logState();
	tResult LoadManeuverList();
    tResult LoadRoadSignMap();

    tInt ParkingSlotMap(int i); //must be updated for each new track!!!

//    tBool WatchFrontForCars();
//    tBool WatchLeftForCars();
//    tBool WatchRightForCars();

//    tBool WatchLeftForPerson();
//    tBool WatchRightForPerson();
//    tBool WatchFrontForPerson();

    tVoid ResetManeuverBool();

    tResult laneUseLeftRoi(tBool bValue);
    tResult laneUseRightRoi(tBool bValue);
    tResult stopLaneChange();

    tResult searchForStartSign(tBool bValue);

    //Open Challenge Functions and Variables
    tVoid performOCManeuverSlave();
    tVoid performOCManeuverMaster();
    tBool m_bCarFound = tFalse;
    tBool m_bOpenChallenge = tFalse;

public:

    /*! Default constructor. */
    cCoreFilter();

    /*! Destructor. */
    virtual ~cCoreFilter() = default;

protected:
    tResult  Init(tInitStage eStage) override;
    tResult  Shutdown(cFilterLevelmachine::tInitStage eStage) override;

    tResult CoreLoop(tTimeStamp tmTimeOfTrigger);
    tResult ProcessJuryStruct(tTimeStamp tmTimeOfTrigger);
    tResult ProcessManeuverList(tTimeStamp tmTimeOfTrigger);
    tResult ProcessOpenDriveMap(tTimeStamp tmTimeOfTrigger);
    tResult ProcessRoadSignMap(tTimeStamp tmTimeOfTrigger);
    tResult ProcessEmergencyCarPin(tTimeStamp tmTimeOfTrigger);
    tResult ProcessComID(tTimeStamp tmTimeOfTrigger);
    tResult ProcessRoadSignExt(tTimeStamp tmTimeOfTrigger);
    tResult ProcessObjectDetection(tTimeStamp tmTimeOfTrigger);
    tResult ProcessCrossingSituation(tTimeStamp tmTimeOfTrigger);
    tResult ProcessOvertake(tTimeStamp tmTimeOfTrigger);
    tResult ProcessRamp(tTimeStamp tmTimeOfTrigger);

    tResult TransmitRamp(tBool bValue);
    tResult TransmitDDCom(tDriveDistanceCom com);
    tResult TransmitLight(Lights light, tBool bValue);
    tResult TransmitLaneChange(tBool bValue);
    tResult TransmitVPStop(tBool bValue);
    tResult TransmitDDSpeedScalar(tFloat32 value);

    tResult TransmitOCSwitch(tBool bValue);
    tResult TransmitOCActivateKI(tBool bValue);
    tResult TransmitOCCom(tBool bValue);

    tResult TransmitSpeedScaleSwitch(tBool bValue);
};


//*************************************************************************************************
