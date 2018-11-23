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

//otherwise cDOM will cause a deprecated warning, however there is no alternative yet
#define A_UTILS_NO_DEPRECATED_WARNING

#include "stdafx.h"
#include "CoreFilter.h"
#include "aadc_structs.h"

#include <chrono>

#define CONSOLE_LOG(_text, _log_level) if (m_properties.jury.enableConsoleOutput) { LOG_ADD_ENTRY(_log_level, _text); }    //!< enables log function if console output is activated
#define CONSOLE_LOG_INFO(_text)      CONSOLE_LOG(_text, A_UTILS_NS::log::tLogLevel::Info)                        //!< log info messages

ADTF_PLUGIN(LABEL_CORE_FILTER, cCoreFilter)

cCoreFilter::cCoreFilter() :
    m_currentManeuver({ tManeuver(), ManeuverStage::Init }),
    m_currentState({ statecar_startup, 0 }),
    m_bJuryStop(tFalse),
    m_startupSent(tFalse),
    m_maneuverListReceived(tFalse),
    m_openDriveMapReceived(tFalse),
    m_roadSignMapReceived(tFalse),
    m_crossingDistance(0.0f)
{

    RegisterPropertyVariable("General - Default Speed Slower", m_properties.general.defaultSpeedSlower);
    RegisterPropertyVariable("General - Default Speed", m_properties.general.defaultSpeed);
    RegisterPropertyVariable("General - Time to wait at Stopsign [s]", m_properties.general.stopWaitTime);
    RegisterPropertyVariable("General - Yaw Offset", m_properties.general.yawOffset);
    RegisterPropertyVariable("General - Steering Offset", m_properties.general.steeringOffset);
    RegisterPropertyVariable("General - use Search fo Init Sign", m_properties.general.useInit);
    RegisterPropertyVariable("Pull Out (both) - pullOutAngle", m_properties.general.pullOutAngle);
    RegisterPropertyVariable("Distance to crossing determined by RoadSigns", m_properties.general.useSignDistance);

    RegisterPropertyVariable("Port number", m_properties.jury.TCPPort);
    RegisterPropertyVariable("Enable console log", m_properties.jury.enableConsoleOutput);
    RegisterPropertyVariable("TCP Sleep Time [microseconds]", m_properties.jury.TCPSleep);

    RegisterPropertyVariable("PullOutLeft - Speed", m_properties.pullOutLeft.speed);
    RegisterPropertyVariable("PullOutLeft - Steering", m_properties.pullOutLeft.steering);
    RegisterPropertyVariable("PullOutLeft - Straight Dist", m_properties.pullOutLeft.straightDist);

    RegisterPropertyVariable("Pull Out Right - Speed", m_properties.pullOutRight.speed);
    RegisterPropertyVariable("Pull Out Right - Steering", m_properties.pullOutRight.steering);
    RegisterPropertyVariable("Pull Out Right - Straight Dist", m_properties.pullOutRight.straightDist);

    RegisterPropertyVariable("Parking - Distance to Parking Space", m_properties.parking.dist1);
    RegisterPropertyVariable("Parking - Width of 1 Parking Slot", m_properties.parking.dist2);
    RegisterPropertyVariable("Parking - Steering", m_properties.parking.steering);
    RegisterPropertyVariable("Parking - Forward Straight Dist", m_properties.parking.forwardStraightDist);
    RegisterPropertyVariable("Parking - Reverse Straight Dist", m_properties.parking.reverseStraightDist);
    RegisterPropertyVariable("Parking - Forward Angle", m_properties.parking.forwardAngle);
    RegisterPropertyVariable("Parking - Reverse Angle", m_properties.parking.reverseAngle);

    RegisterPropertyVariable("Right Turn - Straight Distance", m_properties.turnRight.straightDistance);
    RegisterPropertyVariable("Right Turn - Distance", m_properties.turnRight.distance);
    RegisterPropertyVariable("Right Turn - Steering", m_properties.turnRight.steering);
    RegisterPropertyVariable("Right Turn - Distance to Crossing", m_properties.turnRight.distanceToCrossing);
    RegisterPropertyVariable("Right Turn - Distance to Stop Line", m_properties.turnRight.distanceToStopLine);
    RegisterPropertyVariable("Right Turn - Sign: Straight Distance", m_properties.turnRight.signDistanceStraight);
    RegisterPropertyVariable("Right Turn - Set Lane ROI", m_properties.turnRight.setROI);
    RegisterPropertyVariable("Right Turn - R Curve - Straight Distance Offset", m_properties.curve.rightStraightDistR);
    RegisterPropertyVariable("Right Turn - R Curve - Steer Distance Offset", m_properties.curve.rightSteerDistR);
    RegisterPropertyVariable("Right Turn - R Curve - Steering Offset", m_properties.curve.rightSteeringOffsetR);
    RegisterPropertyVariable("Right Turn - L Curve - Straight Distance Offset", m_properties.curve.rightStraightDistL);
    RegisterPropertyVariable("Right Turn - L Curve - Steer Distance Offset", m_properties.curve.rightSteerDistL);
    RegisterPropertyVariable("Right Turn - L Curve - Steering Offset", m_properties.curve.rightSteeringOffsetL);


    RegisterPropertyVariable("Left Turn - Distance", m_properties.turnLeft.distance);
    RegisterPropertyVariable("Left Turn - Steering", m_properties.turnLeft.steering);
    RegisterPropertyVariable("Left Turn - Distance to Crossing", m_properties.turnLeft.distanceToCrossing);
    RegisterPropertyVariable("Left Turn - Distance to Stop Line", m_properties.turnLeft.distanceToStopLine);
    RegisterPropertyVariable("Left Turn - Sign: Straight Distance", m_properties.turnLeft.signDistanceStraight);
    RegisterPropertyVariable("Left Turn - Set Lane ROI", m_properties.turnLeft.setROI);
    RegisterPropertyVariable("Left Turn - R Curve - Straight Distance Offset", m_properties.curve.leftStraightDistR);
    RegisterPropertyVariable("Left Turn - R Curve - Steer Distance Offset", m_properties.curve.leftSteerDistR);
    RegisterPropertyVariable("Left Turn - R Curve - Steering Offset", m_properties.curve.leftSteeringOffsetR);
    RegisterPropertyVariable("Left Turn - L Curve - Straight Distance Offset", m_properties.curve.leftStraightDistL);
    RegisterPropertyVariable("Left Turn - L Curve - Steer Distance Offset", m_properties.curve.leftSteerDistL);
    RegisterPropertyVariable("Left Turn - L Curve - Steering Offset", m_properties.curve.leftSteeringOffsetL);

    RegisterPropertyVariable("Straight - Distance", m_properties.straight.distance);
    RegisterPropertyVariable("Straight - Use Lanekeeping", m_properties.straight.useLanekeeping);
    RegisterPropertyVariable("Straight - Sign: Straight Distance", m_properties.straight.signDistanceStraight);
    RegisterPropertyVariable("Straight - Set Lane ROI", m_properties.straight.setROI);
    RegisterPropertyVariable("Straight - L Curve - Steering Offset", m_properties.curve.straightSteeringOffsetL);
    RegisterPropertyVariable("Straight - R Curve - Steering Offset", m_properties.curve.straightSteeringOffsetR);
    RegisterPropertyVariable("Straight - R Curve - Distance Offset", m_properties.curve.straightDistR);
    RegisterPropertyVariable("Straight - L Curve - Distance Offset", m_properties.curve.straightDistL);
    RegisterPropertyVariable("Straight - No Curver - Distance Offset", m_properties.straight.nonCurveDist);

    RegisterPropertyVariable("Emergency - Dist Right Steering Out", m_properties.emergency.dist1);
    RegisterPropertyVariable("Emergency - Dist Left Steering Out", m_properties.emergency.dist2);
    RegisterPropertyVariable("Emergency - Dist Right Steering In", m_properties.emergency.dist3);
    RegisterPropertyVariable("Emergency - Dist Left Steering In", m_properties.emergency.dist4);
    RegisterPropertyVariable("Emergency - Reset Time [s]", m_properties.emergency.resetTime);
    RegisterPropertyVariable("Emergency - Wait Time [s]", m_properties.emergency.waitTime);

    RegisterPropertyVariable("Merge Left - Distance from Reset ROI to Look Left", m_properties.mergeLeft.dist1);
    RegisterPropertyVariable("Merge Left - Distance to Merge", m_properties.mergeLeft.distanceLookLeft);
    RegisterPropertyVariable("Merge Left - Wait Time", m_properties.mergeLeft.waitTime);
    RegisterPropertyVariable("Merge Left - use LaneChange", m_properties.mergeLeft.useLaneChange);
    RegisterPropertyVariable("Merge Left - Distance Ramp up", m_properties.mergeLeft.distRamp);
    RegisterPropertyVariable("Merge Left - Distance Reset ROI", m_properties.mergeLeft.distResetRoi);

    RegisterPropertyVariable("Signs - Max distance for signs to be relevant", m_properties.signs.distanceToIgnore);
    RegisterPropertyVariable("Signs - Offset added to max distance for predicting Signs", m_properties.signs.distanceToIgnorePredOffset);
    RegisterPropertyVariable("Signs - Offset added to Distance to Stop Line", m_properties.signs.offsetToStopLine);
    RegisterPropertyVariable("Signs - Distance straight in skip Sign after R", m_properties.signs.distanceSkipR);
    RegisterPropertyVariable("Signs - Distance straight in skip Sign after L", m_properties.signs.distanceSkipL);
    RegisterPropertyVariable("Signs - Shift Stop Line backwards", m_properties.signs.shiftStopLine);

    RegisterPropertyVariable("Overtake - Distance Back Off", m_properties.overtake.distBackOff);
    RegisterPropertyVariable("Overtake - Distance 1 - change into left Lane", m_properties.overtake.dist1);
    RegisterPropertyVariable("Overtake - Distance 2 - straight in left Lane", m_properties.overtake.dist2);
    RegisterPropertyVariable("Overtake - Distance 3 - change into right Lane", m_properties.overtake.dist3);

    RegisterPropertyVariable("Zebra Crossing - Distance to Zebra Crossing for Predicting after right Turn", m_properties.zebra.dist1);
    RegisterPropertyVariable("Zebra Crossing - Distance to drive over Zebra Crossing", m_properties.zebra.dist2);
    RegisterPropertyVariable("Zebra Crossing - Wait time at zebra to detect Barbie", m_properties.zebra.waitTime);
    RegisterPropertyVariable("Zebra Crossing - Distance to zebra after Right turn", m_properties.zebra.distAfterR);

    RegisterPropertyVariable("Child - Distance to drive slower", m_properties.child.dist1);

    RegisterPropertyVariable("Object Detection - Reset Time [s]", m_properties.objectDetection.resetTime);
    RegisterPropertyVariable("Object Detection - Reset Time Barbie [s]", m_properties.objectDetection.resetTimePerson);

    RegisterPropertyVariable("Crossing Wait Max", m_maneuverHelpers.crossing.waitMax);

    RegisterPropertyVariable("Curve - Threshold", m_properties.curve.threshold);

    //Open Challenge Properties
    RegisterPropertyVariable("OC - default speed", m_properties.openChallenge.defaultSpeed);
    RegisterPropertyVariable("OC - default speed slower", m_properties.openChallenge.defaultSpeedSlower);
    RegisterPropertyVariable("OC - distance Overtake", m_properties.openChallenge.distanceOvertake);
    RegisterPropertyVariable("OC - distance Overtake End", m_properties.openChallenge.distanceOvertakeEnd);

    object_ptr<IStreamType> pTypeRoadSignExt;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tRoadSignExt", pTypeRoadSignExt, m_sampleCodecFactories.roadSignExt))
    {
        adtf_ddl::access_element::find_array_index(m_sampleCodecFactories.roadSignExt, cString("af32TVec"), m_sampleIndices.roadSignExt.tVec);
        adtf_ddl::access_element::find_array_index(m_sampleCodecFactories.roadSignExt, cString("af32RVec"), m_sampleIndices.roadSignExt.rVec);
        adtf_ddl::access_element::find_index(m_sampleCodecFactories.roadSignExt, cString("i16Identifier"), m_sampleIndices.roadSignExt.identifier);
    }
    else
    {
        LOG_WARNING("No mediadescription for tRoadSignExt found!");
    }

    object_ptr<IStreamType> pTypeCrossingSituation;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tCrossingSituation", pTypeCrossingSituation, m_sampleCodecFactories.crossingSituation))
    {
        adtf_ddl::access_element::find_index(m_sampleCodecFactories.crossingSituation, cString("situation"), m_sampleIndices.crossingSituation.situation);
        adtf_ddl::access_element::find_index(m_sampleCodecFactories.crossingSituation, cString("distanceToCrossing"), m_sampleIndices.crossingSituation.distance);
    }
    else
    {
        LOG_WARNING("No mediadescription for tCrossingSituation found!");
    }

    object_ptr<IStreamType> pTypeSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalValue, m_sampleCodecFactories.signalValue))
    {
        adtf_ddl::access_element::find_index(m_sampleCodecFactories.signalValue, cString("f32Value"), m_sampleIndices.signalValue.f32Value);
    }
    else
    {
        LOG_WARNING("No mediadescription for tSignalValue found!");
    }

    filter_create_pin(*this, m_pinWriters.SetLaneROI, "LaneROI", pTypeSignalValue);
    filter_create_pin(*this, m_pinReaders.steering, "steering", pTypeSignalValue);
    filter_create_pin(*this, m_pinWriters.DDspeedScalar, "DDspeedScalar", pTypeSignalValue);

    filter_create_pin(*this, m_pinReaders.crossingSituation, "crossingSituation", pTypeCrossingSituation);
    filter_create_pin(*this, m_pinReaders.roadSIgnExt, "roadSignExt", pTypeRoadSignExt);

    object_ptr<IStreamType> pTypeBoolSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tBoolSignalValue", pTypeBoolSignalValue, m_sampleCodecFactories.boolSignalValue))
    {
        adtf_ddl::access_element::find_index(m_sampleCodecFactories.boolSignalValue, cString("bValue"), m_sampleIndices.boolSignalValue.bValue);
    }
    else
    {
        LOG_WARNING("No mediadescription for tBoolSignalValue found!");
    }
    filter_create_pin(*this, m_pinReaders.emergencyCar, "emergency_car_in", pTypeBoolSignalValue);
    filter_create_pin(*this, m_pinReaders.overtake, "overtake", pTypeBoolSignalValue);
    filter_create_pin(*this, m_pinReaders.rampDetector, "ramp_detector", pTypeBoolSignalValue);
    filter_create_pin(*this, m_pinReaders.OCcom, "OC_com", pTypeBoolSignalValue);

    filter_create_pin(*this, m_pinWriters.turnLeft, "turn_signal_left", pTypeBoolSignalValue);
    filter_create_pin(*this, m_pinWriters.turnRight, "turn_signal_right", pTypeBoolSignalValue);
    filter_create_pin(*this, m_pinWriters.hazard, "hazard_light", pTypeBoolSignalValue);
    filter_create_pin(*this, m_pinWriters.head, "head_light", pTypeBoolSignalValue);
    filter_create_pin(*this, m_pinWriters.reverse, "reverse_light", pTypeBoolSignalValue);
    filter_create_pin(*this, m_pinWriters.brake, "brake_light_to_DD", pTypeBoolSignalValue);
    filter_create_pin(*this, m_pinWriters.laneChange, "laneChange", pTypeBoolSignalValue);
    filter_create_pin(*this, m_pinWriters.VPStopBool, "VPStopBool", pTypeBoolSignalValue);
    filter_create_pin(*this, m_pinWriters.ramp, "ramp_switch", pTypeBoolSignalValue);
    filter_create_pin(*this, m_pinWriters.laneUseRoiLeft, "laneRoiLeft", pTypeBoolSignalValue);
    filter_create_pin(*this, m_pinWriters.laneUseRoiRight, "laneRoiRight", pTypeBoolSignalValue);
    filter_create_pin(*this, m_pinWriters.stopLaneChange, "stopLaneChange", pTypeBoolSignalValue);
    filter_create_pin(*this, m_pinWriters.searchForStartSign, "search_for_start_sign", pTypeBoolSignalValue);
    filter_create_pin(*this, m_pinWriters.OCswitch, "OC_switch", pTypeBoolSignalValue);
    filter_create_pin(*this, m_pinWriters.OCactivateKI, "OC_activateKI", pTypeBoolSignalValue);
    filter_create_pin(*this, m_pinWriters.OCcomOut, "OC_comOut", pTypeBoolSignalValue);
    filter_create_pin(*this, m_pinWriters.speedScaleToggle, "speed_scale_toggle", pTypeBoolSignalValue);

    m_runners.rampDetector = runnable<IRunnable::RUN_TRIGGER>(ADTF_RUN_FUNCTION(ProcessRamp));
    RegisterRunner("ramp_detector_runner", m_runners.rampDetector);
    ConfigureDataInTrigger("ramp_detector_runner", "ramp_detector");

    object_ptr<IStreamType> pTypeInertMeas;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tInerMeasUnitData", pTypeInertMeas, m_sampleCodecFactories.inertMeas))
    {
        adtf_ddl::access_element::find_index(m_sampleCodecFactories.inertMeas, cString("ui32ArduinoTimestamp"), m_sampleIndices.inertMeas.ui32ArduinoTimestamp);
        adtf_ddl::access_element::find_index(m_sampleCodecFactories.inertMeas, cString("f32G_z"), m_sampleIndices.inertMeas.f32G_z);
        adtf_ddl::access_element::find_index(m_sampleCodecFactories.inertMeas, cString("f32A_x"), m_sampleIndices.inertMeas.f32A_x);
    }
    else
    {
        LOG_WARNING("No mediadescription for tInerMeasUnitData found!");
    }
    filter_create_pin(*this, m_pinReaders.inertMeas, "inert_meas_in", pTypeInertMeas);

    object_ptr<IStreamType> pTypeJuryStruct;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tJuryStruct", pTypeJuryStruct, m_sampleCodecFactories.juryStruct))
    {
        (adtf_ddl::access_element::find_index(m_sampleCodecFactories.juryStruct, cString("i16ActionID"), m_sampleIndices.juryStruct.actionId));
        (adtf_ddl::access_element::find_index(m_sampleCodecFactories.juryStruct, cString("i16ManeuverEntry"), m_sampleIndices.juryStruct.maneuverEntry));
    }
    else
    {
        LOG_WARNING("No mediadescription for tJuryStruct found!");
    }
    create_pin(*this, m_pinReaders.juryStruct, "jury_struct", pTypeJuryStruct);

    object_ptr<IStreamType> pTypeDriverStruct;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tDriverStruct", pTypeDriverStruct, m_sampleCodecFactories.driverStruct))
    {
        (adtf_ddl::access_element::find_index(m_sampleCodecFactories.driverStruct, cString("i16StateID"), m_sampleIndices.driverStruct.stateId));
        (adtf_ddl::access_element::find_index(m_sampleCodecFactories.driverStruct, cString("i16ManeuverEntry"), m_sampleIndices.driverStruct.maneuverEntry));
    }
    else
    {
        LOG_WARNING("No mediadescription for tDriverStruct found!");
    }
    filter_create_pin(*this, m_pinWriters.driverStruct, "driver_struct", pTypeDriverStruct);

    object_ptr<IStreamType> pTypeDefault = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_anonymous());
    filter_create_pin(*this, m_pinReaders.maneuverList, "maneuver_list", pTypeDefault);
    filter_create_pin(*this, m_pinReaders.openDriveMap, "open_drive_map", pTypeDefault);
    filter_create_pin(*this, m_pinReaders.roadSignMap, "road_sign_map", pTypeDefault);

    m_runners.juryStruct = runnable<IRunnable::RUN_TRIGGER>(ADTF_RUN_FUNCTION(ProcessJuryStruct));
    RegisterRunner("jury_struct_runner", m_runners.juryStruct);
    ConfigureDataInTrigger("jury_struct_runner", "jury_struct");

    object_ptr<IStreamType> pTypeObjectDetectionData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tObjectDetectionData", pTypeObjectDetectionData, m_sampleCodecFactories.objectDetectionData))
    {
        adtf_ddl::access_element::find_index(m_sampleCodecFactories.objectDetectionData, cString("ui32Size"), m_sampleIndices.objectDetectionData.size);
        adtf_ddl::access_element::find_array_index(m_sampleCodecFactories.objectDetectionData, cString("tObjectArray"), m_sampleIndices.objectDetectionData.objectArray);
    }
    else
    {
        LOG_WARNING("No mediadescription for tObjectDectection found!");
    }

    filter_create_pin(*this, m_pinReaders.objectDetection, "objectDetection", pTypeObjectDetectionData);

    m_runners.core = runnable<>(ADTF_RUN_FUNCTION(CoreLoop));
    RegisterRunner("core_loop_runner", m_runners.core);
    ConfigureThreadTrigger("core_loop_runner", tTrue);

    m_runners.emergencyCar = runnable<IRunnable::RUN_TRIGGER>(ADTF_RUN_FUNCTION(ProcessEmergencyCarPin));
    RegisterRunner("emergency_car_runner", m_runners.emergencyCar);
    ConfigureDataInTrigger("emergency_car_runner", "emergency_car_in");

    m_runners.roadSignExt = runnable<IRunnable::RUN_TRIGGER>(ADTF_RUN_FUNCTION(ProcessRoadSignExt));
    RegisterRunner("road_sign_runner", m_runners.roadSignExt);
    ConfigureDataInTrigger("road_sign_runner", "roadSignExt");

    m_runners.maneuverList = runnable<IRunnable::RUN_TRIGGER>(ADTF_RUN_FUNCTION(ProcessManeuverList));
    RegisterRunner("maneuver_list_runner", m_runners.maneuverList);
    ConfigureDataInTrigger("maneuver_list_runner", "maneuver_list");

    m_runners.openDriveMap = runnable<IRunnable::RUN_TRIGGER>(ADTF_RUN_FUNCTION(ProcessOpenDriveMap));
    RegisterRunner("open_drive_map_runner", m_runners.openDriveMap);
    ConfigureDataInTrigger("open_drive_map_runner", "open_drive_map");

    m_runners.roadSignMap = runnable<IRunnable::RUN_TRIGGER>(ADTF_RUN_FUNCTION(ProcessRoadSignMap));
    RegisterRunner("road_sign_map_runner", m_runners.roadSignMap);
    ConfigureDataInTrigger("road_sign_map_runner", "road_sign_map");

    m_runners.objectDetection = runnable<IRunnable::RUN_TRIGGER>(ADTF_RUN_FUNCTION(ProcessObjectDetection));
    RegisterRunner("object_detection_runner", m_runners.objectDetection);
    ConfigureDataInTrigger("object_detection_runner", "objectDetection");

    m_runners.crossingSituation = runnable<IRunnable::RUN_TRIGGER>(ADTF_RUN_FUNCTION(ProcessCrossingSituation));
    RegisterRunner("crossing_situation_runner", m_runners.crossingSituation);
    ConfigureDataInTrigger("crossing_situation_runner", "crossingSituation");

    m_runners.overtake = runnable<IRunnable::RUN_TRIGGER>(ADTF_RUN_FUNCTION(ProcessOvertake));
    RegisterRunner("overtake_runner", m_runners.overtake);
    ConfigureDataInTrigger("overtake_runner", "overtake");

    object_ptr<IStreamType> pTypeIntSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tIntSignalValue", pTypeIntSignalValue, m_sampleCodecFactories.intSignalValue))
    {
        adtf_ddl::access_element::find_index(m_sampleCodecFactories.intSignalValue, cString("value"), m_sampleIndices.intSignalValue.value);
    }
    else
    {
        LOG_WARNING("No mediadescription for tIntSignalValue found!");
    }

    filter_create_pin(*this, m_pinReaders.comID, "comID", pTypeIntSignalValue);
    m_runners.comID = runnable<IRunnable::RUN_TRIGGER>(ADTF_RUN_FUNCTION(ProcessComID));
    RegisterRunner("comID_runner", m_runners.comID);
    ConfigureDataInTrigger("comID_runner", "comID");

    filter_create_pin(*this, m_pinWriters.crossingDetectorActivate, "SwitchCrossingDetector", pTypeIntSignalValue);


    object_ptr<IStreamType> pTypeDriveDistanceCom;
    if( IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tDriveDistanceCom", pTypeDriveDistanceCom, m_sampleCodecFactories.driveDistanceCom)))
    {
        adtf_ddl::access_element::find_index(m_sampleCodecFactories.driveDistanceCom,   cString("comID"), m_sampleIndices.driveDistanceCom.comID);

        adtf_ddl::access_element::find_index(m_sampleCodecFactories.driveDistanceCom,   cString("distanceToDrive"), m_sampleIndices.driveDistanceCom.distanceToDrive);

        adtf_ddl::access_element::find_index(m_sampleCodecFactories.driveDistanceCom,   cString("speedToDrive"), m_sampleIndices.driveDistanceCom.speedToDrive);

        adtf_ddl::access_element::find_index(m_sampleCodecFactories.driveDistanceCom,   cString("stopAfter"), m_sampleIndices.driveDistanceCom.stopAfter);

        adtf_ddl::access_element::find_index(m_sampleCodecFactories.driveDistanceCom,   cString("steeringToDrive"), m_sampleIndices.driveDistanceCom.steeringToDrive);
    }
    else
    {
        LOG_WARNING("No mediadescription for DriveDistanceCom found!");
    }

    filter_create_pin(*this, m_pinWriters.DriveDistanceCom, "DriveDistanceCom", pTypeDriveDistanceCom);

}

tResult cCoreFilter::Init(tInitStage eStage)
{
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    RETURN_NOERROR;
}

tResult cCoreFilter::Shutdown(cFilterLevelmachine::tInitStage eStage)
{
    RETURN_NOERROR;
}

tResult cCoreFilter::CoreLoop(tTimeStamp tmTimeOfTrigger)
{
    std::lock_guard<std::mutex> lock(m_oStateManeuverMutex);
    //LOG_INFO(cString("CoreFilter::CoreLoop"));
    //Jury Startup
    if (!m_startupSent)
    {
        if (m_maneuverListReceived
                && m_openDriveMapReceived
                && m_roadSignMapReceived
                && IS_OK(transmitState()))
        {
            m_startupSent = tTrue;
        }
        RETURN_NOERROR;
    }

//    //Start Sign Search
//    if(m_searchForStartSign && m_properties.general.useInit)
//    {
//        if(tmTimeOfTrigger - m_searchForStartSignInitTime > 1000000)
//        {
//            m_searchForStartSign = tFalse;
//            searchForStartSign(tFalse);
//            LOG_INFO(cString("Start Sign NOT found!"));
//        }
//        RETURN_NOERROR;
//    }

//    //Initialize Route
//    if(!m_routeInitialized && m_startSignSet)
//    {
//        //TO DO!
//        m_routeInitialized = tTrue;
//    }

    //Jury Stop
    if (m_bJuryStop)
    {
        RETURN_NOERROR;
    }

    if(m_bEmergencyCar && tmTimeOfTrigger-m_bEmergencyCarInitTime > 1000000 * m_properties.emergency.resetTime)
    {
        m_bEmergencyCar = tFalse;
    }
    if(m_bChild && tmTimeOfTrigger-m_bChildInitTime > 1000000 * m_properties.child.resetTime)
    {
        m_bChild = tFalse;
    }
    if(m_maneuverHelpers.objectDetection.personRight && tmTimeOfTrigger-m_maneuverHelpers.objectDetection.personRightInitTime > 1000000 * m_properties.objectDetection.resetTimePerson)
    {
        m_maneuverHelpers.objectDetection.personRight = tFalse;
    }

    //Special Maneuver occuring while normal driving
    if(m_bIdleDriving)
    {
        //Overtake
        if(m_bOvertakeManeuver || m_bOvertake)
        {
            if(!m_bOvertakeManeuver)
            {
                LOG_INFO(cString("Overtake Maneuver - initialized!"));
                m_bOvertakeManeuver = tTrue;
                m_i8OvertakeStage = 0;
            }
            performManeuverOvertake();
            RETURN_NOERROR;
        }

        //Emergency Car
        if(m_bEmergencyCarManeuver || m_bEmergencyCar)
        {
            if(!m_bEmergencyCarManeuver)
            {
                LOG_INFO(cString("Emergency Car Maneuver - initialized!"));
                m_bEmergencyCarManeuver = tTrue;
                //m_i8EmergencyStage = 0;
                m_maneuverHelpers.emergency.initTimeWait = tmTimeOfTrigger;
                TransmitLight(Lights::Hazard, tTrue);
            }
            else
            {
                if (m_pClock->GetStreamTime() - m_maneuverHelpers.emergency.initTimeWait > 1000000)
                {
                    TransmitLight(Lights::Hazard, tFalse);
                    m_bEmergencyCarManeuver = tFalse;
                }
            }
            //            performManeuverEmergencyCar();
            //            RETURN_NOERROR;
        }

        //Zebra Crossing
        if(m_bZebraCrossingManeuver || m_bZebraCrossing)
        {
            if(!m_bZebraCrossingManeuver)
            {
                LOG_INFO(cString("Zebra Crossing Maneuver - initialized!"));
                m_bZebraCrossingManeuver = tTrue;
                m_i8ZebraStage = 0;
            }
            performManeuverZebra();
            RETURN_NOERROR;
        }

        //Child
        if(m_bChildManeuver || m_bChild)
        {
            if(!m_bChildManeuver)
            {
                LOG_INFO(cString("Child Maneuver - initialized!"));
                m_bChildManeuver = tTrue;
                m_bChildManeuverInitTime = tmTimeOfTrigger;
                TransmitLight(Lights::Brake, tTrue);
                TransmitDDSpeedScalar(0.5f);
            }
            else if(tmTimeOfTrigger-m_bChildManeuverInitTime > 1000000 * 3)
            {
                TransmitDDSpeedScalar(1.f);
                TransmitLight(Lights::Brake, tFalse);
                m_bChild = tFalse;
                m_bChildManeuver = tFalse;
            }
        }
    }
    else if(m_bChildManeuver)
    {
        TransmitDDSpeedScalar(1.f);
        TransmitLight(Lights::Brake, tFalse);
        m_bChild = tFalse;
        m_bChildManeuver = tFalse;
    }


    if(m_maneuverHelpers.objectDetection.carLeft && tmTimeOfTrigger-m_maneuverHelpers.objectDetection.carLeftInitTime > 1000000 * m_properties.objectDetection.resetTime)
    {
        m_maneuverHelpers.objectDetection.carLeft = tFalse;
    }
    if(m_maneuverHelpers.objectDetection.carMiddle && tmTimeOfTrigger-m_maneuverHelpers.objectDetection.carMiddleInitTime > 1000000 * m_properties.objectDetection.resetTime)
    {
        m_maneuverHelpers.objectDetection.carMiddle = tFalse;
    }
    if(m_maneuverHelpers.objectDetection.carRight && tmTimeOfTrigger-m_maneuverHelpers.objectDetection.carRightInitTime > 1000000 * m_properties.objectDetection.resetTime)
    {
        m_maneuverHelpers.objectDetection.carRight = tFalse;
    }

    //Normal Maneuver
    if (m_currentState.i16StateID == statecar_running)
    {
        performManeuver();
        if (m_currentManeuver.stage == ManeuverStage::Complete)
        {
            nextManeuver();
        }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    RETURN_NOERROR;
}

tVoid cCoreFilter::performManeuver()
{
    switch (m_currentManeuver.maneuver.action)
    {
    case maneuver_left:
        if(m_properties.general.useSignDistance)
        {
            performManeuverLeftSign();
        }
        else
        {
            performManeuverLeft();
        }
        break;
    case maneuver_right:
        if(m_properties.general.useSignDistance)
        {
            performManeuverRightSign();
        }
        else
        {
            performManeuverRight();
        }
        break;
    case maneuver_straight:
        if(m_properties.general.useSignDistance)
        {
            performManeuverStraightSign();
        }
        else
        {
            performManeuverStraight();
        }
        break;
    case maneuver_cross_parking:
        performManeuverCrossParking();
        break;
    case maneuver_pull_out_left:
        performManeuverPullOutLeft();
        break;
    case maneuver_pull_out_right:
        performManeuverPullOutRight();
        break;
    case maneuver_merge_left:
        performManeuverMergeLeft();
        break;

    case manuever_undefined: // use for idle driving when testing
        performManeuverIdle();
        break;

        //use these cases for the Open Challenge!!
    case maneuver_parallel_parking:
        //This is for the "slave" Car
        performOCManeuverSlave();
        break;
    case maneuver_merge_right:
        //This is for the "master" Car
        performOCManeuverMaster();
        break;

    default:
        updateState(statecar_error);
        break;
    }
}

tVoid cCoreFilter::performManeuverEmergencyCar()
{
    //LOG_INFO(cString("CoreFilter::performManeuverEmergencyCar()"));
    switch (m_i8EmergencyStage)
    {
    case 0:
        TransmitLight(Lights::Hazard, tTrue);
        TransmitDDCom({ 30, m_properties.emergency.dist1, m_properties.general.defaultSpeed, tFalse, 100.f });
        m_i8EmergencyStage++;
        LOG_INFO(cString("Emergency Maneuver - Stage 0 complete"));
        break;
    case 1:
    {
        if (m_lastComID == 30)
        {
            TransmitDDCom({ 31, m_properties.emergency.dist2, m_properties.general.defaultSpeed, tTrue, -100.f });
            //m_i8EmergencyStage++;
            m_i8EmergencyStage++;
            LOG_INFO(cString("Emergency Maneuver - Stage 1 complete"));
        }
        break;
    }
    case 2:
    {
        if (!m_bEmergencyCar && m_lastComID == 31)
        {
            m_maneuverHelpers.emergency.initTimeWait = m_pClock->GetStreamTime();
            m_i8EmergencyStage++;
            LOG_INFO(cString("Emergency Maneuver - Stage 2 complete"));
        }
        break;
    }
    case 3:
    {
        if (m_pClock->GetStreamTime()-m_maneuverHelpers.emergency.initTimeWait > 1000000 * m_properties.emergency.waitTime)
        {
            TransmitLight(Lights::Hazard, tFalse);
            TransmitDDCom({0, 1000.f, m_properties.general.defaultSpeed, tTrue, -200.f});
            ResetManeuverBool();
            LOG_INFO(cString("Emergency Maneuver - finished"));
        }
        break;
    }
    default:
        updateState(statecar_error);
        break;
    }
}

tVoid cCoreFilter::performManeuverOvertake()
{
    switch(m_i8OvertakeStage)
    {
    case 0:
        if(!m_maneuverHelpers.overtake.skipBackOff)
        {
            m_lastComID = -1;
            TransmitLight(Lights::Reverse, tTrue);
            TransmitDDCom({40, m_properties.overtake.distBackOff, -m_properties.general.defaultSpeedSlower, tTrue, m_properties.general.steeringOffset});
            LOG_INFO(cString("Overtake Maneuver - Stage BackOff complete"));
        }
        m_i8OvertakeStage++;
        break;
    case 1:
        if(m_maneuverHelpers.overtake.skipBackOff || m_lastComID == 40)
        {
            TransmitLight(Lights::Reverse, tFalse);
            TransmitLight(Lights::Left, tTrue);

            //Watch for oncoming traffic - to do!!

            TransmitLaneChange(tTrue);
            TransmitDDCom({41, m_properties.overtake.dist1, m_properties.general.defaultSpeedSlower, tFalse, -200.f});
            m_i8OvertakeStage++;
            LOG_INFO(cString("Overtake Maneuver - Stage 1 complete"));
        }
        break;
    case 2:
        if(m_lastComID == 41)
        {
            TransmitLight(Lights::Left, tFalse);
            TransmitDDCom({42, m_properties.overtake.dist2, m_properties.general.defaultSpeedSlower, tFalse, -200.0f});
            m_i8OvertakeStage++;
            LOG_INFO(cString("Overtake Maneuver - Stage 2 complete"));
        }
        break;
    case 3:
        if(m_lastComID == 42)
        {
            TransmitLight(Lights::Right, tTrue);
            TransmitLaneChange(tFalse);
            TransmitDDCom({43, m_properties.overtake.dist3, m_properties.general.defaultSpeedSlower, tFalse, -200.0f});
            m_i8OvertakeStage++;
            LOG_INFO(cString("Overtake Maneuver - Stage 3 complete"));
        }
        break;
    case 4:
        if(m_lastComID == 43)
        {
            TransmitLight(Lights::Right, tFalse);
            TransmitDDCom({0, 1000.f, m_properties.general.defaultSpeed, tTrue, -200.f});
            ResetManeuverBool();
            m_bOvertake = tFalse;
            LOG_INFO(cString("Overtake Maneuver - finished"));
        }
        break;
    default:
        updateState(statecar_error);
        break;
    }
}

tVoid cCoreFilter::performManeuverZebra()
{
    switch(m_i8ZebraStage)
    {
    case 0:
    {
        tFloat32 distance = m_maneuverHelpers.zebra.distanceToZebra;
        TransmitDDCom({51, distance, m_properties.general.defaultSpeedSlower, tTrue, -200.0f});
        m_i8ZebraStage++;
        m_maneuverHelpers.zebra.stopSent = tFalse;
        m_maneuverHelpers.zebra.initTimeSet = tFalse;
        LOG_INFO(cString("Zebra Maneuver - Stage 1 Complete!"));
        break;
    }
    case 1:
        if (m_lastComID == 51)
        {
            if(!m_maneuverHelpers.zebra.initTimeSet)
            {
                m_maneuverHelpers.zebra.initTime = m_pClock->GetStreamTime();
                m_maneuverHelpers.zebra.initTimeSet = tTrue;
                return;
            }
            else if(m_pClock->GetStreamTime()-m_maneuverHelpers.zebra.initTime < 1000000 * m_properties.zebra.waitTime)
            {
                return;
            }

            //Watch for person
            tBool stop = m_maneuverHelpers.objectDetection.personRight;

            if(stop)
            {
                if(!m_maneuverHelpers.zebra.stopSent)
                {
                    TransmitDDCom({404, 100.f, 0.f, tTrue, m_properties.general.steeringOffset});
                    m_maneuverHelpers.zebra.stopSent = tTrue;
                }
            }
            else
            {
                SetLaneROI(0.f);
                TransmitDDCom({52, m_properties.zebra.dist2, m_properties.general.defaultSpeedSlower, tFalse, m_properties.general.steeringOffset});
                m_maneuverHelpers.zebra.stopSent = tFalse;
                m_i8ZebraStage ++;
                LOG_INFO(cString("Zebra Maneuver - Stage 2 Complete!"));
            }
        }
        break;
    case 2:
        if (m_lastComID == 52)
        {
            TransmitDDCom({0, 1000.f, m_properties.general.defaultSpeed, tTrue, -200.f});
            ResetManeuverBool();
            m_bZebraCrossing = tFalse;
            LOG_INFO(cString("Zebra Maneuver - Finished!"));
        }
        break;
    default:
        updateState(statecar_error);
        break;
    }
}

tVoid cCoreFilter::performManeuverChild()
{
    switch(m_i8ChildStage)
    {
    case 0:
        TransmitLight(Lights::Brake, tTrue);
        TransmitDDCom({61, 0.1f, m_properties.general.defaultSpeedSlower, tFalse, -200.f});
        m_i8ChildStage++;
        LOG_INFO(cString("Child Maneuver - Stage 1 Complete!"));
        break;
    case 1:
        if(m_lastComID == 61)
        {
            TransmitLight(Lights::Brake, tFalse);
            TransmitDDCom({62, m_properties.child.dist1 - 0.1f, m_properties.general.defaultSpeedSlower, tFalse, -200.f});
            m_i8ChildStage++;
            LOG_INFO(cString("Child Maneuver - Stage 2 Complete!"));
        }
        break;
    case 2:
        if(m_lastComID == 62)
        {
            TransmitDDCom({0, 1000.f, m_properties.general.defaultSpeed, tTrue, -200.f});
            ResetManeuverBool();
            m_bChild = tFalse;
            LOG_INFO(cString("Child Maneuver - finished!"));
        }
        break;
    default:
        updateState(statecar_error);
        break;
    }
}

tVoid cCoreFilter::performManeuverIdle()
{
    //LOG_INFO(cString("CoreFilter::performIdle()"));
    switch (m_currentManeuver.stage)
    {
    case ManeuverStage::Init:
    case ManeuverStage::Search:
        m_currentManeuver.stage = ManeuverStage::Run;
        break;

    case ManeuverStage::Run:
    {
        TransmitDDCom({1,4.0f,m_properties.general.defaultSpeed,tTrue,-200.0f});
        TransmitLaneChange(tTrue);
    }

    case ManeuverStage::Complete:
    default:
        updateState(statecar_error);
        break;
    }
}

tVoid cCoreFilter::performManeuverLeft()
{
    //LOG_INFO(cString("CoreFilter::performManeuverLeft()"));
    switch (m_currentManeuver.stage)
    {
    case ManeuverStage::Init:
        m_bIdleDriving = tTrue;
        m_maneuverHelpers.crossing.step = 0;
        m_maneuverHelpers.crossing.maxDistanceToStopLine = 100.f;
        TransmitDDCom({0, 1000.0f, m_properties.general.defaultSpeed, tTrue, -200.0f});
        if(/*!m_bCrossingBefore &&*/ !m_maneuverHelpers.crossing.skipSign)
        {
            m_maneuverHelpers.crossing.sign = CrossingSigns::None;
        }
        m_currentManeuver.stage = ManeuverStage::Search;
        break;
    case ManeuverStage::Search:
    {
        switch(m_maneuverHelpers.crossing.sign)
        {
        case CrossingSigns::None:
            return;
        case CrossingSigns::StraightOnly:
            updateState(statecar_error);
            return;
        default:
            break;
        }
        m_bIdleDriving = tFalse;
        TransmitLight(Lights::Left, tTrue);
        m_lastComID = -1;
        TransmitDDCom({0, m_maneuverHelpers.crossing.maxDistanceToStopLine, m_properties.general.defaultSpeed, tFalse, -200.f});
        m_maneuverHelpers.crossing.situation = m_maneuverHelpers.crossing.sign;
        m_crossingLeft = tFalse;
        SwitchCrossingDetector(1);
        m_currentManeuver.stage = ManeuverStage::SearchCrossing;
        break;
    }
    case ManeuverStage::SearchCrossing:
    {
        if(m_crossingLeft || m_lastComID == 0)
        {
            SwitchCrossingDetector(0);
            tBool stop = tFalse;
            if(m_bEmergencyCar) {
                stop = tTrue;
            }
            else
            {
                switch(m_maneuverHelpers.crossing.situation)
                {
                case CrossingSigns::Crossing:
                    /*
                 * watch for Cars coming from the right and front
                 */
                    stop = m_maneuverHelpers.objectDetection.carMiddle || m_maneuverHelpers.objectDetection.carRight;

                    /* watch for pedestrians crossing from the right and on the left
                 * need to implement!
                 *
                 */
                    //stop |= WatchLeftForPerson() || m_maneuverHelpers.objectDetection.personRight;
                    break;
                case CrossingSigns::Rocket:
                    /*
                 * watch for Cars coming from the front
                 */
                    stop = m_maneuverHelpers.objectDetection.carMiddle;
                    /* watch for pedestrians crossing on the left
                 * need to implement!
                 *
                 */
                    //stop |= WatchLeftForPerson();
                    break;
                case CrossingSigns::Yield:
                    /*
                 * watch for Cars coming from the front/right/left
                 */
                    stop = m_maneuverHelpers.objectDetection.carMiddle || m_maneuverHelpers.objectDetection.carRight || m_maneuverHelpers.objectDetection.carLeft;
                    /* watch for pedestrians cross crossing and on the left
                 * need to implement!
                 *
                 */
                    //stop |= WatchLeftForPerson() || m_maneuverHelpers.objectDetection.personRight;
                    break;
                case CrossingSigns::Stop:
                    stop = tTrue;
                    break;
                default:
                    updateState(statecar_error);
                    return;
                }
            }

            m_maneuverHelpers.crossing.step = 1;
            m_lastComID = -1;
            tFloat32 dist = m_crossingLeft ? (tFloat32)m_properties.turnLeft.distanceToStopLine : 0.f;
            TransmitDDCom({1, dist, m_properties.general.defaultSpeed, stop, -200.0f});
            if(m_maneuverHelpers.crossing.situation == CrossingSigns::Stop)
            {
                m_maneuverHelpers.crossing.initTimeSet = tFalse;
            }
            m_maneuverHelpers.crossing.stopSent = tFalse;
            m_currentManeuver.stage = ManeuverStage::Run;
        }
        break;
    }
    case ManeuverStage::Run:
    {
        if(m_lastComID != m_maneuverHelpers.crossing.step) return;

        switch(m_lastComID)
        {
        case 1:
        {
            if(m_maneuverHelpers.crossing.situation == CrossingSigns::Stop)
            {
                if(!m_maneuverHelpers.crossing.initTimeSet)
                {
                    m_maneuverHelpers.crossing.initTime = m_pClock->GetStreamTime();
                    m_maneuverHelpers.crossing.initTimeSet = tTrue;
                    return;
                }
                else if((m_pClock->GetStreamTime()-m_maneuverHelpers.crossing.initTime)<1000000*m_properties.general.stopWaitTime)
                {
                    return;
                }
            }

            tBool stop = tFalse;
            if(m_bEmergencyCar)
            {
                stop = tTrue;
            }
            else
            {
                switch(m_maneuverHelpers.crossing.situation)
                {
                case CrossingSigns::Crossing:
                    /*
                     * watch for Cars coming from the right and front
                     */
                    stop = (m_maneuverHelpers.objectDetection.carMiddle || m_maneuverHelpers.objectDetection.carRight);

                    /* watch for pedestrians crossing from the right and on the left
                     * need to implement!
                     *
                     */
                    //stop |= WatchLeftForPerson() || m_maneuverHelpers.objectDetection.personRight;
                    break;
                case CrossingSigns::Rocket:
                    /*
                     * watch for Cars coming from the front
                     */
                    stop = m_maneuverHelpers.objectDetection.carMiddle;
                    /* watch for pedestrians crossing on the left
                     * need to implement!
                     *
                     */
                    //stop |= WatchLeftForPerson();
                    break;
                case CrossingSigns::Yield:
                    /*
                     * watch for Cars coming from the front/right/left
                     */
                    stop = ((m_maneuverHelpers.objectDetection.carMiddle || m_maneuverHelpers.objectDetection.carRight) || m_maneuverHelpers.objectDetection.carLeft);
                    /* watch for pedestrians cross crossing and on the left
                     * need to implement!
                     *
                     */
                    //stop |= WatchLeftForPerson() || m_maneuverHelpers.objectDetection.personRight;
                    break;
                case CrossingSigns::Stop:
                    /*
                     * watch for Cars coming from the front/right/left
                     */
                    stop = ((m_maneuverHelpers.objectDetection.carMiddle || m_maneuverHelpers.objectDetection.carRight) || m_maneuverHelpers.objectDetection.carLeft);
                    /* watch for pedestrians cross crossing and on the left
                     * need to implement!
                     *
                     */
                    //stop |= WatchLeftForPerson() || m_maneuverHelpers.objectDetection.personRight;
                    break;
                default:
                    updateState(statecar_error);
                    return;
                }
            }
            if(!stop)
            {
                m_maneuverHelpers.crossing.step = 2;
                TransmitDDCom({2, (m_properties.turnLeft.distanceToCrossing - m_properties.turnLeft.distanceToStopLine), m_properties.general.defaultSpeed, tFalse, m_properties.general.steeringOffset});
            }
            else if (!m_maneuverHelpers.crossing.stopSent)
            {
                TransmitDDCom({404, 1000.0f, 0.0f, tTrue, -200.0f});
                m_maneuverHelpers.crossing.stopSent = tTrue;
                m_maneuverHelpers.crossing.initTimeWaitMax = m_pClock->GetStreamTime();
            }
            else if (m_pClock->GetStreamTime()-m_maneuverHelpers.crossing.initTime > 1000000 * m_maneuverHelpers.crossing.waitMax)
            {
                m_maneuverHelpers.crossing.step = 2;
                TransmitDDCom({2, (m_properties.turnLeft.distanceToCrossing - m_properties.turnLeft.distanceToStopLine), m_properties.general.defaultSpeed, tFalse, m_properties.general.steeringOffset});
            }
            break;
        }
        case 2:
            m_maneuverHelpers.crossing.step = 3;
            //            m_maneuverHelpers.crossing.sign = CrossingSigns::None;
            //            m_bCrossingBefore = tTrue;
            TransmitDDCom({3, m_properties.turnLeft.distance, (m_properties.general.defaultSpeed), tFalse, m_properties.turnLeft.steering});
            break;
        case 3:
            m_maneuverHelpers.crossing.step = 4;
            SetLaneROI(50.0f);
            TransmitDDCom({4, 1.0f, m_properties.general.defaultSpeed, tTrue, -200.0f});
            TransmitLight(Lights::Left, tFalse);
            if(m_maneuverHelpers.crossing.nextManSkipSign)
            {
                m_maneuverHelpers.crossing.skipSign = tTrue;
                m_maneuverHelpers.crossing.sign = m_maneuverHelpers.crossing.nextManSign;
                m_maneuverHelpers.crossing.nextManSkipSign = tFalse;
                m_maneuverHelpers.crossing.nextManSign = CrossingSigns::None;
            }
            m_currentManeuver.stage = ManeuverStage::Complete;
            break;
        default:
            updateState(statecar_error);
            break;
        }

        break;
    }
    default:
        break;
    }
}

tVoid cCoreFilter::performManeuverRight()
{
    //LOG_INFO(cString("CoreFilter::performManeuverRight()"));
    switch (m_currentManeuver.stage)
    {
    case ManeuverStage::Init:
        m_bIdleDriving = tTrue;
        m_maneuverHelpers.crossing.step = 0;
        m_maneuverHelpers.crossing.maxDistanceToStopLine = 100.f;
        TransmitDDCom({0, 1000.0f, m_properties.general.defaultSpeed, tTrue, -200.0f});
        if(/*!m_bCrossingBefore &&*/ !m_maneuverHelpers.crossing.skipSign)
        {
            m_maneuverHelpers.crossing.sign = CrossingSigns::None;
        }
        m_currentManeuver.stage = ManeuverStage::Search;
        //LOG_INFO(cString("MS::Search"));
        break;
    case ManeuverStage::Search:
    {
        switch(m_maneuverHelpers.crossing.sign)
        {
        case CrossingSigns::None:
            return;
        case CrossingSigns::StraightOnly:
            updateState(statecar_error);
            return;
        default:
            break;
        }
        m_bIdleDriving = tFalse;
        TransmitLight(Lights::Right, tTrue);
        m_lastComID = -1;
        TransmitDDCom({0, m_maneuverHelpers.crossing.maxDistanceToStopLine, m_properties.general.defaultSpeed, tFalse, -200.f});
        m_maneuverHelpers.crossing.situation = m_maneuverHelpers.crossing.sign;
        m_crossingRight = tFalse;
        SwitchCrossingDetector(1);
        m_currentManeuver.stage = ManeuverStage::SearchCrossing;
        break;
    }
    case ManeuverStage::SearchCrossing:
    {
        if(m_crossingRight || m_lastComID == 0)
        {

            SwitchCrossingDetector(0);
            tBool stop = tFalse;
            if(m_bEmergencyCar)
            {
                stop = tTrue;
            }
            else
            {
                switch(m_maneuverHelpers.crossing.situation)
                {
                case CrossingSigns::Crossing:
                    /* watch for pedestrians crossing on the right and front
                     * need to implement!
                     *
                     */
                    //stop = WatchLeftForPerson() || m_maneuverHelpers.objectDetection.personRight;
                    break;
                case CrossingSigns::Rocket:
                    /* watch for pedestrians crossing on right side
                     * need to implement!
                     *
                     */
                    //stop = m_maneuverHelpers.objectDetection.personRight;
                    break;
                case CrossingSigns::Yield:
                    /*
                     * watch for Cars coming from the left
                     */
                    stop = m_maneuverHelpers.objectDetection.carLeft;
                    /* watch for pedestrians cross crossing and on the left
                     * need to implement!
                     *
                     */
                    //stop |= WatchLeftForPerson() || m_maneuverHelpers.objectDetection.personRight;
                    break;
                case CrossingSigns::Stop:
                    stop = tTrue;
                    break;
                default:
                    updateState(statecar_error);
                    return;
                }
            }

            m_maneuverHelpers.crossing.step = 1;
            m_lastComID = -1;
            tFloat32 dist = m_crossingRight ? (tFloat32) (m_crossingDistance - m_properties.turnRight.distanceToStopLine) : 0.f;
            TransmitDDCom({1, dist, m_properties.general.defaultSpeed, stop, -200.0f});
            if(m_maneuverHelpers.crossing.situation == CrossingSigns::Stop)
            {
                m_maneuverHelpers.crossing.initTimeSet = tFalse;
            }
            m_maneuverHelpers.crossing.stopSent = tFalse;
            m_currentManeuver.stage = ManeuverStage::Run;
        }
        break;
    }
    case ManeuverStage::Run:
    {
        if(m_lastComID != m_maneuverHelpers.crossing.step) return;
        switch(m_lastComID)
        {
        case 1:
        {
            if(m_maneuverHelpers.crossing.situation == CrossingSigns::Stop)
            {
                if(!m_maneuverHelpers.crossing.initTimeSet)
                {
                    m_maneuverHelpers.crossing.initTime = m_pClock->GetStreamTime();
                    m_maneuverHelpers.crossing.initTimeSet = tTrue;
                    return;
                }
                else if((m_pClock->GetStreamTime()-m_maneuverHelpers.crossing.initTime)<1000000*m_properties.general.stopWaitTime)
                {
                    return;
                }
            }

            tBool stop = tFalse;
            if(m_bEmergencyCar)
            {
                stop = tTrue;
            }
            else
            {
                switch(m_maneuverHelpers.crossing.situation)
                {
                case CrossingSigns::Crossing:
                    /* watch for pedestrians crossing on the right and front
                     * need to implement!
                     *
                     */
                    //stop = WatchLeftForPerson() || m_maneuverHelpers.objectDetection.personRight;
                    break;
                case CrossingSigns::Rocket:
                    /* watch for pedestrians crossing on right side
                     * need to implement!
                     *
                     */
                    //stop = m_maneuverHelpers.objectDetection.personRight;
                    break;
                case CrossingSigns::Yield:
                    /*
                     * watch for Cars coming from the left
                     */
                    stop = m_maneuverHelpers.objectDetection.carLeft;
                    /* watch for pedestrians cross crossing and on the left
                     * need to implement!
                     *
                     */
                    //stop |= WatchLeftForPerson() || m_maneuverHelpers.objectDetection.personRight;
                    break;
                case CrossingSigns::Stop:
                    /*
                     * watch for Cars coming from the left
                     */
                    stop = m_maneuverHelpers.objectDetection.carLeft;
                    /* watch for pedestrians cross crossing and on the left
                     * need to implement!
                     *
                     */
                    //stop |= WatchLeftForPerson() || m_maneuverHelpers.objectDetection.personRight;
                    break;
                default:
                    updateState(statecar_error);
                    return;
                }
            }
            if(!stop)
            {
                m_maneuverHelpers.crossing.step = 2;
                TransmitDDCom({2, (m_properties.turnRight.straightDistance), m_properties.general.defaultSpeed, tFalse, m_properties.general.steeringOffset});
            }
            else if (!m_maneuverHelpers.crossing.stopSent)
            {
                TransmitDDCom({404, 1000.0f, 0.0f, tTrue, -200.0f});
                m_maneuverHelpers.crossing.stopSent = tTrue;
                m_maneuverHelpers.crossing.initTimeWaitMax = m_pClock->GetStreamTime();
            }
            else if (m_pClock->GetStreamTime()-m_maneuverHelpers.crossing.initTime > 1000000 * m_maneuverHelpers.crossing.waitMax)
            {
                m_maneuverHelpers.crossing.step = 2;
                TransmitDDCom({2, (m_properties.turnRight.straightDistance), m_properties.general.defaultSpeed, tFalse, m_properties.general.steeringOffset});
            }
            break;
        }
        case 2:
            m_maneuverHelpers.crossing.step = 3;
            //            m_maneuverHelpers.crossing.sign = CrossingSigns::None;
            //            m_bCrossingBefore = tTrue;
            TransmitDDCom({3, m_properties.turnRight.distance, (1.2f * m_properties.general.defaultSpeed), tFalse, m_properties.turnRight.steering});
            break;
        case 3:
            m_maneuverHelpers.crossing.step = 4;
            SetLaneROI(100.0f);
            TransmitDDCom({4, 1.0f, m_properties.general.defaultSpeed, tTrue, -200.0f});
            TransmitLight(Lights::Right, tFalse);
            if(m_maneuverHelpers.crossing.nextManSkipSign)
            {
                m_maneuverHelpers.crossing.skipSign = tTrue;
                m_maneuverHelpers.crossing.sign = m_maneuverHelpers.crossing.nextManSign;
                m_maneuverHelpers.crossing.nextManSkipSign = tFalse;
                m_maneuverHelpers.crossing.nextManSign = CrossingSigns::None;
            }
            m_currentManeuver.stage = ManeuverStage::Complete;
            break;
        default:
            updateState(statecar_error);
            break;
        }
        break;
    }
    default:
        break;
    }
}

tVoid cCoreFilter::performManeuverStraight()
{
    //LOG_INFO(cString("CoreFilter::performManeuverStraight()"));
    switch (m_currentManeuver.stage)
    {
    case ManeuverStage::Init:
        m_bIdleDriving = tTrue;
        m_maneuverHelpers.crossing.step = 0;
        m_maneuverHelpers.crossing.maxDistanceToStopLine = 100.f;
        TransmitDDCom({0, 1000.0f, m_properties.general.defaultSpeed, tTrue, -200.0f});
        if(/*!m_bCrossingBefore &&*/ !m_maneuverHelpers.crossing.skipSign)
        {
            m_maneuverHelpers.crossing.sign = CrossingSigns::None;
        }
        m_currentManeuver.stage = ManeuverStage::Search;
        break;
    case ManeuverStage::Search:
    {
        switch(m_maneuverHelpers.crossing.sign)
        {
        case CrossingSigns::None:
            return;
        default:
            break;
        }
        m_bIdleDriving = tFalse;
        m_maneuverHelpers.crossing.situation = m_maneuverHelpers.crossing.sign;
        m_crossingLeft = tFalse;
        m_crossingRight = tFalse;
        m_lastComID = -1;
        TransmitDDCom({0, m_maneuverHelpers.crossing.maxDistanceToStopLine, m_properties.general.defaultSpeed, tFalse, -200.f});
        SwitchCrossingDetector(1);
        m_currentManeuver.stage = ManeuverStage::SearchCrossing;
        break;
    }
    case ManeuverStage::SearchCrossing:
    {
        if(m_crossingLeft || m_crossingRight || m_lastComID == 0)
        {
            SwitchCrossingDetector(0);
            tBool stop = tFalse;
            if(m_bEmergencyCar)
            {
                stop = tTrue;
            }
            else
            {

                switch(m_maneuverHelpers.crossing.situation)
                {
                case CrossingSigns::Crossing:
                    /*
                     * watch for Cars coming from the right
                     */
                    stop = m_maneuverHelpers.objectDetection.carRight;
                    /*
                     * watch for pedestrians crossing from the right
                     * need to implement!
                     *
                     */
                    //stop |= m_maneuverHelpers.objectDetection.personRight;
                    break;
                case CrossingSigns::Rocket:
                    /*
                     * nothing to watch for
                     */
                    break;
                case CrossingSigns::Yield:
                    /*
                     * watch for Cars coming from the left and right
                     */
                    stop = m_maneuverHelpers.objectDetection.carLeft || m_maneuverHelpers.objectDetection.carRight;
                    /* watch for pedestrians cross crossing
                     * need to implement!
                     *
                     */
                    //stop |= m_maneuverHelpers.objectDetection.personRight || WatchLeftForPerson();
                    break;
                case CrossingSigns::Stop:
                    stop = tTrue;
                    break;
                case CrossingSigns::StraightOnly:
                    /*
                     * nothing to watch for
                     */
                    break;
                default:
                    updateState(statecar_error);
                    return;
                }
                //LOG_INFO(cString::Format("SItuation: %i",static_cast<int>(m_maneuverHelpers.crossing.situation)));
            }

            tFloat32 distance = m_crossingLeft ? static_cast<tFloat32>(m_properties.turnLeft.distanceToStopLine) : ( m_crossingRight ? static_cast<tFloat32>(m_crossingDistance - m_properties.turnRight.distanceToStopLine) : 0.f);

            m_maneuverHelpers.crossing.step = 1;
            m_lastComID = -1;
            //if(stop) LOG_INFO(cString("Stop!!!!"));
            TransmitDDCom({1, distance, m_properties.general.defaultSpeed, stop, m_properties.general.steeringOffset});
            if(m_maneuverHelpers.crossing.situation == CrossingSigns::Stop)
            {
                m_maneuverHelpers.crossing.initTimeSet = tFalse;
            }
            m_maneuverHelpers.crossing.stopSent = tFalse;
            m_currentManeuver.stage = ManeuverStage::Run;
        }
        break;
    }
    case ManeuverStage::Run:
    {
        if(m_lastComID != m_maneuverHelpers.crossing.step) return;

        switch(m_lastComID)
        {
        case 1:
        {
            if(m_maneuverHelpers.crossing.situation == CrossingSigns::Stop)
            {
                if(!m_maneuverHelpers.crossing.initTimeSet)
                {
                    m_maneuverHelpers.crossing.initTime = m_pClock->GetStreamTime();
                    m_maneuverHelpers.crossing.initTimeSet = tTrue;
                    return;
                }
                else if((m_pClock->GetStreamTime()-m_maneuverHelpers.crossing.initTime)<1000000*m_properties.general.stopWaitTime)
                {
                    return;
                }
            }

            tBool stop = tFalse;
            if(m_bEmergencyCar)
            {
                stop = tTrue;
            }
            else
            {
                switch(m_maneuverHelpers.crossing.situation)
                {
                case CrossingSigns::Crossing:
                    /*
                     * watch for Cars coming from the right
                     */
                    stop = m_maneuverHelpers.objectDetection.carRight;
                    /*
                     * watch for pedestrians crossing from the right
                     * need to implement!
                     *
                     */
                    //stop |= m_maneuverHelpers.objectDetection.personRight;
                    break;
                case CrossingSigns::Rocket:
                    /*
                     * nothing to watch for
                     */
                    break;
                case CrossingSigns::Yield:
                    /*
                     * watch for Cars coming from the left and right
                     */
                    stop = m_maneuverHelpers.objectDetection.carLeft || m_maneuverHelpers.objectDetection.carRight;
                    /* watch for pedestrians cross crossing
                     * need to implement!
                     *
                     */
                    //stop |= m_maneuverHelpers.objectDetection.personRight || WatchLeftForPerson();
                    break;
                case CrossingSigns::Stop:
                    /*
                     * watch for Cars coming from the left and right
                     */
                    stop = m_maneuverHelpers.objectDetection.carLeft || m_maneuverHelpers.objectDetection.carRight;
                    /* watch for pedestrians cross crossing
                     * need to implement!
                     *
                     */
                    //stop |= m_maneuverHelpers.objectDetection.personRight || WatchLeftForPerson();
                    break;
                case CrossingSigns::StraightOnly:
                    /*
                     * nothing to watch for
                     */
                    break;
                default:
                    updateState(statecar_error);
                    return;
                }
            }
            if(!stop)
            {
                m_maneuverHelpers.crossing.step = 2;
                TransmitDDCom({2, m_properties.straight.distance, m_properties.general.defaultSpeed, tFalse, m_properties.general.steeringOffset});
            }
            else if (!m_maneuverHelpers.crossing.stopSent)
            {
                TransmitDDCom({404, 1000.f, 0.0f, tTrue, m_properties.general.steeringOffset});
                m_maneuverHelpers.crossing.stopSent = tTrue;
                m_maneuverHelpers.crossing.initTimeWaitMax = m_pClock->GetStreamTime();
            }
            else if (m_pClock->GetStreamTime()-m_maneuverHelpers.crossing.initTime > 1000000 * m_maneuverHelpers.crossing.waitMax)
            {
                m_maneuverHelpers.crossing.step = 2;
                TransmitDDCom({2, m_properties.straight.distance, m_properties.general.defaultSpeed, tFalse, m_properties.general.steeringOffset});
            }
            break;
        }
        case 2:
            m_maneuverHelpers.crossing.step = 3;
            //            m_maneuverHelpers.crossing.sign = CrossingSigns::None;
            //            m_bCrossingBefore = tTrue;
            TransmitDDCom({3, 1.f, m_properties.general.defaultSpeed, tTrue, 200.0f});
            if(m_maneuverHelpers.crossing.nextManSkipSign)
            {
                m_maneuverHelpers.crossing.skipSign = tTrue;
                m_maneuverHelpers.crossing.sign = m_maneuverHelpers.crossing.nextManSign;
                m_maneuverHelpers.crossing.nextManSkipSign = tFalse;
                m_maneuverHelpers.crossing.nextManSign = CrossingSigns::None;
            }
            m_currentManeuver.stage = ManeuverStage::Complete;
            break;
        default:
            updateState(statecar_error);
            break;
        }
        break;
    }
    default:
        break;
    }
}

tVoid cCoreFilter::performManeuverCrossParking()
{
    //LOG_INFO(cString("CoreFilter::performManeuverCrossParking()"));
    switch (m_currentManeuver.stage)
    {
    case ManeuverStage::Init:
        m_maneuverHelpers.parking.signFound = tFalse;
        m_bIdleDriving = tTrue;
        TransmitDDCom({0, 1000.0f, m_properties.general.defaultSpeed, tTrue, -200.0f});
        m_currentManeuver.stage = ManeuverStage::Search;
        break;
    case ManeuverStage::Search:
        if(m_maneuverHelpers.parking.signFound)
        {
            m_bIdleDriving = tFalse;
            tInt number = ParkingSlotMap(m_currentManeuver.maneuver.extra);
            tFloat32 distance = m_maneuverHelpers.parking.distanceToSign + m_properties.parking.dist1 + m_properties.parking.dist2 * static_cast<tFloat32>(number - 1);
            m_lastComID = -1;
            TransmitDDCom({1, distance, m_properties.general.defaultSpeed, tFalse, -200.f});
            m_currentManeuver.stage = ManeuverStage::ParkingForwardStraight;
        }
        break;
    case ManeuverStage::ParkingForwardStraight:
        if(m_lastComID == 1)
        {
            m_maneuverHelpers.rotation = 0;
            TransmitLight(Lights::Right, tTrue);
            TransmitDDCom({2, m_properties.parking.forwardStraightDist, m_properties.general.defaultSpeedSlower, tTrue, /*m_properties.general.steeringOffset*/ -200.f});
            m_maneuverHelpers.parking.initTimeSet = tFalse;
            m_currentManeuver.stage = ManeuverStage::CheckTraffic;
        }
        break;
    case ManeuverStage::CheckTraffic:
        if(m_lastComID == 2)
        {
            if(!m_maneuverHelpers.objectDetection.carMiddle)
            {
                if(m_maneuverHelpers.openChallenge.parking)
                {
                    TransmitOCCom(tTrue);
                }
                m_currentManeuver.stage = ManeuverStage::ParkingForwardSteer;
            }
            else if(!m_maneuverHelpers.parking.initTimeSet)
            {
                m_maneuverHelpers.parking.initTime = m_pClock->GetStreamTime();
                m_maneuverHelpers.parking.initTimeSet = tTrue;
            }
            else if(m_pClock->GetStreamTime()-m_maneuverHelpers.parking.initTime > 100000 * 4)
            {
                if(m_maneuverHelpers.openChallenge.parking)
                {
                    TransmitOCCom(tTrue);
                }
                m_currentManeuver.stage = ManeuverStage::ParkingForwardSteer;
            }
        }
        break;
    case ManeuverStage::ParkingForwardSteer:
        m_maneuverHelpers.ts = m_pClock->GetStreamTime();
        TransmitDDCom({3, 1.5f, m_properties.general.defaultSpeedSlower, tTrue, -m_properties.parking.steering});
        m_currentManeuver.stage = ManeuverStage::ParkingReverseSteer;
        break;
    case ManeuverStage::ParkingReverseSteer:
    {
        // turn 45°
        object_ptr<const ISample> pInertSample;
        if (IS_OK(m_pinReaders.inertMeas.GetLastSample(pInertSample)))
        {
            auto oDecoder = m_sampleCodecFactories.inertMeas.MakeDecoderFor(*pInertSample);
            tTimeStamp ts = m_pClock->GetStreamTime();
            tFloat64 dt = (tFloat64)(ts - m_maneuverHelpers.ts)*1e-6;
            tFloat32 yawRate = access_element::get_value(oDecoder, m_sampleIndices.inertMeas.f32G_z);
            m_maneuverHelpers.rotation += (yawRate - m_properties.general.yawOffset) * dt;
            m_maneuverHelpers.ts = ts;

            if (m_maneuverHelpers.rotation >= .9f * m_properties.parking.reverseAngle)
            {
                TransmitLight(Lights::Brake, tTrue);
            }

            if (m_maneuverHelpers.rotation >= m_properties.parking.reverseAngle)
            {
                TransmitLight(Lights::Brake, tFalse);
                TransmitDDCom({4, 1.5f, -m_properties.general.defaultSpeedSlower, tTrue, m_properties.parking.steering});
                m_maneuverHelpers.ts = m_pClock->GetStreamTime();
                m_maneuverHelpers.rotation = 0;
                m_currentManeuver.stage = ManeuverStage::ParkingReverseStraight;
            }
        }
        break;
    }
    case ManeuverStage::ParkingReverseStraight:
    {
        // turn 45°
        object_ptr<const ISample> pInertSample;
        if (IS_OK(m_pinReaders.inertMeas.GetLastSample(pInertSample)))
        {
            auto oDecoder = m_sampleCodecFactories.inertMeas.MakeDecoderFor(*pInertSample);
            tTimeStamp ts = m_pClock->GetStreamTime();
            tFloat64 dt = (tFloat64)(ts - m_maneuverHelpers.ts)*1e-6;
            tFloat32 yawRate = access_element::get_value(oDecoder, m_sampleIndices.inertMeas.f32G_z);
            m_maneuverHelpers.rotation -= (yawRate - m_properties.general.yawOffset) * dt;
            m_maneuverHelpers.ts = ts;
            if (m_maneuverHelpers.rotation <= -m_properties.parking.reverseAngle)
            {
                TransmitDDCom({5, m_properties.parking.reverseStraightDist, -m_properties.general.defaultSpeedSlower, tTrue, m_properties.general.steeringOffset});
                m_maneuverHelpers.parking.initTimeSet = tFalse;
                m_currentManeuver.stage = ManeuverStage::End;
            }
        }
        break;
    }
    case ManeuverStage::End:
        if(m_lastComID == 5)
        {
            if(!m_maneuverHelpers.parking.initTimeSet)
            {
                TransmitLight(Lights::Reverse, tFalse);
                TransmitLight(Lights::Right, tFalse);
                TransmitLight(Lights::Head, tFalse);
                TransmitLight(Lights::Hazard, tTrue);
                m_maneuverHelpers.parking.initTime = m_pClock->GetStreamTime();
                m_maneuverHelpers.parking.initTimeSet = tTrue;
            }
            else if(m_pClock->GetStreamTime()-m_maneuverHelpers.parking.initTime > 1000000 * 3)
            {
                TransmitLight(Lights::Hazard, tFalse);
                TransmitLight(Lights::Head, tTrue);
                m_bCrossingBefore = tFalse;
                m_currentManeuver.stage = ManeuverStage::Complete;
            }
        }
        break;
    case ManeuverStage::Complete:
        break;
    default:
        updateState(statecar_error);
        break;
    }
}

tVoid cCoreFilter::performManeuverPullOutLeft()
{
    // 60cm
    //LOG_INFO(cString("CoreFilter::performManeuverPullOutLeft()"));
    performManeuverPullOut(Lights::Left, m_properties.pullOutLeft.straightDist, m_properties.pullOutLeft.steering);
}

tVoid cCoreFilter::performManeuverPullOutRight()
{
    // 40cm
    //LOG_INFO(cString("CoreFilter::performManeuverPullOutRight()"));
    performManeuverPullOut(Lights::Right, m_properties.pullOutRight.straightDist, m_properties.pullOutRight.steering);
}

tVoid cCoreFilter::performManeuverPullOut(Lights indicator, tFloat32 straightDistance, tFloat32 steering)
{
    switch (m_currentManeuver.stage)
    {
    case ManeuverStage::Init:
        TransmitLight(Lights::Head, tTrue);
        TransmitLight(indicator, tTrue);
        m_maneuverHelpers.rotation = 0;
        m_currentManeuver.stage = ManeuverStage::PullOutStraight;
        break;
    case ManeuverStage::PullOutStraight:
    {
        tBool stop = m_maneuverHelpers.objectDetection.carMiddle;
        if(indicator == Lights::Left)
        {
            stop = stop || m_maneuverHelpers.objectDetection.carLeft || m_maneuverHelpers.objectDetection.carRight;
        }
        else
        {
            stop = stop || m_maneuverHelpers.objectDetection.carLeft;
        }

        if(!stop)
        {
            m_lastComID = -1;
            TransmitDDCom({1, straightDistance, m_properties.general.defaultSpeedSlower, tFalse, m_properties.general.steeringOffset});
            m_currentManeuver.stage = ManeuverStage::PullOutSteer;
        }
        break;
    }
    case ManeuverStage::PullOutSteer:
    {
        if(m_lastComID == 1)
        {
            tFloat32 factor = 1.f;
            if(indicator == Lights::Right) factor = 1.5f;
            m_maneuverHelpers.ts = m_pClock->GetStreamTime();
            TransmitDDCom({2,2.0f, factor * m_properties.general.defaultSpeedSlower, tFalse, steering});
            m_currentManeuver.stage = ManeuverStage::End;
        }
        break;
    }
    case ManeuverStage::End:
    {
        // turn by 70°, lanekeeping will take care of the rest
        object_ptr<const ISample> pInertSample;
        if (IS_OK(m_pinReaders.inertMeas.GetLastSample(pInertSample)))
        {
            auto oDecoder = m_sampleCodecFactories.inertMeas.MakeDecoderFor(*pInertSample);
            tTimeStamp ts = m_pClock->GetStreamTime();
            tFloat64 dt = (tFloat64)(ts - m_maneuverHelpers.ts)*1e-6;
            tFloat32 yawRate = access_element::get_value(oDecoder, m_sampleIndices.inertMeas.f32G_z);
            m_maneuverHelpers.rotation += (yawRate - m_properties.general.yawOffset) * dt;
            m_maneuverHelpers.ts = ts;
            if (std::abs(m_maneuverHelpers.rotation) >= m_properties.general.pullOutAngle)
            {
                TransmitLight(indicator, tFalse);
                TransmitDDCom({3, 1.f, m_properties.general.defaultSpeed, tTrue, -200.f});
                SetLaneROI(0.0f);
                m_bCrossingBefore = tFalse;
                m_currentManeuver.stage = ManeuverStage::Complete;
            }
        }
        break;
    }
    case ManeuverStage::Complete:
        break;
    default:
        updateState(statecar_error);
        break;
    }
}

tVoid cCoreFilter::performManeuverMergeLeft()
{
    //LOG_INFO(cString("CoreFilter::performManeuverMergeLeft()"));
    switch (m_currentManeuver.stage)
    {
    case ManeuverStage::Init:
        m_bIdleDriving = tTrue;
        TransmitRamp(tTrue);
        TransmitDDCom({0, 1000.0f, m_properties.general.defaultSpeed, tTrue, -200.0f});
        m_maneuverHelpers.mergeLeft.foundRamp = tFalse;
        m_currentManeuver.stage = ManeuverStage::Search;
        break;
    case ManeuverStage::Search:
        if(m_maneuverHelpers.mergeLeft.foundRamp)
        {
            m_bIdleDriving = tFalse;
            LOG_INFO(cString("Merge Left - Ramp detected!"));
            m_lastComID = -1;
            TransmitDDCom({8, m_properties.mergeLeft.distRamp, m_properties.general.defaultSpeed, tFalse, -200.f});
            m_currentManeuver.stage = ManeuverStage::BeforeResetRoi;
        }
        break;
    case ManeuverStage::BeforeResetRoi:
        if(m_lastComID == 8)
        {
            TransmitDDCom({9, m_properties.mergeLeft.distResetRoi, m_properties.general.defaultSpeed, tFalse, -200.f});
            m_currentManeuver.stage = ManeuverStage::ResetRoi;
        }
        break;
    case ManeuverStage::ResetRoi:
        SetLaneROI(0.0f);
        if(m_lastComID == 9)
        {
            TransmitDDCom({1, m_properties.mergeLeft.dist1 - m_properties.mergeLeft.distResetRoi, m_properties.general.defaultSpeed, tFalse, -200.f});
            m_maneuverHelpers.mergeLeft.step = 1;
            m_currentManeuver.stage = ManeuverStage::Run;
        }
        break;
    case ManeuverStage::Run:
    {
        if(m_lastComID != m_maneuverHelpers.mergeLeft.step) return;

        switch(m_lastComID)
        {
        case 1:
        {
            m_maneuverHelpers.mergeLeft.wait = m_maneuverHelpers.objectDetection.carLeft;
            TransmitLight(Lights::Left, tTrue);
            TransmitDDCom({2, m_properties.mergeLeft.distanceLookLeft, m_properties.general.defaultSpeed, m_maneuverHelpers.mergeLeft.wait, -200.0f});
            m_maneuverHelpers.mergeLeft.initTimeSet = tFalse;
            m_maneuverHelpers.mergeLeft.step = 2;

            break;
        }
        case 2:
        {
            if(m_maneuverHelpers.mergeLeft.wait)
            {
                if(!m_maneuverHelpers.mergeLeft.initTimeSet)
                {
                    m_maneuverHelpers.mergeLeft.initTime = m_pClock->GetStreamTime();
                    m_maneuverHelpers.mergeLeft.initTimeSet = tTrue;
                    return;
                }
                else if((m_pClock->GetStreamTime()-m_maneuverHelpers.mergeLeft.initTime)<1000000*m_properties.mergeLeft.waitTime)
                {
                    return;
                }
            }

            TransmitRamp(tFalse);

            if(m_properties.mergeLeft.useLaneChange) TransmitLaneChange(tTrue);

            TransmitDDCom({3, 1.0f, m_properties.general.defaultSpeed, tFalse, -200.0f});
            m_maneuverHelpers.mergeLeft.step = 3;

            break;
        }
        case 3:
            LOG_INFO(cString("Merge Left - Complete!"));
            TransmitLight(Lights::Left, tFalse);
            TransmitDDCom({4, 1.0f, m_properties.general.defaultSpeed, tTrue, -200.0f});
            m_maneuverHelpers.mergeLeft.step = 4;

            m_bCrossingBefore = tFalse;
            m_currentManeuver.stage = ManeuverStage::Complete;
            break;
        default:
            updateState(statecar_error);
            break;
        }
        break;
    }
    default:
        m_currentManeuver.stage = ManeuverStage::Complete;
        break;
    }
}

tResult cCoreFilter::ProcessJuryStruct(tTimeStamp tmTimeOfTrigger)
{
    object_ptr<const ISample> pSample;
    if (IS_OK(m_pinReaders.juryStruct.GetNextSample(pSample)))
    {
        auto oDecoder = m_sampleCodecFactories.juryStruct.MakeDecoderFor(*pSample);
        RETURN_IF_FAILED(oDecoder.IsValid());
        tJuryStruct juryInput;
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_sampleIndices.juryStruct.maneuverEntry, &juryInput.i16ManeuverEntry));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_sampleIndices.juryStruct.actionId, &juryInput.i16ActionID));

        tInt8 i8ActionID = juryInput.i16ActionID;
        tInt16 i16entry = juryInput.i16ManeuverEntry;

        std::lock_guard<std::mutex> lock(m_oStateManeuverMutex);
        switch (aadc::jury::juryAction(i8ActionID))
        {
        case action_getready:
            LOG_INFO(cString::Format("Received Jury Action: action_getready, id = %d", i16entry));
            if (initManeuver(i16entry))
            {
                if(!m_startSignSet && m_properties.general.useInit)
                {
                    m_searchForStartSign = tTrue;
                    searchForStartSign(tTrue);
                    m_searchForStartSignInitTime = tmTimeOfTrigger;
                }

                m_bJuryStop = tFalse;
                TransmitLight(Lights::Left, tFalse);
                TransmitLight(Lights::Right, tFalse);
                TransmitLight(Lights::Hazard, tFalse);
                TransmitLight(Lights::Head, tTrue);
                TransmitLight(Lights::Brake, tFalse);
                TransmitLight(Lights::Reverse, tFalse);
                TransmitDDCom({0,100.f,0.f,tTrue, 0.f});
                m_bZebraCrossingManeuver = tFalse;
                m_bZebraCrossing = tFalse;
                m_bChildManeuver = tFalse;
                m_bChild = tFalse;
                m_bEmergencyCarManeuver = tFalse;
                m_bEmergencyCar = tFalse;
                m_bOvertakeManeuver = tFalse;
                m_bOvertake = tFalse;
                m_bIdleDriving = tFalse;
                m_bCrossingBefore = tFalse;

                m_maneuverHelpers.crossing.nextManSkipSign = tFalse;
                m_maneuverHelpers.crossing.nextManSign = CrossingSigns::None;
                m_maneuverHelpers.crossing.skipSign = tFalse;
                m_maneuverHelpers.crossing.sign = CrossingSigns::None;
                m_maneuverHelpers.crossing.stopSent = tFalse;


                SwitchCrossingDetector(0);
                m_crossingLeft = tFalse;
                m_crossingRight = tFalse;
                laneUseLeftRoi(tTrue);
                laneUseRightRoi(tTrue);
                stopLaneChange();

                TransmitSpeedScaleSwitch(tTrue);

                updateState(statecar_ready, i16entry);
                RETURN_NOERROR;
            }
            break;
        case action_start:
            LOG_INFO(cString::Format("Received Jury Action: action_start, id = %d", i16entry));
            if (i16entry == m_currentManeuver.maneuver.id
                    && m_currentManeuver.stage == ManeuverStage::Init && !m_searchForStartSign)
            {
                updateState(statecar_running, i16entry);
                RETURN_NOERROR;
            }
            break;
        case action_stop:
            LOG_INFO(cString::Format("Received Jury Action: action_stop, id = %d", i16entry));
            m_bJuryStop = tTrue;
            TransmitOCSwitch(tFalse);
            TransmitDDCom({ -1, 0.f, 0.f, tTrue, 0.f });

            TransmitSpeedScaleSwitch(tFalse);

            RETURN_NOERROR;
        default:
            break;
        }
        updateState(statecar_error, i16entry);
    }
    RETURN_NOERROR;
}

tVoid cCoreFilter::updateState(stateCar stateID, tInt16 i16ManeuverEntry)
{
    setState(stateID, i16ManeuverEntry);
    transmitState();
}

tVoid cCoreFilter::updateState(stateCar stateID)
{
    setState(stateID);
    transmitState();
}

tVoid cCoreFilter::setState(stateCar stateID, tInt16 i16ManeuverEntry)
{
    m_currentState = { stateID, i16ManeuverEntry };
    logState();
}

tVoid cCoreFilter::setState(stateCar stateID)
{
    m_currentState.i16StateID = stateID;
    logState();
}

tVoid cCoreFilter::logState()
{
    std::string msg = "Current State: " + stateCarToString(stateCar(m_currentState.i16StateID)) + ", Maneuver: " + std::to_string(m_currentState.i16ManeuverEntry) + " - " + maneuverToString(m_currentManeuver.maneuver.action);
    LOG_INFO(cString(msg.c_str()));
}

tResult cCoreFilter::transmitState()
{
    if (m_properties.jury.enableConsoleOutput)
    {
        switch (m_currentState.i16StateID)
        {
        case statecar_ready:
            LOG_INFO(cString::Format("Driver Module: Send state: READY, Maneuver ID %d", m_currentState.i16ManeuverEntry));
            break;
        case statecar_running:
            LOG_INFO(cString::Format("Driver Module: Send state: RUNNING, Maneuver ID %d", m_currentState.i16ManeuverEntry));
            break;
        case statecar_complete:
            LOG_INFO(cString::Format("Driver Module: Send state: COMPLETE, Maneuver ID %d", m_currentState.i16ManeuverEntry));
            break;
        case statecar_error:
            LOG_INFO(cString::Format("Driver Module: Send state: ERROR, Maneuver ID %d", m_currentState.i16ManeuverEntry));
            break;
        case statecar_startup:
            LOG_INFO(cString::Format("Driver Module: Send state: STARTUP, Maneuver ID %d", m_currentState.i16ManeuverEntry));
            break;
        }
    }

    object_ptr<ISample> pWriteSample;
    RETURN_IF_FAILED(alloc_sample(pWriteSample, m_pClock->GetStreamTime()));
    {
        auto oCodec = m_sampleCodecFactories.driverStruct.MakeCodecFor(pWriteSample);
        RETURN_IF_FAILED(oCodec.SetElementValue(m_sampleIndices.driverStruct.stateId, m_currentState.i16StateID));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_sampleIndices.driverStruct.maneuverEntry, m_currentState.i16ManeuverEntry));
    }

    m_pinWriters.driverStruct << pWriteSample << flush << trigger;
    RETURN_NOERROR;
}

tResult cCoreFilter::ProcessManeuverList(tTimeStamp tmTimeOfTrigger)
{
    object_ptr<const ISample> pSampleAnonymous;
    if (IS_OK(m_pinReaders.maneuverList.GetNextSample(pSampleAnonymous)))
    {
        std::vector<tChar> data;
        object_ptr_shared_locked<const ISampleBuffer> pSampleBuffer;
        RETURN_IF_FAILED(pSampleAnonymous->Lock(pSampleBuffer));
        data.resize(pSampleBuffer->GetSize());
        memcpy(data.data(), pSampleBuffer->GetPtr(), pSampleBuffer->GetSize());
        if (data.size() > 0)
        {//maneuverlist
            m_strManeuverFileString.Set(data.data(), data.size());
            LoadManeuverList();
        }
    }
    RETURN_NOERROR;
}

tResult cCoreFilter::LoadManeuverList()
{
    aadc::jury::maneuverList sectorList;
    // create dom from string received from pin
    cDOM oDOM;
    oDOM.FromString(m_strManeuverFileString);
    cDOMElementRefList oSectorElems;
    cDOMElementRefList oManeuverElems;

    //read first Sector Elem
    if (IS_OK(oDOM.FindNodes("AADC-Maneuver-List/AADC-Sector", oSectorElems)))
    {
        //iterate through sectors
        for (cDOMElementRefList::iterator itSectorElem = oSectorElems.begin(); itSectorElem != oSectorElems.end(); ++itSectorElem)
        {
            //if sector found
            tSector sector;
            sector.id = (*itSectorElem)->GetAttributeUInt32("id");

            if (IS_OK((*itSectorElem)->FindNodes("AADC-Maneuver", oManeuverElems)))
            {
                //iterate through maneuvers
                for (cDOMElementRefList::iterator itManeuverElem = oManeuverElems.begin(); itManeuverElem != oManeuverElems.end(); ++itManeuverElem)
                {
                    tManeuver man;
                    man.id = (*itManeuverElem)->GetAttributeUInt32("id");
                    man.action = maneuverFromString((*itManeuverElem)->GetAttribute("action").GetPtr());
                    man.extra = (*itManeuverElem)->GetAttributeUInt32("extra");
                    sector.sector.push_back(man);
                }
            }

            sectorList.push_back(sector);
        }
    }
    std::lock_guard<std::mutex> lock(m_oManeuverListMutex);
    m_sectorList = sectorList;
    if (oSectorElems.size() > 0)
    {
        LOG_INFO("Jury: Loaded Maneuver file successfully.");
        m_maneuverListReceived = tTrue;
    }
    else
    {
        LOG_ERROR("Jury: no valid Maneuver Data found!");
        RETURN_ERROR(ERR_INVALID_FILE);
    }
    RETURN_NOERROR;
}

tResult cCoreFilter::ProcessOpenDriveMap(tTimeStamp tmTimeOfTrigger)
{
    object_ptr<const ISample> pSampleAnonymous;
    if(IS_OK(m_pinReaders.openDriveMap.GetNextSample(pSampleAnonymous)))
    {
        std::vector<tChar> data;
        object_ptr_shared_locked<const ISampleBuffer> pSampleBuffer;
        RETURN_IF_FAILED(pSampleAnonymous->Lock(pSampleBuffer));
        data.resize(pSampleBuffer->GetSize());
        memcpy(data.data(), pSampleBuffer->GetPtr(), pSampleBuffer->GetSize());
        if (data.size() > 0)
        {
            m_openDriveMapString.Set(data.data(), data.size());
            m_roadGraph.load(m_openDriveMapString);
            m_openDriveMapReceived = tTrue;
        }
    }
    RETURN_NOERROR;
}

tResult cCoreFilter::ProcessRoadSignMap(tTimeStamp tmTimeOfTrigger)
{
    // todo: parse parking spaces into member variable
    object_ptr<const ISample> pSampleAnonymous;
    if (IS_OK(m_pinReaders.roadSignMap.GetNextSample(pSampleAnonymous)))
    {
        std::vector<tChar> data;
        object_ptr_shared_locked<const ISampleBuffer> pSampleBuffer;
        RETURN_IF_FAILED(pSampleAnonymous->Lock(pSampleBuffer));
        data.resize(pSampleBuffer->GetSize());
        memcpy(data.data(), pSampleBuffer->GetPtr(), pSampleBuffer->GetSize());
        if (data.size() > 0)
        {
            m_roadSignString.Set(data.data(), data.size());
            LoadRoadSignMap();
        }
    }
    RETURN_NOERROR;
}

tResult cCoreFilter::LoadRoadSignMap()
{
    cDOM oDOM;
    oDOM.FromString(m_roadSignString);
    cDOMElementRefList oSignElems;
    cDOMElementRefList oParkingSpaceElems;

    if (IS_OK(oDOM.FindNodes("roadSign", oSignElems)))
    {
        //iterate through signs
        for (cDOMElementRefList::iterator itSignElem = oSignElems.begin(); itSignElem != oSignElems.end(); ++itSignElem)
        {
            tRoadSign sign;
            sign.id = (*itSignElem)->GetAttributeUInt32("id");
            sign.x = static_cast<tFloat32>((*itSignElem)->GetAttributeFloat64("x"));
            sign.y = static_cast<tFloat32>((*itSignElem)->GetAttributeFloat64("y"));
            sign.direction = (*itSignElem)->GetAttributeInt("direction");

            switch (sign.id)
            {
            case 1:
                sign.type = RoadSigns::Stop;
                break;
            case 2:
                sign.type = RoadSigns::ParkingSpace;
                break;
            case 3:
                sign.type = RoadSigns::Rocket;
                break;
            case 5:
                sign.type = RoadSigns::Yield;
                break;
            case 6:
                sign.type = RoadSigns::ZebraCrossing;
                break;
            case 10:
                continue;
            default:
                sign.initId = (*itSignElem)->GetAttributeUInt32("init");
                if(!sign.initId) continue;
                m_roadSignMap.initSigns.push_back(sign);
                continue;
            }
            m_roadSignMap.signs.push_back(sign);
        }
    }

    if (IS_OK(oDOM.FindNodes("parkingSpace", oParkingSpaceElems)))
    {
        //iterate through parking spaces
        for (cDOMElementRefList::iterator itParkingElem = oParkingSpaceElems.begin(); itParkingElem != oParkingSpaceElems.end(); ++itParkingElem)
        {
            tParkingSpc parkingSpace;
            parkingSpace.id = (*itParkingElem)->GetAttributeUInt32("id");
            parkingSpace.x = static_cast<tFloat32>((*itParkingElem)->GetAttributeFloat64("x"));
            parkingSpace.y = static_cast<tFloat32>((*itParkingElem)->GetAttributeFloat64("y"));
            parkingSpace.direction = (*itParkingElem)->GetAttributeInt("direction");

            m_roadSignMap.parkingSpaces.push_back(parkingSpace);
        }
    }

    if (m_roadSignMap.signs.size() > 0 && m_roadSignMap.initSigns.size() > 0 && m_roadSignMap.parkingSpaces.size() > 0)
    {
        LOG_INFO("Jury: Loaded Road Sign Map successfully.");
        m_roadSignMapReceived = tTrue;
    }
    else
    {
        LOG_ERROR("Jury: no valid Road Sign Map found!");
        RETURN_ERROR(ERR_INVALID_FILE);
    }
    RETURN_NOERROR;
}

tBool cCoreFilter::initManeuver(tInt id)
{
    std::lock_guard<std::mutex> lock(m_oManeuverListMutex);
    for (const auto& s : m_sectorList)
    {
        for (const auto& m : s.sector)
        {
            if (m.id == id)
            {
                m_currentManeuver = { m, ManeuverStage::Init };
                return tTrue;
            }
        }
    }
    return tFalse;
}

tVoid cCoreFilter::nextManeuver()
{
    // todo: via sector id and idx in sector?
    if (initManeuver(m_currentManeuver.maneuver.id + 1))
    {
        updateState(statecar_running, m_currentManeuver.maneuver.id);
    }
    else
    {
        TransmitDDCom({ -1, 0.f, 0.f, tTrue, 0.f });
        updateState(statecar_complete, m_currentManeuver.maneuver.id);
    }
}

tResult cCoreFilter::ProcessEmergencyCarPin(tTimeStamp tmTimeOfTrigger)
{
    object_ptr<const ISample> pSample;
    if (IS_OK(m_pinReaders.emergencyCar.GetLastSample(pSample)))
    {
        auto oDecoder = m_sampleCodecFactories.boolSignalValue.MakeDecoderFor(*pSample);
        RETURN_IF_FAILED(oDecoder.IsValid());
        tBool bValue = tFalse;
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_sampleIndices.boolSignalValue.bValue, &bValue));

        if(bValue)
        {
            LOG_INFO(cString("Emergency Car Pin Triggered: %d", static_cast<tInt32>(m_bEmergencyCar)));
            m_bEmergencyCarInitTime = tmTimeOfTrigger;
            m_bEmergencyCar = tTrue;
        }
    }
    RETURN_NOERROR;
}

tResult cCoreFilter::ProcessRoadSignExt(tTimeStamp tmTimeOfTrigger)
{
    object_ptr<const ISample> pRoadSignExtSample;

    if(IS_OK(m_pinReaders.roadSIgnExt.GetNextSample(pRoadSignExtSample)))
    {
        tInt16 currentRoadSign = -1;

        auto oDecoder = m_sampleCodecFactories.roadSignExt.MakeDecoderFor(*pRoadSignExtSample);

        RETURN_IF_FAILED(oDecoder.IsValid());
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_sampleIndices.roadSignExt.identifier, &currentRoadSign));
        const tFloat32* tVec = static_cast<const tFloat32 *>(oDecoder.GetElementAddress(m_sampleIndices.roadSignExt.tVec));
        const tFloat32* rVec = static_cast<const tFloat32 *>(oDecoder.GetElementAddress(m_sampleIndices.roadSignExt.rVec));
        if (tVec == nullptr || rVec == nullptr)
        {
            return ERR_INVALID_INDEX;
        }

        tFloat32 distance = sqrt(tVec[0]*tVec[0]+(tVec[2] - 0.2f)*(tVec[2] -0.2f));     //distance in [m] to the front of the car (not the camera)
        tFloat32 orientation = rVec[2];                                                 //orientation of the sign compared to the camera

        if(m_searchForStartSign)
        {
            for(tRoadSign initSign : m_roadSignMap.initSigns)
            {
                if(static_cast<tInt16>(initSign.id) == currentRoadSign)
                {
                    m_startSign = initSign;
                    m_startSignSet = tTrue;
                    LOG_INFO("Start Sign found!");
                    m_searchForStartSign = tFalse;
                    RETURN_NOERROR;
                }
            }
        }

        switch (currentRoadSign)
        {
        case RoadSigns::Crossing:
            if( distance < m_properties.signs.distanceToIgnore &&
                    std::abs(orientation) < 1.5f)
            {
                if(m_maneuverHelpers.crossing.sign == CrossingSigns::None)
                {
                    //LOG_INFO(cString("Roadsign Crossing"));
                    m_maneuverHelpers.crossing.signDistanceToStop = tVec[2];
                    m_maneuverHelpers.crossing.sign = CrossingSigns::Crossing;
                }
                else if(tFalse /* min < tVec[2] < max */)
                {
                    m_maneuverHelpers.crossing.maxDistanceToStopLine = tVec[2] + m_properties.signs.offsetToStopLine;
                }
            }
            if( m_maneuverHelpers.crossing.sign != CrossingSigns::None && m_maneuverHelpers.crossing.nextManSign == CrossingSigns::None)
            {
                if( m_currentManeuver.maneuver.action == maneuver_right &&
                    distance < (m_properties.signs.distanceToIgnore + m_properties.signs.distanceToIgnorePredOffset) &&
                    tVec[0] > 0.5f &&
                    std::abs(orientation) > 1.5f)
                {
                    LOG_INFO(cString("Found Sign for next maneuver after right turn!"));
                    m_maneuverHelpers.crossing.nextManSkipSign = tTrue;
                    m_maneuverHelpers.crossing.nextManSign = CrossingSigns::Crossing;
                    m_maneuverHelpers.crossing.nextManDist = m_properties.signs.distanceSkipR;
                }
                else if( m_currentManeuver.maneuver.action == maneuver_left &&
                         distance < (m_properties.signs.distanceToIgnore + 2.f * m_properties.signs.distanceToIgnorePredOffset) &&
                         tVec[0] < -1.f &&
                         std::abs(orientation) > 1.5f)
                {
                    LOG_INFO(cString("Found Sign for next maneuver after left turn!"));
                    m_maneuverHelpers.crossing.nextManSkipSign = tTrue;
                    m_maneuverHelpers.crossing.nextManSign = CrossingSigns::Crossing;
                    m_maneuverHelpers.crossing.nextManDist = m_properties.signs.distanceSkipL;
                }
            }
            break;
        case RoadSigns::Stop:
            if( distance < m_properties.signs.distanceToIgnore &&
                std::abs(orientation) < 1.5f)
            {
                if(m_maneuverHelpers.crossing.sign == CrossingSigns::None)
                {
                    //LOG_INFO(cString("Roadsign Stop"));
                    m_maneuverHelpers.crossing.signDistanceToStop = tVec[2];
                    m_maneuverHelpers.crossing.sign = CrossingSigns::Stop;
                }
                else if(tFalse /* min < tVec[2] < max */)
                {
                    m_maneuverHelpers.crossing.maxDistanceToStopLine = tVec[2] + m_properties.signs.offsetToStopLine;
                }
            }
            if( m_maneuverHelpers.crossing.sign != CrossingSigns::None && m_maneuverHelpers.crossing.nextManSign == CrossingSigns::None)
            {
                if( m_currentManeuver.maneuver.action == maneuver_right &&
                    distance < (m_properties.signs.distanceToIgnore + m_properties.signs.distanceToIgnorePredOffset) &&
                    tVec[0] > 0.5f &&
                    std::abs(orientation) > 1.5f)
                {
                    LOG_INFO(cString("Found Sign for next maneuver after right turn!"));
                    m_maneuverHelpers.crossing.nextManSkipSign = tTrue;
                    m_maneuverHelpers.crossing.nextManSign = CrossingSigns::Stop;
                    m_maneuverHelpers.crossing.nextManDist = m_properties.signs.distanceSkipR;
                }
                else if( m_currentManeuver.maneuver.action == maneuver_left &&
                         distance < (m_properties.signs.distanceToIgnore + 2.f * m_properties.signs.distanceToIgnorePredOffset) &&
                         tVec[0] < -1.f &&
                         std::abs(orientation) > 1.5f)
                {
                    LOG_INFO(cString("Found Sign for next maneuver after left turn!"));
                    m_maneuverHelpers.crossing.nextManSkipSign = tTrue;
                    m_maneuverHelpers.crossing.nextManSign = CrossingSigns::Stop;
                    m_maneuverHelpers.crossing.nextManDist = m_properties.signs.distanceSkipL;
                }
            }
            break;
        case RoadSigns::Yield:
            if( distance < m_properties.signs.distanceToIgnore &&
                    std::abs(orientation) < 1.5f)
            {
                if(m_maneuverHelpers.crossing.sign == CrossingSigns::None)
                {
                    //LOG_INFO(cString("Roadsign Yield"));
                    m_maneuverHelpers.crossing.signDistanceToStop = tVec[2];
                    m_maneuverHelpers.crossing.sign = CrossingSigns::Yield;
                }
                else if(tFalse /* min < tVec[2] < max */)
                {
                    m_maneuverHelpers.crossing.maxDistanceToStopLine = tVec[2] + m_properties.signs.offsetToStopLine;
                }
            }
            if( m_maneuverHelpers.crossing.sign != CrossingSigns::None && m_maneuverHelpers.crossing.nextManSign == CrossingSigns::None)
            {
                if( m_currentManeuver.maneuver.action == maneuver_right &&
                    distance < (m_properties.signs.distanceToIgnore + m_properties.signs.distanceToIgnorePredOffset) &&
                    tVec[0] > 0.5f &&
                    std::abs(orientation) > 1.5f)
                {
                    LOG_INFO(cString("Found Sign for next maneuver after right turn!"));
                    m_maneuverHelpers.crossing.nextManSkipSign = tTrue;
                    m_maneuverHelpers.crossing.nextManSign = CrossingSigns::Yield;
                    m_maneuverHelpers.crossing.nextManDist = m_properties.signs.distanceSkipR;
                }
                else if( m_currentManeuver.maneuver.action == maneuver_left &&
                         distance < (m_properties.signs.distanceToIgnore + 2.f * m_properties.signs.distanceToIgnorePredOffset) &&
                         tVec[0] < -1.f &&
                         std::abs(orientation) > 1.5f)
                {
                    LOG_INFO(cString("Found Sign for next maneuver after left turn!"));
                    m_maneuverHelpers.crossing.nextManSkipSign = tTrue;
                    m_maneuverHelpers.crossing.nextManSign = CrossingSigns::Yield;
                    m_maneuverHelpers.crossing.nextManDist = m_properties.signs.distanceSkipL;
                }
            }
            break;
        case RoadSigns::Rocket:
            if( distance < m_properties.signs.distanceToIgnore &&
                    std::abs(orientation) < 1.5f)
            {
                if(m_maneuverHelpers.crossing.sign == CrossingSigns::None)
                {
                    //LOG_INFO(cString("Roadsign Rocket"));
                    m_maneuverHelpers.crossing.signDistanceToStop = tVec[2];
                    m_maneuverHelpers.crossing.sign = CrossingSigns::Rocket;
                }
                else if(tFalse /* min < tVec[2] < max */)
                {
                    m_maneuverHelpers.crossing.maxDistanceToStopLine = tVec[2] + m_properties.signs.offsetToStopLine;
                }
            }
            if( m_maneuverHelpers.crossing.sign != CrossingSigns::None && m_maneuverHelpers.crossing.nextManSign == CrossingSigns::None)
            {
                if( m_currentManeuver.maneuver.action == maneuver_right &&
                    distance < (m_properties.signs.distanceToIgnore + m_properties.signs.distanceToIgnorePredOffset) &&
                    tVec[0] > 0.5f &&
                    std::abs(orientation) > 1.5f)
                {
                    LOG_INFO(cString("Found Sign for next maneuver after right turn!"));
                    m_maneuverHelpers.crossing.nextManSkipSign = tTrue;
                    m_maneuverHelpers.crossing.nextManSign = CrossingSigns::Rocket;
                    m_maneuverHelpers.crossing.nextManDist = m_properties.signs.distanceSkipR;
                }
                else if( m_currentManeuver.maneuver.action == maneuver_left &&
                         distance < (m_properties.signs.distanceToIgnore + 2.f * m_properties.signs.distanceToIgnorePredOffset) &&
                         tVec[0] < -1.f &&
                         std::abs(orientation) > 1.5f)
                {
                    LOG_INFO(cString("Found Sign for next maneuver after left turn!"));
                    m_maneuverHelpers.crossing.nextManSkipSign = tTrue;
                    m_maneuverHelpers.crossing.nextManSign = CrossingSigns::Rocket;
                    m_maneuverHelpers.crossing.nextManDist = m_properties.signs.distanceSkipL;
                }
            }
            break;
        case RoadSigns::StraightOnly:
            if( distance < m_properties.signs.distanceToIgnore &&
                    std::abs(orientation) < 1.5f)
            {
                if(m_maneuverHelpers.crossing.sign == CrossingSigns::None)
                {
                    //LOG_INFO(cString("Roadsign Straight Only"));
                    m_maneuverHelpers.crossing.signDistanceToStop = tVec[2];
                    m_maneuverHelpers.crossing.sign = CrossingSigns::StraightOnly;
                }
                else if(tFalse /* min < tVec[2] < max */)
                {
                    m_maneuverHelpers.crossing.maxDistanceToStopLine = tVec[2] + m_properties.signs.offsetToStopLine;
                }
            }
            if( m_maneuverHelpers.crossing.sign != CrossingSigns::None && m_maneuverHelpers.crossing.nextManSign == CrossingSigns::None)
            {
                if( m_currentManeuver.maneuver.action == maneuver_right &&
                    distance < (m_properties.signs.distanceToIgnore + m_properties.signs.distanceToIgnorePredOffset) &&
                    tVec[0] > 0.5f &&
                    std::abs(orientation) > 1.5f)
                {
                    LOG_INFO(cString("Found Sign for next maneuver after right turn!"));
                    m_maneuverHelpers.crossing.nextManSkipSign = tTrue;
                    m_maneuverHelpers.crossing.nextManSign = CrossingSigns::StraightOnly;
                    m_maneuverHelpers.crossing.nextManDist = m_properties.signs.distanceSkipR;
                }
                else if( m_currentManeuver.maneuver.action == maneuver_left &&
                         distance < (m_properties.signs.distanceToIgnore + 2.f * m_properties.signs.distanceToIgnorePredOffset) &&
                         tVec[0] < -1.f &&
                         std::abs(orientation) > 1.5f)
                {
                    LOG_INFO(cString("Found Sign for next maneuver after left turn!"));
                    m_maneuverHelpers.crossing.nextManSkipSign = tTrue;
                    m_maneuverHelpers.crossing.nextManSign = CrossingSigns::StraightOnly;
                    m_maneuverHelpers.crossing.nextManDist = m_properties.signs.distanceSkipL;
                }
            }
            break;
        case RoadSigns::ZebraCrossing:
            if(     !m_bZebraCrossing &&
                    m_bIdleDriving &&
                    distance < m_properties.signs.distanceToIgnore &&
                    std::abs(orientation) < 1.f)
            {
                //LOG_INFO(cString("Roadsign Zebra"));
                m_bZebraCrossing = tTrue;
                m_maneuverHelpers.zebra.distanceToZebra = distance;
            }
            else if(!m_bZebraCrossing &&
                    !m_bIdleDriving &&
                    m_currentManeuver.maneuver.action == maneuver_right &&
                    distance < (m_properties.signs.distanceToIgnore + m_properties.signs.distanceToIgnorePredOffset) &&
                    tVec[0] > 0.5f &&
                    orientation < -1.5f)
            {
                m_bZebraCrossing = tTrue;
                m_maneuverHelpers.zebra.distanceToZebra = m_properties.zebra.dist1;
            }
            if( !m_maneuverHelpers.openChallenge.zebra &&
                distance < m_properties.signs.distanceToIgnore &&
                std::abs(orientation) < 1.f)
            {
                m_maneuverHelpers.openChallenge.zebra = tTrue;
                m_maneuverHelpers.zebra.distanceToZebra = m_properties.zebra.distAfterR;
            }
            break;
        case RoadSigns::ParkingSpace:
            if( m_currentManeuver.maneuver.action == maneuver_cross_parking &&
                    distance < m_properties.signs.distanceToIgnore &&
                    std::abs(orientation) < 1.f)
            {
                m_maneuverHelpers.parking.distanceToSign = tVec[2]-0.2f;
                m_maneuverHelpers.parking.signFound = tTrue;
            }
            break;
        default:
            break;
        }
    }
    RETURN_NOERROR;
}

tResult cCoreFilter::ProcessComID(tTimeStamp tmTimeOfTrigger)
{
    tInt32 comID;
    object_ptr<const ISample> pSample;
    if(IS_OK(m_pinReaders.comID.GetNextSample(pSample)))
    {
        auto oDecoder = m_sampleCodecFactories.intSignalValue.MakeDecoderFor(*pSample);
        RETURN_IF_FAILED(oDecoder.IsValid());
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_sampleIndices.intSignalValue.value, &comID));

        LOG_INFO(cString("ComID response received"));
        m_lastComID = comID;
    }
    RETURN_NOERROR;
}

tResult cCoreFilter::ProcessObjectDetection(tTimeStamp tmTimeOfTrigger)
{
    object_ptr<const ISample> pSample;
    if(IS_OK(m_pinReaders.objectDetection.GetNextSample(pSample)))
    {
        auto oDecoder = m_sampleCodecFactories.intSignalValue.MakeDecoderFor(*pSample);
        RETURN_IF_FAILED(oDecoder.IsValid());

        tObjectDetectionData detection;
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_sampleIndices.objectDetectionData.size, &detection.ui32Size));
        const tObject* inputData = static_cast<const tObject *>(oDecoder.GetElementAddress(m_sampleIndices.objectDetectionData.objectArray));
        if (inputData == nullptr)
        {
            return ERR_INVALID_INDEX;
        }
        for (tUInt32 i = 0; i < detection.ui32Size; i++)
        {
            detection.tObjectArray[i]=inputData[i];
        }

        for(tObject object : detection.tObjectArray)
        {
            if(object.id == static_cast<tInt32> (Objects::Car))
            {
                if(((object.xBottomLeft + object.xTopRight) / 2) <= 400)
                {
                    m_maneuverHelpers.objectDetection.carLeftInitTime = tmTimeOfTrigger;
                    m_maneuverHelpers.objectDetection.carLeft = tTrue;
                }
                if(((object.xBottomLeft + object.xTopRight) / 2) > 400 && ((object.xBottomLeft + object.xTopRight) / 2) <= 800)
                {
                    m_maneuverHelpers.objectDetection.carMiddleInitTime = tmTimeOfTrigger;
                    m_maneuverHelpers.objectDetection.carMiddle = tTrue;
                }
                if(((object.xBottomLeft + object.xTopRight) / 2) > 800)
                {
                    m_maneuverHelpers.objectDetection.carRightInitTime = tmTimeOfTrigger;
                    m_maneuverHelpers.objectDetection.carRight = tTrue;
                }
            }

            //disable child for now, maneuver can mess up a lot of things!
            if((object.id == static_cast<tInt32> (Objects::Child)) && ((object.xBottomLeft + object.xTopRight) / 2) >= 600) //maybe more strict, to only activate, when child is very close
            {
                m_maneuverHelpers.objectDetection.personRightInitTime = tmTimeOfTrigger;
                m_maneuverHelpers.objectDetection.personRight = tTrue;
                m_bChildInitTime = tmTimeOfTrigger;
                m_bChild = tTrue;
            }

            if((object.id == static_cast<tInt32> (Objects::Person)) && ((object.xBottomLeft + object.xTopRight) / 2) >= 600) //maybe more strict, to only activate, when child is very close
            {
                m_maneuverHelpers.objectDetection.personRightInitTime = tmTimeOfTrigger;
                m_maneuverHelpers.objectDetection.personRight = tTrue;
            }
        }
    }
    RETURN_NOERROR;
}

tResult cCoreFilter::ProcessCrossingSituation(tTimeStamp tmTimeOfTrigger)
{
    tInt32 situation;
    object_ptr<const ISample> pSample;
    if(IS_OK(m_pinReaders.crossingSituation.GetNextSample(pSample)))
    {
        auto oDecoder = m_sampleCodecFactories.crossingSituation.MakeDecoderFor(*pSample);
        RETURN_IF_FAILED(oDecoder.IsValid());
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_sampleIndices.crossingSituation.situation, &situation))
                RETURN_IF_FAILED(oDecoder.GetElementValue(m_sampleIndices.crossingSituation.distance, &m_crossingDistance));
        //SwitchCrossingDetector(tFalse);

        switch(situation)
        {
        case 1:
            m_crossingLeft = tTrue;
            break;
        case 2:
            m_crossingRight = tTrue;
            break;
        case 3:
            m_crossingLeft = tTrue;
            m_crossingRight = tTrue;
            break;
        default:
            break;
        }
    }
    RETURN_NOERROR;
}

tResult cCoreFilter::ProcessOvertake(tTimeStamp tmTimeOfTrigger)
{
    object_ptr<const ISample> pSample;
    if(IS_OK(m_pinReaders.overtake.GetNextSample(pSample)))
    {
        auto oDecoder = m_sampleCodecFactories.boolSignalValue.MakeDecoderFor(*pSample);
        RETURN_IF_FAILED(oDecoder.IsValid());
        tBool bValue;
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_sampleIndices.boolSignalValue.bValue, &bValue));

        m_bOvertake = bValue;
    }
    RETURN_NOERROR;
}

tResult cCoreFilter::ProcessRamp(tTimeStamp tmTimeOfTrigger)
{
    object_ptr<const ISample> pSample;
    if(IS_OK(m_pinReaders.rampDetector.GetNextSample(pSample)))
    {
        auto oDecoder = m_sampleCodecFactories.boolSignalValue.MakeDecoderFor(*pSample);
        RETURN_IF_FAILED(oDecoder.IsValid());
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_sampleIndices.boolSignalValue.bValue, &m_maneuverHelpers.mergeLeft.foundRamp));
    }
    RETURN_NOERROR;
}

tResult cCoreFilter::SwitchCrossingDetector(tInt32 activate)
{
    object_ptr<ISample> pSample;
    if (IS_OK(alloc_sample(pSample)))
    {
        auto oCodec = m_sampleCodecFactories.intSignalValue.MakeCodecFor(pSample);
        RETURN_IF_FAILED(oCodec.IsValid());
        RETURN_IF_FAILED(oCodec.SetElementValue(m_sampleIndices.intSignalValue.value, activate));
        m_pinWriters.crossingDetectorActivate << pSample << flush << trigger;
    }
    RETURN_NOERROR;
}

tResult cCoreFilter::SetLaneROI(tFloat32 offset)
{
    object_ptr<ISample> pSample;
    if (IS_OK(alloc_sample(pSample)))
    {
        auto oCodec = m_sampleCodecFactories.signalValue.MakeCodecFor(pSample);
        RETURN_IF_FAILED(oCodec.IsValid());
        RETURN_IF_FAILED(oCodec.SetElementValue(m_sampleIndices.signalValue.f32Value, offset));
        m_pinWriters.SetLaneROI << pSample << flush << trigger;
    }
    RETURN_NOERROR;
}

tResult cCoreFilter::TransmitDDCom(tDriveDistanceCom com)
{
    LOG_INFO(cString::Format("Trying to transmit DDC with - Distance: %f - Speed: %f - Steering: %f...", com.distanceToDrive, com.speedToDrive, com.steeringToDrive));
    object_ptr< ISample> pWriteSample;
    if (IS_OK(alloc_sample(pWriteSample)))
    {
        auto oCodec = m_sampleCodecFactories.driveDistanceCom.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.IsValid());

        RETURN_IF_FAILED(oCodec.SetElementValue(m_sampleIndices.driveDistanceCom.comID, com.comID));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_sampleIndices.driveDistanceCom.distanceToDrive, com.distanceToDrive));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_sampleIndices.driveDistanceCom.speedToDrive, com.speedToDrive));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_sampleIndices.driveDistanceCom.stopAfter, com.stopAfter));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_sampleIndices.driveDistanceCom.steeringToDrive, com.steeringToDrive));

        m_pinWriters.DriveDistanceCom << pWriteSample << flush << trigger;
    }
    LOG_INFO(cString("...Done!"));
    RETURN_NOERROR;
}

tResult cCoreFilter::TransmitLight(Lights light, tBool bValue)
{
    object_ptr< ISample> pWriteSample;
    if (IS_OK(alloc_sample(pWriteSample)))
    {
        auto oCodec = m_sampleCodecFactories.boolSignalValue.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.IsValid());
        RETURN_IF_FAILED(oCodec.SetElementValue(m_sampleIndices.boolSignalValue.bValue, bValue));
        switch(light)
        {
        case Lights::Left:
            m_pinWriters.turnLeft << pWriteSample << flush << trigger;
            break;
        case Lights::Right:
            m_pinWriters.turnRight << pWriteSample << flush << trigger;
            break;
        case Lights::Head:
            m_pinWriters.head << pWriteSample << flush << trigger;
            break;
        case Lights::Hazard:
            m_pinWriters.hazard << pWriteSample << flush << trigger;
            break;
        case Lights::Reverse:
            m_pinWriters.reverse << pWriteSample << flush << trigger;
            break;
        case Lights::Brake:
            m_pinWriters.brake << pWriteSample << flush << trigger;
            break;
        default:
            break;
        }
    }
    RETURN_NOERROR;
}

tResult cCoreFilter::TransmitLaneChange(tBool bValue)
{
    object_ptr< ISample> pWriteSample;
    if (IS_OK(alloc_sample(pWriteSample)))
    {
        auto oCodec = m_sampleCodecFactories.boolSignalValue.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.IsValid());
        RETURN_IF_FAILED(oCodec.SetElementValue(m_sampleIndices.boolSignalValue.bValue, bValue));

        m_pinWriters.laneChange << pWriteSample << flush << trigger;
    }
    RETURN_NOERROR;
}

tResult cCoreFilter::TransmitVPStop(tBool bValue)
{
    object_ptr< ISample> pWriteSample;
    if (IS_OK(alloc_sample(pWriteSample)))
    {
        auto oCodec = m_sampleCodecFactories.boolSignalValue.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.IsValid());
        RETURN_IF_FAILED(oCodec.SetElementValue(m_sampleIndices.boolSignalValue.bValue, bValue));

        m_pinWriters.VPStopBool << pWriteSample << flush << trigger;
    }
    RETURN_NOERROR;
}

tResult cCoreFilter::TransmitRamp(tBool bValue)
{
    object_ptr< ISample> pWriteSample;
    if (IS_OK(alloc_sample(pWriteSample)))
    {
        auto oCodec = m_sampleCodecFactories.boolSignalValue.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.IsValid());
        RETURN_IF_FAILED(oCodec.SetElementValue(m_sampleIndices.boolSignalValue.bValue, bValue));

        m_pinWriters.ramp << pWriteSample << flush << trigger;
    }
    RETURN_NOERROR;
}

tResult cCoreFilter::TransmitDDSpeedScalar(tFloat32 value)
{
    object_ptr< ISample> pWriteSample;
    if (IS_OK(alloc_sample(pWriteSample)))
    {
        auto oCodec = m_sampleCodecFactories.signalValue.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.IsValid());
        RETURN_IF_FAILED(oCodec.SetElementValue(m_sampleIndices.signalValue.f32Value, value));

        m_pinWriters.DDspeedScalar << pWriteSample << flush << trigger;
    }
    RETURN_NOERROR;
}

tInt cCoreFilter::ParkingSlotMap(int i)
{
    //depends on the course
    if(i<=4)
    {
        return i;
    }
    else if (i<=8)
    {
        return i-4;
    }
    return 1;
}

tVoid cCoreFilter::ResetManeuverBool()
{
    m_bOvertakeManeuver = tFalse;
    m_bEmergencyCarManeuver = tFalse;
    m_bChildManeuver = tFalse;
    m_bZebraCrossingManeuver = tFalse;
}

tResult cCoreFilter::laneUseLeftRoi(tBool bValue)
{
    object_ptr< ISample> pWriteSample;
    if (IS_OK(alloc_sample(pWriteSample)))
    {
        auto oCodec = m_sampleCodecFactories.boolSignalValue.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.IsValid());
        RETURN_IF_FAILED(oCodec.SetElementValue(m_sampleIndices.boolSignalValue.bValue, bValue));

        m_pinWriters.laneUseRoiLeft << pWriteSample << flush << trigger;
    }
    RETURN_NOERROR;
}

tResult cCoreFilter::laneUseRightRoi(tBool bValue)
{
    object_ptr< ISample> pWriteSample;
    if (IS_OK(alloc_sample(pWriteSample)))
    {
        auto oCodec = m_sampleCodecFactories.boolSignalValue.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.IsValid());
        RETURN_IF_FAILED(oCodec.SetElementValue(m_sampleIndices.boolSignalValue.bValue, bValue));

        m_pinWriters.laneUseRoiRight << pWriteSample << flush << trigger;
    }
    RETURN_NOERROR;
}

tResult cCoreFilter::stopLaneChange()
{
    object_ptr< ISample> pWriteSample;
    if (IS_OK(alloc_sample(pWriteSample)))
    {
        auto oCodec = m_sampleCodecFactories.boolSignalValue.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.IsValid());
        RETURN_IF_FAILED(oCodec.SetElementValue(m_sampleIndices.boolSignalValue.bValue, tTrue));

        m_pinWriters.stopLaneChange << pWriteSample << flush << trigger;
    }
    RETURN_NOERROR;
}

tResult cCoreFilter::searchForStartSign(tBool bValue)
{
    object_ptr< ISample> pWriteSample;
    if (IS_OK(alloc_sample(pWriteSample)))
    {
        auto oCodec = m_sampleCodecFactories.boolSignalValue.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.IsValid());
        RETURN_IF_FAILED(oCodec.SetElementValue(m_sampleIndices.boolSignalValue.bValue, bValue));

        m_pinWriters.searchForStartSign << pWriteSample << flush << trigger;
    }
    RETURN_NOERROR;
}


//Open Challenge Functions here
tVoid cCoreFilter::performOCManeuverSlave()
{
    switch (m_currentManeuver.stage)
    {
    case ManeuverStage::Init:        
        TransmitDDCom({0, 1000.f, m_properties.general.defaultSpeed, tTrue, 200.f});
        m_maneuverHelpers.openChallenge.zebra = tFalse;
        m_bIdleDriving = tFalse;
        m_lastComID = -1;
        m_currentManeuver.stage = ManeuverStage::Search;
        break;
    case ManeuverStage::Search:

        if(m_lastComID == -1 && m_maneuverHelpers.openChallenge.zebra)
        {
            TransmitDDCom({7, 0.6f, m_properties.general.defaultSpeed, tFalse, 200.f});
            m_lastComID = -2;
        }
        else if(m_lastComID == 7)
        {
            TransmitDDCom({7, 0.f, 0.f, tFalse, 0.f});
            TransmitLight(Lights::Hazard, tTrue);
            TransmitOCSwitch(tTrue);
            m_currentManeuver.stage = ManeuverStage::Run;
            return;
        }
        if(m_maneuverHelpers.mergeLeft.foundRamp)
        {
            m_lastComID = 7;
            TransmitDDCom({0, 100.f, 0.f, tTrue, 200.f});
            m_currentManeuver.stage = ManeuverStage::Search;
            return;
        }
        break;
    case ManeuverStage::Run:
    {
        object_ptr<const ISample> pReadSample;
        if(IS_OK(m_pinReaders.OCcom.GetNextSample(pReadSample)))
        {
            auto oDecoder = m_sampleCodecFactories.boolSignalValue.MakeDecoderFor(*pReadSample);
            tBool bValue = access_element::get_value(oDecoder, m_sampleIndices.boolSignalValue.bValue);
            if(bValue)
            {
                TransmitDDCom({404, 1000.f, 0.f, tTrue, -200.f});
                TransmitOCSwitch(tFalse);
                m_currentManeuver.stage = ManeuverStage::ParkingForwardStraight;
            }
            else
            {
                TransmitDDCom({404, 1000.f, 0.f, tTrue, -200.f});
                TransmitOCSwitch(tFalse);
                m_currentManeuver.stage = ManeuverStage::Complete;
                updateState(statecar_error);
            }
        }
        break;
    }
        //parking from this on!
    case ManeuverStage::ParkingForwardStraight:
        m_maneuverHelpers.rotation = 0;
        TransmitLight(Lights::Hazard, tFalse);
        TransmitLight(Lights::Right, tTrue);
        TransmitDDCom({2, m_properties.openChallenge.parkingStraightDist, m_properties.general.defaultSpeedSlower, tTrue, -200.f});
        m_currentManeuver.stage = ManeuverStage::ParkingForwardSteer;
        break;
    case ManeuverStage::ParkingForwardSteer:
        m_maneuverHelpers.ts = m_pClock->GetStreamTime();
        TransmitDDCom({3, 1.5f, m_properties.general.defaultSpeedSlower, tTrue, -m_properties.parking.steering});
        m_currentManeuver.stage = ManeuverStage::ParkingReverseSteer;
        break;
    case ManeuverStage::ParkingReverseSteer:
    {
        // turn 45°
        object_ptr<const ISample> pInertSample;
        if (IS_OK(m_pinReaders.inertMeas.GetLastSample(pInertSample)))
        {
            auto oDecoder = m_sampleCodecFactories.inertMeas.MakeDecoderFor(*pInertSample);
            tTimeStamp ts = m_pClock->GetStreamTime();
            tFloat64 dt = (tFloat64)(ts - m_maneuverHelpers.ts)*1e-6;
            tFloat32 yawRate = access_element::get_value(oDecoder, m_sampleIndices.inertMeas.f32G_z);
            m_maneuverHelpers.rotation += (yawRate - m_properties.general.yawOffset) * dt;
            m_maneuverHelpers.ts = ts;

            if (m_maneuverHelpers.rotation >= .9f * m_properties.parking.reverseAngle)
            {
                TransmitLight(Lights::Brake, tTrue);
            }

            if (m_maneuverHelpers.rotation >= m_properties.parking.reverseAngle)
            {
                TransmitLight(Lights::Brake, tFalse);
                TransmitDDCom({4, 1.5f, -m_properties.general.defaultSpeedSlower, tTrue, m_properties.parking.steering});
                m_maneuverHelpers.ts = m_pClock->GetStreamTime();
                m_maneuverHelpers.rotation = 0;
                m_currentManeuver.stage = ManeuverStage::ParkingReverseStraight;
            }
        }
        break;
    }
    case ManeuverStage::ParkingReverseStraight:
    {
        // turn 45°
        object_ptr<const ISample> pInertSample;
        if (IS_OK(m_pinReaders.inertMeas.GetLastSample(pInertSample)))
        {
            auto oDecoder = m_sampleCodecFactories.inertMeas.MakeDecoderFor(*pInertSample);
            tTimeStamp ts = m_pClock->GetStreamTime();
            tFloat64 dt = (tFloat64)(ts - m_maneuverHelpers.ts)*1e-6;
            tFloat32 yawRate = access_element::get_value(oDecoder, m_sampleIndices.inertMeas.f32G_z);
            m_maneuverHelpers.rotation -= (yawRate - m_properties.general.yawOffset) * dt;
            m_maneuverHelpers.ts = ts;
            if (m_maneuverHelpers.rotation <= -m_properties.parking.reverseAngle)
            {
                TransmitDDCom({5, m_properties.parking.reverseStraightDist, -m_properties.general.defaultSpeedSlower, tTrue, m_properties.general.steeringOffset});
                m_maneuverHelpers.parking.initTimeSet = tFalse;
                m_currentManeuver.stage = ManeuverStage::End;
            }
        }
        break;
    }
    case ManeuverStage::End:
        if(m_lastComID == 5)
        {
            if(!m_maneuverHelpers.parking.initTimeSet)
            {
                TransmitLight(Lights::Reverse, tFalse);
                TransmitLight(Lights::Right, tFalse);
                TransmitLight(Lights::Head, tFalse);
                TransmitLight(Lights::Hazard, tTrue);
                m_maneuverHelpers.parking.initTime = m_pClock->GetStreamTime();
                m_maneuverHelpers.parking.initTimeSet = tTrue;
            }
            else if(m_pClock->GetStreamTime()-m_maneuverHelpers.parking.initTime > 1000000 * 3)
            {
                TransmitLight(Lights::Hazard, tFalse);
                m_currentManeuver.stage = ManeuverStage::Complete;
            }
        }
        break;
    default:
        updateState(statecar_error);
        return;
    }
}

tVoid cCoreFilter::performOCManeuverMaster()
{
    switch (m_currentManeuver.stage)
    {
    case ManeuverStage::Init:
        m_bIdleDriving = tFalse;
        TransmitDDCom({0,1000.f, m_properties.general.defaultSpeed, tTrue, -200.f});
        m_i8OvertakeStage = 0;
        m_currentManeuver.stage = ManeuverStage::Search;
        break;
    case ManeuverStage::Search:
    {
            switch(m_i8OvertakeStage)
            {
            case 0:
                if(m_bOvertake)
                {
                    m_lastComID = -1;
                    TransmitLight(Lights::Reverse, tTrue);
                    TransmitDDCom({40, m_properties.overtake.distBackOff, -m_properties.general.defaultSpeedSlower, tTrue, m_properties.general.steeringOffset});
                    m_i8OvertakeStage++;
                }
                break;
            case 1:
                if(m_lastComID == 40)
                {
                    TransmitLight(Lights::Left, tTrue);
                    TransmitLaneChange(tTrue);
                    TransmitDDCom({41, m_properties.overtake.dist1, m_properties.general.defaultSpeedSlower, tFalse, -200.f});
                    m_i8OvertakeStage++;
                }
                break;
            case 2:
                if(m_lastComID == 41)
                {
                    TransmitLight(Lights::Left, tFalse);
                    TransmitDDCom({42, m_properties.overtake.dist2, m_properties.general.defaultSpeedSlower, tFalse, -200.0f});
                    m_i8OvertakeStage++;
                }
                break;
            case 3:
                if(m_lastComID == 42)
                {
                    TransmitLight(Lights::Right, tTrue);
                    TransmitLaneChange(tFalse);
                    TransmitDDCom({43, m_properties.overtake.dist3 + m_properties.openChallenge.distanceOvertakeEnd, m_properties.general.defaultSpeedSlower, tTrue, -200.0f});
                    m_i8OvertakeStage++;
                }
                break;
            case 4:
                if(m_lastComID == 43)
                {
                    TransmitLight(Lights::Right, tFalse);
                    TransmitLight(Lights::Hazard, tTrue);
                    TransmitOCActivateKI(tTrue);
                    m_maneuverHelpers.openChallenge.initTime = m_pClock->GetStreamTime();
                    m_currentManeuver.stage = ManeuverStage::Run;
                }
                break;
            default:
                updateState(statecar_error);
                return;
            }
        }
        break;
    case ManeuverStage::Run:
        if(m_pClock->GetStreamTime()-m_maneuverHelpers.openChallenge.initTime > 2 * 1000000)
        {
            m_properties.general.defaultSpeed = m_properties.openChallenge.defaultSpeed;
            m_maneuverHelpers.openChallenge.parking = tTrue;
            m_currentManeuver.stage = ManeuverStage::Complete;
        }
        break;
    default:
        updateState(statecar_error);
        return;
    }
}

tResult cCoreFilter::TransmitOCSwitch(tBool bValue)
{
    object_ptr< ISample> pWriteSample;
    if (IS_OK(alloc_sample(pWriteSample)))
    {
        auto oCodec = m_sampleCodecFactories.boolSignalValue.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.IsValid());
        RETURN_IF_FAILED(oCodec.SetElementValue(m_sampleIndices.boolSignalValue.bValue, bValue));

        m_pinWriters.OCswitch << pWriteSample << flush << trigger;
    }
    RETURN_NOERROR;
}

tResult cCoreFilter::TransmitOCActivateKI(tBool bValue)
{
    object_ptr< ISample> pWriteSample;
    if (IS_OK(alloc_sample(pWriteSample)))
    {
        auto oCodec = m_sampleCodecFactories.boolSignalValue.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.IsValid());
        RETURN_IF_FAILED(oCodec.SetElementValue(m_sampleIndices.boolSignalValue.bValue, bValue));

        m_pinWriters.OCactivateKI << pWriteSample << flush << trigger;
    }
    RETURN_NOERROR;
}

tVoid cCoreFilter::performManeuverLeftSign()
{
    //LOG_INFO(cString("CoreFilter::performManeuverLeft()"));
    switch (m_currentManeuver.stage)
    {
    case ManeuverStage::Init:
        if(!m_maneuverHelpers.crossing.skipSign)
        {
            m_bIdleDriving = tTrue;
            m_maneuverHelpers.crossing.step = 0;
            TransmitDDCom({0, 1000.0f, m_properties.general.defaultSpeed, tTrue, -200.0f});
            m_maneuverHelpers.crossing.sign = CrossingSigns::None;
        }
        m_bCurveParamsSet = tFalse;
        m_currentManeuver.stage = ManeuverStage::Search;
        break;
    case ManeuverStage::Search:
    {
        switch(m_maneuverHelpers.crossing.sign)
        {
        case CrossingSigns::None:
            return;
        case CrossingSigns::StraightOnly:
            updateState(statecar_error);
            return;
        default:
            break;
        }
        m_bIdleDriving = tFalse;
        m_maneuverHelpers.crossing.situation = m_maneuverHelpers.crossing.sign;

        if(!m_bCurveParamsSet && !m_maneuverHelpers.crossing.skipSign)
        {
            switch(getSteering())
            {
            case 0: //no curve
                m_maneuverHelpers.curve.straightDist = 0.f;
                m_maneuverHelpers.curve.steerDist = 0.f;
                m_maneuverHelpers.curve.steeringOffset = 0.f;
                m_maneuverHelpers.curve.straightSteeringOffset = 0.f;
                break;
            case -1: //left curve
                m_maneuverHelpers.curve.straightDist = m_properties.curve.leftStraightDistL;
                m_maneuverHelpers.curve.steerDist = m_properties.curve.leftSteerDistL;
                m_maneuverHelpers.curve.steeringOffset = m_properties.curve.leftSteeringOffsetL;
                m_maneuverHelpers.curve.straightSteeringOffset = m_properties.curve.straightSteeringOffsetL;
                break;
            case 1: //right curve
                m_maneuverHelpers.curve.straightDist = m_properties.curve.leftStraightDistR;
                m_maneuverHelpers.curve.steerDist = m_properties.curve.leftSteerDistR;
                m_maneuverHelpers.curve.steeringOffset = m_properties.curve.leftSteeringOffsetR;
                m_maneuverHelpers.curve.straightSteeringOffset = m_properties.curve.straightSteeringOffsetR;
                break;
            default:
                updateState(statecar_error);
                return;
            }
            m_bCurveParamsSet = tTrue;
        }

        tBool stop = tFalse;
        if(m_bEmergencyCar) {
            stop = tTrue;
        }
        else
        {
            switch(m_maneuverHelpers.crossing.situation)
            {
            case CrossingSigns::Crossing:
                break;
            case CrossingSigns::Rocket:
                break;
            case CrossingSigns::Yield:
                stop = m_maneuverHelpers.objectDetection.carLeft;
                break;
            case CrossingSigns::Stop:
                stop = tTrue;
                break;
            default:
                updateState(statecar_error);
                return;
            }
        }

        TransmitLight(Lights::Left, tTrue);
        m_maneuverHelpers.crossing.step = 1;
        m_lastComID = -1;
        tFloat32 distance = (m_maneuverHelpers.crossing.signDistanceToStop - m_properties.signs.shiftStopLine)  + m_maneuverHelpers.curve.straightDist;
        TransmitDDCom({1, distance, m_properties.general.defaultSpeed, stop, -200.0f});
        if(m_maneuverHelpers.crossing.situation == CrossingSigns::Stop)
        {
            m_maneuverHelpers.crossing.initTimeSet = tFalse;
        }
        m_maneuverHelpers.crossing.stopSent = tFalse;
        m_currentManeuver.stage = ManeuverStage::Run;
    }
    break;
    case ManeuverStage::Run:
    {
        if(m_lastComID != m_maneuverHelpers.crossing.step) return;

        switch(m_lastComID)
        {
        case 1:
        {

            if(m_maneuverHelpers.crossing.situation == CrossingSigns::Stop)
            {
                if(!m_maneuverHelpers.crossing.initTimeSet)
                {
                    m_maneuverHelpers.crossing.initTime = m_pClock->GetStreamTime();
                    m_maneuverHelpers.crossing.initTimeSet = tTrue;
                    return;
                }
                else if((m_pClock->GetStreamTime()-m_maneuverHelpers.crossing.initTime)<1000000*m_properties.general.stopWaitTime)
                {
                    return;
                }
            }

            tBool stop = tFalse;
            if(m_bEmergencyCar)
            {
                stop = tTrue;
            }
            else
            {
                switch(m_maneuverHelpers.crossing.situation)
                {
                case CrossingSigns::Crossing:
                    break;
                case CrossingSigns::Rocket:
                    break;
                case CrossingSigns::Yield:
                    stop = m_maneuverHelpers.objectDetection.carLeft;
                    break;
                case CrossingSigns::Stop:
                    stop = m_maneuverHelpers.objectDetection.carLeft;
                    break;
                default:
                    updateState(statecar_error);
                    return;
                }
            }
            if(!stop)
            {
                m_maneuverHelpers.crossing.step = 2;
                tFloat32 distance = (m_properties.turnLeft.signDistanceStraight + ((m_maneuverHelpers.crossing.signDistanceToStop < m_properties.signs.shiftStopLine) ? 0.f : static_cast<tFloat32>(m_properties.signs.shiftStopLine)));
                tFloat32 steering = m_properties.general.steeringOffset + m_maneuverHelpers.curve.straightSteeringOffset;
                TransmitDDCom({2, distance, m_properties.general.defaultSpeed, tFalse, steering});
            }
            else if (!m_maneuverHelpers.crossing.stopSent)
            {
                TransmitDDCom({404, 1000.0f, 0.0f, tTrue, -200.0f});
                m_maneuverHelpers.crossing.stopSent = tTrue;
                m_maneuverHelpers.crossing.initTimeWaitMax = m_pClock->GetStreamTime();
            }
            else if (m_pClock->GetStreamTime()-m_maneuverHelpers.crossing.initTime > 1000000 * m_maneuverHelpers.crossing.waitMax)
            {
                m_maneuverHelpers.crossing.step = 2;
                tFloat32 distance = (m_properties.turnLeft.signDistanceStraight + ((m_maneuverHelpers.crossing.signDistanceToStop < m_properties.signs.shiftStopLine) ? 0.f : static_cast<tFloat32>(m_properties.signs.shiftStopLine)));
                tFloat32 steering = m_properties.general.steeringOffset + m_maneuverHelpers.curve.straightSteeringOffset;
                TransmitDDCom({2, distance, m_properties.general.defaultSpeed, tFalse, steering});
            }
            break;
        }
        case 2:
        {
            m_maneuverHelpers.crossing.step = 3;
            tFloat32 distance = m_properties.turnLeft.distance + m_maneuverHelpers.curve.steerDist;
            tFloat32 steering = m_properties.turnLeft.steering + m_maneuverHelpers.curve.steeringOffset;
            TransmitDDCom({3, distance, m_properties.general.defaultSpeed, tFalse, steering});
            break;
        }
        case 3:
            m_maneuverHelpers.crossing.step = 4;
            SetLaneROI(m_properties.turnLeft.setROI);
            TransmitDDCom({4, 1.0f, m_properties.general.defaultSpeed, tTrue, -200.0f});
            TransmitLight(Lights::Left, tFalse);
            if(m_maneuverHelpers.crossing.nextManSkipSign)
            {
                m_maneuverHelpers.crossing.skipSign = tTrue;
                m_maneuverHelpers.crossing.sign = m_maneuverHelpers.crossing.nextManSign;
                m_maneuverHelpers.crossing.nextManSkipSign = tFalse;
                m_maneuverHelpers.crossing.nextManSign = CrossingSigns::None;
                m_maneuverHelpers.crossing.signDistanceToStop = m_maneuverHelpers.crossing.nextManDist;
            }
            else
            {
                m_maneuverHelpers.crossing.skipSign = tFalse;
            }
            m_currentManeuver.stage = ManeuverStage::Complete;
            break;
        default:
            updateState(statecar_error);
            break;
        }
        break;
    }
    default:
        break;
    }
}

tVoid cCoreFilter::performManeuverRightSign()
{
    //LOG_INFO(cString("CoreFilter::performManeuverRightSign()"));
    switch (m_currentManeuver.stage)
    {
    case ManeuverStage::Init:
        if(!m_maneuverHelpers.crossing.skipSign)
        {
            m_bIdleDriving = tTrue;
            m_maneuverHelpers.crossing.step = 0;
            TransmitDDCom({0, 1000.0f, m_properties.general.defaultSpeed, tTrue, -200.0f});
            m_maneuverHelpers.crossing.sign = CrossingSigns::None;
        }
        m_bCurveParamsSet = tFalse;
        m_currentManeuver.stage = ManeuverStage::Search;
        break;
    case ManeuverStage::Search:
    {
        switch(m_maneuverHelpers.crossing.sign)
        {
        case CrossingSigns::None:
            return;
        case CrossingSigns::StraightOnly:
            updateState(statecar_error);
            return;
        default:
            break;
        }
        m_bIdleDriving = tFalse;
        m_maneuverHelpers.crossing.situation = m_maneuverHelpers.crossing.sign;

        if(!m_bCurveParamsSet && !m_maneuverHelpers.crossing.skipSign)
            {
            switch(getSteering())
            {
            case 0: //no curve
                m_maneuverHelpers.curve.straightDist = 0.f;
                m_maneuverHelpers.curve.steerDist = 0.f;
                m_maneuverHelpers.curve.steeringOffset = 0.f;
                m_maneuverHelpers.curve.straightSteeringOffset = 0.f;
                break;
            case -1: //left curve
                m_maneuverHelpers.curve.straightDist = m_properties.curve.rightStraightDistL;
                m_maneuverHelpers.curve.steerDist = m_properties.curve.rightSteerDistL;
                m_maneuverHelpers.curve.steeringOffset = m_properties.curve.rightSteeringOffsetL;
                m_maneuverHelpers.curve.straightSteeringOffset = m_properties.curve.straightSteeringOffsetL;
                break;
            case 1: //right curve
                m_maneuverHelpers.curve.straightDist = m_properties.curve.rightStraightDistR;
                m_maneuverHelpers.curve.steerDist = m_properties.curve.rightSteerDistR;
                m_maneuverHelpers.curve.steeringOffset = m_properties.curve.rightSteeringOffsetR;
                m_maneuverHelpers.curve.straightSteeringOffset = m_properties.curve.straightSteeringOffsetR;
                break;
            default:
                updateState(statecar_error);
                return;
            }
            m_bCurveParamsSet = tTrue;
            }

        tBool stop = tFalse;
        if(m_bEmergencyCar) {
            stop = tTrue;
        }
        else
        {
            switch(m_maneuverHelpers.crossing.situation)
            {
            case CrossingSigns::Crossing:
                stop = m_maneuverHelpers.objectDetection.carMiddle || m_maneuverHelpers.objectDetection.carRight;
                break;
            case CrossingSigns::Rocket:
                stop = m_maneuverHelpers.objectDetection.carMiddle;
                break;
            case CrossingSigns::Yield:
                stop = m_maneuverHelpers.objectDetection.carMiddle || m_maneuverHelpers.objectDetection.carRight || m_maneuverHelpers.objectDetection.carLeft;
                break;
            case CrossingSigns::Stop:
                stop = tTrue;
                break;
            default:
                updateState(statecar_error);
                return;
            }
        }

        TransmitLight(Lights::Right, tTrue);
        m_maneuverHelpers.crossing.step = 1;
        m_lastComID = -1;
        tFloat32 distance = (m_maneuverHelpers.crossing.signDistanceToStop - m_properties.signs.shiftStopLine) + m_maneuverHelpers.curve.straightDist;
        TransmitDDCom({1, distance, m_properties.general.defaultSpeed, stop, -200.0f});
        if(m_maneuverHelpers.crossing.situation == CrossingSigns::Stop)
        {
            m_maneuverHelpers.crossing.initTimeSet = tFalse;
        }
        m_maneuverHelpers.crossing.stopSent = tFalse;
        m_currentManeuver.stage = ManeuverStage::Run;
    }
    break;
    case ManeuverStage::Run:
    {
        if(m_lastComID != m_maneuverHelpers.crossing.step) return;

        switch(m_lastComID)
        {
        case 1:
        {
            if(m_maneuverHelpers.crossing.situation == CrossingSigns::Stop)
            {
                if(!m_maneuverHelpers.crossing.initTimeSet)
                {
                    m_maneuverHelpers.crossing.initTime = m_pClock->GetStreamTime();
                    m_maneuverHelpers.crossing.initTimeSet = tTrue;
                    return;
                }
                else if((m_pClock->GetStreamTime()-m_maneuverHelpers.crossing.initTime)<1000000*m_properties.general.stopWaitTime)
                {
                    return;
                }
            }

            tBool stop = tFalse;
            if(m_bEmergencyCar)
            {
                stop = tTrue;
            }
            else
            {
                switch(m_maneuverHelpers.crossing.situation)
                {
                case CrossingSigns::Crossing:
                    stop = (m_maneuverHelpers.objectDetection.carMiddle || m_maneuverHelpers.objectDetection.carRight);
                    break;
                case CrossingSigns::Rocket:
                    stop = m_maneuverHelpers.objectDetection.carMiddle;
                    break;
                case CrossingSigns::Yield:
                    stop = ((m_maneuverHelpers.objectDetection.carMiddle || m_maneuverHelpers.objectDetection.carRight) || m_maneuverHelpers.objectDetection.carLeft);
                    break;
                case CrossingSigns::Stop:
                    stop = ((m_maneuverHelpers.objectDetection.carMiddle || m_maneuverHelpers.objectDetection.carRight) || m_maneuverHelpers.objectDetection.carLeft);
                    break;
                default:
                    updateState(statecar_error);
                    return;
                }
            }
            if(!stop)
            {
                m_maneuverHelpers.crossing.step = 2;
                tFloat32 distance = (m_properties.turnRight.signDistanceStraight + ((m_maneuverHelpers.crossing.signDistanceToStop < m_properties.signs.shiftStopLine) ? 0.f : static_cast<tFloat32>(m_properties.signs.shiftStopLine)));
                tFloat32 steering = m_properties.general.steeringOffset + m_maneuverHelpers.curve.straightSteeringOffset;
                TransmitDDCom({2, distance, m_properties.general.defaultSpeed, tFalse, steering});
            }
            else if (!m_maneuverHelpers.crossing.stopSent)
            {
                TransmitDDCom({404, 1000.0f, 0.0f, tTrue, -200.0f});
                m_maneuverHelpers.crossing.stopSent = tTrue;
                m_maneuverHelpers.crossing.initTimeWaitMax = m_pClock->GetStreamTime();
            }
            else if (m_pClock->GetStreamTime()-m_maneuverHelpers.crossing.initTime > 1000000 * m_maneuverHelpers.crossing.waitMax)
            {
                m_maneuverHelpers.crossing.step = 2;
                tFloat32 distance = (m_properties.turnRight.signDistanceStraight + ((m_maneuverHelpers.crossing.signDistanceToStop < m_properties.signs.shiftStopLine) ? 0.f : static_cast<tFloat32>(m_properties.signs.shiftStopLine)));
                tFloat32 steering = m_properties.general.steeringOffset + m_maneuverHelpers.curve.straightSteeringOffset;
                TransmitDDCom({2, distance, m_properties.general.defaultSpeed, tFalse, steering});
            }
            break;
        }
        case 2:
        {
            m_maneuverHelpers.crossing.step = 3;
            tFloat32 distance = m_properties.turnRight.distance + m_maneuverHelpers.curve.steerDist;
            tFloat32 steering = m_properties.turnRight.steering + m_maneuverHelpers.curve.steeringOffset;
            TransmitDDCom({3, distance, m_properties.general.defaultSpeed, tFalse, steering});
            break;
        }
        case 3:
            m_maneuverHelpers.crossing.step = 4;
            SetLaneROI(m_properties.turnRight.setROI);
            TransmitDDCom({4, 1.0f, m_properties.general.defaultSpeed, tTrue, -200.0f});
            TransmitLight(Lights::Right, tFalse);
            if(m_maneuverHelpers.crossing.nextManSkipSign)
            {
                m_maneuverHelpers.crossing.skipSign = tTrue;
                m_maneuverHelpers.crossing.sign = m_maneuverHelpers.crossing.nextManSign;
                m_maneuverHelpers.crossing.nextManSkipSign = tFalse;
                m_maneuverHelpers.crossing.nextManSign = CrossingSigns::None;
                m_maneuverHelpers.crossing.signDistanceToStop = m_maneuverHelpers.crossing.nextManDist;
            }
            else
            {
                m_maneuverHelpers.crossing.skipSign = tFalse;
            }
            m_currentManeuver.stage = ManeuverStage::Complete;
            break;
        default:
            updateState(statecar_error);
            break;
        }
        break;
    }
    default:
        break;
    }
}

tVoid cCoreFilter::performManeuverStraightSign()
{
    //LOG_INFO(cString("CoreFilter::performManeuverStraightSign()"));
    switch (m_currentManeuver.stage)
    {
    case ManeuverStage::Init:
        if(!m_maneuverHelpers.crossing.skipSign)
        {
            m_bIdleDriving = tTrue;
            m_maneuverHelpers.crossing.step = 0;
            TransmitDDCom({0, 1000.0f, m_properties.general.defaultSpeed, tTrue, -200.0f});
            m_maneuverHelpers.crossing.sign = CrossingSigns::None;
        }
        m_bCurveParamsSet = tFalse;
        m_currentManeuver.stage = ManeuverStage::Search;
        break;
    case ManeuverStage::Search:
    {
        switch(m_maneuverHelpers.crossing.sign)
        {
        case CrossingSigns::None:
            return;
        default:
            break;
        }

        if(!m_bCurveParamsSet && !m_maneuverHelpers.crossing.skipSign)
        {
            switch(getSteering())
            {
            case 0: //no curve
                m_maneuverHelpers.curve.straightDist = m_properties.straight.nonCurveDist;
                m_maneuverHelpers.curve.straightSteeringOffset = 0.f;
                break;
            case -1: //left curve
                m_maneuverHelpers.curve.straightDist = m_properties.curve.straightDistL;
                m_maneuverHelpers.curve.straightSteeringOffset = m_properties.curve.straightSteeringOffsetL;
                break;
            case 1: //right curve
                m_maneuverHelpers.curve.straightDist = m_properties.curve.straightDistR;
                m_maneuverHelpers.curve.straightSteeringOffset = m_properties.curve.straightSteeringOffsetR;
                break;
            default:
                updateState(statecar_error);
                return;
            }
            m_bCurveParamsSet = tTrue;
        }

        m_bIdleDriving = tFalse;
        m_maneuverHelpers.crossing.situation = m_maneuverHelpers.crossing.sign;

        tBool stop = tFalse;
        if(m_bEmergencyCar) {
            stop = tTrue;
        }
        else
        {
            switch(m_maneuverHelpers.crossing.situation)
            {
            case CrossingSigns::Crossing:
                stop = m_maneuverHelpers.objectDetection.carRight;
                break;
            case CrossingSigns::Rocket:
                break;
            case CrossingSigns::Yield:
                stop = m_maneuverHelpers.objectDetection.carLeft || m_maneuverHelpers.objectDetection.carRight;
                break;
            case CrossingSigns::Stop:
                stop = tTrue;
                break;
            case CrossingSigns::StraightOnly:
                break;
            default:
                updateState(statecar_error);
                return;
            }
        }

        m_maneuverHelpers.crossing.step = 1;
        m_lastComID = -1;
        tFloat32 distance = m_properties.straight.signDistanceStraight + (m_maneuverHelpers.crossing.signDistanceToStop - m_properties.signs.shiftStopLine)  + m_maneuverHelpers.curve.straightDist;
        TransmitDDCom({1, distance, m_properties.general.defaultSpeed, stop, -200.0f});
        if(m_maneuverHelpers.crossing.situation == CrossingSigns::Stop)
        {
            m_maneuverHelpers.crossing.initTimeSet = tFalse;
        }
        m_maneuverHelpers.crossing.stopSent = tFalse;
        m_currentManeuver.stage = ManeuverStage::Run;
    }
    break;
    case ManeuverStage::Run:
    {
        if(m_lastComID != m_maneuverHelpers.crossing.step) return;

        switch(m_lastComID)
        {
        case 1:
        {
            if(m_maneuverHelpers.crossing.situation == CrossingSigns::Stop)
            {
                if(!m_maneuverHelpers.crossing.initTimeSet)
                {
                    m_maneuverHelpers.crossing.initTime = m_pClock->GetStreamTime();
                    m_maneuverHelpers.crossing.initTimeSet = tTrue;
                    return;
                }
                else if((m_pClock->GetStreamTime()-m_maneuverHelpers.crossing.initTime)<1000000*m_properties.general.stopWaitTime)
                {
                    return;
                }
            }

            tBool stop = tFalse;
            if(m_bEmergencyCar)
            {
                stop = tTrue;
            }
            else
            {
                switch(m_maneuverHelpers.crossing.situation)
                {
                case CrossingSigns::Crossing:
                    stop = m_maneuverHelpers.objectDetection.carRight;
                    break;
                case CrossingSigns::Rocket:
                    break;
                case CrossingSigns::Yield:
                    stop = m_maneuverHelpers.objectDetection.carLeft || m_maneuverHelpers.objectDetection.carRight;
                    break;
                case CrossingSigns::Stop:
                    stop = m_maneuverHelpers.objectDetection.carLeft || m_maneuverHelpers.objectDetection.carRight;
                    break;
                case CrossingSigns::StraightOnly:
                    break;
                default:
                    updateState(statecar_error);
                    return;
                }
            }
            if(!stop)
            {
                m_maneuverHelpers.crossing.step = 2;
                tFloat32 distance = (m_properties.straight.distance + ((m_maneuverHelpers.crossing.signDistanceToStop < m_properties.signs.shiftStopLine) ? 0.f : static_cast<tFloat32>(m_properties.signs.shiftStopLine)));
                tFloat32 steering = m_properties.general.steeringOffset + m_maneuverHelpers.curve.straightSteeringOffset;
                TransmitDDCom({2, distance, m_properties.general.defaultSpeed, tFalse, steering});
            }
            else if (!m_maneuverHelpers.crossing.stopSent)
            {
                TransmitDDCom({404, 1000.0f, 0.0f, tTrue, -200.0f});
                m_maneuverHelpers.crossing.stopSent = tTrue;
                m_maneuverHelpers.crossing.initTimeWaitMax = m_pClock->GetStreamTime();
            }
            else if (m_pClock->GetStreamTime()-m_maneuverHelpers.crossing.initTime > 1000000 * m_maneuverHelpers.crossing.waitMax)
            {
                m_maneuverHelpers.crossing.step = 2;
                tFloat32 distance = (m_properties.straight.distance + ((m_maneuverHelpers.crossing.signDistanceToStop < m_properties.signs.shiftStopLine) ? 0.f : static_cast<tFloat32>(m_properties.signs.shiftStopLine)));
                tFloat32 steering = m_properties.general.steeringOffset + m_maneuverHelpers.curve.straightSteeringOffset;
                TransmitDDCom({2, distance, m_properties.general.defaultSpeed, tFalse, steering});
            }
            break;
        }
        case 2:
            m_maneuverHelpers.crossing.step = 3;
            SetLaneROI(m_properties.straight.setROI);
            TransmitDDCom({3, 1.0f, m_properties.general.defaultSpeed, tTrue, -200.0f});
            m_maneuverHelpers.crossing.skipSign = tFalse;
            m_currentManeuver.stage = ManeuverStage::Complete;
            break;
        default:
            updateState(statecar_error);
            break;
        }
        break;
    }
    default:
        break;
    }
}

tInt32 cCoreFilter::getSteering()
{
    object_ptr<const ISample> pSample;
    if (IS_OK(m_pinReaders.steering.GetLastSample(pSample)))
    {
        auto oDecoder = m_sampleCodecFactories.signalValue.MakeDecoderFor(*pSample);
        tFloat32 steering = access_element::get_value(oDecoder, m_sampleIndices.signalValue.f32Value);

        return ((steering > m_properties.curve.threshold) - (steering < -m_properties.curve.threshold));
    }
    return 0;
}

tResult cCoreFilter::TransmitSpeedScaleSwitch(tBool bValue)
{
    object_ptr< ISample> pWriteSample;
    if (IS_OK(alloc_sample(pWriteSample)))
    {
        auto oCodec = m_sampleCodecFactories.boolSignalValue.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.IsValid());
        RETURN_IF_FAILED(oCodec.SetElementValue(m_sampleIndices.boolSignalValue.bValue, bValue));

        m_pinWriters.speedScaleToggle << pWriteSample << flush << trigger;
    }
    RETURN_NOERROR;
}


tResult cCoreFilter::TransmitOCCom(tBool bValue)
{
    object_ptr< ISample> pWriteSample;
    if (IS_OK(alloc_sample(pWriteSample)))
    {
        auto oCodec = m_sampleCodecFactories.boolSignalValue.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.IsValid());
        RETURN_IF_FAILED(oCodec.SetElementValue(m_sampleIndices.boolSignalValue.bValue, bValue));

        m_pinWriters.OCcomOut << pWriteSample << flush << trigger;
    }
    RETURN_NOERROR;
}
