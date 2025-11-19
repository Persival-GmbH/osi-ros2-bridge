//
// Copyright 2016 -- 2018 PMSF IT Consulting Pierre R. Mai
// Copyright 2023 BMW AG
// SPDX-License-Identifier: MPL-2.0
//

#pragma once

#ifndef FMU_SHARED_OBJECT
#define FMI2_FUNCTION_PREFIX OSI-ROS2-Bridge_
#endif
#include "fmi2Functions.h"

/*
 * Logging Control
 *
 * Logging is controlled via three definitions:
 *
 * - If PRIVATE_LOG_PATH is defined it gives the name of a file
 *   that is to be used as a private log file.
 * - If PUBLIC_LOGGING is defined then we will (also) log to
 *   the FMI logging facility where appropriate.
 * - If VERBOSE_FMI_LOGGING is defined then logging of basic
 *   FMI calls is enabled, which can get very verbose.
 */

/*
 * Variable Definitions
 *
 * Define FMI_*_LAST_IDX to the zero-based index of the last variable
 * of the given type (0 if no variables of the type exist).  This
 * ensures proper space allocation, initialisation and handling of
 * the given variables in the template code.  Optionally you can
 * define FMI_TYPENAME_VARNAME_IDX definitions (e.g. FMI_REAL_MYVAR_IDX)
 * to refer to individual variables inside your code, or for example
 * FMI_REAL_MYARRAY_OFFSET and FMI_REAL_MYARRAY_SIZE definitions for
 * array variables.
 */

/* Boolean Variables */
#define FMI_BOOLEAN_VALID_IDX 0
#define FMI_BOOLEAN_LAST_IDX FMI_BOOLEAN_VALID_IDX
#define FMI_BOOLEAN_VARS (FMI_BOOLEAN_LAST_IDX + 1)

/* Integer Variables */
#define FMI_INTEGER_SENSORDATA_IN_BASELO_IDX 0
#define FMI_INTEGER_SENSORDATA_IN_BASEHI_IDX 1
#define FMI_INTEGER_SENSORDATA_IN_SIZE_IDX 2
#define FMI_INTEGER_DETECTION_INTERVAL_MS 3
#define FMI_INTEGER_LAST_IDX FMI_INTEGER_DETECTION_INTERVAL_MS
#define FMI_INTEGER_VARS (FMI_INTEGER_LAST_IDX + 1)

/* Real Variables */
#define FMI_REAL_LAST_IDX 0
#define FMI_REAL_VARS (FMI_REAL_LAST_IDX + 1)

/* String Variables */
#define FMI_STRING_PCL_TOPIC_IDX 0
#define FMI_STRING_FRAME_ID_IDX 1
#define FMI_STRING_LAST_IDX FMI_STRING_FRAME_ID_IDX
#define FMI_STRING_VARS (FMI_STRING_LAST_IDX+1)

#include <cstdarg>
#include <fstream>
#include <iostream>
#include <set>
#include <string>

#undef min
#undef max
#include "OsiRos2Bridge.h"
#include "osi_sensordata.pb.h"
#include "osi_sensorview.pb.h"

using namespace std;

/* FMU Class */
class OSMP
{
 public:
    /* FMI2 Interface mapped to C++ */
    OSMP(fmi2String theinstance_name,
         fmi2Type thefmu_type,
         fmi2String thefmu_guid,
         fmi2String thefmu_resource_location,
         const fmi2CallbackFunctions *thefunctions,
         fmi2Boolean thevisible,
         fmi2Boolean thelogging_on);
    fmi2Status SetDebugLogging(fmi2Boolean thelogging_on, size_t n_categories, const fmi2String categories[]);
    static fmi2Component Instantiate(fmi2String instance_name,
                                     fmi2Type fmu_type,
                                     fmi2String fmu_guid,
                                     fmi2String fmu_resource_location,
                                     const fmi2CallbackFunctions *functions,
                                     fmi2Boolean visible,
                                     fmi2Boolean logging_on);
    fmi2Status SetupExperiment(fmi2Boolean tolerance_defined, fmi2Real tolerance, fmi2Real start_time, fmi2Boolean stop_time_defined, fmi2Real stop_time);
    fmi2Status EnterInitializationMode();
    fmi2Status ExitInitializationMode();
    fmi2Status DoStep(fmi2Real current_communication_point, fmi2Real communication_step_size, fmi2Boolean no_set_fmu_state_prior_to_current_pointfmi_2_component);
    fmi2Status Terminate();
    fmi2Status Reset();
    void FreeInstance();
    fmi2Status GetReal(const fmi2ValueReference vr[], size_t nvr, fmi2Real value[]);
    fmi2Status GetInteger(const fmi2ValueReference vr[], size_t nvr, fmi2Integer value[]);
    fmi2Status GetBoolean(const fmi2ValueReference vr[], size_t nvr, fmi2Boolean value[]);
    fmi2Status GetString(const fmi2ValueReference vr[], size_t nvr, fmi2String value[]);
    fmi2Status SetReal(const fmi2ValueReference vr[], size_t nvr, const fmi2Real value[]);
    fmi2Status SetInteger(const fmi2ValueReference vr[], size_t nvr, const fmi2Integer value[]);
    fmi2Status SetBoolean(const fmi2ValueReference vr[], size_t nvr, const fmi2Boolean value[]);
    fmi2Status SetString(const fmi2ValueReference vr[], size_t nvr, const fmi2String value[]);

 protected:
    /* Internal Implementation */
    fmi2Status DoInit();
    fmi2Status DoStart(fmi2Boolean tolerance_defined, fmi2Real tolerance, fmi2Real start_time, fmi2Boolean stop_time_defined, fmi2Real stop_time);
    fmi2Status DoEnterInitializationMode();
    fmi2Status DoExitInitializationMode();
    fmi2Status DoCalc(fmi2Real current_communication_point, fmi2Real communication_step_size, fmi2Boolean no_set_fmu_state_prior_to_current_pointfmi_2_component);
    fmi2Status DoTerm();
    void DoFree();

    /* Private File-based Logging just for Debugging */
#ifdef PRIVATE_LOG_PATH
    static ofstream private_log_file;
#endif

    static void FmiVerboseLogGlobal(const char *format, ...)
    {
#ifdef VERBOSE_FMI_LOGGING
#ifdef PRIVATE_LOG_PATH
        va_list ap;
        va_start(ap, format);
        char buffer[1024];
        if (!private_log_file.is_open())
            private_log_file.open(PRIVATE_LOG_PATH, ios::out | ios::app);
        if (private_log_file.is_open())
        {
#ifdef _WIN32
            vsnprintf_s(buffer, 1024, format, ap);
#else
            vsnprintf(buffer, 1024, format, ap);
#endif
            private_log_file << "OSI-ROS2-Bridge"
                             << "::Global:FMI: " << buffer << endl;
            private_log_file.flush();
        }
#endif
#endif
    }

    void InternalLog(const char *category, const char *format, va_list arg)
    {
#if defined(PRIVATE_LOG_PATH) || defined(PUBLIC_LOGGING)
        char buffer[1024];
#ifdef _WIN32
        vsnprintf_s(buffer, 1024, format, arg);
#else
        vsnprintf(buffer, 1024, format, arg);
#endif
#ifdef PRIVATE_LOG_PATH
        if (!private_log_file.is_open())
            private_log_file.open(PRIVATE_LOG_PATH, ios::out | ios::app);
        if (private_log_file.is_open())
        {
            private_log_file << "OSI-ROS2-Bridge"
                             << "::" << instanceName << "<" << ((void*)this) << ">:" << category << ": " << buffer << endl;
            private_log_file.flush();
        }
#endif
#ifdef PUBLIC_LOGGING
        if (loggingOn && loggingCategories.count(category))
            functions.logger(functions.componentEnvironment, instanceName.c_str(), fmi2OK, category, buffer);
#endif
#endif
    }

    void FmiVerboseLog(const char *format, ...)
    {
#if defined(VERBOSE_FMI_LOGGING) && (defined(PRIVATE_LOG_PATH) || defined(PUBLIC_LOGGING))
        va_list ap;
        va_start(ap, format);
        internal_log("FMI", format, ap);
        va_end(ap);
#endif
    }

    /* Normal Logging */
    void NormalLog(const char *category, const char *format, ...)
    {
#if defined(PRIVATE_LOG_PATH) || defined(PUBLIC_LOGGING)
        va_list ap;
        va_start(ap, format);
        internal_log(category, format, ap);
        va_end(ap);
#endif
    }

 private:
    /* Members */
    string instance_name_;
    fmi2Type fmu_type_;
    string fmu_guid_;
    string fmu_resource_location_;
    bool visible_;
    bool logging_on_;
    set<string> logging_categories_;
    fmi2CallbackFunctions functions_;
    fmi2Boolean boolean_vars_[FMI_BOOLEAN_VARS];
    fmi2Integer integer_vars_[FMI_INTEGER_VARS];
    fmi2Real real_vars_[FMI_REAL_VARS];
    string string_vars_[FMI_STRING_VARS];
    bool simulation_started_;

    OsiRos2Bridge osi_ros2_bridge_;
    bool first_time_step = true;

    /* Simple Accessors */
    fmi2Boolean FmiValid()
    {
        return boolean_vars_[FMI_BOOLEAN_VALID_IDX];
    }
    void SetFmiValid(fmi2Boolean value)
    {
        boolean_vars_[FMI_BOOLEAN_VALID_IDX] = value;
    }

    string FmiPclTopic() { return string_vars_[FMI_STRING_PCL_TOPIC_IDX]; }
    string FmiFrameId() { return string_vars_[FMI_STRING_FRAME_ID_IDX]; }
    int FmiDetectionIntervalMS() { return integer_vars_[FMI_INTEGER_DETECTION_INTERVAL_MS]; }

    /* Protocol Buffer Accessors */
    bool GetFmiSensorDataIn(osi3::SensorData &data);

};
