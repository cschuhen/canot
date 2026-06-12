#!/bin/bash
#
MODE=" --release"

BIN_RTIC_PWRS=" --bin pwrsns_rtic"
FEATURES_RTIC_PWRS=" --features=power_sensors"
ARGS_RTIC_PWRS="${MODE} ${BIN_RTIC_PWRS} ${FEATURES_RTIC_PWRS}"
BUILD_RTIC_PWSNS="cargo build ${ARGS_RTIC_PWRS}"

BIN_RTIC_TERMINAL=" --bin terminal_rtic"
FEATURES_RTIC_TERMINAL=" --features=terminal"
ARGS_RTIC_TERMINAL="${MODE} ${BIN_RTIC_TERMINAL} ${FEATURES_RTIC_TERMINAL}"
BUILD_RTIC_TERMINAL="cargo build ${ARGS_RTIC_TERMINAL}"


BIN_EMBASSY_PWRS=" --bin pwrsns_embassy"
FEATURES_EMBASSY_PWRS=" --features=power_sensors"
ARGS_EMBASSY_PWRS="${MODE} ${BIN_EMBASSY_PWRS} ${FEATURES_EMBASSY_PWRS}"
BUILD_EMBASSY_PWSNS="cargo build ${ARGS_EMBASSY_PWRS}"

BIN_EMBASSY_TERMINAL=" --bin terminal_embassy"
FEATURES_EMBASSY_TERMINAL=" --features=terminal"
ARGS_EMBASSY_TERMINAL="${MODE} ${BIN_EMBASSY_TERMINAL} ${FEATURES_EMBASSY_TERMINAL}"
BUILD_EMBASSY_TERMINAL="cargo build ${ARGS_EMBASSY_TERMINAL}"


echo "${BUILD_RTIC_PWSNS} && ${BUILD_RTIC_TERMINAL} && ${BUILD_EMBASSY_PWSNS} && ${BUILD_EMBASSY_TERMINAL}"

if ${BUILD_RTIC_PWSNS} && ${BUILD_RTIC_TERMINAL} && ${BUILD_EMBASSY_PWSNS} && ${BUILD_EMBASSY_TERMINAL}
then
    echo "Build OK"
    #cargo size ${ARGS_RTIC_PWRS} 2> /dev/null
    #cargo size ${ARGS_RTIC_TERMINAL} 2> /dev/null
    #cargo size ${ARGS_EMBASSY_PWRS} 2> /dev/null
    #cargo size ${ARGS_EMBASSY_TERMINAL} 2> /dev/null
else
    echo "Build FAILED"
fi
date
