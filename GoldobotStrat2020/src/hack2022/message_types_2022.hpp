#pragma once
#include <cstdint>

namespace goldobot {

enum class CommMessageType : uint16_t {
  /*      0 - 0x0000 */ CommUartStats,
  /*      1 - 0x0001 */ CommUartPing,
  /*      2 - 0x0002 */ Heartbeat,
  /*      3 - 0x0003 */ HeapStats,
  /*      4 - 0x0004 */ Reset,
  /*      5 - 0x0005 */ GetNucleoFirmwareVersion,
  /*      6 - 0x0006 */ TaskStats,

  /*     10 - 0x000a */ MatchTimer = 10,
  /*     11 - 0x000b */ MatchTimerStart,
  /*     12 - 0x000c */ MatchEnd,

  /*     20 - 0x0014 */ DbgGpioGet = 20,
  /*     21 - 0x0015 */ DbgGpioGetStatus,
  /*     22 - 0x0016 */ DbgGpioSet,
  /*     23 - 0x0017 */ DbgPwmSet,

  /*     29 - 0x001d */ DbgGoldo = 29,

  /*     30 - 0x001e */ FpgaReadReg = 30,
  /*     31 - 0x001f */ FpgaReadRegStatus,
  /*     32 - 0x0020 */ FpgaWriteReg,
  /*     33 - 0x0021 */ SensorsState,
  /*     34 - 0x0022 */ FpgaGpioState,
  /*     35 - 0x0023 */ FpgaReadRegInternal,

  /*     40 - 0x0028 */ ServoAck = 40,
  /*     41 - 0x0029 */ ServoMoveMultiple,
  /*     42 - 0x002a */ ServoSetEnable,
  /*     43 - 0x002b */ ServoState,
  /*     44 - 0x002c */ ServosMoving,
  /*     45 - 0x002d */ ServoDisableAll,
  /*     46 - 0x002e */ ServoSetLiftEnable,
  /*     47 - 0x002f */ ServoGetState,
  /*     48 - 0x0030 */ ServoLiftDoHoming,
  /*     49 - 0x0031 */ ServoSetMaxTorques,

  /*     50 - 0x0032 */ ODriveRequestPacket = 50,
  /*     51 - 0x0033 */ ODriveResponsePacket,
  /*     52 - 0x0034 */ ODriveTelemetry,
  /*     53 - 0x0035 */ ODriveCommStats,

  /*     60 - 0x003c */ DynamixelsRequest = 60,
  /*     61 - 0x003d */ DynamixelsResponse,

  /*    100 - 0x0064 */ PropulsionEnableSet = 100,
  /*    101 - 0x0065 */ PropulsionMotorsEnableSet,
  /*    102 - 0x0066 */ PropulsionMotorsVelocitySetpointsSet,
  /*    103 - 0x0067 */ PropulsionSetTargetSpeed,
  /*    104 - 0x0068 */ PropulsionSetAccelerationLimits,
  /*    105 - 0x0069 */ PropulsionSetPose,
  /*    106 - 0x006a */ PropulsionSetTargetPose,
  /*    107 - 0x006b */ PropulsionEmergencyStop,
  /*    108 - 0x006c */ PropulsionClearError,
  /*    109 - 0x006d */ PropulsionClearCommandQueue,
  /*    110 - 0x006e */ PropulsionSetSimulationMode,
  /*    111 - 0x006f */ PropulsionScopeConfig,
  /*    112 - 0x0070 */ PropulsionMotorsTorqueLimitsSet,
  /*    113 - 0x0071 */ PropulsionTransformPose,

  /*    120 - 0x0078 */ PropulsionTelemetry = 120,
  /*    121 - 0x0079 */ PropulsionTelemetryEx,
  /*    122 - 0x007a */ PropulsionPose,
  /*    123 - 0x007b */ PropulsionState,
  /*    124 - 0x007c */ PropulsionODriveTelemetry,
  /*    125 - 0x007d */ PropulsionScopeData,
  /*    126 - 0x007e */ PropulsionOdometryStream,
  /*    127 - 0x007f */ PropulsionODriveStream,

  /*    130 - 0x0082 */ PropulsionCommandEvent = 130,
  /*    131 - 0x0083 */ PropulsionControllerEvent,

  /*    140 - 0x008c */ PropulsionExecuteTranslation = 140,
  /*    141 - 0x008d */ PropulsionExecuteMoveTo,
  /*    142 - 0x008e */ PropulsionExecuteRotation,
  /*    143 - 0x008f */ PropulsionExecutePointTo,
  /*    144 - 0x0090 */ PropulsionExecuteFaceDirection,
  /*    145 - 0x0091 */ PropulsionExecuteTrajectory,
  /*    146 - 0x0092 */ PropulsionMeasureNormal,
  /*    147 - 0x0093 */ PropulsionSetControlLevels,
  /*    148 - 0x0094 */ PropulsionEnterManualControl,
  /*    149 - 0x0095 */ PropulsionExecuteReposition,
  /*    150 - 0x0096 */ PropulsionExitManualControl,
  /*    151 - 0x0097 */ PropulsionCalibrateODrive,
  /*    152 - 0x0098 */ PropulsionODriveClearErrors,
  /*    153 - 0x0099 */ PropulsionExecutePointToBack,

  /*    180 - 0x00b4 */ PropulsionODriveStatistics = 180,
  /*    181 - 0x00b5 */ PropulsionODriveAxisStates,
  /*    182 - 0x00b6 */ PropulsionODriveAxisErrors,

  /*    200 - 0x00c8 */ RobotConfigLoadBegin = 200,
  /*    201 - 0x00c9 */ RobotConfigLoadChunk,
  /*    202 - 0x00ca */ RobotConfigLoadEnd,
  /*    203 - 0x00cb */ RobotConfigLoadStatus,

  /*    210 - 0x00d2 */ OdometryConfigGet = 210,
  /*    211 - 0x00d3 */ OdometryConfigGetStatus,
  /*    212 - 0x00d4 */ OdometryConfigSet,

  /*    215 - 0x00d7 */ PropulsionConfigGet = 215,
  /*    216 - 0x00d8 */ PropulsionConfigGetStatus,
  /*    217 - 0x00d9 */ PropulsionConfigSet,

  /*    250 - 0x00fa */ WatchdogReset = 250,
  /*    251 - 0x00fb */ WatchdogStatus,

  /*    300 - 0x012c */ UartCommTaskStatistics = 300,
  /*    301 - 0x012d */ ODriveCommTaskStatistics = 301,
  /*    302 - 0x012e */ PropulsionTaskStatistics = 302
};

// task ids for watchdog
// 0: main, 1: propulsion, 2: fpga, 3, odrive_comm, 4: dynamixels_comm, 5: servos

}  // namespace goldobot
