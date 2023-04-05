
"use strict";

let CliffEvent = require('./CliffEvent.js');
let DockInfraRed = require('./DockInfraRed.js');
let DigitalInputEvent = require('./DigitalInputEvent.js');
let SensorState = require('./SensorState.js');
let ScanAngle = require('./ScanAngle.js');
let KeyboardInput = require('./KeyboardInput.js');
let VersionInfo = require('./VersionInfo.js');
let ControllerInfo = require('./ControllerInfo.js');
let BumperEvent = require('./BumperEvent.js');
let ButtonEvent = require('./ButtonEvent.js');
let Led = require('./Led.js');
let Sound = require('./Sound.js');
let WheelDropEvent = require('./WheelDropEvent.js');
let ExternalPower = require('./ExternalPower.js');
let RobotStateEvent = require('./RobotStateEvent.js');
let MotorPower = require('./MotorPower.js');
let DigitalOutput = require('./DigitalOutput.js');
let PowerSystemEvent = require('./PowerSystemEvent.js');
let AutoDockingResult = require('./AutoDockingResult.js');
let AutoDockingActionGoal = require('./AutoDockingActionGoal.js');
let AutoDockingActionFeedback = require('./AutoDockingActionFeedback.js');
let AutoDockingFeedback = require('./AutoDockingFeedback.js');
let AutoDockingGoal = require('./AutoDockingGoal.js');
let AutoDockingAction = require('./AutoDockingAction.js');
let AutoDockingActionResult = require('./AutoDockingActionResult.js');

module.exports = {
  CliffEvent: CliffEvent,
  DockInfraRed: DockInfraRed,
  DigitalInputEvent: DigitalInputEvent,
  SensorState: SensorState,
  ScanAngle: ScanAngle,
  KeyboardInput: KeyboardInput,
  VersionInfo: VersionInfo,
  ControllerInfo: ControllerInfo,
  BumperEvent: BumperEvent,
  ButtonEvent: ButtonEvent,
  Led: Led,
  Sound: Sound,
  WheelDropEvent: WheelDropEvent,
  ExternalPower: ExternalPower,
  RobotStateEvent: RobotStateEvent,
  MotorPower: MotorPower,
  DigitalOutput: DigitalOutput,
  PowerSystemEvent: PowerSystemEvent,
  AutoDockingResult: AutoDockingResult,
  AutoDockingActionGoal: AutoDockingActionGoal,
  AutoDockingActionFeedback: AutoDockingActionFeedback,
  AutoDockingFeedback: AutoDockingFeedback,
  AutoDockingGoal: AutoDockingGoal,
  AutoDockingAction: AutoDockingAction,
  AutoDockingActionResult: AutoDockingActionResult,
};
