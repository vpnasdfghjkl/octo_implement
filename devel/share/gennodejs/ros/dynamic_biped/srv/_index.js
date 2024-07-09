
"use strict";

let changeArmCtrlMode = require('./changeArmCtrlMode.js')
let srvChangePhases = require('./srvChangePhases.js')
let changeAMBACCtrlMode = require('./changeAMBACCtrlMode.js')
let srvClearPositionCMD = require('./srvClearPositionCMD.js')
let srvchangeCtlMode = require('./srvchangeCtlMode.js')
let controlEndHand = require('./controlEndHand.js')
let srvChangeJoller = require('./srvChangeJoller.js')

module.exports = {
  changeArmCtrlMode: changeArmCtrlMode,
  srvChangePhases: srvChangePhases,
  changeAMBACCtrlMode: changeAMBACCtrlMode,
  srvClearPositionCMD: srvClearPositionCMD,
  srvchangeCtlMode: srvchangeCtlMode,
  controlEndHand: controlEndHand,
  srvChangeJoller: srvChangeJoller,
};
