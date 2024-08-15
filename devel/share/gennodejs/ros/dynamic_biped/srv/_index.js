
"use strict";

let changeAMBACCtrlMode = require('./changeAMBACCtrlMode.js')
let srvchangeCtlMode = require('./srvchangeCtlMode.js')
let changeArmCtrlMode = require('./changeArmCtrlMode.js')
let srvChangePhases = require('./srvChangePhases.js')
let srvClearPositionCMD = require('./srvClearPositionCMD.js')
let srvChangeJoller = require('./srvChangeJoller.js')
let controlEndHand = require('./controlEndHand.js')

module.exports = {
  changeAMBACCtrlMode: changeAMBACCtrlMode,
  srvchangeCtlMode: srvchangeCtlMode,
  changeArmCtrlMode: changeArmCtrlMode,
  srvChangePhases: srvChangePhases,
  srvClearPositionCMD: srvClearPositionCMD,
  srvChangeJoller: srvChangeJoller,
  controlEndHand: controlEndHand,
};
