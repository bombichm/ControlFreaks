/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegister;

/**
 * Register Op Modes
 */
public class FtcOpModeRegister implements OpModeRegister {

  /**
   * The Op Mode Manager will call this method when it wants a list of all
   * available op modes. Add your op mode to the list to enable it.
   *
   * @param manager op mode manager
   */
  public void register(OpModeManager manager) {

    /*
     * register your op modes here.
     * The first parameter is the name of the op mode
     * The second parameter is the op mode class property
     *
     * If two or more op modes are registered with the same name, the app will display an error.
     */

    //manager.register("NullOp", NullOp.class);

    //manager.register("MatrixK9TeleOp", MatrixK9TeleOp.class);
    //manager.register("K9TeleOp", K9TeleOp.class);
    //manager.register("K9TankDrive", K9TankDrive.class);
    //manager.register("HTRGBExample", HTRGBExample.class);
    manager.register ("Manual Mode", CFPushBotManual.class);
    manager.register ("Test Do Not Use", CFPushBotAuto.class);
    //manager.register ("Funny Dance", CFPushBotAuto_Funny.class);
    manager.register ("Blue 2 45 Climb Home", CFPushBotAuto_Blue2_ClimbHome_Short.class);
    manager.register ("Blue 2 Climb Visitor", CFPushBotAuto_Blue2_ClimbVisitors.class);
    manager.register ("Blue 3 Climb Visitor", CFPushBotAuto_Blue3_ClimbVisitors.class);
    //manager.register ("Blue 4 Climb Home Long", CFPushBotAuto_Blue4_ClimbHome_Long.class);
    manager.register ("Blue 4 Climb Home Short", CFPushBotAuto_Blue4_ClimbHome_Short.class);
    manager.register ("Blue 4 LightBar Long", CFPushBotAuto_Blue4_LightBar_Long.class);

    manager.register ("Red 2 Climb Home", CFPushBotAuto_Red2_ClimbHome_Short.class);
    manager.register ("Red 2 Climb Visitor", CFPushBotAuto_Red2_ClimbVisitors.class);
    manager.register ("Red 3 Climb Home", CFPushBotAuto_Red3_ClimbHome_Short.class);
    manager.register ("Red 3 Climb Visitor", CFPushBotAuto_Red3_ClimbVisitors.class);
    //manager.register ("Red 4 Climb Home Long", CFPushBotAuto_Red4_ClimbHome_Long.class);
    //manager.register ("Red 4 Climb Home Short", CFPushBotAuto_Red4_ClimbHome_Short.class);
    //manager.register ("Drive Forward 94 Inch", CFPushBotAuto_Blue2_ClimbHome_Short.class);
    //manager.register ("Red 4 LightBar Long", CFPushBotAuto_Red4_LightBar_Long.class);
    //manager.register("LCD Backpack Test", I2CLED7Seg.class);
    //manager.register("DetectColor", DetectColor.class);
    //manager.register("LinearDetectColor", LinearDetectColor.class);


    /*
     * Uncomment any of the following lines if you want to register an op mode.
     */
    //manager.register("MR Gyro Test", MRGyroTest.class);

    //manager.register("AdafruitRGBExample", AdafruitRGBExample.class);
    //manager.register("ColorSensorDriver", ColorSensorDriver.class);

    //manager.register("IrSeekerOp", IrSeekerOp.class);
    //manager.register("CompassCalibration", CompassCalibration.class);
    //manager.register("I2cAddressChangeExample", LinearI2cAddressChange.class);


    //manager.register("NxtTeleOp", NxtTeleOp.class);

    //manager.register("LinearK9TeleOp", LinearK9TeleOp.class);
    //manager.register("LinearIrExample", LinearIrExample.class);


    //manager.register ("PushBotManual1", PushBotManual1.class);
    //manager.register ("PushBotAutoSensors", PushBotAutoSensors.class);
    //manager.register ("PushBotIrEvent", PushBotIrEvent.class);

    //manager.register ("PushBotManualSensors", PushBotManualSensors.class);
    //manager.register ("PushBotOdsDetectEvent", PushBotOdsDetectEvent.class);
    //manager.register ("PushBotOdsFollowEvent", PushBotOdsFollowEvent.class);
    //manager.register ("PushBotTouchEvent", PushBotTouchEvent.class);

    //manager.register("PushBotDriveTouch", PushBotDriveTouch.java);
    //manager.register("PushBotIrSeek", PushBotIrSeek.java);
    //manager.register("PushBotSquare", PushBotSquare.java);
  }
}
