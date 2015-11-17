/* Copyright (c) 2015 Qualcomm Technologies Inc

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

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.LED;

import java.util.concurrent.locks.Lock;

/*
 *
 * This is an example LinearOpMode that shows how to use
 * the Adafruit RGB Sensor.  It assumes that the I2C
 * cable for the sensor is connected to an I2C port on the
 * Core Device Interface Module.
 *
 * It also assuems that the LED pin of the sensor is connected
 * to the digital signal pin of a digital port on the
 * Core Device Interface Module.
 *
 * You can use the digital port to turn the sensor's onboard
 * LED on or off.
 *
 * The op mode assumes that the Core Device Interface Module
 * is configured with a name of "dim" and that the Adafruit color sensor
 * is configured as an I2C device with a name of "color".
 *
 * It also assumes that the LED pin of the RGB sensor
 * is connected to the signal pin of digital port #5 (zero indexed)
 * of the Core Device Interface Module.
 *
 * You can use the X button on either gamepad to turn the LED on and off.
 *
 */
public class AdafruitRGBExample extends LinearOpMode {

  ColorSensor sensorRGB;
  //DeviceInterfaceModule cdim;
  private DeviceInterfaceModule v_dim;
  private LED v_heartbeat;
  private I2cDevice v_i2c_led7seg;
  private static final int v_i2c_led7seg_port = 1;
  private static final int v_i2c_led7seg_address = 0xe0;
  private byte[] v_i2c_led7seg_readCache;
  private Lock v_i2c_led7seg_readLock;
  private byte[] v_i2c_led7seg_writeCache;
  private Lock v_i2c_led7seg_writeLock;

  // we assume that the LED pin of the RGB sensor is connected to
  // digital port 5 (zero indexed).
  static final int LED_CHANNEL = 5;

  @Override
  public void runOpMode() throws InterruptedException {

    // write some device information (connection info, name and type)
    // to the log file.
    hardwareMap.logDevices();

    // get a reference to our DeviceInterfaceModule object.
    v_dim = hardwareMap.deviceInterfaceModule.get("dim");

    // set the digital channel to output mode.
    // remember, the Adafruit sensor is actually two devices.
    // It's an I2C sensor and it's also an LED that can be turned on or off.
    v_dim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);

    // get a reference to our ColorSensor object.
    sensorRGB = hardwareMap.colorSensor.get("color2");

    // bEnabled represents the state of the LED.
    boolean bEnabled = true;

    // turn the LED on in the beginning, just so user will know that the sensor is active.
    v_dim.setDigitalChannelState(LED_CHANNEL, bEnabled);

    // wait one cycle.
    waitOneFullHardwareCycle();

    // wait for the start button to be pressed.
    waitForStart();

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F,0F,0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    // get a reference to the RelativeLayout so we can change the background
    // color of the Robot Controller app to match the hue detected by the RGB sensor.
    final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);

    // bPrevState and bCurrState represent the previous and current state of the button.
    boolean bPrevState = false;
    boolean bCurrState = false;
    try {
      v_i2c_led7seg =  hardwareMap.i2cDevice.get("ledseg"); // = new I2cDevice(v_dim,v_i2c_led7seg_port);
      v_i2c_led7seg_readCache = v_i2c_led7seg.getI2cReadCache();
      v_i2c_led7seg_readLock = v_i2c_led7seg.getI2cReadCacheLock();
      v_i2c_led7seg_writeCache = v_i2c_led7seg.getI2cWriteCache();
      v_i2c_led7seg_writeLock = v_i2c_led7seg.getI2cWriteCacheLock();
      //enable Oclator
      try {
        v_i2c_led7seg_writeLock.lock();
        v_i2c_led7seg_writeCache[0] = 0;
        v_i2c_led7seg_writeCache[1] = (byte) v_i2c_led7seg_address;
        v_i2c_led7seg_writeCache[2] = (byte) (0x21);  // | 0x80
        v_i2c_led7seg_writeCache[3] = 0;
        v_i2c_led7seg_writeCache[31] = -1;
      }catch (Exception p_exeception)
      {
        debugLogException("03", "led7seg", "Enable Oscolator", p_exeception);
      }finally {
        v_i2c_led7seg_writeLock.unlock();
      }
      v_i2c_led7seg.writeI2cCacheToController();

      //Enable Display 0x80 | 0x01  to enableDisplay
      try {
        v_i2c_led7seg_writeLock.lock();
        v_i2c_led7seg_writeCache[0] = 0;
        v_i2c_led7seg_writeCache[1] = (byte) v_i2c_led7seg_address;
        v_i2c_led7seg_writeCache[2] = (byte) (0x81);  // Enable Display
        v_i2c_led7seg_writeCache[3] = 0;
        v_i2c_led7seg_writeCache[31] = -1;
      }catch (Exception p_exeception)
      {
        debugLogException("04", "i2c_led7seg", "Enable Display", p_exeception);
      }finally {
        v_i2c_led7seg_writeLock.unlock();
      }
      v_i2c_led7seg.writeI2cCacheToController();
              /*
              //Write 0000 to the display
              try {
                  v_i2c_led7seg_writeLock.lock();
                  v_i2c_led7seg_writeCache[0] = 0;
                  v_i2c_led7seg_writeCache[1] = (byte) v_i2c_led7seg_address;
                  v_i2c_led7seg_writeCache[2] = (byte) (0x00);  // Enable Display
                  v_i2c_led7seg_writeCache[3] = (byte)2;
                  v_i2c_led7seg_writeCache[4] = (byte)0x3F;
                  v_i2c_led7seg_writeCache[5] = (byte)0x00;
                  v_i2c_led7seg_writeCache[31] = -1;
              }catch (Exception p_exeception)
              {
                  debugLogException("05", "i2c_led7seg", "missing", p_exeception);
              }finally {
                  v_i2c_led7seg_writeLock.unlock();
              }
              v_i2c_led7seg.writeI2cCacheToController();
              */
  }catch (Exception p_exeception)
  {
    debugLogException("06", "i2c_led7seg", "missing", p_exeception);
    v_i2c_led7seg = null;
  }
    // while the op mode is active, loop and read the RGB data.
    // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
    while (opModeIsActive()) {
      // check the status of the x button on either gamepad.
      bCurrState = gamepad1.x || gamepad2.x;

      // check for button state transitions.
      if (bCurrState == true && bCurrState != bPrevState)  {
        // button is transitioning to a pressed state.

        // print a debug statement.
        DbgLog.msg("MY_DEBUG - x button was pressed!");

        // update previous state variable.
        bPrevState = bCurrState;

        // on button press, enable the LED.
        bEnabled = true;

        // turn on the LED.
        v_dim.setDigitalChannelState(LED_CHANNEL, bEnabled);

      } else if (bCurrState == false && bCurrState != bPrevState)  {
        // button is transitioning to a released state.

        // print a debug statement.
        DbgLog.msg("MY_DEBUG - x button was released!");

        // update previous state variable.
        bPrevState = bCurrState;

        // on button press, enable the LED.
        bEnabled = false;

        // turn off the LED.
        v_dim.setDigitalChannelState(LED_CHANNEL, bEnabled);
      }

      // convert the RGB values to HSV values.
      Color.RGBToHSV((sensorRGB.red() * 255) / 800, (sensorRGB.green() * 255) / 800, (sensorRGB.blue() * 255) / 800, hsvValues);

      // send the info back to driver station using telemetry function.
      telemetry.addData("Clear", sensorRGB.alpha());
      telemetry.addData("Red  ", sensorRGB.red());
      telemetry.addData("Green", sensorRGB.green());
      telemetry.addData("Blue ", sensorRGB.blue());
      telemetry.addData("Hue", hsvValues[0]);

      // change the background color to match the color detected by the RGB sensor.
      // pass a reference to the hue, saturation, and value array as an argument
      // to the HSVToColor method.
      relativeLayout.post(new Runnable() {
        public void run() {
          relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
        }
      });

      // wait a hardware cycle before iterating.
      waitOneFullHardwareCycle();
    }
  }
  void debugLogException(String line, String type, String msg, Exception ex){

    String debugMessage = type + ":" + msg;
    if (ex != null) {
      String errMsg = ex.getLocalizedMessage();
      if (errMsg != null) {
        debugMessage = debugMessage + errMsg;
      }else{
        debugMessage = debugMessage + " error. is null";
      }
    }else{
      debugMessage = debugMessage + " error is null";
    }

    DbgLog.msg(debugMessage);
    telemetry.addData(line, debugMessage);
  }
}
