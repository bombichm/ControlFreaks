package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

import org.usfirst.ControlFreaks.Wire;
import org.usfirst.ControlFreaks.AdafruitLEDBackpack7Seg;
import java.util.concurrent.locks.Lock;

/**
 * Created by adevries on 11/13/2015.
 */
public class I2CLED7Seg extends OpMode {
    //public HardwareMap hardwareMap = new HardwareMap();
    private DeviceInterfaceModule v_dim;
    private LED v_heartbeat;

    private AdafruitLEDBackpack7Seg v_ledseg;


    public I2CLED7Seg ()

    {
        //
        // Initialize base classes.
        //
        // All via self-construction.

        //
        // Initialize class members.
        //
        // All via self-construction.

    } // PushBotHardware

    @Override
    public void init() {

        // write some device information (connection info, name and type)
        // to the log file.
        hardwareMap.logDevices();

        try {
            //Connect the Core Interface Device or Dim

            try {

                // set up the hardware devices we are going to use
                v_dim = hardwareMap.deviceInterfaceModule.get("dim");
                v_dim.setLED(0, true);
                v_dim.setLED(1, true);

            } catch (Exception p_exeception) {
                debugLogException("00", "dim", "missing", p_exeception);

                v_dim = null;
            }

            try {

                // set up the hardware devices we are going to use
                v_heartbeat = hardwareMap.led.get("heartbeat");
                v_heartbeat.enable(true);

            } catch (Exception p_exeception) {
                debugLogException("01", "heartbeat", "missing", p_exeception);

                v_heartbeat = null;
            }

            try {
                v_ledseg = new AdafruitLEDBackpack7Seg(hardwareMap, "ledseg");
                v_ledseg.writeDigits(0);
            } catch (Exception p_exeception) {
                debugLogException("03", "led7seg", "missing", p_exeception);
            }
        } catch (Exception p_exeception) {
            debugLogException("03", "led7seg", "missing", p_exeception);
        }
    }

    boolean ranloop = false;
    @Override
    public void loop() {
        v_ledseg.loop();

        if (ranloop == false){
            //
            //Connect the Core Interface Device or Dim
            try {

                // set up the hardware devices we are going to use
                v_dim = hardwareMap.deviceInterfaceModule.get("dim");
                v_dim.setLED(1,true);

            }catch (Exception p_exeception)
            {
                debugLogException("00","dim","missing",p_exeception);

                v_dim = null;
            }
        }
            ranloop=true;
            //hardwareMap.wait(100);

    }
    @Override
    public void stop() {
        v_ledseg.stop();
    }
    void test(){


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
