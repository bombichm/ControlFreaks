package org.usfirst.ControlFreaks;

import android.location.Address;
import android.util.Log;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by adevries on 11/16/2015.
 */
public class AdafruitLEDBackpack7Seg {
    static final String logId   = "Ada7Seg:";       // Tag identifier in FtcRobotController.LogCat
    private String v_deviceName = "Not Set";
    private byte I2CAddress = 0x70;
    private Wire v_ledseg;
    private final static byte numbertable[] = {
            0x3F, /* 0 */
            0x06, /* 1 */
            0x5B, /* 2 */
            0x4F, /* 3 */
            0x66, /* 4 */
            0x6D, /* 5 */
            0x7D, /* 6 */
            0x07, /* 7 */
            0x7F, /* 8 */
            0x6F, /* 9 */
            0x77, /* a */
            0x7C, /* b */
            0x39, /* C */
            0x5E, /* d */
            0x79, /* E */
            0x71, /* F */
    };
    public AdafruitLEDBackpack7Seg (HardwareMap hardwareMap, String deviceName, byte RealI2CAddress) throws Exception
    {
        try{
            I2CAddress = RealI2CAddress;
            v_deviceName = deviceName;
            v_ledseg = new Wire(hardwareMap,deviceName,I2CAddress * 2);
            init();
        }catch (Exception p_exeception)
        {
            debugLogException("Error Creating Wire", p_exeception);
            v_ledseg = null;
            throw p_exeception;
        }
    }
    public AdafruitLEDBackpack7Seg (HardwareMap hardwareMap, String deviceName) throws Exception

    {
        //
        // Initialize base classes.
        //
        // All via self-construction.

        //
        // Initialize class members.
        //
        // All via self-construction.
        try{
            v_deviceName = deviceName;
            v_ledseg = new Wire(hardwareMap,deviceName,I2CAddress * 2);
            init();

        }catch (Exception p_exeception)
        {
            debugLogException("Error Creating Wire", p_exeception);
            v_ledseg = null;
        }

    }

    private void init() throws Exception{
        try{
            if (v_ledseg != null) {
                v_ledseg.write(0x21, 0);
                v_ledseg.write(0x81, 0);
                v_ledseg.beginWrite(0x00);
                v_ledseg.write(numbertable[0]);
                v_ledseg.write(numbertable[0]);
                v_ledseg.write(numbertable[0]);
                v_ledseg.write(numbertable[0]);
                v_ledseg.endWrite();
            }
        }catch (Exception p_exeception)
        {
            debugLogException("Error Init", p_exeception);
            throw p_exeception;
        }
    }
    /**
     * put this in your loop so its called over and over in the loop
     */
    public void loop(){

    }

    private boolean v_timer_mode = false;

    private int v_seconds = 0;
    public boolean startTimer(int seconds){
        if (v_timer_mode == false) {
            v_seconds = seconds;
            return true;
        }else{
            debugLogException("timer already running call stopTimer() or wait for it to finish", null);
            return false;
        }
    }

    public boolean writeDigits(int number){
        v_ledseg.beginWrite(0x00);
        v_ledseg.write(numbertable[0]);
        v_ledseg.write(numbertable[1]);
        v_ledseg.write(numbertable[2]);
        v_ledseg.write(numbertable[3]);
        v_ledseg.endWrite();
        return true;
    }

    public void stop() {
        v_ledseg.close();
    }
    void debugLogException( String msg, Exception ex){

        String debugMessage = logId + msg;
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
        //telemetry.addData(line, debugMessage);
    }
}
