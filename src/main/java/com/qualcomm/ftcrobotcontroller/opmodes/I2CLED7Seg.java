package com.qualcomm.ftcrobotcontroller.opmodes;

import android.media.AudioManager;
import android.media.ToneGenerator;

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
    ToneGenerator v_tone_generator;
    @Override
    public void init() {

        // write some device information (connection info, name and type)
        // to the log file.
        hardwareMap.logDevices();

        try {
            //Connect the Core Interface Device or Dim
            v_tone_generator = new ToneGenerator(AudioManager.STREAM_RING, ToneGenerator.MAX_VOLUME);

            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_3, 250);
            sleep(300);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_3, 250);
            sleep(300);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_3, 250);
            sleep(500);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_3, 250);
            sleep(300);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_3, 250);
            sleep(300);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_3, 250);
            sleep(500);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_3, 250);
            sleep(300);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_9, 250);
            sleep(300);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_1, 250);
            sleep(300);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_2, 250);
            sleep(300);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_3, 250);
            sleep(500);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_6, 250);
            sleep(300);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_6, 250);
            sleep(300);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_6, 250);
            sleep(500);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_6, 250);
            sleep(300);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_6, 250);
            sleep(300);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_3, 250);
            sleep(300);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_3, 250);
            sleep(500);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_3, 250);
            sleep(300);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_3, 250);
            sleep(300);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_3, 250);
            sleep(300);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_2, 250);
            sleep(300);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_2, 250);
            sleep(300);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_3, 250);
            sleep(300);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_2, 250);
            sleep(500);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_9, 250);
            sleep(500);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_3, 250);
            sleep(300);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_3, 250);
            sleep(300);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_3, 250);
            sleep(500);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_3, 250);
            sleep(300);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_3, 250);
            sleep(300);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_3, 250);
            sleep(500);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_3, 250);
            sleep(300);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_9, 250);
            sleep(300);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_1, 250);
            sleep(300);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_2, 250);
            sleep(300);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_3, 250);
            sleep(500);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_6, 250);
            sleep(300);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_6, 250);
            sleep(300);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_6, 250);
            sleep(500);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_6, 250);
            sleep(300);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_6, 250);
            sleep(300);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_3, 250);
            sleep(300);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_3, 250);
            sleep(300);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_3, 250);
            sleep(300);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_3, 250);
            sleep(300);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_9, 250);
            sleep(300);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_9, 250);
            sleep(300);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_6, 250);
            sleep(300);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_2, 250);
            sleep(300);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_1, 250);
            /*sleep(600);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_0, 666);
            sleep(600);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_0, 6633);
            sleep(600);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_0, 3332232);
            v_tone_generator.wait(100);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_9, 9);
            sleep(600);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_0, 333);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_0, 333);
            sleep(600);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_0, 39123);
            sleep(600);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_0, 666);
            sleep(600);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_0, 6633);*/

        } catch (Exception p_exeception) {
            debugLogException("03", "led7seg", "missing", p_exeception);
        }
    }

    public void sleep(long time){
        try {
            Thread.sleep(time);
        }catch(Exception ex){

        }
    }
    boolean ranloop = false;
    @Override
    public void loop() {
        try {
            Thread.sleep(1000);
        }catch(Exception ex){

        }

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
