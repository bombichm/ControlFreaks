package com.qualcomm.ftcrobotcontroller.opmodes;

import android.media.ToneGenerator;

/**
 * Created by adevries on 11/6/2015.
 */
public class CFPushBotAuto extends CFPushBotTelemetry {

    boolean v_use_gyro = true;
    boolean v_turn_slow = false;
    int v_drive_inches = 12;
    int v_turn_degrees = 90;
    double v_drive_power = 1.0;
    //boolean v_drive_inches_use_gyro = true;

    //--------------------------------------------------------------------------
    //
    // CFPushBotAuto
    //
    /**
     * Construct the class.
     *
     * The system calls this member when the class is instantiated.
     */
    public CFPushBotAuto ()

    {
        //
        // Initialize base classes.
        //
        // All via self-construction.

        //
        // Initialize class members.
        //
        // All via self-construction.

    } // PushBotAuto

    //--------------------------------------------------------------------------
    //
    // start
    //
    /**
     * Perform any actions that are necessary when the OpMode is enabled.
     *
     * The system calls this member once when the OpMode is enabled.
     */
    @Override public void start ()

    {
        //
        // Call the PushBotHardware (super/base class) start method.
        //
        super.start();


    } // start

    //--------------------------------------------------------------------------
    //
    // loop
    //
    /**
     * Implement a state machine that controls the robot during auto-operation.
     * The state machine uses a class member and encoder input to transition
     * between states.
     *
     * The system calls this member repeatedly while the OpMode is running.
     */
    @Override public void loop ()

    {
        hardware_loop();
        //----------------------------------------------------------------------
        //
        // State: Initialize (i.e. state_0).
        //
        switch (v_state) {
            //
            // Synchronize the state machine and hardware.
            //
            case 0:
                //
                //
                if (gamepad1.left_bumper) {
                    //drive forward 12 or 48 inches
                    v_turn_slow = false;
                    v_drive_power = 1.0f;
                } else {
                    v_turn_slow = true;
                    v_drive_power = .5f;
                }

                if (gamepad1.right_bumper) {
                    //drive backward 12 or 48 inches
                    v_use_gyro = false;
                } else{
                    v_use_gyro = true;
                }
                if (gamepad1.b) {
                    //drive backward 12 or 48 inches
                    v_turn_degrees = 45;
                    v_drive_inches = 48;
                } else{
                    v_turn_degrees = 90;
                    v_drive_inches = 12;
                }
                if(gamepad1.x){
                    v_state=9;
                }
                if(gamepad1.y){
                    v_state=10;
                }
                if (gamepad1.dpad_up) {
                    //drive forward no gyro 12 or 48 inches
                    v_state = 1;
                }else if (gamepad1.dpad_down) {
                    //drive backward no gyro 12 or 48 inches
                    v_state = 5;
                }else if (gamepad1.dpad_left) {
                    //turn left no gyro 45 or 90
                    v_state = 3;
                }else if (gamepad1.dpad_right) {
                    //turn right no gyro 45 or 90
                    v_state = 7;
                }

                break;
            case 1:
                drive_inches(v_drive_power,v_drive_inches, v_use_gyro);
                v_state++;
            case 2:

                //
                // Transition to the next state when this method is called again.
                if (drive_inches_complete()) {
                    sound_play_dtmf(ToneGenerator.TONE_DTMF_2,500);
                    sleep(200);
                    set_second_message("drive " + v_drive_inches + " inches forward final h:" + sensor_gyro_get_heading() + ",re:" + a_right_encoder_count() + ",le:" + a_left_encoder_count());
                    v_state = 0;

                }


                break;

            case 3:
                // positive is right turn
                set_second_message("turn " + v_turn_degrees + " to the left");
                        turn_degrees(0 - v_turn_degrees, v_turn_slow, v_use_gyro);

                v_state++;
                break;
            //
            // Wait...
            //
            case 4:
                //keep checking if we have reached the distance we need to reach
                if (turn_complete ())
                {
                    //set_second_message("turn Complete");
                    sound_play_dtmf(ToneGenerator.TONE_DTMF_2,500);
                    sleep(200);
                    set_second_message("turn left " + v_turn_degrees + " final h:" + sensor_gyro_get_heading() + ",re:" + a_right_encoder_count() + ",le:" + a_left_encoder_count());
                    v_state=0;
                }
                break;

            case 5:
                //
                // drive backwards 12 inches
                //
                drive_inches(0-v_drive_power,v_drive_inches, v_use_gyro);
                v_state++;
                break;
            case 6:

                //
                // Transition to the next state when this method is called again.
                if (drive_inches_complete()) {
                    sound_play_dtmf(ToneGenerator.TONE_DTMF_2,500);
                    sleep(200);
                    set_second_message("drive backwards " + v_drive_inches + " final h:" + sensor_gyro_get_heading() + ",re:" + a_right_encoder_count() + ",le:" + a_left_encoder_count());
                    v_state=0;
                }

                break;
            case 7:
                // positive is right turn
                set_second_message("turn " + v_turn_degrees + " to the right");
                turn_degrees(v_turn_degrees, v_turn_slow, v_use_gyro);
                v_state++;
                break;
            //
            // Wait...
            //
            case 8:
                //keep checking if we have reached the distance we need to reach
                if (turn_complete ())
                {
                    sound_play_dtmf(ToneGenerator.TONE_DTMF_3, 500);
                    sleep(500);
                    set_second_message("turn right " + v_turn_degrees + " final h:" + sensor_gyro_get_heading() + ",re:" + a_right_encoder_count() + ",le:" + a_left_encoder_count());

                    v_state=0;
                }
                break;
            case 9:
                sound_play_dtmf(ToneGenerator.TONE_DTMF_3, 500);
                if (led7seg_is_enabled()){
                    led7seg_enabled(false);
                }else{

                    led7seg_timer_start(30);
                    sleep(10);
                    led7seg_enabled(true);
                }


                sleep(500);
                v_state=0;
                break;
            case 10:
                sound_play_dtmf(ToneGenerator.TONE_DTMF_3, 500);
                led7seg_test();
                sleep(500);
                v_state=0;
                break;
            default:
                //
                // The autonomous actions have been accomplished (i.e. the state has
                // transitioned into its final state.
                //
                break;
        }

        //
        // Send telemetry data to the driver station.
        //
        if(gamepad1.back){
            set_drive_power(0.0d, 0.0d);
            sound_play_dtmf(ToneGenerator.TONE_DTMF_2,500);
            sleep(200);
            set_second_message("cancel detected");
            v_state = 0;
        }
        if ((loopCounter() % 10) == 0) {
            update_telemetry(); // Update common telemetry
            telemetry.addData("18", "State: " + v_state);
        }

    } // loop

    //--------------------------------------------------------------------------
    //
    // v_state
    //
    /**
     * This class member remembers which state is currently active.  When the
     * start method is called, the state will be initialized (0).  When the loop
     * starts, the state will change from initialize to state_1.  When state_1
     * actions are complete, the state will change to state_2.  This implements
     * a state machine for the loop method.
     */
    private int v_state = 0;


}
