package com.qualcomm.ftcrobotcontroller.opmodes;

import android.media.ToneGenerator;

/**
 * Created by adevries on 11/6/2015.
 */
public class CFPushBotAuto_Blue2_ClimbVisitors extends CFPushBotTelemetry {

    //--------------------------------------------------------------------------
    //
    // CFPushBotAuto
    //
    /**
     * Construct the class.
     *
     * The system calls this member when the class is instantiated.
     */
    public CFPushBotAuto_Blue2_ClimbVisitors()

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
        super.start ();
        blueled_on();
        led7seg_timer_init(30);

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
    long  v_rpa_move_delay;
    @Override public void loop ()

    {
        hardware_loop();
        //----------------------------------------------------------------------
        //
        // State: Initialize (i.e. state_0).
        //
        if (led7seg_timer_complete()== true){
            set_second_message("timer complete stop");
            set_drive_power(0f,0f);
            v_state = 100;
        }
        switch (v_state)
        {
            //
            // Synchronize the state machine and hardware.
            //
            case 0:
                //
                // drive Forward 24 inches
                //

                led7seg_timer_start(30);
                v_state++;


                break;
            case 1:
                /*if(loopCounter() > 1400) {
                    v_state++;
                }*/
                timewait(9);
                v_state++;
                break;
            case 2:
                if(timewait_Complete()== true){
                    v_state++;
                }
                break;
            case 3:
                //
                // drive Forward 24 inches
                //


                // 28 inches  was 85
                drive_inches(.7f,85, true);
                v_state++;
                break;
            case 4:

                //
                // Transition to the next state when this method is called again.
                if (drive_inches_complete()) {
                    //
                    sound_play_dtmf(ToneGenerator.TONE_DTMF_2,500);
                    v_state++;
                }

                break;

            case 5:
                // positive is right turn
                turn_degrees(-45, true, true);
                set_second_message("turn 45 degrees to the left");
                v_state++;
                break;
            //
            // Wait...
            //
            case 6:
                //keep checking if we have reached the distance we need to reach
                if (turn_complete ())
                {
                    sound_play_dtmf(ToneGenerator.TONE_DTMF_2,500);
                    set_second_message("turn Complete");
                    v_state++;
                }
                break;
            case 7:
                //
                // drive Forward 12 inches
                //
                //drive_inches(.7f,10, true);
                rpabase_moveToClimb();
                v_rpa_move_delay = 250 + loopCounter();
                //set_drive_power(1.0d, 1.0d);
                v_state++;
                break;
            case 8:
                //
                // Transition to the next state when this method is called again.
                if (v_rpa_move_delay < loopCounter()) {
                    //
                    v_state++;
                }
                break;
            case 9:
                //
                // drive Forward 12 inches
                //

                //this drives to high spot 105 inches drive_inches(.7f,105, true);
                //70 midle
                drive_inches(.7f,106, true);
                //set_drive_power(1.0d, 1.0d);
                v_state++;
                break;
            case 10:
                //
                // Transition to the next state when this method is called again.
                if (drive_inches_complete()) {
                    //
                    v_state++;
                }
                break;
            default:
                //
                // The autonomous actions have been accomplished (i.e. the state has
                // transitioned into its final state.
                //
                sleep(10);
                break;
        }

        //
        // Send telemetry data to the driver station.
        //
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
