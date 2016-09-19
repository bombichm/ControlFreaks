package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by adevries on 11/6/2015.
 */
public class CFPushBotAuto_Blue4_ClimbHome_Short extends CFPushBotTelemetry {

    //--------------------------------------------------------------------------
    //
    // CFPushBotAuto
    //
    /**
     * Construct the class.
     *
     * The system calls this member when the class is instantiated.
     */
    public CFPushBotAuto_Blue4_ClimbHome_Short()

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
                drive_inches(.7f,30, true);
                v_state++;
                break;
            case 1:

                //
                // Transition to the next state when this method is called again.
                if (drive_inches_complete()) {
                    //
                    v_state++;
                }

                break;

            case 2:
                // positive is right turn
                turn_degrees(45, true, true);
                set_second_message("turn 45 degrees to the right");
                v_state++;
                break;
            //
            // Wait...
            //
            case 3:
                //keep checking if we have reached the distance we need to reach
                if (turn_complete ())
                {
                    set_second_message("turn Complete");
                    v_state++;
                }
                break;
            case 4:
                //
                // drive Forward 12 inches
                //
                drive_inches(.7f,30, true);

                //set_drive_power(1.0d, 1.0d);
                v_state++;
                break;
            case 5:
                //
                // Transition to the next state when this method is called again.
                if (drive_inches_complete()) {
                    //
                    v_state++;
                }
                break;
            case 6:
                // positive is right turn
                turn_degrees(87, true, true);
                set_second_message("turn 90 degrees to the right");
                v_state++;
                break;
            //
            // Wait...
            //
            case 7:
                //keep checking if we have reached the distance we need to reach
                if (turn_complete ())
                {
                    set_second_message("turn Complete");
                    v_state++;
                }
                break;
            case 8:
                //
                // drive Forward 12 inches
                //
                //drive_inches(.7f,10, true);
                rpabase_moveToClimb();
                v_rpa_move_delay = 250 + loopCounter();
                //set_drive_power(1.0d, 1.0d);
                v_state++;
                break;
            case 9:
                //
                // Transition to the next state when this method is called again.
                if (v_rpa_move_delay < loopCounter()) {
                    //
                    v_state++;
                }
                break;
            case 10:
                //
                // drive Forward 12 inches
                //
                drive_inches(.7f,60, true);

                //set_drive_power(1.0d, 1.0d);
                v_state++;
                break;
            case 11:
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
