package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by adevries on 11/6/2015.
 */
public class CFPushBotAuto_Red4_LightBar_Long extends CFPushBotTelemetry {

    //--------------------------------------------------------------------------
    //
    // CFPushBotAuto
    //
    /**
     * Construct the class.
     *
     * The system calls this member when the class is instantiated.
     */
    public CFPushBotAuto_Red4_LightBar_Long()

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
        redled_on();
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
    @Override public void loop ()

    {
        hardware_loop();
        //----------------------------------------------------------------------
        //
        // State: Initialize (i.e. state_0).
        //
        switch (v_state)
        {
            //
            // Synchronize the state machine and hardware.
            //
            case 0:
                //
                // drive Forward 24 inches
                //
                led7seg_timer_init(30);
                drive_inches(1.0f,74.5f, true);
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
                turn_degrees(-88, false, true);
                set_second_message("turn 88 degrees to the left");
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
                drive_inches(1.0f,47, true);

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
