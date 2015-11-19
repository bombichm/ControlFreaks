package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by adevries on 11/6/2015.
 */
public class CFPushBotAuto_Blue4 extends CFPushBotTelemetry {

    //--------------------------------------------------------------------------
    //
    // CFPushBotAuto
    //
    /**
     * Construct the class.
     *
     * The system calls this member when the class is instantiated.
     */
    public CFPushBotAuto_Blue4()

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

        //
        // Reset the motor encoders on the drive wheels.
        //
        reset_drive_encoders ();

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
                // Reset the encoders to ensure they are at a known good value.
                //
                reset_drive_encoders ();
                v_state++;
                break;
            case 1:
                //
                // Reset the encoders to ensure they are at a known good value.
                //
                //reset_drive_encoders ();

                //
                // Transition to the next state when this method is called again.
                if (have_drive_encoders_reset()) {
                    //
                    v_state++;
                }

                break;
            //
            // Drive forward until the encoders exceed the specified values.
            //
            case 2:
                //Drive forward 74.5 inches Positive Power is forward
                drive_inches(1.0f,74.5f);
                v_state++;
                break;
            //
            // Wait...
            //
            case 3:
                //keep checking if we have reached the distance we need to reach
                if (drive_inches_complete ())
                {
                    v_state++;
                }
                break;
            case 4:
                //
                // Reset the encoders to ensure they are at a known good value.
                //
                reset_drive_encoders ();
                v_state++;
                break;
            case 5:
                //
                // Reset the encoders to ensure they are at a known good value.
                //
                //reset_drive_encoders ();

                //
                // Transition to the next state when this method is called again.
                if (have_drive_encoders_reset()) {
                    //
                    v_state++;
                }

                break;
            //
            // Turn right 90 degres.
            //
            case 6:
                // positive is right turn
               turn_degrees(90, true, true);
                v_state++;
                break;
            //
            // Wait...
            //
            case 7:
                if (turn_complete ());
                {
                    v_state++;
                }
                break;
//            //
//            // Turn right until the encoders exceed the specified values.
//            //
            /*case 6:
                drive_inches(1.0f,52.5f);
                {

                    v_state++;
                }
                break;

            case 7:
                if (drive_inches_complete())
                {
                    v_state++;
                }
               break;*/
//            //
//            // Perform no action - stay in this case until the OpMode is stopped.
//            // This method will still be called regardless of the state machine.
//            //
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
        update_telemetry (); // Update common telemetry
        telemetry.addData ("18", "State: " + v_state);

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
