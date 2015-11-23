package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by adevries on 11/6/2015.
 */
public class CFPushBotAuto_Funny extends CFPushBotTelemetry {

    //--------------------------------------------------------------------------
    //
    // CFPushBotAuto
    //
    /**
     * Construct the class.
     *
     * The system calls this member when the class is instantiated.
     */
    public CFPushBotAuto_Funny()

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
        run_using_encoders();
        //
        // Reset the motor encoders on the drive wheels.
        //
        //reset_drive_encoders ();
        //run_without_drive_encoders();
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
                // drive Forward 12 inches
                //
                drive_inches(1.0f, 12, true);

                //set_drive_power(1.0d, 1.0d);
                v_state++;
                break;
            case 1:
                if(drive_inches_complete()) {
                    v_state++;
                }
                break;

            case 2:
                //
                // drive Forward 12 inches
                //
                //drive_inches(1.0f,12, true);

                sleep(2000);
                v_state++;
                break;
            case 3:
//
                // drive Forward 12 inches
                //
                drive_inches(1.0f, 12, true);

                //set_drive_power(1.0d, 1.0d);
                v_state++;
                break;
            case 4:
                if(drive_inches_complete()) {
                    v_state++;
                }
                break;
            case 5:

                //
                // Transition to the next state when this method is called again.
                sleep(1000);
                v_state++;


                break;

            case 6:
                //
                // drive Forward 12 inches
                //
                drive_inches(1.0f, 12, true);

                //set_drive_power(1.0d, 1.0d);
                v_state++;
                break;
            case 7:
                if(drive_inches_complete()) {
                    v_state++;
                }
                break;
//            //
//            // Wait...
//            //
//            case 5:
//                //keep checking if we have reached the distance we need to reach
//                if (loopCounter() % 50 == 0)
//                {
//                    set_second_message("Flip Down");
//                    v_state++;
//                }
//                break;
//            case 6:
//                // positive is right turn
//                // positive is right turn
//                m_flip_left_position(FlipLeftServo_MinPosition+.2);
//                m_flip_right_position(FlipRightServo_MaxPosition - .2);
//                v_state++;
//
//                break;
//
//            case 7:
//            // positive is right turn
//                if (loopCounter() % 50 == 0)
//                {
//                    set_second_message("reset");
//                    v_state = 2;
//                }
//                break;

            default:
                //
                // The autonomous actions have been accomplished (i.e. the state has
                // transitioned into its final state.
                //
                sleep(100);
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
