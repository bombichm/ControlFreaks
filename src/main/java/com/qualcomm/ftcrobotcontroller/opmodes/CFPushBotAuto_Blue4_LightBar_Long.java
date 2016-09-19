package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by adevries on 11/6/2015.
 */
public class CFPushBotAuto_Blue4_LightBar_Long extends CFPushBotTelemetry {

    //--------------------------------------------------------------------------
    //
    // CFPushBotAuto
    //
    /**
     * Construct the class.
     *
     * The system calls this member when the class is instantiated.
     */
    public CFPushBotAuto_Blue4_LightBar_Long()

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
    boolean moveRpa = true;
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
                // drive Forward 12 inches
                //
                led7seg_timer_start(30);
                //this is offset for articulating arm which we removed drive_inches(.7f,74.5f, true);

                drive_inches(.7f,12, true);
                v_state++;
                break;
            case 1:

                //
                // Transition to the next state when this method is called again.
                if (moveRpa == true){
                    m_rpabase_position(.45);
                    moveRpa = false;
                }

                if (drive_inches_complete()) {
                    //
                    v_state++;
                }

                break;

            case 2:
                // positive is right turn

                turn_degrees(30, true, true);
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

                drive_inches(.7f,74, true);

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

                turn_degrees(60, true, true);
                set_second_message("turn 60 degrees to the right");
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
                m_rpabase_position(RPABaseServo_DumpPosition - .1);
                drive_inches(.7f,15, true);

                //set_drive_power(1.0d, 1.0d);
                v_state++;
                break;
            case 9:
                //
                // Transition to the next state when this method is called again.
                if (drive_inches_complete()) {
                    //
                    v_state++;
                }
                break;
            case 10:
                rpabase_moveToDump();
                v_rpa_move_delay = 300 + loopCounter();
                v_state++;
                break;
            case 11:
                if (v_rpa_move_delay < loopCounter()) {
                    //
                    ;
                    v_rpa_move_delay = 300 + loopCounter();
                    v_state++;
                }

                break;
            case 12:

                rpaarm_moveUp(true);
                if (v_rpa_move_delay < loopCounter() || rpa_arm_extended()) {
                    //
                    m_rpa_arm_power(0.0f);
                    m_servo_dump_climber_position(v_servo_dump_climbers_MaxPosition);

                    v_rpa_move_delay = 100 + loopCounter();
                    v_state++;
                }
                break;
            case 13:
                if (v_rpa_move_delay < loopCounter()) {
                    //
                    rpaarm_moveDown(true);
                    m_servo_dump_climber_position(v_servo_dump_climbers_MinPosition);
                    m_rpabase_position(RPABaseServo_DumpPosition - .1);
                    v_rpa_move_delay = 500 + loopCounter();
                    v_state++;
                }
                break;
            case 14:
                if (v_rpa_move_delay < loopCounter() || rpa_arm_retracted()) {
                    //
                    m_rpa_arm_power(0.0f);

                    v_state++;
                }
                break;
            case 15:
                drive_inches(-.7f,15, true);

                //set_drive_power(1.0d, 1.0d);
                v_state++;
                break;
            case 16:
                //
                // Transition to the next state when this method is called again.
                if (drive_inches_complete()) {
                    //
                    v_state++;
                }
                break;
            case 17:
                // positive is right turn
                turn_degrees(-45, true, true);
                m_rpabase_position(.45);
                set_second_message("turn 45 degrees to the right");
                v_state++;
                break;
            //
            // Wait...
            //
            case 18:
                //keep checking if we have reached the distance we need to reach
                if (turn_complete ())
                {
                    set_second_message("turn Complete");
                    v_state++;
                }
                break;

            case 19:

                drive_inches(-.7f,32, true);

                //set_drive_power(1.0d, 1.0d);
                v_state++;
                break;
            case 20:
                //
                // Transition to the next state when this method is called again.
                if (drive_inches_complete()) {
                    //
                    v_state++;
                }
                break;

            case 21:
                // positive is right turn
                turn_degrees(90, true, true);
                set_second_message("turn 90 degrees to the right");
                v_state++;
                break;
            //
            // Wait...
            //
            case 22:
                //keep checking if we have reached the distance we need to reach
                if (turn_complete ())
                {
                    set_second_message("turn Complete");
                    v_state++;
                }
                break;
            case 23:
                rpabase_moveToClimb();
                drive_inches(.7f,90, true);

                //set_drive_power(1.0d, 1.0d);
                v_state++;
                break;
            case 24:
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
