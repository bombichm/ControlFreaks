package com.qualcomm.ftcrobotcontroller.opmodes;

//------------------------------------------------------------------------------
//
// PushBotManual
//
/**
 * Provide a basic manual operational mode that uses the left and right
 * drive motors, left arm motor, servo motors and gamepad input from two
 * gamepads for the Push Bot.
 *
 * @author SSI Robotics
 * @version 2015-08-01-06-01
 */
public class PushBotManual extends PushBotTelemetry

{
    //--------------------------------------------------------------------------
    //
    // PushBotManual
    //
    /**
     * Construct the class.
     *
     * The system calls this member when the class is instantiated.
     */

    private static boolean bothControllersEnabled = false;
    public PushBotManual ()

    {
        //
        // Initialize base classes.
        //
        // All via self-construction.

        //
        // Initialize class members.
        //
        // All via self-construction.

    } // PushBotManual

    //--------------------------------------------------------------------------
    //
    // loop
    //
    /**
     * Implement a state machine that controls the robot during
     * manual-operation.  The state machine uses gamepad input to transition
     * between states.
     *
     * The system calls this member repeatedly while the OpMode is running.
     */
    @Override public void loop ()

    {
        //----------------------------------------------------------------------
        //
        // DC Motors
        //
        // Obtain the current values of the joystick controllers.
        //
        // Note that x and y equal -1 when the joystick is pushed all of the way
        // forward (i.e. away from the human holder's body).
        //
        // The clip method guarantees the value never exceeds the range +-1.
        //
        // The DC motors are scaled to make it easier to control them at slower
        // speeds.
        //
        // The setPower methods write the motor power values to the DcMotor
        // class, but the power levels aren't applied until this method ends.
        //

        //
        // Manage the drive wheel motors.
        //
        float l_left_drive_power = scale_motor_power (-gamepad1.left_stick_y);
        float l_right_drive_power = scale_motor_power (-gamepad1.right_stick_y);

        set_drive_power (l_left_drive_power, l_right_drive_power);

        //
        // Manage the arm motor.
        //
        //m_left_arm_power (l_left_arm_power);

        //float l_left_arm_power = scale_motor_power (-gamepad2.left_stick_y);
        //----------------------------------------------------------------------
        //
        // Servo Motors
        //
        // Obtain the current values of the gamepad 'x' and 'b' buttons.
        //
        // Note that x and b buttons have boolean values of true and false.
        //
        // The clip method guarantees the value never exceeds the allowable range of
        // [0,1].
        //
        // The setPosition methods write the motor power values to the Servo
        // class, but the positions aren't applied until this method ends.
        //

        if (gamepad2.dpad_right || (gamepad1.dpad_right && bothControllersEnabled) )
        {
            //arm_wrist_moveRight(gamepad2.left_bumper);
            arm_shoulder_moveUp((gamepad2.left_bumper || (gamepad1.left_bumper && bothControllersEnabled)));
        }
        else if (gamepad2.dpad_left || (gamepad1.dpad_left && bothControllersEnabled))
        {

            arm_shoulder_moveDown(gamepad2.left_bumper || (gamepad1.left_bumper && bothControllersEnabled) );
        }


        if (gamepad2.dpad_up || ( gamepad1.dpad_up && bothControllersEnabled))
        {

            arm_elbow_moveUp(gamepad2.left_bumper || (gamepad1.left_bumper && bothControllersEnabled));

        }
        else if (gamepad2.dpad_down || (gamepad1.dpad_down && bothControllersEnabled))
        {
            arm_elbow_moveDown(gamepad2.left_bumper || (gamepad1.left_bumper && bothControllersEnabled));
        }
        if (gamepad2.b || (gamepad1.b && bothControllersEnabled))
        {
            //move RPABase Servo in the up direction in left bumper down move fast
            rpabase_moveUp(gamepad2.left_bumper || (gamepad1.left_bumper && bothControllersEnabled));
        }
        else if (gamepad2.x || (gamepad1.x && bothControllersEnabled))
        {
            //move RPABase Servo in the up direction in left bumper down move fast
            rpabase_moveDown(gamepad2.left_bumper || (gamepad1.left_bumper && bothControllersEnabled));
        }
        if (rpa_arm_extended()){
            m_rpa_arm_power(0.0f);
        }
        if (rpa_arm_retracted()){
            m_rpa_arm_power(0.0f);
        }
        if (gamepad2.y || (gamepad2.y && bothControllersEnabled))
        {
            rpaarm_moveUp(gamepad2.left_bumper || (gamepad1.left_bumper && bothControllersEnabled));
        }
        else if (gamepad2.a || (gamepad1.a && bothControllersEnabled))
        {
            //move RPABase Servo in the up direction in left bumper down move fast
            rpaarm_moveDown(gamepad2.left_bumper || (gamepad1.left_bumper && bothControllersEnabled));
        }else{
            m_rpa_arm_power(0.0f);
        }

        if (gamepad2.left_trigger > ArmWristTrigger_Threshold_Fast || (bothControllersEnabled && gamepad1.left_trigger > ArmWristTrigger_Threshold_Fast) ){
            arm_wrist_moveLeft(true);
        }else if(gamepad2.left_trigger > ArmWristTrigger_Threshold || (bothControllersEnabled && gamepad1.left_trigger > ArmWristTrigger_Threshold) ){
            arm_wrist_moveLeft(false);
        }else if (gamepad2.right_trigger > ArmWristTrigger_Threshold || (bothControllersEnabled && gamepad1.right_trigger > ArmWristTrigger_Threshold_Fast) ){
            arm_wrist_moveRight(true);
        }else if(gamepad2.right_trigger > ArmWristTrigger_Threshold || (bothControllersEnabled && gamepad1.right_trigger > ArmWristTrigger_Threshold) ){
            arm_wrist_moveRight(false);
        }
        //
        // Send telemetry data to the driver station.
        //
        update_telemetry (); // Update common telemetry
        update_gamepad_telemetry ();

    } // loop

} // PushBotManual
