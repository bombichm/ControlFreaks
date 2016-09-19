package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by adevries on 11/6/2015.
 */
public class CFPushBotManual extends CFPushBotTelemetry{

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
    private byte v_neopixels_mode = 0;
    public CFPushBotManual ()

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
    boolean isMovingArm = false;
    float stickdeadzone = .1f;
    boolean isFirstTimeButtonPress = true;
    @Override public void loop ()

    {
        hardware_loop();
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

        if (isFirstTimeButtonPress && (gamepad1.left_stick_y < 0 || gamepad1.left_stick_y > 0 || gamepad1.right_stick_y < 0 || gamepad1.right_stick_y > 0 )){
            manualModeButtonPress();
            isFirstTimeButtonPress = false;
        }
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

        /*if (gamepad2.dpad_right )
        {
            //arm_wrist_moveRight(gamepad2.left_bumper);
            arm_shoulder_moveUp(!gamepad2.left_bumper);

        }
        else if (gamepad2.dpad_left )
        {
            arm_shoulder_moveDown(!gamepad2.left_bumper);
        }


        if (gamepad2.dpad_up )
        {
            arm_elbow_moveUp(!gamepad2.left_bumper);
        }
        else if (gamepad2.dpad_down )
        {
            arm_elbow_moveDown(!gamepad2.left_bumper);
        }*/

        if (gamepad2.dpad_up )
        {
            set_second_message("dpad up");
            m_servo_dump_climber_position(v_servo_dump_climbers_MinPosition);
        }
        else if (gamepad2.dpad_down )
        {
            set_second_message("dpad down");
            m_servo_dump_climber_position(v_servo_dump_climbers_MaxPosition);
        }

        if (gamepad2.b || gamepad2.left_stick_x > stickdeadzone )
        {
            //move RPABase Servo in the up direction in left bumper down move fast
            rpabase_moveUp(!gamepad2.left_bumper);
            isMovingArm = true;
        }
        else if (gamepad2.x || gamepad2.left_stick_x < (0 - stickdeadzone) )
        {
            isMovingArm = true;
            //move RPABase Servo in the up direction in left bumper down move fast
            rpabase_moveDown(!gamepad2.left_bumper);
        }else if(isMovingArm && !gamepad2.x && !gamepad2.b && gamepad2.left_stick_x > (0 - stickdeadzone) && gamepad2.left_stick_x < stickdeadzone ){
            m_rpabase_position(a_rpabase_position_actual());
            isMovingArm = false;
        }
        if (gamepad1.y) {
            m_winch_power(v_motor_winch_Speed);
        }
        else{
            m_winch_power(0.0f);
        }

        if (gamepad2.y )
        {
            rpaarm_moveUp(!gamepad2.left_bumper);
        }
        else if (gamepad2.a )
        {
            //move RPABase Servo in the up direction in left bumper down move fast
            rpaarm_moveDown(!gamepad2.left_bumper);
        }else{
            m_rpa_arm_power(0.0f);
        }

        /*if (gamepad2.left_trigger > ArmWristTrigger_Threshold_Fast ){

            arm_wrist_moveLeft(true);
        }else if (gamepad2.left_trigger > ArmWristTrigger_Threshold  ){
            arm_wrist_moveLeft(false);
        }else if (gamepad2.right_trigger > ArmWristTrigger_Threshold_Fast ){
            arm_wrist_moveRight(true);
        }else if(gamepad2.right_trigger > ArmWristTrigger_Threshold ){
            arm_wrist_moveRight(false);
        }
*/
        if(gamepad1.right_trigger > FlipRightServo_MinPosition) {
            m_flip_right_position(FlipRightServo_MaxPosition - gamepad1.right_trigger);
        }else {
            m_flip_right_position(FlipRightServo_MaxPosition);
        }

/*
        if (gamepad1.left_trigger > FlipLeftServo_MinPosition){
            m_flip_left_position(gamepad1.left_trigger);
        }else {
            m_flip_left_position(FlipLeftServo_MinPosition);
        }*/
        if(gamepad1.y && gamepad1.b){
            rpabase_moveToClimb();
        }

        if(gamepad1.a && gamepad1.x){
            neopixels_set_mode((byte)4);
            play_jingle_bells();
        }
        if(gamepad1.dpad_up){
            if(loopCounter() % 100 == 0){
                v_neopixels_mode++;
                neopixels_set_mode(v_neopixels_mode);
                set_second_message("neopixel mode:" + v_neopixels_mode);
            }
        }
        if(gamepad1.dpad_down){
            if(loopCounter() % 100 == 0){
                v_neopixels_mode--;
                neopixels_set_mode(v_neopixels_mode);
                set_second_message("neopixel mode:" + v_neopixels_mode);
            }
        }
        if(gamepad1.dpad_left){
            neopixels_set_rgb((byte)0xFF, (byte)0x00, (byte)0x00);
            set_second_message("neopixel color:red");

        }
        if(gamepad1.dpad_right){
            neopixels_set_rgb((byte)0x00, (byte)0x00, (byte)0xFF);
            set_second_message("neopixel color:blue");

        }
        //
        // Send telemetry data to the driver station.
        //
        update_telemetry (); // Update common telemetry
        //update_gamepad_telemetry();

    } // loop
}
