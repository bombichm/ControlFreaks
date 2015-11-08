package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by adevries on 11/6/2015.
 */
public class CFPushBotTelemetry extends CFPushBotHardware {
    String secondMessage = "N/A";
    //--------------------------------------------------------------------------
    //
    // PushBotTelemetry
    //
    /**
     * Construct the class.
     *
     * The system calls this member when the class is instantiated.
     */
    public CFPushBotTelemetry ()

    {
        //
        // Initialize base classes.
        //
        // All via self-construction.

        //
        // Initialize class members.
        //
        // All via self-construction.

    } // PushBotTelemetry

    //--------------------------------------------------------------------------
    //
    // update_telemetry
    //
    /**
     * Update the telemetry with current values from the base class.
     */
    public void update_telemetry ()

    {
        if (a_warning_generated ())
        {
            set_first_message (a_warning_message ());
        }
        //
        // Send telemetry data to the driver station.
        //
        telemetry.addData ( "01", secondMessage);
        telemetry.addData
                ( "02"
                        , "Left Drive: "
                                + a_left_drive_power ()
                                + ", "
                                + a_left_encoder_count ()
                );
        telemetry.addData
                ( "03"
                        , "Right Drive: "
                                + a_right_drive_power()
                                + ", "
                                + a_right_encoder_count()
                );
        telemetry.addData
                ( "04"
                        , "Arm Shoulder: " + a_arm_shoulder_position()
                );
        telemetry.addData
                ( "05"
                        , "Arm Elbow: " + a_arm_elbow_position()
                );
        telemetry.addData
                ( "06"
                        , "Arm Wrist: " + a_arm_wrist_position()
                );
        telemetry.addData
                ( "07"
                        , "RPA Base Position: " + a_rpabase_position()
                );
        telemetry.addData
                ( "08"
                        , "RPA Arm Position: " + a_rpa_arm_power() + ":" + rpa_arm_extended() + ":" + rpa_arm_retracted()
                );
        telemetry.addData(
                "09", "Color RGBA: " + sensor_color_getLast_rgb()[0]
                        + "," + sensor_color_getLast_rgb()[1]
                        + "," + sensor_color_getLast_rgb()[2]
                        + "," + sensor_color_getLast_rgb()[3]
        );
        telemetry.addData(
                "10", "Gyro: " + sensor_gyro_getLast_heading()
        );

    } // update_telemetry

    //--------------------------------------------------------------------------
    //
    // update_gamepad_telemetry
    //
    /**
     * Update the telemetry with current gamepad readings.
     */
    public void update_gamepad_telemetry ()

    {
        //
        // Send telemetry data concerning gamepads to the driver station.
        //
        telemetry.addData ("10", "GP1 Left: " + -gamepad1.left_stick_y);
        telemetry.addData ("11", "GP1 Right: " + -gamepad1.right_stick_y);
        telemetry.addData ("12", "GP2 Left: " + -gamepad2.left_stick_y);
        telemetry.addData ("13", "GP2 X: " + gamepad2.x);
        telemetry.addData ("14", "GP2 Y: " + gamepad2.y);
        telemetry.addData ("15", "GP2 A: " + gamepad2.a);
        telemetry.addData ("16", "GP1 LT: " + gamepad1.left_trigger);
        telemetry.addData ("17", "GP1 RT: " + gamepad1.right_trigger);

    } // update_gamepad_telemetry

    //--------------------------------------------------------------------------
    //
    // set_first_message
    //
    /**
     * Update the telemetry's first message with the specified message.
     */
    public void set_first_message (String p_message)

    {
        telemetry.addData ( "00", p_message);

    } // set_first_message

    //--------------------------------------------------------------------------
    //
    // set_first_message
    //
    /**
     * Update the telemetry's first message with the specified message.
     */
    public void set_second_message (String p_message)

    {
        secondMessage = p_message;


    } // set_first_message
    //--------------------------------------------------------------------------
    //
    // set_error_message
    //
    /**
     * Update the telemetry's first message to indicate an error.
     */
    public void set_error_message (String p_message)

    {
        set_first_message ("ERROR: " + p_message);

    } // set_error_message
}