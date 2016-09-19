package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;

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
        //push ref to our base so it can call our functions
        this.setTelemetry(this);
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
        try {
            if (a_warning_generated()) {
                set_first_message(a_warning_message());
            }
            telemetry.addData("01", loopCounter() + ":" + hardware_loop_slowtime_milliseconds() + ":" + secondMessage);
            telemetry.addData("02", "Gyro: H:" + sensor_gyro_get_heading() + ",X:" + sensor_gyro_get_rawX() + ",Y:" + sensor_gyro_get_rawY() + ",Z:" + sensor_gyro_get_rawZ());
            if (v_debug) {
                //
                // Send telemetry data to the driver station.
                //


                telemetry.addData
                        ("03"
                                , "Left Drive: "
                                        + a_left_drive_power()
                                        + ", "
                                        + a_left_encoder_count()
                                        + ", "
                                        + a_left_drive_mode()
                        );
                telemetry.addData
                        ("04"
                                , "Right Drive: "
                                        + a_right_drive_power()
                                        + ", "
                                        + a_right_encoder_count()
                                        + ", "
                                        + a_right_drive_mode()
                        );
                                telemetry.addData
                        ("05"
                                , "RPA Base Position: " + a_rpabase_position()
                        );
                telemetry.addData
                        ("06"
                                , "RPA Arm Position: " + a_rpa_arm_power() + ":" + rpa_arm_extended() + ":" + rpa_arm_retracted()
                        );
                telemetry.addData(
                        "07", "Flip: Right:" + a_flip_right_position()
                );
                /*telemetry.addData
                        ("05"
                                , "Arm Shoulder: " + a_arm_shoulder_position()
                        );
                telemetry.addData
                        ("06"
                                , "Arm Elbow: " + a_arm_elbow_position()
                        );
                telemetry.addData
                        ("07"
                                , "Arm Wrist: " + a_arm_wrist_position()
                        );

                int[] v_color_rgba = sensor_color_get_rgba();
                telemetry.addData(
                        "10", "Color RGBA: " + v_color_rgba[0]
                                + "," + v_color_rgba[1]
                                + "," + v_color_rgba[2]
                                + "," + v_color_rgba[3]
                );

                telemetry.addData(
                        "1l", "Flip: Right:" + a_flip_right_position() + ", Left:" + a_flip_left_position()
                );
                telemetry.addData(
                        "12", "Ultra: " + sensor_ultraLegecy_distance()
                );
                telemetry.addData(
                        "13", "Light: tape:" + sensor_lightLegecy_white_tape_detected() + "," + sensor_lightLegecy_amountDetected()
                );*/
            }
        }catch (Exception p_exeception)
        {
            set_first_message("updateTelmetry: " + p_exeception.getLocalizedMessage());
        }
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
        if (v_debug) {
            telemetry.addData("14", "GP1 Left: " + -gamepad1.left_stick_y);
            telemetry.addData("15", "GP1 Right: " + -gamepad1.right_stick_y);
            telemetry.addData("16", "GP2 Left: " + -gamepad2.left_stick_y);
            telemetry.addData("17", "GP2 X: " + gamepad2.x);
            telemetry.addData("18", "GP2 Y: " + gamepad2.y);
            telemetry.addData("19", "GP2 A: " + gamepad2.a);
            telemetry.addData("20", "GP1 LT: " + gamepad1.left_trigger);
            telemetry.addData("21", "GP1 RT: " + gamepad1.right_trigger);
        }
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
        if (v_debug) {
            DbgLog.msg(loopCounter() + ": " + p_message);
        }

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
