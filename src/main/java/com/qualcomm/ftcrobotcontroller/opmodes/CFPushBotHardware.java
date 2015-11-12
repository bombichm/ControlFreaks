package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by adevries on 11/6/2015.
 */

        import android.graphics.Color;
        import com.qualcomm.ftccommon.DbgLog;
        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorController;
        import com.qualcomm.robotcore.hardware.I2cDevice;
        import com.qualcomm.robotcore.hardware.LED;
        import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.hardware.TouchSensor;
        import com.qualcomm.robotcore.util.Range;
        import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
        import com.qualcomm.robotcore.hardware.IrSeekerSensor;
        import com.qualcomm.robotcore.util.RobotLog;
        import com.qualcomm.robotcore.util.TypeConversion;
//import com.qualcomm.ftcrobotcontroller.opmodes.ColorSensorDriver;
        import com.qualcomm.robotcore.hardware.ColorSensor;

public class CFPushBotHardware extends OpMode {

    private long v_loop_ticks = 0;

    private static final double driveInches_ticksPerInch = 182.35;
    private static final double driveInches_ticksSlowDown1 = 300;
    private static final double driveInches_ticksSlowDown2 = 150;
    private static final float v_drive_power_slowdown1 = .5f;
    private static final float v_drive_power_slowdown2 = .25f;
    //Global Vars to the class
    private static final double ServoErrorResultPosition = -0.0000000001;
    //RPA Base Varables
    private Servo v_servo_rpa_base;
    private static final double RPABaseServo_Delta = 0.0005;
    private static final double RPABaseServo_Delta_Fast = 0.01;
    private double RPABaseServo_MinPosition = 0.01;  //Need to unhook gear int so it goes to zero then rehook servo gear
    private double RPABaseServo_MaxPosition = 0.61;
    private double RPABaseServo_MaxPosition_Delta = 0.60;
    private double l_rpa_base_position = 0.163D;  //init RPA Base Position and to control RPA Base position as servo.getPosition seems to be flaky

    //RPA Arm Varables
    // v_motor_rpa_arm
    private DcMotor v_motor_rpa_arm;
    private TouchSensor v_sensor_touch_rpa_arm_retract;
    private TouchSensor v_sensor_touch_rpa_arm_extend;
    private static final double RPAArmMotor_Speed = 0.3;
    private static final double RPAArmMotor_Speed_Fast = 1.0;
    private static final String RPAArmMotor_Retract_TouchSensorName = "rpa_retract";
    private static final String RPAArmMotor_Extend_TouchSensorName = "rpa_extend";


    // v_servo_arm_shoulder
    private Servo v_servo_arm_shoulder;
    private static final double ArmShoulderServo_Delta = 0.0008;
    private static final double ArmShoulderServo_Delta_Fast = 0.002;
    private static final double ArmShoulderServo_MinPosition = 0.19;
    private static final double ArmShoulderServo_MaxPosition = 0.80;
    private double l_arm_shoulder_position = 0.2D;  //init arm shoulder Position


    // v_servo_arm_elbow
    private Servo v_servo_arm_elbow;
    private static final double ArmElbowServo_Delta = 0.0008;
    private static final double ArmElbowServo_Delta_Fast = 0.002;
    private static final double ArmElbowServo_MinPosition = 0.20;
    private static final double ArmElbowServo_MaxPosition = 0.99;
    private double l_arm_elbow_position = 0.20D;  //init arm elbow Position

    // v_servo_arm_wrist
    private Servo v_servo_arm_wrist;
    private static final double ArmWristServo_Delta = 0.005;
    private static final double ArmWristServo_Delta_Fast = 0.05;
    private static final double ArmWristServo_MinPosition = 0.05;
    private static final double ArmWristServo_MaxPosition = 0.99;
    private double l_arm_wrist_position = 0.45D;  //init arm elbow Position

    /**
     * Used in Manual mode to set min trigger pull for slow wrist action
     *
     */
    public static final double ArmWristTrigger_Threshold = 0.2;
    /**
     * Used in Manual mode to set min trigger pull for fast wrist action
     *
     */
    public static final double ArmWristTrigger_Threshold_Fast = 0.9;

    //Legecy Color Sensor
    private ColorSensor v_sensor_colorLegecy;
    private boolean v_sensor_colorLegecy_led_enabled = false;
    // v_sensor_color_hsvValues is an array that will hold the hue, saturation, and value information.
    private float v_sensor_colorLegecy_hsvValues[] = {0F,0F,0F};
    // values is a reference to the v_sensor_color_hsvValues array.
    private final float v_sensor_colorLegecy_values[] = v_sensor_colorLegecy_hsvValues;
    private int v_sensor_colorLegecy_rgbValues[] = {0,0,0,0};

    //Legecy OSD Sensor
    private OpticalDistanceSensor v_sensor_odsLegecy;
    private boolean v_sensor_odsLegecy_enabled = false;

    private int v_sensor_gyro_heading = -1;
    private CFSensorGY521 v_sensor_gy521;

    private LED v_led_heartbeat;
    private boolean v_led_heartbeat_enabled = false;
    private final int v_led_heartbeat_tickPerToggle = 20;
    private int v_led_heartbeat_ticks = 0;

    private DeviceInterfaceModule v_dim;

    //--------------------------------------------------------------------------
    //
    // v_motor_left_drive
    //
    /**
     * Manage the aspects of the left drive motor.
     */
    private DcMotor v_motor_left_drive;

    //--------------------------------------------------------------------------
    //
    // v_motor_right_drive
    //
    /**
     * Manage the aspects of the right drive motor.
     */
    private DcMotor v_motor_right_drive;



    //--------------------------------------------------------------------------
    //
    // v_warning_generated
    //
    /**
     * Indicate whether a message is a available to the class user.
     */
    private boolean v_warning_generated = false;

    //--------------------------------------------------------------------------
    //
    // v_warning_message
    //
    /**
     * Store a message to the user if one has been generated.
     */
    private String v_warning_message;

    //--------------------------------------------------------------------------
    //
    // PushBotHardware
    //
    /**
     * Construct the class.
     *
     * The system calls this member when the class is instantiated.
     */
    public CFPushBotHardware ()

    {
        //
        // Initialize base classes.
        //
        // All via self-construction.

        //
        // Initialize class members.
        //
        // All via self-construction.

    } // PushBotHardware

    //--------------------------------------------------------------------------
    //
    // init
    //
    /**
     * Perform any actions that are necessary when the OpMode is enabled.
     *
     * The system calls this member once when the OpMode is enabled.
     */
    @Override public void init ()

    {
        //
        // Use the hardwareMap to associate class members to hardware ports.
        //
        // Note that the names of the devices (i.e. arguments to the get method)
        // must match the names specified in the configuration file created by
        // the FTC Robot Controller (Settings-->Configure Robot).
        //
        // The variable below is used to provide telemetry data to a class user.
        //
        v_warning_generated = false;
        v_warning_message = "Can't map; ";

        //
        //Connect the Core Interface Device or Dim
        try {
            if(v_dim !=null) {
                // set up the hardware devices we are going to use
                v_dim = hardwareMap.deviceInterfaceModule.get("dim");
            }

        }catch (Exception p_exeception)
        {
            m_warning_message ("dim");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_dim = null;
        }

        //if we have a dim then connect to the gyro
        try {
            //I2cDevice ic2GY521 = hardwareMap.i2cDevice.get("gy521");
            v_sensor_gy521 = new CFSensorGY521(v_dim,5);

        }catch(Exception p_exeception){

            m_warning_message ("dim gy-521");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_dim = null;
        }

        //
        // Connect the drive wheel motors.
        //
        // The direction of the right motor is reversed, so joystick inputs can
        // be more generically applied.
        //
        try
        {
            v_motor_left_drive = hardwareMap.dcMotor.get ("left_drive");
            v_motor_left_drive.setDirection (DcMotor.Direction.REVERSE);
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("left_drive");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_motor_left_drive = null;
        }

        try
        {
            v_motor_right_drive = hardwareMap.dcMotor.get ("right_drive");
            //v_motor_right_drive.setDirection (DcMotor.Direction.REVERSE);
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("right_drive");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_motor_right_drive = null;
        }

        try
        {
            //// Get a reference to the touch sensor
            v_sensor_touch_rpa_arm_retract = hardwareMap.touchSensor.get(RPAArmMotor_Retract_TouchSensorName);
            v_sensor_touch_rpa_arm_extend = hardwareMap.touchSensor.get(RPAArmMotor_Extend_TouchSensorName);
            v_motor_rpa_arm = hardwareMap.dcMotor.get ("rpa_arm");
            //v_motor_right_drive.setDirection (DcMotor.Direction.REVERSE);
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("rpa_arm");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_motor_rpa_arm = null;
        }



        //
        // Connect the arm sholder servo.
        //
        try
        {

            v_servo_arm_shoulder = hardwareMap.servo.get("arm_shoulder");
            v_servo_arm_shoulder.setPosition (l_arm_shoulder_position);
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("arm_shoulder");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_servo_arm_shoulder = null;
        }

        //
        // Connect the arm elbow servo.
        //
        try
        {

            v_servo_arm_elbow = hardwareMap.servo.get("arm_elbow");
            v_servo_arm_elbow.setPosition (l_arm_elbow_position);

        }
        catch (Exception p_exeception)
        {
            m_warning_message ("arm_elbow");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_servo_arm_elbow = null;
        }

        //
        // Connect the arm wrist servo.
        //
        try
        {
            v_servo_arm_wrist = hardwareMap.servo.get("arm_wrist");
            v_servo_arm_wrist.setPosition (l_arm_wrist_position);
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("arm_wrist");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_servo_arm_wrist = null;
        }

        //
        // Connect the heartbeat led.
        //
        try
        {
            v_led_heartbeat = hardwareMap.led.get("heartbeat");
            v_led_heartbeat.enable(v_led_heartbeat_enabled);
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("heartbeat");
            DbgLog.msg (p_exeception.getLocalizedMessage ());
            v_led_heartbeat = null;
        }

        //
        // Connect the RPA Base servo.
        //

        try
        {
            v_servo_rpa_base = hardwareMap.servo.get ("rpa_base");
            v_servo_rpa_base.scaleRange(RPABaseServo_MinPosition,RPABaseServo_MaxPosition); //set the max range to allow the servo to move
            v_servo_rpa_base.setPosition (l_rpa_base_position);
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("rpa_base");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_servo_rpa_base = null;
        }

        try
        {
            // get a reference to our ColorSensor object.
            v_sensor_colorLegecy = hardwareMap.colorSensor.get("color1");
            // bEnabled represents the state of the LED.
            boolean v_sensor_colorLegecy_led_enabled = true;
            // turn the LED on in the beginning, just so user will know that the sensor is active.
            v_sensor_colorLegecy.enableLed(false);

        }
        catch (Exception p_exeception)
        {
            m_warning_message ("color1");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_sensor_colorLegecy = null;
        }

        try
        {
            v_sensor_odsLegecy = hardwareMap.opticalDistanceSensor.get ("ods1");
        }
        catch (Exception p_exeception)
        {
            try
            {
                v_sensor_odsLegecy = hardwareMap.opticalDistanceSensor.get
                        ( "ods1"
                        );
            }
            catch (Exception p_exeception_eopd)
            {

                m_warning_message ("sensor_odsLegecy");
                DbgLog.msg
                        ( "Can't map sensor_odsLegecy "
                                        + p_exeception_eopd.getLocalizedMessage ()
                                        + ").\n"
                        );

                v_sensor_odsLegecy = null;

            }
        }

    } // init

    //--------------------------------------------------------------------------
    //
    // a_warning_generated
    //
    /**
     * Access whether a warning has been generated.
     */
    boolean a_warning_generated ()

    {
        return v_warning_generated;

    } // a_warning_generated

    //--------------------------------------------------------------------------
    //
    // a_warning_message
    //
    /**
     * Access the warning message.
     */
    String a_warning_message ()

    {
        return v_warning_message;

    } // a_warning_message

    //--------------------------------------------------------------------------
    //
    // m_warning_message
    //
    /**
     * Mutate the warning message by ADDING the specified message to the current
     * message; set the warning indicator to true.
     *
     * A comma will be added before the specified message if the message isn't
     * empty.
     */
    void m_warning_message (String p_exception_message)

    {
        if (v_warning_generated)
        {
            v_warning_message += ", ";
        }
        v_warning_generated = true;
        v_warning_message += p_exception_message;

    } // m_warning_message

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
        // Only actions that are common to all Op-Modes (i.e. both automatic and
        // manual) should be implemented here.
        //
        // This method is designed to be overridden.
        //

    } // start

    //--------------------------------------------------------------------------
    //
    // loop
    //
    /**
     * Perform any actions that are necessary while the OpMode is running.
     *
     * The system calls this member repeatedly while the OpMode is running.
     */
    @Override public void loop ()

    {

        //
        // Only actions that are common to all OpModes (i.e. both auto and\
        // manual) should be implemented here.
        //
        // This method is designed to be overridden.
        //
        loop_tick();
    } // loop

    //--------------------------------------------------------------------------
    //
    // stop
    //
    /**
     * Perform any actions that are necessary when the OpMode is disabled.
     *
     * The system calls this member once when the OpMode is disabled.
     */
    @Override public void stop ()
    {
        //
        // Nothing needs to be done for this method.
        //

    } // stop


    /** called each time through the loop needed to sync hardware and look for status changes
     *
     */
    public void loop_tick(){
        v_loop_ticks++;
        heartbeat_tick();
    }

    //--------------------------------------------------------------------------
    //
    // scale_motor_power
    //
    /**
     * Scale the joystick input using a nonlinear algorithm.
     */
    float scale_motor_power (float p_power)
    {
        //
        // Assume no scaling.
        //
        float l_scale = 0.0f;

        //
        // Ensure the values are legal.
        //
        float l_power = Range.clip (p_power, -1, 1);

        float[] l_array =
                { 0.00f, 0.05f, 0.09f, 0.10f, 0.12f
                        , 0.15f, 0.18f, 0.24f, 0.30f, 0.36f
                        , 0.43f, 0.50f, 0.60f, 0.72f, 0.85f
                        , 1.00f, 1.00f
                };

        //
        // Get the corresponding index for the specified argument/parameter.
        //
        int l_index = (int)(l_power * 16.0);
        if (l_index < 0)
        {
            l_index = -l_index;
        }
        else if (l_index > 16)
        {
            l_index = 16;
        }

        if (l_power < 0)
        {
            l_scale = -l_array[l_index];
        }
        else
        {
            l_scale = l_array[l_index];
        }

        return l_scale;

    } // scale_motor_power

    //--------------------------------------------------------------------------
    //
    // a_left_drive_power
    //
    /**
     * Access the left drive motor's power level.
     */
    double a_left_drive_power ()
    {
        double l_return = 0.0;

        if (v_motor_left_drive != null)
        {
            l_return = v_motor_left_drive.getPower ();
        }

        return l_return;

    } // a_left_drive_power

    //--------------------------------------------------------------------------
    //
    // a_right_drive_power
    //
    /**
     * Access the right drive motor's power level.
     */
    double a_right_drive_power ()
    {
        double l_return = 0.0;

        if (v_motor_right_drive != null)
        {
            l_return = v_motor_right_drive.getPower ();
        }

        return l_return;

    } // a_right_drive_power

    //--------------------------------------------------------------------------
    //
    // set_drive_power
    //
    /**
     * Scale the joystick input using a nonlinear algorithm.
     */
    void set_drive_power (double p_left_power, double p_right_power)

    {
        if (v_motor_left_drive != null)
        {
            v_motor_left_drive.setPower (p_left_power);
        }
        if (v_motor_right_drive != null)
        {
            v_motor_right_drive.setPower(p_right_power);
        }

    } // set_drive_power

    //--------------------------------------------------------------------------
    //
    // run_using_left_drive_encoder
    //
    /**
     * Set the left drive wheel encoder to run, if the mode is appropriate.
     */
    public void run_using_left_drive_encoder ()

    {
        if (v_motor_left_drive != null)
        {
            v_motor_left_drive.setMode
                    (DcMotorController.RunMode.RUN_USING_ENCODERS
                    );
        }

    } // run_using_left_drive_encoder

    //--------------------------------------------------------------------------
    //
    // run_using_right_drive_encoder
    //
    /**
     * Set the right drive wheel encoder to run, if the mode is appropriate.
     */
    public void run_using_right_drive_encoder ()

    {
        if (v_motor_right_drive != null)
        {
            v_motor_right_drive.setMode
                    (DcMotorController.RunMode.RUN_USING_ENCODERS
                    );
        }

    } // run_using_right_drive_encoder

    //--------------------------------------------------------------------------
    //
    // run_using_encoders
    //
    /**
     * Set both drive wheel encoders to run, if the mode is appropriate.
     */
    public void run_using_encoders ()

    {
        //
        // Call other members to perform the action on both motors.
        //
        run_using_left_drive_encoder ();
        run_using_right_drive_encoder ();

    } // run_using_encoders

    //--------------------------------------------------------------------------
    //
    // run_without_left_drive_encoder
    //
    /**
     * Set the left drive wheel encoder to run, if the mode is appropriate.
     */
    public void run_without_left_drive_encoder ()

    {
        if (v_motor_left_drive != null)
        {
            if (v_motor_left_drive.getMode() ==
                    DcMotorController.RunMode.RESET_ENCODERS)
            {
                v_motor_left_drive.setMode
                        (DcMotorController.RunMode.RUN_WITHOUT_ENCODERS
                        );
            }
        }

    } // run_without_left_drive_encoder

    //--------------------------------------------------------------------------
    //
    // run_without_right_drive_encoder
    //
    /**
     * Set the right drive wheel encoder to run, if the mode is appropriate.
     */
    public void run_without_right_drive_encoder ()

    {
        if (v_motor_right_drive != null)
        {
            if (v_motor_right_drive.getMode() ==
                    DcMotorController.RunMode.RESET_ENCODERS)
            {
                v_motor_right_drive.setMode
                        (DcMotorController.RunMode.RUN_WITHOUT_ENCODERS
                        );
            }
        }

    } // run_without_right_drive_encoder

    //--------------------------------------------------------------------------
    //
    // run_without_drive_encoders
    //
    /**
     * Set both drive wheel encoders to run, if the mode is appropriate.
     */
    public void run_without_drive_encoders ()

    {
        //
        // Call other members to perform the action on both motors.
        //
        run_without_left_drive_encoder ();
        run_without_right_drive_encoder ();

    } // run_without_drive_encoders

    //--------------------------------------------------------------------------
    //
    // reset_left_drive_encoder
    //
    /**
     * Reset the left drive wheel encoder.
     */
    public void reset_left_drive_encoder ()

    {
        if (v_motor_left_drive != null)
        {
            v_motor_left_drive.setMode
                    (DcMotorController.RunMode.RESET_ENCODERS
                    );
        }

    } // reset_left_drive_encoder

    //--------------------------------------------------------------------------
    //
    // reset_right_drive_encoder
    //
    /**
     * Reset the right drive wheel encoder.
     */
    public void reset_right_drive_encoder ()

    {
        if (v_motor_right_drive != null)
        {
            v_motor_right_drive.setMode
                    (DcMotorController.RunMode.RESET_ENCODERS
                    );
        }

    } // reset_right_drive_encoder

    //--------------------------------------------------------------------------
    //
    // reset_drive_encoders
    //
    /**
     * Reset both drive wheel encoders.
     */
    public void reset_drive_encoders ()

    {
        //
        // Reset the motor encoders on the drive wheels.
        //
        reset_left_drive_encoder ();
        reset_right_drive_encoder ();

    } // reset_drive_encoders

    //--------------------------------------------------------------------------
    //
    // a_left_encoder_count
    //
    /**
     * Access the left encoder's count.
     */
    int a_left_encoder_count ()
    {
        int l_return = 0;

        if (v_motor_left_drive != null)
        {
            l_return = v_motor_left_drive.getCurrentPosition ();
        }

        return l_return;

    } // a_left_encoder_count

    //--------------------------------------------------------------------------
    //
    // a_right_encoder_count
    //
    /**
     * Access the right encoder's count.
     */
    int a_right_encoder_count ()

    {
        int l_return = 0;

        if (v_motor_right_drive != null)
        {
            l_return = v_motor_right_drive.getCurrentPosition ();
        }

        return l_return;

    } // a_right_encoder_count

    //--------------------------------------------------------------------------
    //
    // has_left_drive_encoder_reached
    //
    /**
     * Indicate whether the left drive motor's encoder has reached a value.
     */
    boolean has_left_drive_encoder_reached (double p_count)

    {
        //
        // Assume failure.
        //
        boolean l_return = false;

        if (v_motor_left_drive != null)
        {
            //
            // Has the encoder reached the specified values?
            //
            // TODO Implement stall code using these variables.
            //
            if (Math.abs (v_motor_left_drive.getCurrentPosition ()) > p_count)
            {
                //
                // Set the status to a positive indication.
                //
                l_return = true;
            }
        }

        //
        // Return the status.
        //
        return l_return;

    } // has_left_drive_encoder_reached

    //--------------------------------------------------------------------------
    //
    // has_right_drive_encoder_reached
    //
    /**
     * Indicate whether the right drive motor's encoder has reached a value.
     */
    boolean has_right_drive_encoder_reached (double p_count)

    {
        //
        // Assume failure.
        //
        boolean l_return = false;

        if (v_motor_right_drive != null)
        {
            //
            // Have the encoders reached the specified values?
            //
            // TODO Implement stall code using these variables.
            //
            if (Math.abs (v_motor_right_drive.getCurrentPosition ()) > p_count)
            {
                //
                // Set the status to a positive indication.
                //
                l_return = true;
            }
        }

        //
        // Return the status.
        //
        return l_return;

    } // has_right_drive_encoder_reached

    //--------------------------------------------------------------------------
    //
    // have_drive_encoders_reached
    //
    /**
     * Indicate whether the drive motors' encoders have reached a value.
     */
    boolean have_drive_encoders_reached
    ( double p_left_count
            , double p_right_count
    )

    {
        //
        // Assume failure.
        //
        boolean l_return = false;

        //
        // Have the encoders reached the specified values?
        //
        if (has_left_drive_encoder_reached (p_left_count) &&
                has_right_drive_encoder_reached (p_right_count))
        {
            //
            // Set the status to a positive indication.
            //
            l_return = true;
        }

        //
        // Return the status.
        //
        return l_return;

    } // have_encoders_reached

    //--------------------------------------------------------------------------
    //
    // drive_using_encoders
    //
    /**
     * Indicate whether the drive motors' encoders have reached a value.
     */
    boolean drive_using_encoders
    ( double p_left_power
            , double p_right_power
            , double p_left_count
            , double p_right_count
    )

    {
        //
        // Assume the encoders have not reached the limit.
        //
        boolean l_return = false;

        //
        // Tell the system that motor encoders will be used.
        //
        run_using_encoders ();

        //
        // Start the drive wheel motors at full power.
        //
        set_drive_power (p_left_power, p_right_power);

        //
        // Have the motor shafts turned the required amount?
        //
        // If they haven't, then the op-mode remains in this state (i.e this
        // block will be executed the next time this method is called).
        //
        if (have_drive_encoders_reached (p_left_count, p_right_count))
        {
            //
            // Reset the encoders to ensure they are at a known good value.
            //
            reset_drive_encoders ();

            //
            // Stop the motors.
            //
            set_drive_power (0.0f, 0.0f);

            //
            // Transition to the next state when this method is called
            // again.
            //
            l_return = true;
        }

        //
        // Return the status.
        //
        return l_return;

    } // drive_using_encoders

    //--------------------------------------------------------------------------
    //
    // has_left_drive_encoder_reset
    //
    /**
     * Indicate whether the left drive encoder has been completely reset.
     */
    boolean has_left_drive_encoder_reset ()
    {
        //
        // Assume failure.
        //
        boolean l_return = false;

        //
        // Has the left encoder reached zero?
        //
        if (a_left_encoder_count() == 0)
        {
            //
            // Set the status to a positive indication.
            //
            l_return = true;
        }

        //
        // Return the status.
        //
        return l_return;

    } // has_left_drive_encoder_reset

    //--------------------------------------------------------------------------
    //
    // has_right_drive_encoder_reset
    //
    /**
     * Indicate whether the left drive encoder has been completely reset.
     */
    boolean has_right_drive_encoder_reset ()
    {
        //
        // Assume failure.
        //
        boolean l_return = false;

        //
        // Has the right encoder reached zero?
        //
        if (a_right_encoder_count() == 0)
        {
            //
            // Set the status to a positive indication.
            //
            l_return = true;
        }

        //
        // Return the status.
        //
        return l_return;

    } // has_right_drive_encoder_reset

    //--------------------------------------------------------------------------
    //
    // have_drive_encoders_reset
    //
    /**
     * Indicate whether the encoders have been completely reset.
     */
    boolean have_drive_encoders_reset ()
    {
        //
        // Assume failure.
        //
        boolean l_return = false;

        //
        // Have the encoders reached zero?
        //
        if (has_left_drive_encoder_reset() && has_right_drive_encoder_reset ())
        {
            //
            // Set the status to a positive indication.
            //
            l_return = true;
        }

        //
        // Return the status.
        //
        return l_return;

    } // have_drive_encoders_reset

    private long v_drive_inches_ticks;
    private float v_drive_inches_power;
    public void drive_inches(float power,float inches){
        //
        // Tell the system that motor encoders will be used.  This call MUST
        // be in this state and NOT the previous or the encoders will not
        // work.  It doesn't need to be in subsequent states.
        //
        run_using_encoders ();
        v_drive_inches_power = power;
        v_drive_inches_ticks = Math.round(inches * driveInches_ticksPerInch);
        //
        // Start the drive wheel motors at full power.
        //
        set_drive_power (v_drive_inches_power, v_drive_inches_power);
    }

    public boolean drive_inches_complete(){


        //
        // Have the motor shafts turned the required amount?
        //
        // If they haven't, then the op-mode remains in this state (i.e this
        // block will be executed the next time this method is called).
        //
        if (have_drive_encoders_reached (v_drive_inches_ticks - driveInches_ticksSlowDown1, v_drive_inches_ticks - driveInches_ticksSlowDown1))
        {

            //
            // slow the motors to slowdown 1
            //
            if(v_drive_inches_power > v_drive_power_slowdown1) {
                set_drive_power(v_drive_power_slowdown1, v_drive_power_slowdown1);
            }
        }else if (have_drive_encoders_reached (v_drive_inches_ticks - driveInches_ticksSlowDown2, v_drive_inches_ticks - driveInches_ticksSlowDown2))
        {
            //
            // slow the motors to slowdown 2
            //
            if(v_drive_inches_power > v_drive_power_slowdown2) {
                set_drive_power(v_drive_power_slowdown2, v_drive_power_slowdown2);
            }

        }else if (have_drive_encoders_reached (v_drive_inches_ticks , v_drive_inches_ticks ))
        {
            //
            // Stop the motors.
            //
            set_drive_power (0.0f, 0.0f);
            //
            // Reset the encoders to ensure they are at a known good value.
            //
            reset_drive_encoders();
            //
            // Transition to the next state when this method is called
            // again.
            //
            return true;
        }
        return false;
    }

    public boolean rpa_arm_extended(){
        if (v_sensor_touch_rpa_arm_extend != null) {
            return v_sensor_touch_rpa_arm_extend.isPressed();
        }else {
            return true;
        }
    }
    private float v_turn_ticks_per_degree = 18.8f;
    private long v_turn_degrees_ticks;
    public void turn_degrees(int degrees){
        run_using_encoders();
        if (degrees > 0) {
            v_turn_degrees_ticks = Math.round(degrees * v_turn_ticks_per_degree);
            set_drive_power(-1.0f, 1.0f);
        }else{
            v_turn_degrees_ticks = Math.round((0 - degrees) * v_turn_ticks_per_degree);
            set_drive_power(1.0f, -1.0f);
        }
    }

    public boolean turn_complete(){
        if (have_drive_encoders_reached (v_turn_degrees_ticks, v_turn_degrees_ticks))
        {
            set_drive_power (0.0f, 0.0f);
            reset_drive_encoders ();
            return true;
        }
        return false;
    }

   public boolean rpa_arm_retracted(){
        if (v_sensor_touch_rpa_arm_extend != null) {
            return v_sensor_touch_rpa_arm_retract.isPressed();
        }else {
            return true;
        }
    }

    //--------------------------------------------------------------------------
    //
    // a_left_arm_power
    //
    /**
     * Access the rpa arm motor's power level.
     */
    double a_rpa_arm_power ()
    {
        double l_return = 0.0;

        if (v_motor_rpa_arm != null)
        {
            l_return = v_motor_rpa_arm.getPower ();
            if(l_return > 0 && rpa_arm_extended()==true){
                v_motor_rpa_arm.setPower (0);
            }
            if(l_return < 0 && rpa_arm_retracted()==true){
                v_motor_rpa_arm.setPower(0);
            }

        }

        return l_return;

    } // a_rpa_arm_power

    //--------------------------------------------------------------------------
    //
    // m_rpa_arm_power
    //
    /**
     * Access the rpa arm motor's power level.
     */
    void m_rpa_arm_power (double p_level)
    {
        if (v_motor_rpa_arm != null)
        {
            if(p_level > 0){
                //We are moving up
               if (rpa_arm_extended()==false) {
                   //The switch is not pressed so move up
                   v_motor_rpa_arm.setPower(p_level);
               }else {
                   v_motor_rpa_arm.setPower(0);
               }

            }else if( p_level < 0) {
                //we are moving down
                if (rpa_arm_retracted() == false) {
                    //The switch is not pressed so move down
                    v_motor_rpa_arm.setPower(p_level);
                } else {
                    v_motor_rpa_arm.setPower(0);
                }
            }
           else{
                v_motor_rpa_arm.setPower(0);
            }
        }

    } // m_left_arm_power






    //--------------------------------------------------------------------------
    //
    // rpaarm_moveUp
    //
    /**
     * move the rpabase servo in the up Direction.
     */
    boolean rpaarm_moveUp (boolean fast)
    {
        if(rpa_arm_extended() == true) {

            return false;
        }else{
            if(fast){
                m_rpa_arm_power(RPAArmMotor_Speed_Fast);
            }else{
                m_rpa_arm_power(RPAArmMotor_Speed);
            }
            return true;
        }

    } // rpaarm_moveUp


    //--------------------------------------------------------------------------
    //
    // rpaarm_moveDown
    //
    /**
     * move the rpaarm motor in the down Direction.
     */
    boolean rpaarm_moveDown (boolean fast)
    {
        if(rpa_arm_retracted() == true) {
            return false;
        }else{
            if(fast){
                m_rpa_arm_power(0-RPAArmMotor_Speed_Fast);
            }else{
                m_rpa_arm_power(0-RPAArmMotor_Speed);
            }
            return true;
        }
    } // rpabase_moveDown


    //--------------------------------------------------------------------------
    //
    // rpabase_moveUp
    //
    /**
     * move the rpabase servo in the up Direction.
     */
    double rpabase_moveUp (boolean fast)
    {
        double l_temptarget;
        if (fast) {
            l_temptarget = a_rpabase_position() + RPABaseServo_Delta_Fast;
        }else{
            l_temptarget = a_rpabase_position() + RPABaseServo_Delta;
        }
        return m_rpabase_position(l_temptarget);
    } // rpabase_moveUp


    //--------------------------------------------------------------------------
    //
    // rpabase_moveDown
    //
    /**
     * move the rpabase servo in the down Direction.
     */
    double rpabase_moveDown (boolean fast)
    {
        double l_temptarget;
        if (fast) {
            l_temptarget = a_rpabase_position() - RPABaseServo_Delta_Fast;
        }else{
            l_temptarget = a_rpabase_position() - RPABaseServo_Delta;
        }
        return m_rpabase_position(l_temptarget);
    } // rpabase_moveDown


    //--------------------------------------------------------------------------
    //
    // a_rpabase_position
    //
    /**
     * Access the rpabase position.
     */
    double a_rpabase_position ()
    {
        //there is a bug where the getPosition() does return correctly so use an internal
       /* double l_return = 0.0;

        if (v_servo_rpa_base != null)
        {
            l_return = v_servo_rpa_base.getPosition ();
        }

        return l_return;*/
        return l_rpa_base_position;
    } // a_rpabase_position

    //--------------------------------------------------------------------------
    //
    // a_rpabase_position
    //
    /**
     * Access the rpabase position.
     */
//    double rpabase_position_zero ()
//    {
//        //Because it is posible to slip the gear and get out of position we allow for a rezero
//       /* double l_return = 0.0;
//
//        if (v_servo_rpa_base != null)
//        {
//            l_return = v_servo_rpa_base.getPosition ();
//        }
//
//        return l_return;*/
//        return l_rpa_base_position;
//    } // a_rpabase_position

    //--------------------------------------------------------------------------
    //
    // m_rpabase_position
    //
    /**
     * Mutate the rpa_base position.
     */
    double m_rpabase_position (double p_position)
    {
        //
        // Ensure the specific value is legal.
        //
        l_rpa_base_position = Range.clip
                ( p_position
                        , RPABaseServo_MinPosition
                        , RPABaseServo_MaxPosition
                );
        try {
            if (v_servo_rpa_base != null) {
                v_servo_rpa_base.setPosition(l_rpa_base_position);
                return l_rpa_base_position;
            } else {
                return ServoErrorResultPosition;
            }
        }catch (Exception p_exeception)
        {
            m_warning_message("rpa_base");
            DbgLog.msg (p_exeception.getLocalizedMessage ());
            return ServoErrorResultPosition;
        }


    } // m_rpabase_position


    //--------------------------------------------------------------------------
    //
    // arm_shoulder_moveUp
    //
    /**
     * move the arm shoulder servo in the up Direction.
     */
    double arm_shoulder_moveUp (boolean fast)
    {
        double l_temptarget;
        if (fast) {
            l_temptarget = a_arm_shoulder_position() + ArmShoulderServo_Delta_Fast;
        }else{
            l_temptarget = a_arm_shoulder_position() + ArmShoulderServo_Delta;
        }
        return m_arm_shoulder_position(l_temptarget);
    } // arm_shoulder_moveUp


    //--------------------------------------------------------------------------
    //
    // arm_shoulder_moveDown
    //
    /**
     * move the arm_shoulder servo in the down Direction.
     */
    double arm_shoulder_moveDown (boolean fast)
    {
        double l_temptarget;
        if (fast) {
            l_temptarget = a_arm_shoulder_position() - ArmShoulderServo_Delta_Fast;
        }else{
            l_temptarget = a_arm_shoulder_position() - ArmShoulderServo_Delta;
        }
        return m_arm_shoulder_position(l_temptarget);
    } // arm_shoulder_moveDown


    //--------------------------------------------------------------------------
    //
    // a_arm_shoulder_position
    //
    /**
     * Access the arm_shoulder position.
     */
    double a_arm_shoulder_position ()
    {

        return l_arm_shoulder_position;
    } // a_arm_shoulder_position


    //--------------------------------------------------------------------------
    //
    // m_arm_shoulder_position
    //
    /**
     * Mutate the arm shoulder position.
     */
    double m_arm_shoulder_position (double p_position)
    {
        //
        // Ensure the specific value is legal.
        //
        l_arm_shoulder_position = Range.clip
                ( p_position
                        , ArmShoulderServo_MinPosition
                        , ArmShoulderServo_MaxPosition
                );
        try {
            if (v_servo_arm_shoulder != null) {
                v_servo_arm_shoulder.setPosition(l_arm_shoulder_position);
                return l_arm_shoulder_position;
            } else {
                return ServoErrorResultPosition;
            }
        }catch (Exception p_exeception)
        {
            m_warning_message("arm_sholder");
            DbgLog.msg (p_exeception.getLocalizedMessage ());
            return ServoErrorResultPosition;
        }


    } // m_arm_shoulder_position

    //--------------------------------------------------------------------------
    //
    // arm_elbow_moveUp
    //
    /**
     * move the arm elbow servo in the up Direction.
     */
    double arm_elbow_moveUp (boolean fast)
    {
        double l_temptarget;
        if (fast) {
            l_temptarget = a_arm_elbow_position() + ArmElbowServo_Delta_Fast;
        }else{
            l_temptarget = a_arm_elbow_position() + ArmElbowServo_Delta;
        }
        return m_arm_elbow_position(l_temptarget);
    } // arm_elbow_moveUp


    //--------------------------------------------------------------------------
    //
    // arm_elbow_moveDown
    //
    /**
     * move the arm_shoulder servo in the down Direction.
     */
    double arm_elbow_moveDown (boolean fast)
    {
        double l_temptarget;
        if (fast) {
            l_temptarget = a_arm_elbow_position() - ArmElbowServo_Delta_Fast;
        }else{
            l_temptarget = a_arm_elbow_position() - ArmElbowServo_Delta;
        }
        return m_arm_elbow_position(l_temptarget);
    } // arm_elbow_moveDown


    //--------------------------------------------------------------------------
    //
    // a_arm_elbow_position
    //
    /**
     * Access the arm_elbow position.
     */
    double a_arm_elbow_position ()
    {

        return l_arm_elbow_position;
    } // a_arm_elbow_position


    //--------------------------------------------------------------------------
    //
    // m_arm_elbow_position
    //
    /**
     * Mutate the arm elbow position.
     */
    double m_arm_elbow_position (double p_position)
    {
        //
        // Ensure the specific value is legal.
        //
        l_arm_elbow_position = Range.clip
                ( p_position
                        , ArmElbowServo_MinPosition
                        , ArmElbowServo_MaxPosition
                );
        try {
            if (v_servo_arm_elbow != null) {
                v_servo_arm_elbow.setPosition(l_arm_elbow_position);
                return l_arm_elbow_position;
            } else {
                return ServoErrorResultPosition;
            }
        }catch (Exception p_exeception)
        {
            m_warning_message("arm_elbow");
            DbgLog.msg (p_exeception.getLocalizedMessage ());
            return ServoErrorResultPosition;
        }


    } // m_arm_elbow_position


    //--------------------------------------------------------------------------
    //
    // arm_wrist_moveLeft
    //
    /**
     * move the arm wrist servo to the Left.
     */
    double arm_wrist_moveLeft (boolean fast)
    {
        double l_temptarget;
        if (fast) {
            l_temptarget = a_arm_wrist_position() + ArmWristServo_Delta_Fast;
        } else{
            l_temptarget = a_arm_wrist_position() + ArmWristServo_Delta;
        }
        return m_arm_wrist_position(l_temptarget);
    } // arm_wrist_moveLeft


    //--------------------------------------------------------------------------
    //
    // arm_wrist_moveRight
    //
    /**
     * move the arm_wrist servo to the Right.
     */
    double arm_wrist_moveRight (boolean fast)
    {
        double l_temptarget;
        if (fast) {
            l_temptarget = a_arm_wrist_position() - ArmWristServo_Delta_Fast;
        }else{
            l_temptarget = a_arm_wrist_position() - ArmWristServo_Delta;
        }
        return m_arm_wrist_position(l_temptarget);
    } // arm_wrist_moveRight


    //--------------------------------------------------------------------------
    //
    // a_arm_elbow_position
    //
    /**
     * Access the arm_wrist position.
     */
    double a_arm_wrist_position ()
    {

        return l_arm_wrist_position;
    } // a_arm_wrist_position


    //--------------------------------------------------------------------------
    //
    // m_arm_wrist_position
    //
    /**
     * Mutate the arm wrist position.
     */
    double m_arm_wrist_position (double p_position)
    {
        //
        // Ensure the specific value is legal.
        //
        l_arm_wrist_position = Range.clip
                ( p_position
                        , ArmWristServo_MinPosition
                        , ArmWristServo_MaxPosition
                );
        try {
            if (v_servo_arm_wrist != null) {
                v_servo_arm_wrist.setPosition(l_arm_wrist_position);
                return l_arm_wrist_position;
            } else {
                return ServoErrorResultPosition;
            }
        }catch (Exception p_exeception)
        {
            m_warning_message("arm_wrist");
            DbgLog.msg (p_exeception.getLocalizedMessage ());
            return ServoErrorResultPosition;
        }


    } // m_arm_elbow_position

    /**
     * ticks the heartbeat which should happen every time though our loop
     * the heartbeat is wired to the Device Interface Module and is used to make sure our loop is still running
     */

    public void heartbeat_tick(){
        v_led_heartbeat_ticks++;
        if (v_led_heartbeat_ticks > v_led_heartbeat_tickPerToggle){
            heartbeat_toggle();
            v_led_heartbeat_ticks = 0;
        }
    }

    private boolean heartbeat_toggle () {
        if (v_led_heartbeat != null) {
            if (v_led_heartbeat_enabled) {
                v_led_heartbeat_enabled = false;
            } else {
                v_led_heartbeat_enabled = true;
            }
            v_led_heartbeat.enable(v_led_heartbeat_enabled);
            return v_led_heartbeat_enabled;
        }else {
            return false;
        }

    }

    /**
     * Turn on the red led located in the Device Interface Module
     * @return returns true is successfull in turning on the led returns false on error
     */
    public boolean redled_on () {
        try {
            if (v_dim != null) {
                v_dim.setLED(1, true);
                return true;
            }
            return false;
        }catch (Exception p_exeception)
        {
            m_warning_message("dim redled");
            DbgLog.msg (p_exeception.getLocalizedMessage ());
            return false;
        }
    }

    /**
     * Turn off the red led located in the Device Interface Module
     * @return returns true is successfull in turning on the led returns false on error
     */
    public boolean redled_off () {
        try {
            if (v_dim != null) {
                v_dim.setLED(1, false);
                return true;
            }
            return false;
        }catch (Exception p_exeception)
        {
            m_warning_message("dim redled");
            DbgLog.msg (p_exeception.getLocalizedMessage ());
            return false;
        }
    }

    /**
     * Toggles the current state of the red led located in the Device Interface Module
     * <p>calling the function repeataly will give a blink effect.
     * @return returns true is successfull in turning on the led returns false on error
     */

    public boolean redled_toggle () {
        try {
        if (v_dim != null) {
            boolean isEnabled = v_dim.getLEDState(1);
            if (isEnabled) {
                isEnabled = false;
            } else {
                isEnabled = true;
            }
            v_dim.setLED(1, isEnabled);
            return isEnabled;
        }else {
            return false;
        }
        }catch (Exception p_exeception)
        {
            m_warning_message("dim redled");
            DbgLog.msg (p_exeception.getLocalizedMessage ());
            return false;
        }
    }

    /**
     * Turn on the blue led located in the Device Interface Module
     * @return returns true is successfull in turning on the led returns false on error
     */
    public boolean blueled_on () {
        try {
            if (v_dim != null) {
                v_dim.setLED(2, true);
                return true;
            }
            return false;
        }catch (Exception p_exeception)
        {
            m_warning_message("dim blueled");
            DbgLog.msg (p_exeception.getLocalizedMessage ());
            return false;
        }
    }

    /**
     * Turn off the blue led located in the Device Interface Module
     * @return returns true is successfull in turning on the led returns false on error
     */
    public boolean blueled_off () {
        try {
            if (v_dim != null) {
                v_dim.setLED(2, false);
                return true;
            }
            return false;
        }catch (Exception p_exeception)
        {
            m_warning_message("dim blueled");
            DbgLog.msg (p_exeception.getLocalizedMessage ());
            return false;
        }
    }



    /**
     * Toggle the blue led located in the Device Interface Module
     * @return returns true is successfull in turning on the led returns false on error
     */
    public boolean blueled_toggle () {
        try {
            if (v_dim != null) {
                boolean isEnabled = v_dim.getLEDState(2);
                if (isEnabled) {
                    isEnabled = false;
                } else {
                    isEnabled = true;
                }
                v_dim.setLED(2, isEnabled);
                return isEnabled;
            } else {
                return false;
            }
        }catch (Exception p_exeception)
        {
            m_warning_message("dim blueled");
            DbgLog.msg (p_exeception.getLocalizedMessage ());
            return false;
        }
    }

     //--------------------------------------------------------------------------
    //
    // sensor_legecyColor_led
    //

    private boolean sensor_colorLegecy_led (boolean enable)
    {
        try {
            if (v_sensor_colorLegecy != null) {
                v_sensor_colorLegecy_led_enabled = enable;
                v_sensor_colorLegecy.enableLed(enable);
                return enable;
            } else {
                return false;
            }
        }catch (Exception p_exeception)
        {
            m_warning_message("sensor_color");
            DbgLog.msg (p_exeception.getLocalizedMessage ());
            return false;
        }


    } // sensor_legecyColor_led

    //GetgyroHeading
    int sensor_gyro_get_heading(){
        try{
            v_sensor_gy521.portIsReady(5);
            return 0;
        }catch (Exception p_exeception)
        {
            m_warning_message("sensor_gyro");
            DbgLog.msg (p_exeception.getLocalizedMessage ());
            return v_sensor_gyro_heading;
        }
    }

    //GetgyroHeading
    int sensor_gyro_getLast_heading(){
        try{
            return v_sensor_gyro_heading;
        }catch (Exception p_exeception)
        {
            m_warning_message("sensor_gyro");
            DbgLog.msg (p_exeception.getLocalizedMessage ());
            return v_sensor_gyro_heading;
        }
    }

    /**
     * Enable the Legecy Color Sensor
     * @return returns true is successfull returns false on error
     */
    public boolean sensor_colorLegecy_start(){
        try{
            // convert the RGB values to HSV values.
            if(v_sensor_colorLegecy_rgbValues !=null) {
                //turn on the led this is the only way legecy color will detect anything
                sensor_colorLegecy_led(true);
                return true;
            }
            return false;
        }catch (Exception p_exeception)
        {
            m_warning_message("sensor_colorLegecy");
            DbgLog.msg (p_exeception.getLocalizedMessage ());
            return false;
        }
    }
    /**
     * Disables the Legecy Color Sensor
     * @return returns true is successfull returns false on error
     */
    public boolean sensor_colorLegecy_stop(){
        try{
            // convert the RGB values to HSV values.
            if(v_sensor_colorLegecy_rgbValues !=null) {
                //turn on the led this is the only way legecy color will detect anything
                sensor_colorLegecy_led(false);
                return true;
            }
            return false;
        }catch (Exception p_exeception)
        {
            m_warning_message("sensor_colorLegecy");
            DbgLog.msg (p_exeception.getLocalizedMessage ());
            return false;
        }
    }

    private int[] sensor_colorLegecy_read_rgb(){
        try{
            // convert the RGB values to HSV values.
            if(v_sensor_colorLegecy_rgbValues !=null) {
                //v_sensor_color.enableLed(true);
                // wait one cycle.
                //waitOneFullHardwareCycle();
                v_sensor_colorLegecy_rgbValues[0] = v_sensor_colorLegecy.red();
                v_sensor_colorLegecy_rgbValues[1] = v_sensor_colorLegecy.green();
                v_sensor_colorLegecy_rgbValues[2] = v_sensor_colorLegecy.blue();
                v_sensor_colorLegecy_rgbValues[3] = v_sensor_colorLegecy.alpha();
                // wait one cycle.
                //waitOneFullHardwareCycle();
               // v_sensor_color.enableLed(false);
            }
            //Color.RGBToHSV(v_sensor_color.red(), v_sensor_color.green(), v_sensor_color.blue(), v_sensor_color_hsvValues);
            return v_sensor_colorLegecy_rgbValues;

        }catch (Exception p_exeception)
        {
            m_warning_message("sensor_colorLegecy");
            DbgLog.msg (p_exeception.getLocalizedMessage ());
            return v_sensor_colorLegecy_rgbValues;
        }
    }


    public int[] sensor_colorLegecy_getLast_rgb(){
        try{
            return v_sensor_colorLegecy_rgbValues;

        }catch (Exception p_exeception)
        {
            m_warning_message("sensor_colorLegecy");
            DbgLog.msg (p_exeception.getLocalizedMessage ());
            return v_sensor_colorLegecy_rgbValues;
        }
    }


    //osd Legecy Sensor Methods

    //--------------------------------------------------------------------------
    //
    // a_ods_light_detected
    //
    /**
     * Access the amount of light detected by the Optical Distance Sensor.
     */
    private double a_ods_light_detected ()

    {
        double l_return = 0.0;

        if (v_sensor_odsLegecy != null)
        {
            v_sensor_odsLegecy.getLightDetected ();
        }

        return l_return;

    } // a_ods_light_detected

    public boolean sensor_odsLegecy_white_tape_detected(){
        return a_ods_white_tape_detected();
    }

    //--------------------------------------------------------------------------
    //
    // a_ods_white_tape_detected
    //
    /**
     * Access whether the EOP is detecting white tape.
     */
    private boolean a_ods_white_tape_detected ()
    {
        //
        // Assume not.
        //
        boolean l_return = false;

        if (v_sensor_odsLegecy != null)
        {
            //
            // Is the amount of light detected above the threshold for white
            // tape?
            //
            if (v_sensor_odsLegecy.getLightDetected () > 0.8)
            {
                l_return = true;
            }
        }

        //
        // Return
        //
        return l_return;

    } // a_ods_white_tape_detected

//Don't use these inless we are in linerOpMode
//    public void waitOneFullHardwareCycle() throws InterruptedException {
//        this.waitForNextHardwareCycle();
//        Thread.sleep(1L);
//        this.waitForNextHardwareCycle();
//    }
//
//    public void waitForNextHardwareCycle() throws InterruptedException {
//        synchronized(this) {
//            this.wait();
//        }
//    }
//
//    public void sleep(long milliseconds) throws InterruptedException {
//        Thread.sleep(milliseconds);
//    }
}
