package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by adevries on 11/6/2015.
 */

        import android.graphics.Color;
        import android.media.AudioManager;
        import android.media.ToneGenerator;

        import com.qualcomm.ftccommon.DbgLog;
        import com.qualcomm.hardware.ModernRoboticsI2cGyro;
        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorController;
        import com.qualcomm.robotcore.hardware.DigitalChannelController;
        import com.qualcomm.robotcore.hardware.GyroSensor;
        import com.qualcomm.robotcore.hardware.I2cDevice;
        import com.qualcomm.robotcore.hardware.LED;
        import com.qualcomm.robotcore.hardware.LightSensor;
        import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.hardware.TouchSensor;
        import com.qualcomm.robotcore.hardware.UltrasonicSensor;
        import com.qualcomm.robotcore.util.Range;
        import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
        import com.qualcomm.robotcore.hardware.IrSeekerSensor;
        import com.qualcomm.robotcore.util.RobotLog;
        import com.qualcomm.robotcore.util.TypeConversion;
//import com.qualcomm.ftcrobotcontroller.opmodes.ColorSensorDriver;
        import com.qualcomm.robotcore.hardware.ColorSensor;

        import org.usfirst.ControlFreaks.AdafruitLEDBackpack7Seg;

        import java.util.concurrent.locks.Lock;

public class CFPushBotHardware extends OpMode {

    //We Increment the v_loop_ticks each time through our loop
    private long v_loop_ticks = 0;
    // each time through the loop we check to see if v_loop_ticks % v_loop_ticks_slow_count == 0 is so then slow loop = true
    private int v_loop_ticks_slow_count = 20;
    private boolean v_loop_ticks_slow = false;

    //old treads
    //private static final float v_turn_ticks_per_degree = 18.8f;
    //new treads
    private static final float v_turn_ticks_per_degree = 22.6f;

    private static final double v_turn_motorspeed = .5d;
    private static final double v_turn_motorspeed_slow = .2d;
    private static final int v_turn_degrees_heading_drift_error = 5;

    //old treads
    //private static final double driveInches_ticksPerInch = 182.35;

    //new Treads
    private static final double v_drive_inches_ticksPerInch = 200.35;
    //This is the value used to stop before reaching target to account for delay in stop command being processed
    private static final int v_drive_inches_ticksStop = 100;

    //ticks before target to slow to slowdown 1 speed hapens before slowdown 2
    private static final int v_drive_inches_ticksSlowDown1 = 500;
    //ticks before target to slow to slowdown 2 speed
    //private static final double v_drive_inches_ticksSlowDown2 = 1000;
    private static final float v_drive_power_slowdown1 = .30f;
    //private static final float v_drive_power_slowdown2 = .30f;

    //Global Vars to the class
    private static final double ServoErrorResultPosition = -0.0000000001;
    //RPA Base Varables
    private Servo v_servo_rpa_base;
    private static final double RPABaseServo_Delta = 0.0005;
    private static final double RPABaseServo_Delta_Fast = 0.001;
    private double RPABaseServo_MinPosition = 0.01;  //Need to unhook gear int so it goes to zero then rehook servo gear
    private double RPABaseServo_ClimbPosition = 0.69;
    private double RPABaseServo_MaxPosition = 0.70;
    //private double RPABaseServo_MaxPosition_Delta = 0.60;
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

    //wench motor
    private DcMotor v_motor_wench;
    public static final double v_motor_wench_Speed = 0.3;
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
    private static final double ArmWristServo_MinPosition = 0.00;
    private static final double ArmWristServo_MaxPosition = 1.00;
    private double l_arm_wrist_position = 0.0D;  //init arm elbow Position
    //Used in Manual mode to set min trigger pull for slow wrist action

    public static final double ArmWristTrigger_Threshold = 0.2;
    //Used in Manual mode to set min trigger pull for fast wrist action

    public static final double ArmWristTrigger_Threshold_Fast = 0.9;


    // v_servo_flip_right
    private Servo v_servo_flip_right;
    //private static final double FlipRightServo_Delta = 0.005;
    //private static final double FlipRightServo_Delta_Fast = 0.05;
    public static final double FlipRightServo_MinPosition = 0.13;
    public static final double FlipRightServo_MaxPosition = 0.99;
    private double l_flip_right_position = 0.99D;  //init flip_right Position

    // v_servo_flip_left
    private Servo v_servo_flip_left;
   // private static final double FlipLeftServo_Delta = 0.005;
   // private static final double FlipLeftServo_Delta_Fast = 0.05;
    public static final double FlipLeftServo_MinPosition = 0.05;
    public static final double FlipLeftServo_MaxPosition = 0.89;
    private double l_flip_left_position = 0.05D;  //init flip_right Position



    //Legecy Color Sensor
    private ColorSensor v_sensor_colorLegecy;
    private final static String v_sensor_colorLegecy_name="color1";
    private boolean v_sensor_colorLegecy_led_enabled = false;
    // v_sensor_color_hsvValues is an array that will hold the hue, saturation, and value information.
    private float v_sensor_colorLegecy_hsvValues[] = {0F,0F,0F};
    // values is a reference to the v_sensor_color_hsvValues array.
    private final float v_sensor_colorLegecy_values[] = v_sensor_colorLegecy_hsvValues;
    private int v_sensor_colorLegecy_rgbValues[] = {0,0,0,0};

    //Adafruit RGB Sensor
    private ColorSensor v_sensor_color_i2c;
    private static final int v_sensor_color_i2c_led_pin = 5;
    // bEnabled represents the state of the LED.
    private boolean v_sensor_color_i2c_led_enabled = false;
    //red, green, blue, alpha
    private int v_sensor_color_i2c_rgbaValues[]= {0,0,0,0};
    //we read the values in the loop only if the sensor is enabled as they take resourses
    private boolean v_sensor_color_i2c_enabled = false;

    //Legecy OSD Sensor
    //private OpticalDistanceSensor v_sensor_odsLegecy;
    //private boolean v_sensor_odsLegecy_enabled = false;

    //Legecy Light Sensor
    private LightSensor v_sensor_lightLegecy;
    private static final String v_sensor_lightLegecy_name = "light1";
    private boolean v_sensor_lightLegecy_enabled = false;

    private static final String v_sensor_ultraLegecy_name = "ultra1";
    private UltrasonicSensor v_sensor_ultraLegecy;
    private int v_sensor_ultraLegecy_ticksPerRead = 20;
    private double v_sensor_ultraLegecy_distance;

   //Modern Robotics gyro1
    ModernRoboticsI2cGyro v_sensor_gyro_mr;
    GyroSensor v_sensor_gyro;
    //private int v_sensor_gyro_x, v_sensor_gyro_y, v_sensor_gyro_z = 0;
    //private int v_sensor_gyro_heading = 0;

    //Tone Generator to make noise
    ToneGenerator v_tone_generator;

// (tone type, tone duration in ms)
// from a list of predefined tone types


    private LED v_led_heartbeat;
    private boolean v_led_heartbeat_enabled = true;
    private  static final int v_led_heartbeat_tickPerToggle = 20;
    //private int v_led_heartbeat_ticks = 0;

    private DeviceInterfaceModule v_dim;

    private AdafruitLEDBackpack7Seg v_ledseg;
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

    private CFPushBotTelemetry v_CFPushBotTelemetry; //hack to allow use to set error messages using telmentry

    public void setTelemetry(CFPushBotTelemetry objCFPushBotTelemetry){
        v_CFPushBotTelemetry= objCFPushBotTelemetry;
    }
    private void setSecondMessage(String message){
        if (v_CFPushBotTelemetry != null){

            v_CFPushBotTelemetry.set_second_message(message);
        }
    }
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

                // set up the hardware devices we are going to use
            v_dim = hardwareMap.deviceInterfaceModule.get("dim");


        }catch (Exception p_exeception)
        {
            debugLogException("dim","missing",p_exeception);

            v_dim = null;
        }


        try {
            // get a reference to our GyroSensor object.
            v_sensor_gyro = hardwareMap.gyroSensor.get("gyro1");
            // calibrate the gyro.
            v_sensor_gyro.calibrate();
            // make sure the gyro is calibrated.
            while (v_sensor_gyro.isCalibrating())  {
                sleep(50);
            }
            //v_sensor_gyro_mr = (ModernRoboticsI2cGyro) v_sensor_gyro;
            //v_sensor_gyro_mr.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARDINAL);
            sleep(100);
            v_sensor_gyro.resetZAxisIntegrator();
            sleep(200);
            //v_sensor_gyro_heading = v_sensor_gyro.getHeading();
            setSecondMessage("Gyro isCalibrated H:" + v_sensor_gyro.getHeading() );
        }catch(Exception p_exeception){

            m_warning_message ("gyro1");
            DbgLog.msg(p_exeception.getLocalizedMessage());
            v_sensor_gyro = null;
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
            debugLogException("left_drive","missing",p_exeception);
            v_motor_left_drive = null;
        }

        try
        {
            v_motor_right_drive = hardwareMap.dcMotor.get ("right_drive");

            //v_motor_right_drive.setDirection (DcMotor.Direction.REVERSE);
        }
        catch (Exception p_exeception)
        {
            debugLogException("right_drive", "missing", p_exeception);
            v_motor_right_drive = null;
        }
        try {
            int counter = 0;
            reset_drive_encoders();
            while (counter < 10 && have_drive_encoders_reset() == false){
                counter++;
                sleep(100);
                debugLogException("init", "waiting on  rest_drive_encoders() complete r:" + v_motor_right_drive.getMode() + ",l:" + v_motor_left_drive.getMode(), null);

            }
            run_using_encoders();
//             counter = 0;
//            while (counter < 10 && v_motor_right_drive.getMode() != DcMotorController.RunMode.RUN_WITHOUT_ENCODERS && v_motor_left_drive.getMode() != DcMotorController.RunMode.RUN_USING_ENCODERS ){
//                counter++;
//                sleep(100);
//                debugLogException("init", "waiting on  run_using_encoders() complete r:" + v_motor_right_drive.getMode() + ",l:" + v_motor_left_drive.getMode(), null);
//            }

            debugLogException("init", "run_using_encoders() and rest_drive_encoders() complete", null);
        }catch (Exception p_exeception)
        {
            debugLogException("run_using encoders", "error", p_exeception);
            v_motor_right_drive = null;
        }

        try{
            v_tone_generator = new ToneGenerator(AudioManager.STREAM_RING, ToneGenerator.MAX_VOLUME);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_0, 500);
            sleep(500);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_9, 500);
            sleep(500);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_0, 500);
            sleep(500);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_9, 500);
            sleep(500);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_4, 500);
            sleep(200);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_7, 500);
            sleep(200);
            v_tone_generator.startTone(ToneGenerator.TONE_DTMF_9, 500);
        }catch (Exception p_exeception)
        {
            debugLogException("toneGenerator", "missing", p_exeception);

            v_tone_generator = null;
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
            debugLogException("rpa_arm", "missing", p_exeception);

            v_motor_rpa_arm = null;
        }

        try{
            if(v_dim != null) {
                // set the digital channel to output mode.
                // remember, the Adafruit sensor is actually two devices.
                // It's an I2C sensor and it's also an LED that can be turned on or off.
                v_dim.setDigitalChannelMode(v_sensor_color_i2c_led_pin, DigitalChannelController.Mode.OUTPUT);
                // get a reference to our ColorSensor object.
                v_sensor_color_i2c = hardwareMap.colorSensor.get("color2");
                // turn the LED on in the beginning, just so user will know that the sensor is active.
                v_dim.setDigitalChannelState(v_sensor_color_i2c_led_pin, v_sensor_color_i2c_led_enabled);
            }
        } catch (Exception p_exeception)
        {
            debugLogException("sensor_color_i2c", "missing", p_exeception);
            v_sensor_color_i2c = null;
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
            debugLogException("arm_shoulder", "missing", p_exeception);
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
            debugLogException("arm_elbow", "missing", p_exeception);
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
            debugLogException("arm_wrist", "missing", p_exeception);
            v_servo_arm_wrist = null;
        }

        //
        // Connect the flip right servo.
        //
        try
        {
            v_servo_flip_right = hardwareMap.servo.get("flip_right");
            v_servo_flip_right.setPosition (l_flip_right_position);
        }
        catch (Exception p_exeception)
        {
            debugLogException("flip_right", "missing", p_exeception);
            v_servo_flip_right = null;
        }

        //
        // Connect the flip left servo.
        //
        try
        {
            v_servo_flip_left = hardwareMap.servo.get("flip_left");
            v_servo_flip_left.setPosition(l_flip_left_position);
        }
        catch (Exception p_exeception)
        {
            debugLogException("flip_left", "missing", p_exeception);
            v_servo_flip_left = null;
        }

        /*//
        // Connect the heartbeat led.
        //
        try
        {
            v_led_heartbeat = hardwareMap.led.get("heartbeat");
            v_led_heartbeat.enable(v_led_heartbeat_enabled);
        }
        catch (Exception p_exeception)
        {
            debugLogException("heartbeat", "missing", p_exeception);
            v_led_heartbeat = null;
        }*/

        //
        // Connect the RPA Base servo.
        //

        try
        {
            v_servo_rpa_base = hardwareMap.servo.get ("rpa_base");
            v_servo_rpa_base.scaleRange(RPABaseServo_MinPosition,RPABaseServo_MaxPosition); //set the max range to allow the servo to move
            v_servo_rpa_base.setPosition(l_rpa_base_position);
        }
        catch (Exception p_exeception)
        {
            debugLogException("rpa_base", "missing", p_exeception);
            v_servo_rpa_base = null;
        }

        try
        {
            v_motor_wench = hardwareMap.dcMotor.get ("wench");
          //  v_motor_wench.setDirection(DcMotor.Direction.REVERSE);
        }
        catch (Exception p_exeception)
        {
            debugLogException("wench","missing",p_exeception);
            v_motor_wench = null;
        }
        /*try
        {
            // get a reference to our ColorSensor object.
            v_sensor_colorLegecy = hardwareMap.colorSensor.get(v_sensor_colorLegecy_name);
            // bEnabled represents the state of the LED.
            boolean v_sensor_colorLegecy_led_enabled = true;
            // turn the LED on in the beginning, just so user will know that the sensor is active.
            v_sensor_colorLegecy.enableLed(false);

        }
        catch (Exception p_exeception)
        {
            debugLogException(v_sensor_colorLegecy_name, "missing", p_exeception);
            v_sensor_colorLegecy = null;
        }*/

       /* try
        {
            v_sensor_odsLegecy = hardwareMap.opticalDistanceSensor.get ("ods1");

        }
        catch (Exception p_exeception)
        {
            debugLogException("ods1", "missing", p_exeception);
            v_sensor_odsLegecy = null;

        }*/
       /* try
        {
            v_sensor_lightLegecy = hardwareMap.lightSensor.get (v_sensor_lightLegecy_name);

        }
        catch (Exception p_exeception)
        {
            debugLogException(v_sensor_lightLegecy_name, "missing", p_exeception);
            v_sensor_lightLegecy = null;

        }*/

        /*try
        {
            v_sensor_ultraLegecy = hardwareMap.ultrasonicSensor.get (v_sensor_ultraLegecy_name);

        }
        catch (Exception p_exeception)
        {
            debugLogException(v_sensor_ultraLegecy_name, "missing", p_exeception);
            v_sensor_ultraLegecy = null;

        }*/

        try{

            v_ledseg = new AdafruitLEDBackpack7Seg(hardwareMap, "ledseg");

        }catch (Exception p_exeception)
        {
            debugLogException("ledseg", "missing", p_exeception);
            v_ledseg = null;

        }

        //update our telmentry after init so we know if we are missing anything
        if(v_CFPushBotTelemetry != null){
            v_CFPushBotTelemetry.update_telemetry();
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

    /**
     * Used to retrive the total loop count
     * @return The number of time loop has been executed
     */
    public long loopCounter(){
        return v_loop_ticks;
    }

    void debugLogException(String type, String msg, Exception ex){
        if (ex != null){
            m_warning_message(type);
        }
        String debugMessage = type + ":" + msg;
        if (ex != null) {
            String errMsg = ex.getLocalizedMessage();
            if (errMsg != null) {
                debugMessage = debugMessage + errMsg;
            }
        }

        DbgLog.msg(debugMessage );
    }

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
        hardware_stop();
    } // stop



    public boolean is_slow_tick(){
        return  v_loop_ticks_slow;
    }


    /** called each time through the loop needed to sync hardware and look for status changes
     *
     */
    public void hardware_loop(){
        v_loop_ticks++;
        if(v_loop_ticks % v_loop_ticks_slow_count == 0 ){
            v_loop_ticks_slow = true;
        }else{
            v_loop_ticks_slow = false;
        }
        heartbeat_tick();
        if(v_ledseg != null){
            v_ledseg.loop();
        }
        /*if(v_rpabase_moveToClimb == true){
            if (rpa_arm_extended() == false){
                rpaarm_moveUp(true);
            }else{
                v_rpabase_moveToClimb = false;
            }
        }*/
        if(v_loop_ticks_slow){
            // get the heading info.
            // the Modern Robotics' gyro sensor keeps
            // track of the current heading for the Z axis only.
//            if(v_sensor_gyro != null) {
//                v_sensor_gyro_heading = v_sensor_gyro.getHeading();
//                v_sensor_gyro_x = v_sensor_gyro.rawX();
//                v_sensor_gyro_y = v_sensor_gyro.rawY();
//                v_sensor_gyro_z = v_sensor_gyro.rawZ();
//            }
            //the i2c color sensor uses a memory lock that is taxing so we only do this if we are using the color sensor and ever slow loop count
            if(v_sensor_color_i2c_enabled == true){
                v_sensor_color_i2c_rgbaValues[0] = v_sensor_color_i2c.red();
                v_sensor_color_i2c_rgbaValues[1] = v_sensor_color_i2c.green();
                v_sensor_color_i2c_rgbaValues[2] = v_sensor_color_i2c.blue();
                v_sensor_color_i2c_rgbaValues[3] = v_sensor_color_i2c.alpha();
            }
        }

    }

    public void hardware_stop(){
        if(v_led_heartbeat !=null){
            v_led_heartbeat.enable(false);
        }
        if(v_ledseg != null){
            v_ledseg.stop();
        }
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
        //setSecondMessage("set_drive_power " + p_left_power + ":" + p_right_power);
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
/*

    //--------------------------------------------------------------------------
    //
    // run_without_left_drive_encoder
    //
    */
/**
     * Set the left drive wheel encoder to run, if the mode is appropriate.
     *//*

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
*/

    public boolean sound_play_dtmf(int tone, int duration){
        if (v_tone_generator != null) {
            v_tone_generator.startTone(tone, duration);
            return true;
        }else{
            return false;
        }
    }

    /*//--------------------------------------------------------------------------
    //
    // run_without_right_drive_encoder
    //
    *//**
     * Set the right drive wheel encoder to run, if the mode is appropriate.
     *//*
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
    *//**
     * Set both drive wheel encoders to run, if the mode is appropriate.
     *//*
    public void run_without_drive_encoders ()

    {
        //
        // Call other members to perform the action on both motors.
        //
        run_without_left_drive_encoder ();
        run_without_right_drive_encoder ();

    } // run_without_drive_encoders
*/
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

    public boolean isInDriveMode(DcMotorController.RunMode RunMode){
        if (v_motor_left_drive != null && v_motor_right_drive != null){
            if(v_motor_left_drive.getMode() == RunMode && v_motor_right_drive.getMode() == RunMode){
                return true;
            }else{
                return false;
            }
        }
        return true;
    }

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
        reset_left_drive_encoder();
        reset_right_drive_encoder();

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

    /**
     * Access the left drive mode.
     */
    DcMotorController.RunMode a_left_drive_mode ()
    {


        if (v_motor_left_drive != null)
        {
            return v_motor_left_drive.getMode();
        }

        return DcMotorController.RunMode.RUN_TO_POSITION;

    } // a_left_drive_mode

    /**
     * Access the right drive mode.
     */
    DcMotorController.RunMode a_right_drive_mode ()
    {


        if (v_motor_right_drive != null)
        {
            return v_motor_right_drive.getMode();
        }

        return DcMotorController.RunMode.RUN_TO_POSITION;

    } // a_right_drive_mode


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
     * *
     * Indicate whether the drive motors' encoders have reached a value.
     * @param p_left_power  Power 0.0-1.0 for left motor
     * @param p_right_power Power 0.0-1.0 for left motor
     * @param p_left_count  Encoder ticks to travel before stopping
     * @param p_right_count Encoder ticks to travel before stopping
     * @param useGyro   If true then the gyro will try to maintain a heading by slightly decreasing a tracks power
     * @param desiredHeading the desired track call sensor_gyro_heading to get current heading
     * @return true if we have reached the desired distance
     */

    public boolean drive_using_encoders
    ( double p_left_power
            , double p_right_power
            , double p_left_count
            , double p_right_count
            , boolean useGyro
            , int desiredHeading
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

    private long v_drive_inches_ticks_target_right_slowdown;
    private long v_drive_inches_ticks_target_left_slowdown;
    private long v_drive_inches_ticks_target_right_stop;
    private long v_drive_inches_ticks_target_left_stop;
    private long v_drive_inches_ticks_target_right;
    private long v_drive_inches_ticks_target_left;
    private double v_drive_inches_power;
    private boolean v_drive_inches_useGyro;
    private int v_drive_inches_state;
    private int v_drive_inches_heading;
    private boolean v_drive_slowdown1_already_set = false;
    private static final float v_drive_inches_power_gyro_correction = 0.1f;
    private static final int v_drive_inches_power_gyro_correction_max_times = 6;

    public void drive_inches(double power,float inches, boolean useGyro){
        try {

            if (v_motor_left_drive == null || v_motor_right_drive == null){
                return;
            }
            //
            // Tell the system that motor encoders will be used.  This call MUST
            // be in this state and NOT the previous or the encoders will not
            // work.  It doesn't need to be in subsequent states.
            //
            //reset_drive_encoders();
            v_drive_inches_power = power;
            if (power >= 0 ){
                //we are going forward
                v_drive_inches_ticks_target_right = v_motor_right_drive.getCurrentPosition() +  Math.round(inches * v_drive_inches_ticksPerInch);
                v_drive_inches_ticks_target_right_slowdown = v_drive_inches_ticks_target_right - v_drive_inches_ticksSlowDown1;
                v_drive_inches_ticks_target_right_stop = v_drive_inches_ticks_target_right - v_drive_inches_ticksStop;
                v_drive_inches_ticks_target_left = v_motor_left_drive.getCurrentPosition() +  Math.round(inches * v_drive_inches_ticksPerInch);
                v_drive_inches_ticks_target_left_slowdown = v_drive_inches_ticks_target_left - v_drive_inches_ticksSlowDown1;
                v_drive_inches_ticks_target_left_stop = v_drive_inches_ticks_target_left - v_drive_inches_ticksStop;
            }else{
                //we are going backward
                v_drive_inches_ticks_target_right = v_motor_right_drive.getCurrentPosition() -  Math.round(inches * v_drive_inches_ticksPerInch);
                v_drive_inches_ticks_target_right_slowdown = v_drive_inches_ticks_target_right + v_drive_inches_ticksSlowDown1;
                v_drive_inches_ticks_target_right_stop = v_drive_inches_ticks_target_right + v_drive_inches_ticksStop;
                v_drive_inches_ticks_target_left = v_motor_left_drive.getCurrentPosition() -  Math.round(inches * v_drive_inches_ticksPerInch);
                v_drive_inches_ticks_target_left_slowdown = v_drive_inches_ticks_target_left + v_drive_inches_ticksSlowDown1;
                v_drive_inches_ticks_target_left_stop = v_drive_inches_ticks_target_left + v_drive_inches_ticksStop;

            }

            v_drive_inches_useGyro = useGyro;
            v_drive_inches_state = 0;
            v_drive_slowdown1_already_set = false;
            setSecondMessage("drive_inches: p: " + v_drive_inches_power
                    + ",tl:" + v_drive_inches_ticks_target_left + ",tr:" + v_drive_inches_ticks_target_right
                    + ",tlsl:" + v_drive_inches_ticks_target_left_slowdown + ",trsl:" + v_drive_inches_ticks_target_right_slowdown
                    + ",tlst:" + v_drive_inches_ticks_target_left_stop + ",trst:" + v_drive_inches_ticks_target_right_stop
                    + ",l:" + a_left_encoder_count() + ",r:" + a_right_encoder_count() );

        }catch (Exception p_exeception)
        {
            debugLogException("drive inches", "drive_inches", p_exeception);


        }
    }

    /**
     * Inits the led 7 segment counter to start a count down in seconds
     * @param seconds
     * @return
     */
    public boolean led7seg_timer_init(int seconds){
        return false;
    }

    private boolean isFirstButtonPress = true;

    public boolean manualModeButtonPress(){
        if (isFirstButtonPress){
            isFirstButtonPress = false;
            v_ledseg.startTimer(120);
        }
        return true;
    }

    /**
     * Inits the led 7 segment counter to start a count down in seconds
     * @return
     */

    public boolean led7seg_test(){
        if (v_ledseg != null){

            v_ledseg.writetest();
            return true;
        }
        return false;
    }

    /**
     * starts led 7 segment counter to to count down in seconds
     * @param seconds
     * @return
     */
    int v_led7seg_timer_seconds = 0;
    boolean v_led7seg_timer_running = false;
    public boolean led7seg_timer_start(int seconds){
        if (v_ledseg != null){

            v_ledseg.startTimer(seconds);
            return true;
        }
        return false;
    }

    public boolean led7seg_is_enabled(){
        if (v_ledseg != null){
            return v_ledseg.isEnabled();
        }
        return false;
    }
    public boolean led7seg_enabled(boolean enabled){
        if (v_ledseg != null){
            return v_ledseg.enabled(enabled);
        }
        return false;
    }

    public boolean drive_inches_complete(){
        if (v_motor_left_drive == null || v_motor_right_drive == null){
            return true;
        }

        int v_drive_inches_ticks_left = v_motor_left_drive.getCurrentPosition();
        int v_drive_inches_ticks_right = v_motor_right_drive.getCurrentPosition();

        switch(v_drive_inches_state){

            case 0:
                if (v_drive_inches_useGyro) {
                    v_drive_inches_heading = sensor_gyro_get_heading();
                }
                //add a delay to handle any stupid issues with the ftc dc motor controller timming
                sleep(2);
                set_drive_power(v_drive_inches_power, v_drive_inches_power);
                setSecondMessage("drive_inches_complete: set the drive power "
                     +  " p: " + v_drive_inches_power
                        + ",tl:" + v_drive_inches_ticks_target_left + ",tr:" + v_drive_inches_ticks_target_right
                        + ",tlsl:" + v_drive_inches_ticks_target_left_slowdown + ",trsl:" + v_drive_inches_ticks_target_right_slowdown
                        + ",tlst:" + v_drive_inches_ticks_target_left_stop + ",trst:" + v_drive_inches_ticks_target_right_stop
                    + ",l:" + v_drive_inches_ticks_left + ",r:" + v_drive_inches_ticks_right);
                v_drive_inches_state++;
                break;
            case 1:
                //
                // Have the motor shafts turned the required amount?
                //
                // If they haven't, then the op-mode remains in this state (i.e this
                // block will be executed the next time this method is called).
                //
                if (

                        (v_drive_inches_power >= 0.0d &&  (v_drive_inches_ticks_left >= v_drive_inches_ticks_target_left_stop &&  v_drive_inches_ticks_right >= v_drive_inches_ticks_target_right_stop ))
                ||
                        (v_drive_inches_power < 00.0d &&  (v_drive_inches_ticks_left <= v_drive_inches_ticks_target_left_stop &&  v_drive_inches_ticks_right <= v_drive_inches_ticks_target_right_stop ))
                )
                {
                    //
                    // Stop the motors.
                    //
                    set_drive_power(0.0f, 0.0f);
                    //
                    sleep(50);
                    setSecondMessage("drive_inches_complete: drive complete "
                            +  " p: " + v_drive_inches_power
                            + ",tl:" + v_drive_inches_ticks_target_left + ",tr:" + v_drive_inches_ticks_target_right
                            + ",tlsl:" + v_drive_inches_ticks_target_left_slowdown + ",trsl:" + v_drive_inches_ticks_target_right_slowdown
                            + ",tlst:" + v_drive_inches_ticks_target_left_stop + ",trst:" + v_drive_inches_ticks_target_right_stop
                            + ",l:" + v_drive_inches_ticks_left + ",r:" + v_drive_inches_ticks_right );
                    v_drive_inches_state++;
                    return true;
                /*}
                else if (
                            (v_drive_inches_power >= 0 &&  (v_drive_inches_ticks_left >= (v_drive_inches_ticks_target_left - v_drive_inches_ticksSlowDown2)  ||  v_drive_inches_ticks_right >= (v_drive_inches_ticks_target_right -  v_drive_inches_ticksSlowDown2) ))
                            ||
                            (v_drive_inches_power < 0 &&  (v_drive_inches_ticks_left <= (v_drive_inches_ticks_target_left + v_drive_inches_ticksSlowDown2) ||  v_drive_inches_ticks_right <= (v_drive_inches_ticks_target_right + v_drive_inches_ticksSlowDown2) ))
                        )
                {
                    //
                    // slow the motors to slowdown 2
                    //
                    if(v_drive_inches_power > v_drive_power_slowdown2) {
                        set_drive_power(v_drive_power_slowdown2, v_drive_power_slowdown2);
                        v_drive_inches_power = v_drive_power_slowdown2;
                        setSecondMessage("drive_inches_complete: slowdown 2 "
                                +  " p: " + v_drive_inches_power + ",tl:" + v_drive_inches_ticks_target_left + ",tr:" + v_drive_inches_ticks_target_right
                                + ",l:" + v_drive_inches_ticks_left + ",r:" + v_drive_inches_ticks_right);
                    }*/

                }else if ( v_drive_slowdown1_already_set == false &&
                            (v_drive_inches_power >= 0.0d &&  (v_drive_inches_ticks_left >= v_drive_inches_ticks_target_left_slowdown  ||  v_drive_inches_ticks_right >= v_drive_inches_ticks_target_right_slowdown ))
                            ||
                            (v_drive_inches_power < 0.0d &&  (v_drive_inches_ticks_left <= v_drive_inches_ticks_target_left_slowdown ||  v_drive_inches_ticks_right <= v_drive_inches_ticks_target_right_slowdown ))
                        )
                {

                    //
                    // slow the motors to slowdown 1
                    //
                    if((v_drive_inches_power > 0.0d && v_drive_inches_power > v_drive_power_slowdown1)
                        || (v_drive_inches_power < 00.0d && v_drive_inches_power < (0- v_drive_power_slowdown1))
                        ) {
                        if(v_drive_inches_power > 0.0d){
                            set_drive_power(v_drive_power_slowdown1, v_drive_power_slowdown1);
                        }else{
                            set_drive_power(0-v_drive_power_slowdown1, 0-v_drive_power_slowdown1);
                        }
                        v_drive_slowdown1_already_set = true;
                        v_drive_inches_power = v_drive_power_slowdown1;
                        setSecondMessage("drive_inches_complete: slow down 1 "
                                +  " p: " + v_drive_inches_power
                                + ",tl:" + v_drive_inches_ticks_target_left + ",tr:" + v_drive_inches_ticks_target_right
                                + ",tlsl:" + v_drive_inches_ticks_target_left_slowdown + ",trsl:" + v_drive_inches_ticks_target_right_slowdown
                                + ",tlst:" + v_drive_inches_ticks_target_left_stop + ",trst:" + v_drive_inches_ticks_target_right_stop
                                + ",l:" + v_drive_inches_ticks_left + ",r:" + v_drive_inches_ticks_right
                        );
                    }
                }
                else if(v_drive_inches_useGyro){
                    //the logic here is to try to hold a gyro heading by slowing a track down a touch v_drive_inches_power_gyro_correction
                    //the issue is our gyro is slow to refresh so may need to only do this a couple of loops then turn off
                    //not sure yet still testing 11/19/2015 Two days to first competition wow this is tight deadline
                    int headingDifference;
                    int currentHeading = sensor_gyro_get_heading();
                    if ((v_drive_inches_heading - currentHeading) > 180 ) {
                        currentHeading = 360 + currentHeading;
                    }else if ((v_drive_inches_heading - currentHeading) < -180 ) {
                        currentHeading = currentHeading - 360;
                    }
                    headingDifference = Math.abs(currentHeading-v_drive_inches_heading);
                    //hard limit of no more then 3 times the correction
                    if (headingDifference > v_drive_inches_power_gyro_correction_max_times){
                        headingDifference = v_drive_inches_power_gyro_correction_max_times;
                    }
                    float powerCorrectAmount = v_drive_inches_power_gyro_correction * headingDifference;
                    if (v_drive_inches_power < 0.0d){
                        powerCorrectAmount = 0 - powerCorrectAmount;
                    }
                    if ((v_drive_inches_heading > currentHeading && v_drive_inches_power > 0) || (v_drive_inches_heading < currentHeading && v_drive_inches_power < 0) ){
                        set_drive_power(v_drive_inches_power, v_drive_inches_power - powerCorrectAmount );
                    }else if ((v_drive_inches_heading < currentHeading && v_drive_inches_power > 0) || (v_drive_inches_heading > currentHeading && v_drive_inches_power < 0)){
                        set_drive_power(v_drive_inches_power - powerCorrectAmount, v_drive_inches_power );
                    }else {
                        set_drive_power(v_drive_inches_power, v_drive_inches_power );
                    }
                    if(v_loop_ticks_slow) {
                        setSecondMessage("drive_inches_complete: gyro ch:" + currentHeading + ", th:" + v_drive_inches_heading
                                +  " p: " + v_drive_inches_power
                                + ",tl:" + v_drive_inches_ticks_target_left + ",tr:" + v_drive_inches_ticks_target_right
                                + ",tlsl:" + v_drive_inches_ticks_target_left_slowdown + ",trsl:" + v_drive_inches_ticks_target_right_slowdown
                                + ",tlst:" + v_drive_inches_ticks_target_left_stop + ",trst:" + v_drive_inches_ticks_target_right_stop
                                + ",l:" + v_drive_inches_ticks_left + ",r:" + v_drive_inches_ticks_right
                                + ",lp:" + v_motor_left_drive.getPower() + ",rp:" + v_motor_right_drive.getPower());
                    }
                }else{
                    if(v_loop_ticks_slow) {
                        setSecondMessage("drive_inches_complete: "
                            +  " p: " + v_drive_inches_power
                            + ",tl:" + v_drive_inches_ticks_target_left + ",tr:" + v_drive_inches_ticks_target_right
                            + ",tlsl:" + v_drive_inches_ticks_target_left_slowdown + ",trsl:" + v_drive_inches_ticks_target_right_slowdown
                            + ",tlst:" + v_drive_inches_ticks_target_left_stop + ",trst:" + v_drive_inches_ticks_target_right_stop
                            + ",l:" + v_drive_inches_ticks_left + ",r:" + v_drive_inches_ticks_right
                            + ",lp:" + v_motor_left_drive.getPower() + ",rp:" + v_motor_right_drive.getPower()
                        );
                    }
                }
            break;
            default:
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

    private long v_turn_degrees_ticks_target_right;
    private long v_turn_degrees_ticks_target_left;

    private int v_turn_degrees_heading_target;
    private int v_turn_degrees_heading_target_slow;
    private int v_turn_degrees_heading_target_stop;
    private boolean v_turn_degrees_heading_target_360round;
    private boolean v_turn_degrees_heading_start_error_360round;
    private boolean v_turn_degrees_heading_target_slow_360round;
    private boolean v_turn_degrees_heading_target_stop_360round;
    private int v_turn_degrees_heading_start;
    private static final int v_turn_degrees_heading_overshoot_slowdown = 30;
    private static final int v_turn_degrees_heading_overshoot_stop = 5;
    private int v_turn_degrees_heading_start_error;
    private boolean v_turn_degrees_usingGyro;
    private boolean v_turn_degrees_iscwturn;
    private boolean v_turn_degrees_isSlowTurn;
    private int v_turn_degrees_state;
    /**
     *
     * @param degrees the amount in degrees you want to turn postive number is to the right negitive to the left
     * @param turnSlow make a slowTurn
     * @param useGyro use the Gyro to turn if false then ticks of the encoder will be used
     *
     */

    public void turn_degrees(int degrees, boolean turnSlow, boolean useGyro){
        //Do nothing is turn is zero
        if(degrees == 0 || v_motor_left_drive == null || v_motor_right_drive == null){
            return;
        }


        v_turn_degrees_state = 0;
        v_turn_degrees_usingGyro = useGyro;
        v_turn_degrees_isSlowTurn = turnSlow;
        if (degrees > 0) {
            //greater then 0 turn cw or to the right
            v_turn_degrees_iscwturn = true;
        } else {
            v_turn_degrees_iscwturn = false;
        }

        if (v_turn_degrees_usingGyro == true && v_sensor_gyro == null ){
            setSecondMessage("turn_degrees() no Gyro to use");
            v_turn_degrees_usingGyro = false;
        }
        if (v_turn_degrees_usingGyro == true) {

            //run_without_drive_encoders();
            //set the state to 1 as we don't need to wait for drive encoder reset

            //v_sensor_gyro.resetZAxisIntegrator();
            v_turn_degrees_heading_start = sensor_gyro_get_heading();

            if (v_turn_degrees_iscwturn) {
                //used to account for gyro drift so a drift -1 on the intial start doesn't mark turn complete
                v_turn_degrees_heading_start_error = v_turn_degrees_heading_start - v_turn_degrees_heading_drift_error;
                if (v_turn_degrees_heading_start_error < 0){
                    v_turn_degrees_heading_start_error = 360 + v_turn_degrees_heading_start_error;
                    v_turn_degrees_heading_start_error_360round = true;
                }else{
                    v_turn_degrees_heading_start_error_360round = false;
                }
                //clockwise turn to the right so gyro will count up
                v_turn_degrees_heading_target = v_turn_degrees_heading_start + degrees;
                //add above could be greater then 360 so correct
                if (v_turn_degrees_heading_target >= 360){
                    v_turn_degrees_heading_target =  v_turn_degrees_heading_target - 360;
                    v_turn_degrees_heading_target_360round = true;
                }else{
                    v_turn_degrees_heading_target_360round = false;
                }

                //set our slowdown target
                v_turn_degrees_heading_target_slow = v_turn_degrees_heading_start + (degrees - v_turn_degrees_heading_overshoot_slowdown);
                //add above could be greater then 360 so correct
                if (v_turn_degrees_heading_target_slow >= 360){
                    v_turn_degrees_heading_target_slow =  v_turn_degrees_heading_target_slow - 360;
                    v_turn_degrees_heading_target_slow_360round = true;
                }else{
                    v_turn_degrees_heading_target_slow_360round = false;
                }

                //set our stop target

                v_turn_degrees_heading_target_stop = v_turn_degrees_heading_start + (degrees - v_turn_degrees_heading_overshoot_stop);
                //add above could be greater then 360 so correct
                if (v_turn_degrees_heading_target_stop >= 360){
                    v_turn_degrees_heading_target_stop =  v_turn_degrees_heading_target_stop - 360;
                    v_turn_degrees_heading_target_stop_360round = true;
                }else{
                    v_turn_degrees_heading_target_stop_360round = false;
                }

            } else {
                v_turn_degrees_heading_start_error = v_turn_degrees_heading_start + v_turn_degrees_heading_drift_error;
                if (v_turn_degrees_heading_start_error >= 360){
                    v_turn_degrees_heading_start_error =  v_turn_degrees_heading_start_error - 360;
                    v_turn_degrees_heading_start_error_360round = true;
                }else{
                    v_turn_degrees_heading_start_error_360round = false;
                }
                //clockwise turn to the left so gyro will count down degrees is a negative so we just add below which will subtract
                v_turn_degrees_heading_target = v_turn_degrees_heading_start + degrees;
                //degrees is a negitive so add above could be greater then 360
                if (v_turn_degrees_heading_target < 0){
                    v_turn_degrees_heading_target = 360 + v_turn_degrees_heading_target;
                    v_turn_degrees_heading_target_360round = true;
                }else{
                    v_turn_degrees_heading_target_360round = false;
                }

                //set our slow down target
                v_turn_degrees_heading_target_slow = v_turn_degrees_heading_start + (degrees + v_turn_degrees_heading_overshoot_slowdown);
                //degrees is a negitive so add above could be greater then 360
                if (v_turn_degrees_heading_target_slow < 0){
                    v_turn_degrees_heading_target_slow = 360 + v_turn_degrees_heading_target_slow;
                    v_turn_degrees_heading_target_slow_360round = true;
                }else{
                    v_turn_degrees_heading_target_slow_360round = false;
                }

                //set our stop target
                v_turn_degrees_heading_target_stop = v_turn_degrees_heading_start + (degrees + v_turn_degrees_heading_overshoot_stop);
                //degrees is a negitive so add above could be greater then 360
                if (v_turn_degrees_heading_target_stop < 0){
                    v_turn_degrees_heading_target_stop = 360 + v_turn_degrees_heading_target_stop;
                    v_turn_degrees_heading_target_stop_360round = true;
                }else{
                    v_turn_degrees_heading_target_stop_360round = false;
                }

            }
            setSecondMessage("turn_degrees: d:" + degrees
                            + ",ch:" + v_turn_degrees_heading_start
                            + ",ht:" + v_turn_degrees_heading_target
                            + ",htsl:" + v_turn_degrees_heading_target_slow
                            + ",htst:" + v_turn_degrees_heading_target_stop
                            + ",hte:" + v_turn_degrees_heading_start_error
                            + ",htr:" + v_turn_degrees_heading_target_360round
                            + ",htslr:" + v_turn_degrees_heading_target_slow_360round
                            + ",htstr:" + v_turn_degrees_heading_target_stop_360round
                            + ",hte:" + v_turn_degrees_heading_start_error_360round
            );

        }else {
            int ticks = Math.round(Math.abs(degrees) * v_turn_ticks_per_degree);
            if (v_turn_degrees_iscwturn) {
                v_turn_degrees_ticks_target_left = v_motor_left_drive.getCurrentPosition() + ticks;
                v_turn_degrees_ticks_target_right = v_motor_right_drive.getCurrentPosition() - ticks;
            } else {
                v_turn_degrees_ticks_target_left = v_motor_left_drive.getCurrentPosition() - ticks;
                v_turn_degrees_ticks_target_right =  v_motor_right_drive.getCurrentPosition() + ticks;
            }
            setSecondMessage("turn_degrees: ticks:" + ticks + ", lt:" + v_turn_degrees_ticks_target_left + " rt:" + v_turn_degrees_ticks_target_right + ", re:" + v_motor_right_drive.getCurrentPosition() + ", le:" + v_motor_left_drive.getCurrentPosition() );

        }


    }

    /**
     * Used to tell if the turn is complete turn_degrees must be called first
     *
     * @return true if the turn is complete false if not
     */

    public boolean turn_complete(){

        try{

            if(v_motor_left_drive == null || v_motor_right_drive == null){
                setSecondMessage("turn_complete: no motors");
                v_turn_degrees_state = -1;
                return true;
            }

            switch(v_turn_degrees_state){
                case 0:
                    if (v_turn_degrees_iscwturn) {
                        if (v_turn_degrees_isSlowTurn) {
                            //have an issue where motors not turning on at same time so need to call directly
                            //turning right so turn on left motor first
                            v_motor_left_drive.setPower(v_turn_motorspeed_slow);
                            v_motor_right_drive.setPower(0 - v_turn_motorspeed_slow);
                            //set_drive_power(v_turn_motorspeed_slow, 0-v_turn_motorspeed_slow);
                            sleep(2);
                            setSecondMessage("turn_complete: set slow turn cw r:" + v_motor_right_drive.getPower() + "l:" + v_motor_left_drive.getPower());


                        }else {
                            //turning right so turn on left motor first
                            v_motor_left_drive.setPower(v_turn_motorspeed);
                            v_motor_right_drive.setPower(0 - v_turn_motorspeed);
                            //set_drive_power(v_turn_motorspeed, 0- v_turn_motorspeed);
                            sleep(2);
                            setSecondMessage("turn_complete: set fast turn cw r:" + v_motor_right_drive.getPower() + "l:" + v_motor_left_drive.getPower());

                        }
                        //v_motor_left_drive.setPowerFloat();
                        //v_motor_right_drive.setPowerFloat();
                    } else {
                        if (v_turn_degrees_isSlowTurn) {
                            //turning left so turn on right motor first
                            v_motor_right_drive.setPower(v_turn_motorspeed_slow);
                            v_motor_left_drive.setPower(0 - v_turn_motorspeed_slow);
                            //set_drive_power(0-v_turn_motorspeed_slow,  v_turn_motorspeed_slow);
                            sleep(2);
                            setSecondMessage("turn_complete: set slow turn ccw r:" + v_motor_right_drive.getPower() + "l:" + v_motor_left_drive.getPower());
                        }else {
                            //turning left so turn on right motor first
                            v_motor_right_drive.setPower(v_turn_motorspeed);
                            v_motor_left_drive.setPower(0 - v_turn_motorspeed);
                            set_drive_power(0-v_turn_motorspeed, v_turn_motorspeed);
                            sleep(2);
                            setSecondMessage("turn_complete: set fast turn ccw r:" + v_motor_right_drive.getPower() + "l:" + v_motor_left_drive.getPower() );
                        }
                        //v_motor_right_drive.setPowerFloat();
                        //v_motor_left_drive.setPowerFloat();
                    }

                    v_turn_degrees_state++;
                    break;
                case 1:
                    if (v_turn_degrees_usingGyro) {
                        int currentHeading = sensor_gyro_get_heading();
                        //debugLogException("turn_complete ", "H:" + currentHeading + ",T:" + v_turn_degrees_heading_target + ",SH:" + v_turn_degrees_heading_start + ",SHE:" + v_turn_degrees_heading_start_error, null );

                        if(v_turn_degrees_iscwturn){
                            //if we are turning clockwise then we stop >= then our target

                            if(
                                        (
                                                (v_turn_degrees_heading_target_360round == false &&
                                            currentHeading >= v_turn_degrees_heading_target
                                                )
                                                        ||
                                                (v_turn_degrees_heading_target_360round == true &&
                                                        currentHeading >= v_turn_degrees_heading_target &&
                                                        currentHeading < v_turn_degrees_heading_start_error)
                                        )
                                            &&(
                                                ( v_turn_degrees_heading_target_360round == true
                                                        && v_turn_degrees_heading_start_error_360round == true &&
                                                    currentHeading < v_turn_degrees_heading_start_error
                                                 ) ||
                                                ( v_turn_degrees_heading_target_360round == false && v_turn_degrees_heading_start_error_360round == false &&
                                                        currentHeading > v_turn_degrees_heading_start_error
                                                ) ||
                                                ( v_turn_degrees_heading_target_360round == true && v_turn_degrees_heading_start_error_360round == false &&
                                                        currentHeading > v_turn_degrees_heading_start_error
                                                )
                                                ||
                                                ( v_turn_degrees_heading_target_360round == false && v_turn_degrees_heading_start_error_360round == true &&
                                                        currentHeading < v_turn_degrees_heading_start_error
                                                )
                                            )

                                    )

                            {
                                //turning right so stop right first then left
                                v_motor_right_drive.setPower(0.0d);
                                v_motor_left_drive.setPower(0.0d);
                                //v_motor_right_drive.setPowerFloat();
                                //v_motor_left_drive.setPowerFloat();
                                //set_drive_power(0.0f, 0.0f);
                                v_turn_degrees_state++;
                                setSecondMessage("turn_complete: done cw hc:" + currentHeading
                                        + ",ht:" + v_turn_degrees_heading_target
                                        + ",htsl:" + v_turn_degrees_heading_target_slow
                                        + ",htst:" + v_turn_degrees_heading_target_stop
                                        + ",hte:" + v_turn_degrees_heading_start_error
                                        + ",htr:" + v_turn_degrees_heading_target_360round
                                        + ",htslr:" + v_turn_degrees_heading_target_slow_360round
                                        + ",htstr:" + v_turn_degrees_heading_target_stop_360round
                                        + ",hte:" + v_turn_degrees_heading_start_error_360round);
                                return true;
                            }
                            //else are we within our slowdown offset of the target then
                            else if(
                                    (
                                            (v_turn_degrees_heading_target_360round == false &&
                                                    currentHeading >= v_turn_degrees_heading_target
                                            )
                                                    ||
                                                    (v_turn_degrees_heading_target_360round == true &&
                                                            currentHeading >= v_turn_degrees_heading_target &&
                                                            currentHeading < v_turn_degrees_heading_start_error)
                                    )
                                            &&(
                                            ( v_turn_degrees_heading_target_360round == true
                                                    && v_turn_degrees_heading_start_error_360round == true &&
                                                    currentHeading < v_turn_degrees_heading_start_error
                                            ) ||
                                                    ( v_turn_degrees_heading_target_360round == false && v_turn_degrees_heading_start_error_360round == false &&
                                                            currentHeading > v_turn_degrees_heading_start_error
                                                    ) ||
                                                    ( v_turn_degrees_heading_target_360round == true && v_turn_degrees_heading_start_error_360round == false &&
                                                            currentHeading < v_turn_degrees_heading_start_error
                                                    )
                                                    ||
                                                    ( v_turn_degrees_heading_target_360round == false && v_turn_degrees_heading_start_error_360round == true &&
                                                            currentHeading < v_turn_degrees_heading_start_error
                                                    )
                                    )

                                    )

                            {
                                //turning right so stop right first then left
                                v_motor_right_drive.setPower(0.0d);
                                v_motor_left_drive.setPower(0.0d);
                                //v_motor_right_drive.setPowerFloat();
                                //v_motor_left_drive.setPowerFloat();
                                //set_drive_power(0.0f, 0.0f);
                                v_turn_degrees_state++;
                                setSecondMessage("turn_complete: done cw hc:" + currentHeading
                                        + ",ht:" + v_turn_degrees_heading_target
                                        + ",htsl:" + v_turn_degrees_heading_target_slow
                                        + ",htst:" + v_turn_degrees_heading_target_stop
                                        + ",hte:" + v_turn_degrees_heading_start_error
                                        + ",htr:" + v_turn_degrees_heading_target_360round
                                        + ",htslr:" + v_turn_degrees_heading_target_slow_360round
                                        + ",htstr:" + v_turn_degrees_heading_target_stop_360round
                                        + ",hte:" + v_turn_degrees_heading_start_error_360round);
                                return true;
                            }


                        }else{
                            //we are turning counterclockwise so we stop <= our target heading
                            if(
                                    (

                                            (v_turn_degrees_heading_target_360round == false &&
                                                    currentHeading <= v_turn_degrees_heading_target)
                                                    ||
                                                    (v_turn_degrees_heading_target_360round == true &&
                                                            currentHeading <= v_turn_degrees_heading_target &&
                                                            currentHeading > v_turn_degrees_heading_start_error)
                                    )
                                    && ( //the following prevents early motor stop due to drift of gyro or error on start of turn the wrong way
                                        ( v_turn_degrees_heading_target_360round == true && v_turn_degrees_heading_start_error_360round == true &&
                                                currentHeading > v_turn_degrees_heading_start_error
                                        ) ||
                                        ( v_turn_degrees_heading_start_error_360round == false && v_turn_degrees_heading_start_error_360round == false
                                                && currentHeading < v_turn_degrees_heading_start_error
                                        )||
                                        ( v_turn_degrees_heading_target_360round == true && v_turn_degrees_heading_start_error_360round == false &&
                                                currentHeading > v_turn_degrees_heading_start_error
                                        )
                                        ||
                                        ( v_turn_degrees_heading_target_360round == false && v_turn_degrees_heading_start_error_360round == true &&
                                                currentHeading > v_turn_degrees_heading_start_error
                                        )
                                    )
                                    ){
                                //turning left so stop left first then right
                                v_motor_left_drive.setPower(0.0d);
                                v_motor_right_drive.setPower(0.0d);
                                //v_motor_left_drive.setPowerFloat();
                                //v_motor_right_drive.setPowerFloat();
                                //set_drive_power(0.0f, 0.0f);
                                v_turn_degrees_state++;
                                setSecondMessage("turn_complete: done ccw "
                                        + "hc:" + currentHeading
                                        + ",ht:" + v_turn_degrees_heading_target
                                        + ",htsl:" + v_turn_degrees_heading_target_slow
                                        + ",htst:" + v_turn_degrees_heading_target_stop
                                        + ",hte:" + v_turn_degrees_heading_start_error
                                        + ",htr:" + v_turn_degrees_heading_target_360round
                                        + ",htslr:" + v_turn_degrees_heading_target_slow_360round
                                        + ",htstr:" + v_turn_degrees_heading_target_stop_360round
                                        + ",hte:" + v_turn_degrees_heading_start_error_360round);
                                return true;
                            }
                        }
                        if (is_slow_tick()){
                            setSecondMessage("turn_complete: Waiting on heading "
                                    + " hc:" + currentHeading
                                    + ",ht:" + v_turn_degrees_heading_target
                                    + ",htsl:" + v_turn_degrees_heading_target_slow
                                    + ",htst:" + v_turn_degrees_heading_target_stop
                                    + ",hte:" + v_turn_degrees_heading_start_error
                                    + ",htr:" + v_turn_degrees_heading_target_360round
                                    + ",htslr:" + v_turn_degrees_heading_target_slow_360round
                                    + ",htstr:" + v_turn_degrees_heading_target_stop_360round
                                    + ",hte:" + v_turn_degrees_heading_start_error_360round);
                        }


                    }else {
                        if ((v_turn_degrees_iscwturn == true &&  v_motor_left_drive.getCurrentPosition() >= v_turn_degrees_ticks_target_left &&  v_motor_right_drive.getCurrentPosition() <= v_turn_degrees_ticks_target_right )
                                ||
                                (v_turn_degrees_iscwturn == false &&  v_motor_left_drive.getCurrentPosition() <= v_turn_degrees_ticks_target_left &&  v_motor_right_drive.getCurrentPosition() >= v_turn_degrees_ticks_target_right )
                                ) {
                            set_drive_power(0.0f, 0.0f);
                            setSecondMessage("turn_complete: encoders reached value lt:" + v_turn_degrees_ticks_target_left + " rt:" + v_turn_degrees_ticks_target_right + ", re:" + v_motor_right_drive.getCurrentPosition() + ", le:" + v_motor_left_drive.getCurrentPosition() + ", rp:" + v_motor_right_drive.getPower() + ", lp:" + v_motor_left_drive.getPower());
                            v_turn_degrees_state++;
                            return true;
                        }else {
                            //if (is_slow_tick()){
                                setSecondMessage("turn_complete: Waiting on encoders lt:" + v_turn_degrees_ticks_target_left + " rt:" + v_turn_degrees_ticks_target_right + ", re:" + v_motor_right_drive.getCurrentPosition() + ", le:" + v_motor_left_drive.getCurrentPosition() + ", rp:" + v_motor_right_drive.getPower() + ", lp:" + v_motor_left_drive.getPower() );
                            //}
                        }
                    }
                    break;
                default:
                    return true;
            }

            return false;
        } catch (Exception p_exeception)
        {
            debugLogException("turn_complete:", "error " + p_exeception.getMessage() , p_exeception);
            return false;

        }
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



    //pull wench
    boolean wench_moveUp ()
    {
        m_wench_power(v_motor_wench_Speed);
        return true;

    } // rpaarm_moveUp
    //--------------------------------------------------------------------------
    //
    // m_rpa_arm_power
    //
    /**
     * Access the rpa arm motor's power level.
     */
    void m_wench_power (double p_level)
    {
        if (v_motor_wench != null)
        {
            if(p_level > 0){
                v_motor_wench.setPower(p_level);
            }
            else{
                v_motor_wench.setPower(0);
            }
        }

    } // m_left_arm_power




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
    } // rpaarm_moveDown


    //--------------------------------------------------------------------------
    //
    // rpabase_moveUp
    //
    /**
     * move the rpabase servo in the up Direction.
     */
    public double rpabase_moveUp (boolean fast)
    {
        double l_temptarget;
        //move the wrist out of the way
        m_arm_wrist_position(ArmWristServo_MinPosition);
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
    public double rpabase_moveDown (boolean fast)
    {
        double l_temptarget;
        //move the wrist out of the way
        m_arm_wrist_position(ArmWristServo_MinPosition);
        if (fast) {
            l_temptarget = a_rpabase_position() - RPABaseServo_Delta_Fast;
        }else{
            l_temptarget = a_rpabase_position() - RPABaseServo_Delta;
        }
        return m_rpabase_position(l_temptarget);
    } // rpabase_moveDown


    //--------------------------------------------------------------------------
    //
    // rpabase_moveDown
    //
    /**
     * move the rpabase servo in the down Direction.
     */
    private boolean v_rpabase_moveToClimb = false;
    public double rpabase_moveToClimb ()
    {
        //move the wrist out of the way
        v_rpabase_moveToClimb = true;
        //m_arm_wrist_position(ArmWristServo_MinPosition);
        return m_rpabase_position(RPABaseServo_ClimbPosition);
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

    double a_rpabase_position_actual ()
    {
        //there is a bug where the getPosition() does return correctly so use an internal
        double l_return = 0.0;

        if (v_servo_rpa_base != null)
        {
            l_return = v_servo_rpa_base.getPosition ();
        }

        return l_return;
        //return l_rpa_base_position;
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
            debugLogException("rpa_base", "m_rpabase_position", p_exeception);
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
            debugLogException("arm_sholder", "m_arm_shoulder_position", p_exeception);
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

        try {
            //
            // Ensure the specific value is legal.
            //
            l_arm_elbow_position = Range.clip
                    ( p_position
                            , ArmElbowServo_MinPosition
                            , ArmElbowServo_MaxPosition
                    );
            if (v_servo_arm_elbow != null) {
                v_servo_arm_elbow.setPosition(l_arm_elbow_position);
                return l_arm_elbow_position;
            } else {
                return ServoErrorResultPosition;
            }
        }catch (Exception p_exeception)
        {
            debugLogException("arm_elbow", "m_arm_elbow_position", p_exeception);
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
            debugLogException("arm_wrist", "missing", p_exeception);
            return ServoErrorResultPosition;
        }


    } // m_arm_elbow_position



    /**
     * Mutate the flip right position.
     */
    double m_flip_right_position (double p_position)
    {
        //
        // Ensure the specific value is legal.
        //
        l_flip_right_position = Range.clip
                ( p_position
                        , FlipRightServo_MinPosition
                        , FlipRightServo_MaxPosition
                );
        try {
            if (v_servo_flip_right != null) {
                v_servo_flip_right.setPosition(l_flip_right_position);
                return l_flip_right_position;
            } else {
                return ServoErrorResultPosition;
            }
        }catch (Exception p_exeception)
        {
            debugLogException("flip_right", "m_flip_right_position", p_exeception);
            return ServoErrorResultPosition;
        }
    } // m_flip_right_position

    /**
     * Access the flip_right position.
     */
    double a_flip_right_position ()
    {
        return l_flip_right_position;
    } // a_flip_right_position

    /**
     * Mutate the flip right position.
     */
    double m_flip_left_position (double p_position)
    {
        //
        // Ensure the specific value is legal.
        //
        l_flip_left_position = Range.clip
                ( p_position
                        , FlipLeftServo_MinPosition
                        , FlipLeftServo_MaxPosition
                );
        try {
            if (v_servo_flip_left != null) {
                v_servo_flip_left.setPosition(l_flip_left_position);
                return l_flip_left_position;
            } else {
                return ServoErrorResultPosition;
            }
        }catch (Exception p_exeception)
        {
            debugLogException("flip_left", "m_flip_left_position", p_exeception);
            return ServoErrorResultPosition;
        }
    } // m_flip_left_position

    /**
     * Access the flip_right position.
     */
    double a_flip_left_position ()
    {
        return l_flip_left_position;
    } // a_flip_left_position


    /**
     * ticks the heartbeat which should happen every time though our loop
     * the heartbeat is wired to the Device Interface Module and is used to make sure our loop is still running
     */

    public void heartbeat_tick(){
        try {

            if ((v_loop_ticks % v_led_heartbeat_tickPerToggle) == 0) {
                heartbeat_toggle();
            }
        }catch (Exception p_exeception)
        {
            debugLogException("led_heartbeat", "heartbeat_tick", p_exeception);
        }
    }

    private boolean heartbeat_toggle () {
        try{
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
        }catch (Exception p_exeception)
        {
            debugLogException("led_heartbeat", "heartbeat_toggle", p_exeception);
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
            debugLogException("dim redled", "redled_on", p_exeception);
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
            debugLogException("dim redled", "redled_off", p_exeception);
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
            debugLogException("dim redled", "redled_toggle", p_exeception);
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
                v_dim.setLED(0, true);
                return true;
            }
            return false;
        }catch (Exception p_exeception)
        {
            debugLogException("dim blueled", "blueled_on", p_exeception);
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
                v_dim.setLED(0, false);
                return true;
            }
            return false;
        }catch (Exception p_exeception)
        {
            debugLogException("dim blueled", "blueled_off", p_exeception);
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
                v_dim.setLED(0, isEnabled);
                return isEnabled;
            } else {
                return false;
            }
        }catch (Exception p_exeception)
        {
            debugLogException("dim blueled", "blueled_toggle", p_exeception);
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
            debugLogException("sensor_color", "sensor_colorLegecy_led", p_exeception);
            return false;
        }


    } // sensor_legecyColor_led


    /**
     * reset the gyro heading to zero
     */
    private boolean sensor_gyro_resetHeading(){
        try{
            if(v_sensor_gyro != null){
                // get the x, y, and z values (rate of change of angle).

                v_sensor_gyro.resetZAxisIntegrator();
                return true;
            }
            return false;
        }catch(Exception p_exeception)
        {
            debugLogException("sensor_gyro", "sensor_gyro_resetHeading", p_exeception);
            return false;
        }
    }


    /**
     * Enable the Legecy Color Sensor
     * @return returns true is successfull returns false on error
     */
    public boolean sensor_color_led(boolean enable){
        try{
            // convert the RGB values to HSV values.
            if(v_sensor_color_i2c !=null) {
                //turn on the led this is the only way legecy color will detect anything
                v_sensor_color_i2c.enableLed(enable);
                return true;
            }
            return false;
        }catch (Exception p_exeception)
        {
            debugLogException("sensor_color", "sensor_color_led", p_exeception);
            return false;
        }
    }

    /**
     * Enable the Legecy Color Sensor
     * @return returns true is successfull returns false on error
     */
    public boolean sensor_color_enable(boolean enable){
        try{
            // convert the RGB values to HSV values.
            if(v_sensor_color_i2c !=null) {
                //turn on the led this is the only way legecy color will detect anything
                v_sensor_color_i2c_enabled = enable;
                return true;
            }
            return false;
        }catch (Exception p_exeception)
        {
            debugLogException("sensor_color", "sensor_color_enable", p_exeception);
            return false;
        }
    }
    public int[] sensor_color_get_rgba(){
        try{
            // we are now doing this in the hardware_loop and only if the sensor is enabled
            // convert the RGB values to HSV values.
//            if(v_sensor_color_i2c_rgbaValues !=null) {
//                //v_sensor_color.enableLed(true);
//                // wait one cycle.
//                //waitOneFullHardwareCycle();
//                v_sensor_color_i2c_rgbaValues[0] = v_sensor_color_i2c.red();
//                v_sensor_color_i2c_rgbaValues[1] = v_sensor_color_i2c.green();
//                v_sensor_color_i2c_rgbaValues[2] = v_sensor_color_i2c.blue();
//                v_sensor_color_i2c_rgbaValues[3] = v_sensor_color_i2c.alpha();
//                // wait one cycle.
//                //waitOneFullHardwareCycle();
//                // v_sensor_color.enableLed(false);
//            }
//            //Color.RGBToHSV(v_sensor_color.red(), v_sensor_color.green(), v_sensor_color.blue(), v_sensor_color_hsvValues);
            return v_sensor_color_i2c_rgbaValues;

        }catch (Exception p_exeception)
        {
            debugLogException("sensor_color", "sensor_color_read_rgb", p_exeception);
            return v_sensor_color_i2c_rgbaValues;
        }
    }


    /**
     *
     * @return gyro heading in degrees since reset
     */
    public int sensor_gyro_get_heading(){
        try{
            if(v_sensor_gyro != null) {
                return v_sensor_gyro.getHeading();
            }else{
                return 0;
            }
        }catch (Exception p_exeception)
        {
            debugLogException("sensor_gyro", "sensor_gyro_get_heading", p_exeception);
            return 0;
        }


    }

    /**
     * return the rawX rate
     * @return gyro heading in degrees since reset
     */
    public int sensor_gyro_get_rawX(){
        try{
            // get the x info.
            if(v_sensor_gyro != null) {
               return v_sensor_gyro.rawX();
            }else{
                return 0;
            }
        }catch (Exception p_exeception)
        {
            debugLogException("sensor_gyro", "sensor_gyro_get_rawX", p_exeception);
            return 0;
        }

    }

    /**
     * return the rawX rate
     * @return gyro heading in degrees since reset
     */
    public int sensor_gyro_get_rawY(){
        try{
            // get the heading info.
            // the Modern Robotics' gyro sensor keeps
            // track of the current heading for the Z axis only.
            if(v_sensor_gyro != null) {
                return v_sensor_gyro.rawY();
            }
            else{
                return 0;
            }
        }catch (Exception p_exeception)
        {
            debugLogException("sensor_gyro", "sensor_gyro_get_rawY", p_exeception);
            return 0;
        }
    }

    /**
     * return the rawX rate
     * @return gyro heading in degrees since reset
     */
    public int sensor_gyro_get_rawZ(){
        try{
            // get the heading info.
            // the Modern Robotics' gyro sensor keeps
            // track of the current heading for the Z axis only.
            if(v_sensor_gyro != null) {
                return v_sensor_gyro.rawZ();
            }
            else{
                return 0;
            }
        }catch (Exception p_exeception)
        {
            debugLogException("sensor_gyro", "sensor_gyro_get_rawZ", p_exeception);
            return 0;
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
            debugLogException("sensor_colorLegecy", "sensor_colorLegecy_start", p_exeception);
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
            debugLogException("sensor_colorLegecy", "sensor_colorLegecy_stop", p_exeception);

            return false;
        }
    }

    private int[] sensor_colorLegecy_read_rgba(){
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
            debugLogException("sensor_colorLegecy", "sensor_colorLegecy_read_rgb", p_exeception);
            return v_sensor_colorLegecy_rgbValues;
        }
    }


    public int[] sensor_colorLegecy_getLast_rgba(){
        try{
            return v_sensor_colorLegecy_rgbValues;

        }catch (Exception p_exeception)
        {
            debugLogException("sensor_colorLegecy", "sensor_colorLegecy_getLast_rgb", p_exeception);
            return v_sensor_colorLegecy_rgbValues;
        }
    }

public double sensor_ultraLegecy_distance(){
    try{
        if(v_sensor_ultraLegecy != null){
            if ((v_loop_ticks % v_sensor_ultraLegecy_ticksPerRead) == 0) {
               v_sensor_ultraLegecy_distance = v_sensor_ultraLegecy.getUltrasonicLevel();
            }
            return v_sensor_ultraLegecy_distance;
        }else{
            return 9999.9999;
        }
    }catch (Exception p_exeception)
    {
        debugLogException("sensor_ultraLegecy", "sensor_ultraLegecy_distance", p_exeception);
        return 9999.9999;
    }
}

    //Lego Light Legecy Sensor Methods

    //--------------------------------------------------------------------------
    //
    // a_ods_light_detected
    //
    /**
     * Access the amount of light detected by the Optical Distance Sensor.
     */
    public double sensor_lightLegecy_amountDetected ()

    {
        double l_return = 0.0;

        if (v_sensor_lightLegecy != null)
        {
            v_sensor_lightLegecy.getLightDetected ();
        }

        return l_return;

    }
public boolean sensor_lightLegecy_led(boolean enable){
    if(v_sensor_lightLegecy != null) {
        v_sensor_lightLegecy_enabled = enable;
        v_sensor_lightLegecy.enableLed(enable);
        return true;
    }else{
        return false;
    }
}

    public boolean sensor_lightLegecy_led_status(){
        return v_sensor_lightLegecy_enabled;
    }
    public boolean sensor_lightLegecy_white_tape_detected(){
        return a_light_white_tape_detected();
    }

    //--------------------------------------------------------------------------
    //
    // a_light_white_tape_detected
    //
    /**
     * Access whether the Light Sensor is detecting white tape.
     */
    private boolean a_light_white_tape_detected ()
    {

        //
        // Assume not.
        //
        boolean l_return = false;

        if (v_sensor_lightLegecy != null)
        {
            //
            // Is the amount of light detected above the threshold for white
            // tape?
            //
            if (v_sensor_lightLegecy.getLightDetected () > 0.8)
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
    public void sleep(long milliseconds) {
        try{

            Thread.sleep(milliseconds);
        }catch(InterruptedException ex){
            //do stuff
        }
    }
}
