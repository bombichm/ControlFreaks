package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
/**
 * Created by Robot on 10/8/2015.
 */
public class EncoderTest1 extends PushBotTelemetry {

    DcMotor motorRight;
    DcMotor motorLeft;


    @Override
    public void init() {
        motorRight = hardwareMap.dcMotor.get("motor_2");
        motorLeft = hardwareMap.dcMotor.get("motor_1");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        reset_drive_encoders();
    }

    @Override
    public void loop() {



        run_using_encoders();

        //
        // Start the drive wheel motors at full power.
        //
        // set_drive_power (1.0f, 1.0f);
        motorRight.setPower(.5);
       motorLeft.setPower(.5);

        // Have the motor shafts turned the required amount?
        //
        // If they haven't, then the op-mode remains in this state (i.e this
        // block will be executed the next time this method is called).
        //
        if (have_drive_encoders_reached(5,5))
        {
            //
            // Reset the encoders to ensure they are at a known good value.
            //
            reset_drive_encoders();

            //
            // Stop the motors.
            //
            set_drive_power(0.0, 0.0);



            stop();
        }





    }

}