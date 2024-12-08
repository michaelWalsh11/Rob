package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;


@TeleOp(name="VertTest", group="Training")
//@Disabled
public class VertTest extends OpMode {

    RobotBackBetter robot = new RobotBackBetter();

    private double leftSliderSpeed = 0;
    private double rightSliderSpeed = 0;

    // Constants for motor power limits
    private final int MAX_SLIDER_SPEED = 5;


    @Override
    public void init() {

        robot.init(hardwareMap);


        robot.outTake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.outTake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.outTake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.outTake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void loop() {
        controlSliders();


    }

    public void controlSliders() {

        if (Math.abs(gamepad2.left_stick_y) > 0.4)
        {
            leftSliderSpeed = gamepad2.left_stick_y * MAX_SLIDER_SPEED;
        }
        else
        {
            leftSliderSpeed = 0;
        }
        robot.outTake1.setPower(leftSliderSpeed);


        if (Math.abs(gamepad2.right_stick_y) > 0.4)
        {
            rightSliderSpeed = gamepad2.right_stick_y * MAX_SLIDER_SPEED;
        }
        else
        {
            rightSliderSpeed = 0;
        }
        robot.outTake2.setPower(rightSliderSpeed);
    }
}
