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
        if (Math.abs(gamepad2.left_stick_y) > 0.4) {
            // Move sliders based on gamepad input
            leftSliderSpeed = gamepad2.left_stick_y * MAX_SLIDER_SPEED;
            robot.outTake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Enable manual control
            robot.outTake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            // Hold position when no input
            leftSliderSpeed = 0.0;
            robot.outTake1.setTargetPosition(robot.outTake1.getCurrentPosition());
            robot.outTake2.setTargetPosition(robot.outTake2.getCurrentPosition());
            robot.outTake1.setMode(DcMotor.RunMode.RUN_TO_POSITION);  // Set to hold the current position
            robot.outTake2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if (gamepad2.a)
        {
            leftSliderSpeed = 0.9;
            robot.outTake1.setTargetPosition(0);
            robot.outTake2.setTargetPosition(0);
            robot.outTake1.setMode(DcMotor.RunMode.RUN_TO_POSITION);  // Set to hold the current position
            robot.outTake2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if (gamepad2.y)
        {
            leftSliderSpeed = 0.9;
            robot.outTake1.setTargetPosition(1000);
            robot.outTake2.setTargetPosition(1000);
            robot.outTake1.setMode(DcMotor.RunMode.RUN_TO_POSITION);  // Set to hold the current position
            robot.outTake2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        // Apply power to both motors
        robot.outTake1.setPower(leftSliderSpeed);
        robot.outTake2.setPower(leftSliderSpeed);
    }

}
