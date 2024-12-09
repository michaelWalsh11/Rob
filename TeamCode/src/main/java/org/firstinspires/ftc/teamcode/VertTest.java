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

    boolean bool = false;
    boolean False = false;


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

        if (gamepad2.x && !bool)
        {
            bool = true;
            False = !False;
        }
        else {
            bool = false;
        }

        if (!False)
        {
            controlSliders();
        }
        else {
            controlSlidersRUN();
        }


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

    public void controlSlidersRUN() {
        if (Math.abs(gamepad2.left_stick_y) > 0.4) {
            // Move sliders based on gamepad input
            leftSliderSpeed = gamepad2.left_stick_y * MAX_SLIDER_SPEED;

            // Calculate new target positions based on current positions and input speed
            int newTargetPosition1 = robot.outTake1.getCurrentPosition() + (int) (leftSliderSpeed * 100);  // Adjust multiplier as needed
            int newTargetPosition2 = robot.outTake2.getCurrentPosition() + (int) (leftSliderSpeed * 100);

            // Set the target positions
            robot.outTake1.setTargetPosition(newTargetPosition1);
            robot.outTake2.setTargetPosition(newTargetPosition2);

            // Set the mode to RUN_TO_POSITION to move to the target
            robot.outTake1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.outTake2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Apply power to both motors
            robot.outTake1.setPower(Math.abs(leftSliderSpeed));
            robot.outTake2.setPower(Math.abs(leftSliderSpeed));

        } else {
            // Hold position when no input
            robot.outTake1.setTargetPosition(robot.outTake1.getCurrentPosition());
            robot.outTake2.setTargetPosition(robot.outTake2.getCurrentPosition());
            robot.outTake1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.outTake2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Apply minimal power to hold the position
            robot.outTake1.setPower(0.1);  // Adjust holding power as needed
            robot.outTake2.setPower(0.1);
        }

        if (gamepad2.a) {
            // Move sliders to position 0 when button A is pressed
            robot.outTake1.setTargetPosition(0);
            robot.outTake2.setTargetPosition(0);
            robot.outTake1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.outTake2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Apply power to move
            robot.outTake1.setPower(0.9);
            robot.outTake2.setPower(0.9);
        }

        if (gamepad2.y) {
            // Move sliders to position 1000 when button Y is pressed
            robot.outTake1.setTargetPosition(1000);
            robot.outTake2.setTargetPosition(1000);
            robot.outTake1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.outTake2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Apply power to move
            robot.outTake1.setPower(0.9);
            robot.outTake2.setPower(0.9);
        }
    }


}
