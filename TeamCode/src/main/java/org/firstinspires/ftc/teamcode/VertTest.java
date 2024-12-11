package org.firstinspires.ftc.teamcode;

import android.app.admin.SystemUpdatePolicy;
import android.provider.ContactsContract;

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

    private int lArmPos = 0;
    private int rArmPos = 0;


    @Override
    public void init() {

        robot.init(hardwareMap);


        robot.outTake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.outTake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.outTake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.outTake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lArmPos = robot.outTake1.getCurrentPosition();
        rArmPos = robot.outTake2.getCurrentPosition();

    }

    @Override
    public void loop() {

        controlMotors();


    }









    public void controlMotors() {

        // Get input from the gamepad stick
        double motorPower = -gamepad2.left_stick_y;  // Invert so up is positive

        // Threshold for small movements (deadzone)
        if (Math.abs(motorPower) > 0.4) {
            // If moving, set power to both motors
            robot.outTake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.outTake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.outTake1.setPower(motorPower);
            robot.outTake2.setPower(-motorPower);
            lArmPos = robot.outTake1.getCurrentPosition();
            rArmPos = robot.outTake2.getCurrentPosition();
        } else {
            // If stick is neutral, hold the position
            //robot.outTake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //robot.outTake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.outTake1.setTargetPosition(lArmPos);
            robot.outTake2.setTargetPosition(rArmPos);
            robot.outTake1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.outTake2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.outTake1.setPower(0.4); // Holding power (tune as necessary)
            robot.outTake2.setPower(0.4);
            //robot.outTake1.setPower(0);
        }

        telemetry.addLine("" + robot.outTake1.getCurrentPosition());
        telemetry.addLine("" + robot.outTake2.getCurrentPosition());
        telemetry.addLine("" + motorPower);
        // Button to go to position 0

}












    public void controlSlidersRUN() {
        if (Math.abs(gamepad2.left_stick_y) > 0.4) {

            leftSliderSpeed = gamepad2.left_stick_y * MAX_SLIDER_SPEED;

            int newTargetPosition1 = robot.outTake1.getCurrentPosition() + (int) (leftSliderSpeed * 100);  // Adjust multiplier as needed
            int newTargetPosition2 = robot.outTake2.getCurrentPosition() + (int) (leftSliderSpeed * 100);

            robot.outTake1.setTargetPosition(newTargetPosition1);
            robot.outTake2.setTargetPosition(newTargetPosition2);

            robot.outTake1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.outTake2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.outTake1.setPower(Math.abs(leftSliderSpeed));
            robot.outTake2.setPower(Math.abs(leftSliderSpeed));

            telemetry.addLine("" + newTargetPosition1);

        } else {
            robot.outTake1.setTargetPosition(robot.outTake1.getCurrentPosition());
            robot.outTake2.setTargetPosition(robot.outTake1.getCurrentPosition());
            robot.outTake1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.outTake2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.outTake1.setPower(0.1);
            robot.outTake2.setPower(0.1);
        }

        if (gamepad2.a) {
            robot.outTake1.setTargetPosition(0);
            robot.outTake2.setTargetPosition(0);
            robot.outTake1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.outTake2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.outTake1.setPower(0.9);
            robot.outTake2.setPower(0.9);
        }

        if (gamepad2.y) {
            robot.outTake1.setTargetPosition(-1500);
            robot.outTake2.setTargetPosition(-1500);
            robot.outTake1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.outTake2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.outTake1.setPower(0.9);
            robot.outTake2.setPower(0.9);
        }
    }
}
