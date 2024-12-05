package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="Motor / Servo Tester", group="Training")
//@Disabled
public class ServoTester extends OpMode {

    RobotBackBetter robot   = new RobotBackBetter(); // use the class created to define a Robot's hardware
    // could also use HardwarePushbotMatrix class.
    double          clawOffset  = 0.0 ;                  // Servo mid position
    final double    CLAW_SPEED  = 0.02 ;


    public double OUT_ARM_SPEED = 0.0;
    public double IN_ARM_SPEED = 0.0;// sets rate to move servo

    public double rotatePos;
    public double rotatePos1;
    public double rotatePos2;
    public double rotatePos3;
    public double rotatePos4;


    public int inPos1;
    public int outPos1;




    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Robot Ready");

        //

        // Set to Run without Encoder for Tele Operated


    }


    public void outTake(int amp)
    {
        robot.outTake1.setPower(1.0);
        robot.outTake1.setTargetPosition(amp);
        robot.outTake1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.outTake1.setPower(OUT_ARM_SPEED);

        robot.outTake2.setPower(1.0);
        robot.outTake2.setTargetPosition(amp);
        robot.outTake2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.outTake2.setPower(OUT_ARM_SPEED);
    }

    public void inTake(int amp)
    {
        //armMover Action
        robot.inTake1.setPower(1.0);
        robot.inTake1.setTargetPosition(amp);
        robot.inTake1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.inTake1.setPower(IN_ARM_SPEED);

        robot.inTake2.setPower(1.0);
        robot.inTake2.setTargetPosition(-amp);
        robot.inTake2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.inTake2.setPower(IN_ARM_SPEED);
    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        rotatePos = robot.rotator.getPosition();
        rotatePos1 = robot.grasper.getPosition();
        rotatePos2 = robot.outtake.getPosition();
        rotatePos3 = robot.kickLeft.getPosition();
        rotatePos4 = robot.kickRight.getPosition();

        outPos1 = robot.outTake1.getCurrentPosition();
        inPos1 = robot.outTake1.getCurrentPosition();

        //armPos = 50; // Lifts Arm for Driving
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {


        if (gamepad2.left_bumper)
        {
            rotatePos = Math.min(rotatePos + 0.01, 1.0);;
        }
        if (gamepad2.right_bumper)
        {
            rotatePos = Math.max(rotatePos - 0.01, 0.0);
        }

        robot.rotator.setPosition(rotatePos);


        if (gamepad2.y)
        {
            rotatePos1 = Math.min(rotatePos1 + 0.01, 1.0);;
        }
        if (gamepad2.a)
        {
            rotatePos1 = Math.max(rotatePos1 - 0.01, 0.0);
        }

        robot.grasper.setPosition(rotatePos1);

        if (gamepad2.b)
        {
            rotatePos2 = Math.min(rotatePos2 + 0.01, 1.0);;
        }
        if (gamepad2.x)
        {
            rotatePos2 = Math.max(rotatePos2 - 0.01, 0.0);
        }

        robot.outtake.setPosition(rotatePos2);

        if (gamepad1.dpad_up)
        {
            rotatePos3 = Math.min(rotatePos3 + 0.01, 1.0);;
        }
        if (gamepad1.dpad_down)
        {
            rotatePos3 = Math.max(rotatePos3 - 0.01, 0.0);
        }

        robot.kickLeft.setPosition(rotatePos3);

        if (gamepad1.a)
        {
            rotatePos4 = Math.min(rotatePos4 + 0.01, 1.0);;
        }
        if (gamepad1.y)
        {
            rotatePos4 = Math.max(rotatePos4 - 0.01, 0.0);
        }

        robot.kickRight.setPosition(rotatePos4);


        if (gamepad2.left_stick_y > 0.4)
        {
            OUT_ARM_SPEED = 1.0;
            outPos1 += (int) (gamepad2.left_stick_y * 15.0);
        }

        if (gamepad2.left_stick_y < -0.4)
        {
            OUT_ARM_SPEED = 1.0;
            outPos1 -= (int) (Math.abs(gamepad2.left_stick_y) * 15.0);
        }

        //auto intake
        if (gamepad2.right_stick_y > 0.4)
        {
            IN_ARM_SPEED = 1.0;
            inPos1 += (int) (gamepad2.right_stick_y * 15.0);
        }

        if (gamepad2.right_stick_y < -0.4)
        {
            IN_ARM_SPEED = 1.0;
            inPos1 -= (int) (Math.abs(gamepad2.right_stick_y) * 15.0);
        }

        outTake(outPos1);
        inTake(inPos1);


        telemetry.addLine("Servo Tuning (controller 2");
        telemetry.addLine("");
        telemetry.addLine("rotatorPos (left and right bumper) " + rotatePos);
        telemetry.addLine("grasperPos (y and a) " + rotatePos1);
        telemetry.addLine("outtakePos (b and x) " + rotatePos2);
        telemetry.addLine("");
        telemetry.addLine("");
        telemetry.addLine("Arm Tuning (controller 2");
        telemetry.addLine("");
        telemetry.addLine("outtake (dpad_up and dpad_down) " + outPos1);
        telemetry.addLine("intake (dpad_left and dpad_right) " + inPos1);
        telemetry.addLine("");
        telemetry.addLine("");
        telemetry.addLine("Servo Tuning (controller 1");
        telemetry.addLine("");
        telemetry.addLine("kickstand 1 pos (dpad_up and dpad_down) " + rotatePos3);
        telemetry.addLine("kickstand 2 pos (a and y) " + rotatePos4);


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
