package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.DriveConstants.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="Main", group="Training")
//@Disabled
public class DriveBasic extends OpMode {

    RobotBackBetter robot   = new RobotBackBetter(); // use the class created to define a Robot's hardware
    // could also use HardwarePushbotMatrix class.
    double          clawOffset  = 0.0;                  // Servo mid position
    final double    CLAW_SPEED  = 0.02;                 // sets rate to move servo

    private int outPos1;
    private int outPos2;
    private int inPos1;
    private int inPos2;

    private int number = 0;

    private double ARM_SPEED = 0.7;
    private double ARM_SPEED_ANGLER = 0.7;

    private double IN_ARM_SPEED = 0.7;
    private double OUT_ARM_SPEED = 0.7;

    private int toggle = 0;

    private boolean rand1 = false;
    private boolean rand2 = false;

    private boolean bump1 = false;
    private boolean bump2 = false;

    private boolean other1 = false;
    private boolean other2 = false;

    private double rotatePos = 0.0;
    private double grasperPos = 0.0;


    private double DRIVE_SPEED = 1.0;


    @Override /* * */
    public void init() {

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Robot Ready");    //

        // Set to Run without Encoder for Tele Operated
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.outTake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.outTake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.outTake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.outTake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outPos1 = robot.outTake1.getCurrentPosition();
        outPos2 = robot.outTake2.getCurrentPosition();

        robot.inTake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.inTake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.inTake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.inTake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        inPos1 = robot.outTake1.getCurrentPosition();
        inPos2 = robot.outTake2.getCurrentPosition();


        rotatePos = robot.rotator.getPosition();
        grasperPos = robot.grasper.getPosition();



    }

    @Override /* * */
    public void init_loop()
    {

    }


    @Override /* * */
    public void start()
    {

    }

    public boolean dpadLeftPressed = false;
    public boolean isControlsActive = false;

    @Override
    public void loop() {

        if (gamepad1.dpad_left && !dpadLeftPressed)
        {
            isControlsActive = !isControlsActive;
            dpadLeftPressed = true;
        }
        else if (!gamepad1.dpad_left)
        {
            dpadLeftPressed = false;
        }

        if (isControlsActive) {
            controls();
        }

        drive();
        grasper();
        arms();
        telemetry();

        // Run intake in a separate thread if multi-Threading breaks remove thread
        new Thread(() -> {
            try {
                intake();
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }).start();
    }
    public void controls()
    {
        // Gamepad 1 Controls
        telemetry.addLine("GAMEPAD 1 CONTROLS:");
        telemetry.addLine("LEFT STICK (x, y): Controls movement");
        telemetry.addLine(" - LEFT STICK X: Strafing");
        telemetry.addLine(" - LEFT STICK Y: Forward/backward movement");
        telemetry.addLine("RIGHT STICK (x): Controls turning");
        telemetry.addLine("DPAD_UP: Raises kickstands (left and right)");
        telemetry.addLine("DPAD_LEFT: toggles the control menu");
        telemetry.addLine("DPAD_DOWN: Lowers kickstands (left and right)");
        telemetry.addLine("RIGHT_TRIGGER: Drops the outtake mechanism");
        telemetry.addLine("X: Also drops the outtake mechanism");
        telemetry.addLine("A: Toggles drive speed between full (1.0) and reduced (0.4)");

        telemetry.addLine("");

        // Gamepad 2 Controls
        telemetry.addLine("GAMEPAD 2 CONTROLS:");
        telemetry.addLine("LEFT STICK (y): Controls outtake arm movement");
        telemetry.addLine(" - Positive for up, negative for down");
        telemetry.addLine("RIGHT STICK (y): Controls intake arm movement");
        telemetry.addLine(" - Positive for out, negative for in");
        telemetry.addLine("LEFT_TRIGGER: Opens the grasper");
        telemetry.addLine("RIGHT_TRIGGER: Closes the grasper");
        telemetry.addLine("DPAD_UP: Initiates intake routine");
        telemetry.addLine("A: Sets intake arm to maximum position");
        telemetry.addLine("B: Sets intake arm to minimum position");
        telemetry.addLine("X: Sets outtake arm to maximum position");
        telemetry.addLine("Y: Sets outtake arm to minimum position and moves outtake to intake position");
        telemetry.addLine("LEFT_BUMPER: Rotates the grasper to the left");
        telemetry.addLine("RIGHT_BUMPER: Rotates the grasper to the right");

    }

    public void telemetry()
    {

        telemetry.addLine("");
        telemetry.addLine("armMotorSpeeds");

        telemetry.addData("armSpeed", ARM_SPEED);
        telemetry.addData("SpeedArmAngler", ARM_SPEED_ANGLER);

        telemetry.addLine("");

        telemetry.addLine("intake Arm 1: " + inPos1);
        telemetry.addData("intake Arm 2: ", inPos2);

        telemetry.addLine("");
        telemetry.addLine("ArmAnglers");

        telemetry.addLine("outtake Arm 1: " + outPos1);
        telemetry.addData("outtake Arm 2: ", outPos2);

        telemetry.addLine("");
        telemetry.addLine("grasper: " + robot.grasper.getPosition());
        telemetry.addLine("rotator" + robot.rotator.getPosition());
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

    public void intake() throws  InterruptedException
    {
        if (gamepad2.dpad_up)
        {
            robot.grasper.setPosition(GRASPER_CLOSE);
            Thread.sleep(AFTER_CLOSE_SLEEP);
            robot.rotator.setPosition(ROTATOR_TRANSFER);
            rotatePos = ROTATOR_TRANSFER;
            Thread.sleep(1000);
            robot.outtake.setPosition(OUTTAKE_INTAKE);
            Thread.sleep(TRANSFER_TIME);
            robot.grasper.setPosition(GRASPER_OPEN);
            Thread.sleep(AFTER_OPEN_SLEEP);
            robot.rotator.setPosition(ROTATOR_GROUND);
            rotatePos = ROTATOR_GROUND;
        }
    }

    public void arms() {
        //auto outtake
        if (gamepad2.left_stick_y > 0.4)
        {
            OUT_ARM_SPEED = 1.0;
            outPos1 += (int) (gamepad2.left_stick_y * OUTSLIDE_SPEED);
        }

        if (gamepad1.dpad_down)
        {
            robot.kickLeft.setPosition(KICKLEFT_DOWN); // FIND DATA FOR DOWN
            robot.kickRight.setPosition(KICKRIGHT_DOWN); //FIND DATA FOR DOWN
        }

        if (gamepad1.dpad_up)
        {
            robot.kickLeft.setPosition(KICKLEFT_UP); // FIND DATA FOR UP
            robot.kickRight.setPosition(KICKRIGHT_UP); //FIND DATA FOR UP
        }


        if (gamepad2.left_stick_y < -0.4)
        {
            OUT_ARM_SPEED = 1.0;
            outPos1 -= (int) (Math.abs(gamepad2.left_stick_y) * OUTSLIDE_SPEED);
        }

        //auto intake
        if (gamepad2.right_stick_y > 0.4)
        {
            IN_ARM_SPEED = 1.0;
            inPos1 += (int) (gamepad2.right_stick_y * INSLIDE_SPEED);
        }

        if (gamepad2.right_stick_y < -0.4)
        {
            IN_ARM_SPEED = 1.0;
            inPos1 -= (int) (Math.abs(gamepad2.right_stick_y) * INSLIDE_SPEED);
        }

        if (outPos1 < 0)
        {
            outPos1 = 0;
            OUT_ARM_SPEED = 0;
        }

        if (inPos1 < 0)
        {
            inPos1 = 0;
            IN_ARM_SPEED = 0.0;
        }

        if (gamepad1.right_trigger > 0.4 || gamepad1.x)
        {
            robot.outtake.setPosition(OUTTAKE_DROP);
        }

        //todo
        // - add max values for the arms
        // - also replace the hotkeys for y and b and their maxes
        // - button pick up rotator and transfer and go back




        if (gamepad2.left_trigger > 0.4)
        {
            robot.grasper.setPosition(GRASPER_OPEN);
        }

        if (gamepad2.right_trigger > 0.4)
        {
            robot.grasper.setPosition(GRASPER_CLOSE);
        }

        if (gamepad2.b && !gamepad2.start)
        {
            inPos1 = INTAKE_MIN;
        }

        if (gamepad2.a && !gamepad2.start)
        {
            inPos1 = INTAKE_MAX;
        }

        if (gamepad2.x)
        {
            outPos1 = OUTTAKE_MAX;
        }

        if (gamepad2.y) {

            outPos1 = OUTTAKE_MIN;
            robot.outtake.setPosition(OUTTAKE_INTAKE);
        }

        //outTake(outPos1);
        inTake(inPos1);



    }

    public void grasper()
    {

        //twist left and right
        if (gamepad2.left_bumper)
        {
            rotatePos = Math.min(rotatePos + ROTATE_SPEED, 1.0);;
        }
        if (gamepad2.right_bumper)
        {
            rotatePos = Math.max(rotatePos - ROTATE_SPEED, 0.0);
        }

        robot.rotator.setPosition(rotatePos);

    }

    public void drive()
    {
        //drive
        double leftX;
        double leftY;
        double rightX;

        if (DRIVE_SPEED == 1.0 && gamepad1.a && !rand1)
        {
            DRIVE_SPEED = 0.4;
            rand1 = true;
        }
        if (DRIVE_SPEED == 0.4 && gamepad1.a && !rand1)
        {
            DRIVE_SPEED = 1.0;
            rand1 = true;
        }

        if (!gamepad1.a)
        {
            rand1 = false;
        }

        leftX = gamepad1.left_stick_x * DRIVE_SPEED;
        leftY = gamepad1.left_stick_y * DRIVE_SPEED;
        rightX = gamepad1.right_stick_x * DRIVE_SPEED;

        double leftRearPower = leftY + leftX - rightX;
        double leftFrontPower = leftY - leftX - rightX;
        double rightRearPower = leftY - leftX + rightX;
        double rightFrontPower = leftY + leftX + rightX;

        robot.leftFront.setPower(leftFrontPower);
        robot.leftRear.setPower(leftRearPower);
        robot.rightFront.setPower(rightFrontPower);
        robot.rightRear.setPower(rightRearPower);

    }

    @Override
    public void stop()
    {
        //hello
    }

}