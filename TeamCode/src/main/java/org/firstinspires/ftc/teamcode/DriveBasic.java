package org.firstinspires.ftc.teamcode;

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

    DriveConstants drive = new DriveConstants();


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

    @Override
    public void loop() {

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
            robot.grasper.setPosition(drive.GRASPER_CLOSE); // test pos and find it close pos
            Thread.sleep(200);
            robot.outtake.setPosition(drive.OUTTAKE_INTAKE); // tune to find it down pos to intake
            robot.rotator.setPosition(drive.ROTATOR_TRANSFER); // test pos and find it outtake pos
            Thread.sleep(300);
            robot.grasper.setPosition(drive.GRASPER_OPEN); // open pos
            robot.rotator.setPosition(drive.ROTATOR_GROUND); // down pos
        }
    }

    public void arms() {
        //auto outtake
        if (gamepad2.left_stick_y > 0.4)
        {
            OUT_ARM_SPEED = 1.0;
            outPos1 += (int) (gamepad2.left_stick_y * 15.0);
        }

        if (gamepad1.dpad_down)
        {
            robot.kickLeft.setPosition(drive.KICKLEFT_DOWN); // FIND DATA FOR DOWN
            robot.kickRight.setPosition(drive.KICKRIGHT_DOWN); //FIND DATA FOR DOWN
        }

        if (gamepad1.dpad_up)
        {
            robot.kickLeft.setPosition(drive.KICKLEFT_UP); // FIND DATA FOR UP
            robot.kickRight.setPosition(drive.KICKRIGHT_UP); //FIND DATA FOR UP
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

        if (outPos1 > 0)
        {
            outPos1 = 0;
            OUT_ARM_SPEED = 0;
        }

        if (inPos1 > 0)
        {
            inPos1 = 0;
            IN_ARM_SPEED = 0.0;
        }

        if (gamepad1.right_trigger > 0.4 || gamepad1.x)
        {
            robot.outtake.setPosition(drive.OUTTAKE_DROP); // drop pixel
        }

        //todo
        // - add max values for the arms
        // - also replace the hotkeys for y and b and their maxes
        // - button pick up rotator and transfer and go back




        if (gamepad2.left_trigger > 0.4)
        {
            robot.grasper.setPosition(drive.GRASPER_OPEN);
        }

        if (gamepad2.right_trigger > 0.4)
        {
            robot.grasper.setPosition(drive.GRASPER_CLOSE);
        }

        if (gamepad2.b && !gamepad2.start)
        {
            inPos1 = drive.INTAKE_MIN;
            inTake(inPos1);
        }

        if (gamepad2.a && !gamepad2.start)
        {
            inPos1 = drive.INTAKE_MAX;
            inTake(inPos1);
        }

        if (gamepad2.x)
        {
            outPos1 = drive.OUTTAKE_MAX;
            outTake(outPos1);
        }

        if (gamepad2.y) {
            outPos1 = drive.OUTTAKE_MIN;
            robot.outtake.setPosition(drive.OUTTAKE_DROP);
            outTake(outPos1);
        }

        outTake(outPos1);
        inTake(inPos1);



    }

    public void grasper()
    {

        //todo
        // - tune all values because pure guessing

        //open and close grasper
        if (gamepad2.right_trigger > 0.4)
        {
            robot.grasper.setPosition(1.0);
        }
        if (gamepad2.left_trigger > 0.4)
        {
            robot.grasper.setPosition(0.0);
        }

        //twist left and right
        if (gamepad2.left_bumper)
        {
            rotatePos = Math.min(rotatePos + 0.01, 1.0);;
        }
        if (gamepad2.right_bumper)
        {
            rotatePos = Math.max(rotatePos - 0.01, 0.0);
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