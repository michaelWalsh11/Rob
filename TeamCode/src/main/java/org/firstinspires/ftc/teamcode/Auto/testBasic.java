package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.DriveConstants.*;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="home slice", group="Autonomous")
public class testBasic extends LinearOpMode {

    public class Intake {

        private DcMotorEx inTake1;
        private DcMotorEx inTake2;

        public Intake(HardwareMap hardwareMap) {
            inTake1 = hardwareMap.get(DcMotorEx.class, "intake1");
            inTake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            inTake1.setDirection(DcMotorSimple.Direction.REVERSE);

            inTake2 = hardwareMap.get(DcMotorEx.class, "intake2");
            inTake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            inTake2.setDirection(DcMotorSimple.Direction.FORWARD);

        }

        public class IntakeToPos implements Action {
            private final double targetPosition;
            private final double holdingPower;
            private boolean initialized = false;

            public IntakeToPos(double targetPosition, double holdingPower) {
                this.targetPosition = targetPosition;
                this.holdingPower = holdingPower;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized)
                {
                    inTake1.setPower(0.8);
                    inTake2.setPower(0.8);
                    initialized = true;
                }

                double pos = inTake1.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < targetPosition) {
                    return true; // Continue running the action
                }
                else
                {
                    inTake1.setPower(holdingPower);  // Apply a small power to hold the position
                    inTake2.setPower(holdingPower);
                    return false; // Stop running, but maintain hold
                }
            }

            // Method to create actions with holding power
        }

        public Action in() {
            return new IntakeToPos(INTAKE_MIN, 0.2);
        }

        public Action out() {
            return new IntakeToPos(INTAKE_MAX, 0.2);
        }

    }


    public class Outtake {

        private DcMotorEx outTake1;
        private DcMotorEx outTake2;

        public Outtake(HardwareMap hardwareMap) {
            outTake1 = hardwareMap.get(DcMotorEx.class, "outtake1");
            outTake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            outTake1.setDirection(DcMotorSimple.Direction.FORWARD);

            outTake2 = hardwareMap.get(DcMotorEx.class, "outtake2");
            outTake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            outTake2.setDirection(DcMotorSimple.Direction.REVERSE);


        }

        public class OuttakeToPos implements Action {
            private final int targetPosition;
            private final double holdingPower;
            private final double direction;
            private boolean initialized = false;

            public OuttakeToPos(int targetPosition, double holdingPower, double direction) {
                this.targetPosition = targetPosition;
                this.holdingPower = holdingPower;
                this.direction = direction;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized)
                {

                    initialized = true;
                }

                outTake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                outTake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                outTake1.setPower(direction);
                outTake2.setPower(direction);

                packet.put("outtake1Pos", outTake1.getCurrentPosition());
                packet.put("outtake2Pos", outTake2.getCurrentPosition());


                int pos = outTake1.getCurrentPosition();
                packet.put("liftPos", pos);
                if (Math.abs(targetPosition - pos) > 20) {
                    return true;
                }
                else
                {
                    outTake1.setTargetPosition(pos);
                    outTake2.setTargetPosition(pos);
                    outTake1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    outTake2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    outTake1.setPower(holdingPower);  // Apply a small power to hold the position
                    outTake2.setPower(0.0);
                    return false; // Stop running, but maintain hold
                }
            }



            // Method to create actions with holding power
        }

        public Action in() {
            return new OuttakeToPos(0, 0.4, -1.0);
        }

        public Action out() {
            return new OuttakeToPos(OUTTAKE_MAX, 0.4, 1.0);
        }

    }

    //rotator
    public class Rotator
    {
        private Servo rotator;

        public Rotator(HardwareMap hardwareMap) {
            rotator = hardwareMap.get(Servo.class, "rotator");
        }

        public class RotateTransfer implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotator.setPosition(ROTATOR_TRANSFER);
                return false;
            }
        }

        public class RotateMid implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotator.setPosition(ROTATOR_MID);
                return false;
            }
        }

        public class RotateGround implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotator.setPosition(ROTATOR_GROUND);
                return false;
            }
        }

        public Action ground() {
            return new RotateGround();
        }

        public Action mid() {
            return new RotateMid();
        }

        public Action transfer() {
            return new RotateTransfer();
        }
    }

    public class KickStands
    {
        private Servo kickLeft;
        private Servo kickRight;

        public KickStands(HardwareMap hardwareMap) {
            kickLeft = hardwareMap.get(Servo.class, "kickleft");
            kickRight = hardwareMap.get(Servo.class, "kickright");
        }

        public class KickFlat implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                kickLeft.setPosition(KICKLEFT_FLAT);
                kickRight.setPosition(KICKRIGHT_FLAT);
                return false;
            }
        }

        public Action flat() {
            return new KickFlat();
        }

    }



    public class OuttakeServo
    {
        private Servo outtake;

        public OuttakeServo(HardwareMap hardwareMap) {
            outtake = hardwareMap.get(Servo.class, "outtake");
        }

        public class Drop implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtake.setPosition(OUTTAKE_DROP);
                return false;
            }
        }

        public class Intake implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtake.setPosition(OUTTAKE_INTAKE);
                return false;
            }
        }

        public class Flat implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtake.setPosition(OUTTAKE_INTAKE);
                return false;
            }
        }
        public Action drop() {
            return new Drop();
        }

        public Action flat() {
            return new Flat();
        }

        public Action intake()  {
            return new Intake();
        }
    }


    //angler
    public class Grasper
    {
        private Servo grasper;

        public Grasper(HardwareMap hardwareMap) {
            grasper = hardwareMap.get(Servo.class, "grasper");
        }

        public class Open implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grasper.setPosition(GRASPER_OPEN);
                return false;
            }
        }

        public class Close implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grasper.setPosition(GRASPER_CLOSE);
                return false;
            }
        }


        public Action open() {
            return new Open();
        }

        public Action close() {
            return new Close();
        }
    }

    public class LowHang
    {
        private Servo lowHang;

        public LowHang(HardwareMap hardwareMap) {
            lowHang = hardwareMap.get(Servo.class, "lowHang");
        }

        public class Down implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                lowHang.setPosition(AUTO_HANG_DOWN);
                return false;
            }
        }

        public class Tap implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                lowHang.setPosition(AUTO_HANG_HANG);
                return false;
            }
        }

        public class Vert implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                lowHang.setPosition(AUTO_HANG_VERT);
                return false;
            }
        }


        public Action vert() {
            return new Vert();
        }

        public Action down() {
            return new Down();
        }

        public Action tap() {
            return new Tap();
        }
    }


    public class Sleep
    {
        public Sleep()
        {

        }
        public class oneSecond implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                try {
                    Thread.sleep(800);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                return false;
            }

        }

        public class half implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                return false;
            }

        }

        public Action oneSec()
        {
            return new oneSecond();
        }

        public Action half()
        {
            return new half();
        }
    }

    @Override
    public void runOpMode() {

        Pose2d pose = new Pose2d(0, 0, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, pose);

        Grasper grasper = new Grasper(hardwareMap);
        Rotator rotator = new Rotator(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);
        OuttakeServo bucket = new OuttakeServo(hardwareMap);
        KickStands kickstands = new KickStands(hardwareMap);
        LowHang tapper = new LowHang(hardwareMap);

        Sleep sleep = new Sleep();

        TrajectoryActionBuilder action = drive.actionBuilder(pose)
                .strafeToLinearHeading(new Vector2d(-15, 8), Math.toRadians(45));

        TrajectoryActionBuilder action2 = drive.actionBuilder(new Pose2d(-15, 8, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(-19, 4), Math.toRadians(45));

        TrajectoryActionBuilder action3 = drive.actionBuilder(new Pose2d(-19, 4, Math.toRadians(45)))
                        .strafeToLinearHeading(new Vector2d(-12, 10), Math.toRadians(90));

        TrajectoryActionBuilder action4 = drive.actionBuilder(new Pose2d(-12, 10, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-6.5, 22), Math.toRadians(90));

        TrajectoryActionBuilder action5 = drive.actionBuilder(new Pose2d(-6.5, 22, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-12, 10), Math.toRadians(45));

        TrajectoryActionBuilder action6 = drive.actionBuilder(new Pose2d(-12, 10, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(-19, 4), Math.toRadians(45));

        TrajectoryActionBuilder action7 = drive.actionBuilder(new Pose2d(-19, 4, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(-16, 15), Math.toRadians(90));

        TrajectoryActionBuilder action8 = drive.actionBuilder(new Pose2d(-16, 15, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-20, 22), Math.toRadians(90));

        TrajectoryActionBuilder action9 = drive.actionBuilder(new Pose2d(-20, 22, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-12, 10), Math.toRadians(45));

        TrajectoryActionBuilder action10 = drive.actionBuilder(new Pose2d(-12, 10, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(-19, 4), Math.toRadians(45));

        TrajectoryActionBuilder action11 = drive.actionBuilder(new Pose2d(-19, 4, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(5, 50), Math.toRadians(90),
                        new TranslationalVelConstraint(50),
                        new ProfileAccelConstraint(-50, 50));

        TrajectoryActionBuilder action12 = drive.actionBuilder(new Pose2d(5, 50, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(20, 55), Math.toRadians(90),
                        new TranslationalVelConstraint(50),
                        new ProfileAccelConstraint(-50, 50));


        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                tapper.vert(),
                                rotator.mid(),
                                bucket.intake(),
                                outtake.out(),
                                action.build()
                        ),
                        action2.build(),
                        sleep.half(),
                        bucket.drop(),
                        sleep.half(),
                        new ParallelAction(
                                action3.build(),
                                outtake.in()
                        ),
                        new ParallelAction(
                                rotator.ground(),
                                grasper.open()
                        ),
                        action4.build(),
                        sleep.half(),
                        new ParallelAction(
                                grasper.close(),
                                bucket.intake()
                        ),
                        sleep.half(),
                        rotator.transfer(),
                        sleep.half(),
                        grasper.open(),
                        sleep.half(),
                        new ParallelAction(
                                rotator.mid(),
                                outtake.out(),
                                action5.build()
                        ),
                        sleep.half(),
                        action6.build(),
                        sleep.half(),
                        bucket.drop(),
                        sleep.half(),
                        new ParallelAction(
                                action7.build(),
                                outtake.in()
                        ),
                        bucket.intake(),
                        new ParallelAction(
                                action8.build(),
                                rotator.ground(),
                                grasper.open()
                        ),
                        sleep.half(),
                        new ParallelAction(
                                grasper.close(),
                                bucket.intake()
                        ),
                        sleep.half(),
                        rotator.transfer(),
                        sleep.half(),
                        grasper.open(),
                        sleep.half(),
                        rotator.mid(),
                        new ParallelAction(
                                outtake.out(),
                                action9.build()
                        ),
                        sleep.half(),
                        action10.build(),
                        sleep.half(),
                        bucket.drop(),
                        sleep.half(),
                        new ParallelAction(
                                action11.build(),
                                outtake.in()
                        ),
                        new ParallelAction(
                                action12.build(),
                                kickstands.flat()
                        ),
                        tapper.down()
                )
        );


        while (opModeIsActive())
        {

        }



    }
}

