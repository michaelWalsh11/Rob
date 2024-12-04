package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class AutoCommands {

    public AutoCommands()
    {

    }

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
            return new IntakeToPos(0, 0.2);
        }

        public Action out() {
            return new IntakeToPos(1400, 0.2);
        }

    }


    public class Outtake {

        private DcMotorEx outTake1;
        private DcMotorEx outTake2;

        public Outtake(HardwareMap hardwareMap) {
            outTake1 = hardwareMap.get(DcMotorEx.class, "outtake1");
            outTake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            outTake1.setDirection(DcMotorSimple.Direction.REVERSE);

            outTake2 = hardwareMap.get(DcMotorEx.class, "outtake2");
            outTake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            outTake2.setDirection(DcMotorSimple.Direction.FORWARD);

        }

        public class OuttakeToPos implements Action {
            private final double targetPosition;
            private final double holdingPower;
            private boolean initialized = false;

            public OuttakeToPos(double targetPosition, double holdingPower) {
                this.targetPosition = targetPosition;
                this.holdingPower = holdingPower;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized)
                {
                    outTake1.setPower(0.8);
                    outTake2.setPower(0.8);
                    initialized = true;
                }

                double pos = outTake1.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < targetPosition) {
                    return true; // Continue running the action
                }
                else
                {
                    outTake1.setPower(holdingPower);  // Apply a small power to hold the position
                    outTake2.setPower(holdingPower);
                    return false; // Stop running, but maintain hold
                }
            }

            // Method to create actions with holding power
        }

        public Action in() {
            return new OuttakeToPos(0, 0.2);
        }

        public Action out() {
            return new OuttakeToPos(2500, 0.2);
        }

    }

    //rotator
    public class Rotator
    {
        private Servo rotator;

        public Rotator(HardwareMap hardwareMap) {
            rotator = hardwareMap.get(Servo.class, "rotator");
        }

        public class Rotate1 implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotator.setPosition(1.0);
                return false;
            }
        }

        public class Rotate0 implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotator.setPosition(0.2);
                return false;
            }
        }

        public Action go1() {
            return new Rotate1();
        }

        public Action go0() {
            return new Rotate0();
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
                outtake.setPosition(1.0);
                return false;
            }
        }

        public class Flat implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtake.setPosition(0.2);
                return false;
            }
        }

        public Action drop() {
            return new Drop();
        }

        public Action flat() {
            return new Flat();
        }
    }


    //angler
    public class Grasper
    {
        private Servo grasper;

        public Grasper(HardwareMap hardwareMap) {
            grasper = hardwareMap.get(Servo.class, "grasper");
        }

        public class GrasperOpen implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grasper.setPosition(1.0);
                return false;
            }
        }

        public class GrasperClose implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grasper.setPosition(0.0);
                return false;
            }
        }


        public Action open() {
            return new GrasperOpen();
        }

        public Action close() {
            return new GrasperClose();
        }
    }
}