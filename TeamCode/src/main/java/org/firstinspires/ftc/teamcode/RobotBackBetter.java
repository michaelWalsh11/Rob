package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class RobotBackBetter {

    //Drive Motors
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftRear = null;
    public DcMotor rightRear = null;
    public DcMotor outTake1 = null;
    public DcMotor outTake2 = null;
    public DcMotor inTake1 = null;
    public DcMotor inTake2 = null;

    public Servo grasper = null;
    public Servo rotator = null;
    public Servo outtake = null;

    public Servo kickLeft = null;
    public Servo kickRight = null;

    public IMU imu;
    public double offset = 0;//in degrees

    HardwareMap hwMap = null;

    //todo
    // - write down control hub orientation
    // - write motor stuff on phone
    // - see if we have servos and initialize them

    public RobotBackBetter()
    {

    }

    public void init(HardwareMap ahwMap)
    {
        hwMap = ahwMap;

        leftFront = hwMap.dcMotor.get("leftfront");
        rightFront = hwMap.dcMotor.get("rightfront");
        leftRear = hwMap.dcMotor.get("leftrear");
        rightRear = hwMap.dcMotor.get("rightrear");
        outTake1 = hwMap.dcMotor.get("outtake1");
        outTake2 = hwMap.dcMotor.get("outtake2");
        inTake1 = hwMap.dcMotor.get("intake1");
        inTake2 = hwMap.dcMotor.get("intake2");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        outTake1.setDirection(DcMotor.Direction.FORWARD);
        outTake2.setDirection(DcMotor.Direction.FORWARD);
        inTake1.setDirection(DcMotor.Direction.FORWARD);
        inTake2.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        grasper = hwMap.servo.get("grasper");
        rotator = hwMap.servo.get("rotator");
        outtake = hwMap.servo.get("outtake");

        kickLeft = hwMap.servo.get("kickleft");
        kickRight = hwMap.servo.get("kickright");

        imu = hwMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    public double getOrientation() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES) - offset;
    }

    public double resetImu() {
        offset = imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES));
        return offset;
    }
}
