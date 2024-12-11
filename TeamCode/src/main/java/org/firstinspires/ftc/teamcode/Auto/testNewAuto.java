package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="Trash Auto", group="Autonomous")
public class testNewAuto extends LinearOpMode {

    Pose2d pose = new Pose2d(0, 0, 0);
    //AutoCommands cmd = new AutoCommands();
    MecanumDrive drive = new MecanumDrive(hardwareMap, pose);

    @Override
    public void runOpMode() {


        TrajectoryActionBuilder action = drive.actionBuilder(pose)
                .strafeToLinearHeading(new Vector2d(10, 10), Math.toRadians(90));


        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        action.build()
                )
        );


    }
}
