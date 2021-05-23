package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.utils.RRQuickStart.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Vector2D;
import org.firstinspires.ftc.teamcode.utils.WobbleGoal;

public class TeamWobbleGoal extends LinearOpMode {

    int stackSize;

    SampleMecanumDrive drive;
    WobbleGoal wg;

    Pose2d startingPos;
    Pose2d wgPos4Ring;
    Pose2d wgPos1Ring;
    Pose2d wgPos0Ring;
    Pose2d wgStartPose;

    Trajectory dumpZone;
    Trajectory pickUp;
    Trajectory dumpZone2;
    Trajectory backToStart;

    public void initialize() {
        drive = new SampleMecanumDrive(hardwareMap);
        wg = new WobbleGoal(hardwareMap);

        wgPos4Ring = new Pose2d(54.0,-60.0,0.0);
        wgPos1Ring = new Pose2d(54.0-24,-60.0+24,0.0);
        wgPos0Ring = new Pose2d(12.0,-48.0,Math.toRadians(-90.0));
        wgStartPose = new Pose2d(-38.0,-48.0,Math.toRadians(180.0));

        makeTrajectories();
    }

    public void makeTrajectories() {
        switch (stackSize) {
            case 0:
                startingPos = new Pose2d(12.0,-60.0+24,0.0);
                dumpZone = drive.trajectoryBuilder(startingPos, true)
                        .splineToLinearHeading(wgPos1Ring, 0.0)
                        .build();

                pickUp = drive.trajectoryBuilder(dumpZone.end(), false)
                        .lineToLinearHeading(wgStartPose)
                        .build();

                dumpZone2 = drive.trajectoryBuilder(pickUp.end(), false)
                        .lineToLinearHeading(wgPos1Ring)
                        .build();

                backToStart = drive.trajectoryBuilder(dumpZone2.end(), false)
                        .lineToLinearHeading(startingPos)
                        .build();

            default:
                startingPos = new Pose2d(12.0,-60.0,0.0);
                dumpZone = drive.trajectoryBuilder(startingPos, true)
                        .splineToLinearHeading(wgPos1Ring, 0.0)
                        .build();

                pickUp = drive.trajectoryBuilder(dumpZone.end(), false)
                        .lineToLinearHeading(wgStartPose)
                        .build();

                dumpZone2 = drive.trajectoryBuilder(pickUp.end(), false)
                        .lineToLinearHeading(wgPos4Ring)
                        .build();

                backToStart = drive.trajectoryBuilder(dumpZone2.end(), false)
                        .lineToLinearHeading(startingPos)
                        .build();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        stackSize = 0;

        initialize();
        waitForStart();

        //Wobble Goal dump zone
        drive.followTrajectory(dumpZone);
        wg.lower();
        sleep(250);
        wg.release();
        sleep(250);

        //wobble goal second pick up
        drive.followTrajectory(pickUp);
        wg.grab();
        sleep(250);
        wg.lift();
        sleep(250);

        //wobble goal Dump zone
        drive.followTrajectory(dumpZone2);
        wg.lower();
        sleep(250);
        wg.release();
        sleep(250);
    }
}
