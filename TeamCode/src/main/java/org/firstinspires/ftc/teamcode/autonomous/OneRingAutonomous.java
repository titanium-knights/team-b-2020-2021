package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.Intake;
import org.firstinspires.ftc.teamcode.utils.Outtake;
import org.firstinspires.ftc.teamcode.utils.RRQuickStart.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.WobbleGoal;

/**
 * Created by Alice
 */

public class OneRingAutonomous extends LinearOpMode {
    //Declare Trajectories

    SampleMecanumDrive drive;
    WobbleGoal wg;
    Outtake out;
    Intake intake;

    Trajectory driveToShootingPt;
    Trajectory driveToRing;
    Trajectory ringToShootingPt;
    Trajectory driveToPointB;
    Trajectory driveToStart2;
    Trajectory start2ToPointB;
    Trajectory pointBToLine;

    public void initialize(){
        //initiate all the objects, etc
        drive = new SampleMecanumDrive(hardwareMap);
        wg = new WobbleGoal(hardwareMap);
        out = new Outtake(hardwareMap);
        intake = new Intake(hardwareMap);
        Pose2d startPosition = new Pose2d(-60, -48, Math.toRadians(0));

        driveToShootingPt = drive.trajectoryBuilder(startPosition)
                .splineTo(new Vector2d(0,-36), Math.toRadians(0))
                .build();
        driveToRing = drive.trajectoryBuilder(driveToShootingPt.end())
                .splineTo(new Vector2d(-24,-36),Math.toRadians(180))
                .build();
        ringToShootingPt = drive.trajectoryBuilder(driveToRing.end())
                .splineTo(new Vector2d(0,-36), Math.toRadians(0))
                .build();
        driveToPointB = drive.trajectoryBuilder(ringToShootingPt.end())
                .splineTo(new Vector2d(36,-36), Math.toRadians(0))
                .build();
        driveToStart2 = drive.trajectoryBuilder(driveToPointB.end())
                .splineTo(new Vector2d(-48,-24), Math.toRadians(180))
                .build();
        start2ToPointB = drive.trajectoryBuilder(driveToStart2.end())
                .splineTo(new Vector2d(36,-36), Math.toRadians(0))
                .build();
        pointBToLine = drive.trajectoryBuilder(start2ToPointB.end())
                .splineTo(new Vector2d(12,-36), Math.toRadians(180))
                .build();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();
        drive.followTrajectory(driveToShootingPt);
        out.spin();
        out.push();
        sleep(250);
        out.pull();
        sleep(250);
        out.push();
        sleep(250);
        out.pull();
        sleep(250);
        out.push();
        sleep(250);
        out.pull();
        intake.spin();
        drive.followTrajectory(driveToRing);
        intake.stop();
        drive.followTrajectory(ringToShootingPt);
        out.spin();
        out.push();
        sleep(250);
        out.pull();
        drive.followTrajectory(driveToPointB);
        wg.release();
        drive.followTrajectory(driveToStart2);
        wg.lower();
        sleep(500);
        wg.stop();
        wg.grab();
        wg.lift();
        sleep(500);
        wg.stop();
        drive.followTrajectory(start2ToPointB);
        wg.release();
        drive.followTrajectory(pointBToLine);

    }
}
