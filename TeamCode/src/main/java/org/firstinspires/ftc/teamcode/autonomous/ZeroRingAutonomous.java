package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.MecDrive;
import org.firstinspires.ftc.teamcode.utils.Outtake;
import org.firstinspires.ftc.teamcode.utils.RRQuickStart.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.WobbleGoal;

/**
 * Created by Alice
 */
public class ZeroRingAutonomous extends LinearOpMode {
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    WobbleGoal wg = new WobbleGoal(hardwareMap);
    Outtake out = new Outtake(hardwareMap);
    Trajectory driveToPointA;
    Trajectory driveToShootingPt;
    Trajectory driveToStart2;
    Trajectory start2ToPointA;
    Trajectory pointAToLine;
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        drive.followTrajectory(driveToPointA);
        wg.release();
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
        drive.followTrajectory(driveToStart2);
        //pick up wobble goal
        wg.lower();
        sleep(500);
        wg.stop();
        wg.grab();
        wg.lift();
        sleep(500);
        wg.stop();
        drive.followTrajectory(start2ToPointA);
        wg.release();
        sleep(500);
        wg.stop();


    }
    public void initialize(){

        /*Pose2d startPosition = new Pose2d(-72,-48,Math.toRadians(0));
        Trajectory driveToPointA = drive.trajectoryBuilder(startPosition)
                .splineTo(new Vector2d(12,-60),Math.toRadians(0))
                .build();*/
        //-60,-48
        Pose2d startPosition = new Pose2d(-60, -48, Math.toRadians(0));

        //Create Trajectory to point A (12,-60)
        driveToPointA = drive.trajectoryBuilder(startPosition)
                .splineTo(new Vector2d(12,-60), Math.toRadians(0))
                .build();

        //0,-36
        driveToShootingPt = drive.trajectoryBuilder(driveToPointA.end())
                .splineTo(new Vector2d(0,-36), Math.toRadians(0))
                .build();

        driveToStart2 = drive.trajectoryBuilder(driveToShootingPt.end())
                .splineTo(new Vector2d(-48,-24),Math.toRadians(180))
                .build();

        start2ToPointA = drive.trajectoryBuilder(driveToStart2.end())
                .splineTo(new Vector2d(12,-60),Math.toRadians(0))
                .build();
        waitForStart();
    }

}