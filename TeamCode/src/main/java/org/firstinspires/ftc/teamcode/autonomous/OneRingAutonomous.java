package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.Intake;
import org.firstinspires.ftc.teamcode.utils.Outtake;
import org.firstinspires.ftc.teamcode.utils.RRQuickStart.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.WobbleGoal;

public class OneRingAutonomous extends LinearOpMode {
    //Declare Trajectories

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    WobbleGoal wg = new WobbleGoal(hardwareMap);
    Outtake o = new Outtake(hardwareMap);
    Intake in = new Intake(hardwareMap);
    Trajectory driveToShootingPoint;
    Trajectory driveToRings;

    public void initialize(){
       //Initialize all the objects, etc
        Pose2d startPos = new Pose2d(-60,-48, Math.toRadians(0));

        //Create Trajectory to shooting point
        driveToShootingPoint = drive.trajectoryBuilder(startPos)
                .splineTo(new Vector2d(0,-36), Math.toRadians(0))
                .build();
    }

    public void shoot(int n) {
        o.spin();
        for (int i = 0; i < n; i++) {
            o.push();
        }
        o.stop();
    }

    public void forwardToRings() {
        Pose2d temp = new Pose2d(0,-36, Math.toRadians(0));
        driveToRings = drive.trajectoryBuilder(temp)
                .splineTo(new Vector2d(-36,-36), Math.toRadians(0))
                .build();
    }

    public void driveBack() {
        Pose2d temp = new Pose2d(-36,-36,Math.toRadians(0));
        Trajectory t = drive.trajectoryBuilder(temp)
                .splineTo(new Vector2d(0,-36),Math.toRadians(0))
                .build();

        shoot(1);
    }

    public void driveToPointA() {
        Pose2d temp = new Pose2d(0,-36,Math.toRadians(0));
        Trajectory t = drive.trajectoryBuilder(temp)
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        shoot(3);

        in.spin();

        forwardToRings();

        driveBack();

        driveToPointA();

        wg.release();
    }

}