package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.Intake;
import org.firstinspires.ftc.teamcode.utils.MecDrive;
import org.firstinspires.ftc.teamcode.utils.Outtake;
import org.firstinspires.ftc.teamcode.utils.RRQuickStart.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.WobbleGoal;

public class LearnRoadRunner extends LinearOpMode {
    SampleMecanumDrive drive;
    WobbleGoal wg;
    Outtake o;
    Trajectory driveToPointA;
    Trajectory driveToShootingPt;
    Trajectory driveToStart2;
    Trajectory start2ToPointA;
    Intake intake;
    Trajectory pointAToLine;
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        //Grab and pick up wg
        wg.grab();
        sleep(500);
        wg.lift();
        sleep(500);
        wg.stop();


        drive.followTrajectory(driveToPointA);
        //TODO Release WG
        wg.release();


        drive.followTrajectory(driveToShootingPt);
        //TODO Shoot 3 rings
        o.spin();
        for (int i = 0 ; i < 3; i++) {
            o.push();
            sleep(250);
            o.pull();
            if(i!=2) {
                sleep(250);
            }
        }
        o.stop();


        drive.followTrajectory(driveToStart2);
        //TODO lower wg-elevator and pick up wobble goal
        wg.lower();
        wg.grab();
        wg.lift();

        //TODO Create a trajectory and follow it to Point A
        Pose2d a = new Pose2d(-48,-24, Math.toRadians(0));
        start2ToPointA = drive.trajectoryBuilder(driveToPointA.end()).splineTo(new Vector2d(12,-60), Math.toRadians(0)).build();


        //TODO drop wg
        wg.lower();
        wg.release();
        wg.stop();

        //TODO Park On Line
        

    }
    public void initialize(){
        intake = new Intake(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        wg = new WobbleGoal(hardwareMap);
        o = new Outtake(hardwareMap);
        //-60,-48
        Pose2d startPosition = new Pose2d(-60, -48, Math.toRadians(0));

        //Create Trajectory to point A (12,-60)
        driveToPointA = drive.trajectoryBuilder(startPosition)
                .splineTo(new Vector2d(12,-60), Math.toRadians(0))
                .build();

        //0,-36
        driveToShootingPt = drive.trajectoryBuilder(driveToPointA.end()).splineTo(new Vector2d(0,-36), Math.toRadians(0)).build();
        //(-48,-24)
        driveToStart2 = drive.trajectoryBuilder(driveToShootingPt.end()).splineTo(new Vector2d(-48,-24),Math.toRadians(180)).build();


        waitForStart();
    }

}
