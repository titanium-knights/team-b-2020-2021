package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.Intake;
import org.firstinspires.ftc.teamcode.utils.Outtake;
import org.firstinspires.ftc.teamcode.utils.RRQuickStart.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.utils.RRQuickStart.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Shooter2;
import org.firstinspires.ftc.teamcode.utils.WobbleGoal;

import java.util.Arrays;
import java.util.Vector;

@Autonomous(name="March4Ring")
public class March4Ring extends LinearOpMode {
    private WobbleGoal wg;
    private Shooter2 out;
    private Intake intake;
    private SampleMecanumDrive drive;

    private Pose2d startPose = new Pose2d(-60.0,-36.0,0.0);
    private Vector2d wgDumpZone = new Vector2d(24,-30);
    private Vector2d powerShotLeft = new Vector2d(0,-22);
    private Vector2d intakePre = new Vector2d(-12,-44);
    private Vector2d intakePost = new Vector2d(-24,-44);
    private Vector2d wg2Pos = new Vector2d(-50,-30);
    private Vector2d line = new Vector2d(12,-30);


    private Trajectory startToPSLeft;
    private Trajectory psLeftToPsMid;
    private Trajectory psMidToPsRight;
    private Trajectory psRightToIntakePre;
    private Trajectory intakePreToIntakePost;
    private Trajectory intakePostToThreeRingsShoot;
    private Trajectory intakePostToWGDump1;
    private Trajectory wgDump1ToWG2;
    private Trajectory wg2ToWGDump2;
    private Trajectory intakePostToIntakePre;
    private Trajectory intakePreToIntakePost2;

    @Override
    public void runOpMode(){
        initialize();
        wg.grab();
        sleep(2000);
        wg.lift();
        sleep(500);
        wg.stop();
        waitForStart();
        out.spinPowershot();
        Trajectory[] arr = {psLeftToPsMid,psMidToPsRight, psRightToIntakePre};
        drive.followTrajectory(startToPSLeft);
        sleep(500);
        for(int i=0;i<3;i++){
            out.pull();
            sleep(1500);
            out.push();
            drive.followTrajectory(arr[i]);
        }
        out.stop();

        //Picks up 3 rings
        intake.spinBoth();
        drive.followTrajectory(intakePreToIntakePost);
        sleep(2000);
        out.spinHighGoal();
        drive.followTrajectory(intakePostToIntakePre);
        sleep(250);
        intake.stop();


        //Shoots 3 ring that was intaked

        out.pull();
        sleep(750);
        out.stop();
        out.push();

        intake.spinBoth();
        drive.followTrajectory(intakePreToIntakePost2);
        out.spinHighGoal();
        sleep(1000);
        out.pull();
        sleep(250);
        out.stop();
        out.push();
        //GoesTo dump and dumps
        drive.followTrajectory(intakePostToWGDump1);
        wg.release();
        wg.lower();
        sleep(500);
        wg.stop();
/*
        //Goes to 2nd wg and picks up
        drive.followTrajectory(wgDump1ToWG2);
        wg.grab();
        sleep(1000);
        wg.lift();
        sleep(500);
        wg.stop();

        //Goes to dump and dumps
        drive.followTrajectory(wg2ToWGDump2);
        wg.release();
        sleep(250);

        //Drives to line
        drive.followTrajectory(wgDump2ToPark);
        */
    }
    public void initialize(){
        drive = new SampleMecanumDrive(hardwareMap);
        wg = new WobbleGoal(hardwareMap);
        out=new Shooter2(hardwareMap);
        intake = new Intake(hardwareMap);
        createTrajectories();


    }
    public void createTrajectories(){
        drive.setPoseEstimate(startPose);
        startToPSLeft = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(powerShotLeft.getX(),powerShotLeft.getY(),Math.toRadians(180)),-7.0)
                .build();
        psLeftToPsMid = drive.trajectoryBuilder(startToPSLeft.end())
                .strafeLeft(8)
                .build();
        psMidToPsRight = drive.trajectoryBuilder(psLeftToPsMid.end())
                .strafeLeft(7)
                .build();
        psRightToIntakePre = drive.trajectoryBuilder(psMidToPsRight.end())
                .splineToConstantHeading(intakePre,0)
                .build();
        intakePreToIntakePost = drive.trajectoryBuilder(psRightToIntakePre.end())
                .splineToConstantHeading(intakePost,0)
                .build();

        intakePostToIntakePre = drive.trajectoryBuilder(intakePreToIntakePost.end())
                .splineToConstantHeading(intakePre,0)
                .build();
        intakePreToIntakePost2 = drive.trajectoryBuilder(intakePostToIntakePre.end())
                .splineToConstantHeading(intakePost,0)
                .build();
        intakePostToWGDump1 = drive.trajectoryBuilder(intakePreToIntakePost2.end())
                .splineTo(
                        wgDumpZone,Math.toRadians(45),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(13, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)

                )
                .build();
        wgDump1ToWG2 = drive.trajectoryBuilder(intakePostToWGDump1.end())
                .splineTo(wg2Pos,Math.toRadians(0))
                .build();
        wg2ToWGDump2 = drive.trajectoryBuilder(wgDump1ToWG2.end())
                .splineTo(wgDumpZone,Math.toRadians(45))
                .build();

    }
}


