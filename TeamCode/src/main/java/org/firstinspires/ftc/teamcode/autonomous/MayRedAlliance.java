package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.DualShooterNoPID;
import org.firstinspires.ftc.teamcode.utils.Intake;
import org.firstinspires.ftc.teamcode.utils.Outtake;
import org.firstinspires.ftc.teamcode.utils.RRQuickStart.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.utils.RRQuickStart.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Shooter2;
import org.firstinspires.ftc.teamcode.utils.WobbleGoal;

import java.util.Arrays;
import java.util.Vector;

public class MayRedAlliance extends LinearOpMode {
    private final int Y_MULTIPLIER = 1;
    private WobbleGoal wg;
    private DualShooterNoPID out;
    private Intake intake;
    private SampleMecanumDrive drive;

    private Pose2d startPose = new Pose2d(-60.0,-48.0*Y_MULTIPLIER,0.0);
    //wqqqq
    private Vector2d wgDumpZone = new Vector2d(30,-36*Y_MULTIPLIER);
    private Vector2d powerShotLeft = new Vector2d(0,(-22+14)*Y_MULTIPLIER); // might be -22-14???
    private Vector2d intakePre = new Vector2d(-12,-39*Y_MULTIPLIER);
    private Vector2d intakePost = new Vector2d(-24,-39*Y_MULTIPLIER);

    private Vector2d wg2Pos = new Vector2d(-36,-24*Y_MULTIPLIER);
    private Vector2d line = new Vector2d(12,-30*Y_MULTIPLIER);


    private Trajectory startToPSLeft;
    private Trajectory psLeftToPsMid;
    private Trajectory psMidToPsRight;
    private Trajectory psRightToIntakePre;
    private Trajectory intakePreToIntakePost;
    private Trajectory intakePostToOneRingShoot;
    private Trajectory intakePostToWGDump1;
    private Trajectory wgDump1ToWG2;
    private Trajectory wg2ToWGDump2;
    private Trajectory wgDump2ToPark;

    @Override
    public void runOpMode() {
        initialize();

        wg.grab();
        sleep(250);
        wg.lift();
        sleep(250);
        wg.stop();
        waitForStart();

        out.spinPowershot();
        Trajectory[] arr = {psLeftToPsMid,psMidToPsRight, psRightToIntakePre};
        drive.followTrajectory(startToPSLeft);

        for(int i = 0; i < 3; i++) {
            out.pull();
            sleep(250);
            out.push();
            drive.followTrajectory(arr[i]);
        }
        out.stop();

        //Picks up 1 ring
        intake.spinBoth();
        drive.followTrajectory(intakePreToIntakePost);
        drive.followTrajectory(intakePostToOneRingShoot);
        out.spinHighGoal();
        sleep(2500);
        intake.stop();

        //Shoots 1 ring that was intaken
        out.pull();
        sleep(750);
        out.stop();
        out.push();


        //GoesTo dump and dumps
        drive.followTrajectory(intakePostToWGDump1);
        wg.release();
        wg.lower();
        sleep(500);
        wg.stop();

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
    }

    public void initialize(){
        drive = new SampleMecanumDrive(hardwareMap);
        wg = new WobbleGoal(hardwareMap);
        out=new DualShooterNoPID(hardwareMap);
        intake = new Intake(hardwareMap);
        createTrajectories();
    }

    public void createTrajectories() {
        drive.setPoseEstimate(startPose);

        startToPSLeft = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(powerShotLeft.getX(), powerShotLeft.getY(), Math.toRadians(180)),-7.0)
                .build();
        psLeftToPsMid = drive.trajectoryBuilder(startToPSLeft.end())
                .strafeRight(8) // pretty sure this is right
                .build();
        psMidToPsRight = drive.trajectoryBuilder(psLeftToPsMid.end())
                .strafeRight(7)
                .build();
        psRightToIntakePre = drive.trajectoryBuilder(psMidToPsRight.end())
                .splineToConstantHeading(intakePre,0)
                .build();
        intakePreToIntakePost = drive.trajectoryBuilder(psRightToIntakePre.end())
                .splineToConstantHeading(intakePost,0)
                .build();
        intakePostToOneRingShoot = drive.trajectoryBuilder(intakePreToIntakePost.end())
                .back(12)
                .build();
        intakePostToWGDump1 = drive.trajectoryBuilder(intakePostToOneRingShoot.end())
                .splineTo(wgDumpZone, Math.toRadians(0))
                .build();
        wgDump1ToWG2 = drive.trajectoryBuilder(intakePostToWGDump1.end())
                .splineTo(wg2Pos, Math.toRadians(0))
                .build();
        wg2ToWGDump2 = drive.trajectoryBuilder(wgDump1ToWG2.end())
                .splineTo(wgDumpZone, Math.toRadians(45))
                .build();
        wgDump2ToPark = drive.trajectoryBuilder(wg2ToWGDump2.end())
                .splineToConstantHeading(line,0)
                .build();
    }
}
