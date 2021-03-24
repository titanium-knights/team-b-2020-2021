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
import org.firstinspires.ftc.teamcode.utils.Shooter2;
import org.firstinspires.ftc.teamcode.utils.RRQuickStart.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.utils.RRQuickStart.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.WobbleGoal;

import java.util.Arrays;
import java.util.Vector;

@Autonomous(name="March1RingImproved")
public class March1RingImproved extends LinearOpMode {
    private WobbleGoal wg;
    private Shooter2 out;
    private Intake intake;
    private SampleMecanumDrive drive;
    private Vector2d intakePre = new Vector2d(-12,-38);
    private Vector2d intakePost = new Vector2d(-24,-38);


    private Pose2d trialStartPose = new Pose2d(-60.0,-36.0,Math.toRadians(180));
    private Trajectory trialRing1,trialRing2,trialRing3,trialRing4;
    private Trajectory goToPs2;
    private Trajectory goToPs3;

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
        Trajectory[] arr = {goToPs2,goToPs3, trialRing2};
        drive.followTrajectory(trialRing1);
        sleep(550);
        for(int i=0;i<3;i++){
            out.pull();
            sleep(400);
            out.push();
            if(i==2){
                //intake.spinBoth();
                //out.spinHighGoal();
                break;
            }
            drive.followTrajectory(arr[i]);
        }
        out.push();
        /*
        sleep(500);
        intake.stop();

        //Shoots 1 ring that was intaked

        out.pull();
        sleep(250);
        out.push();
        out.stop();



        //GoesTo dump and dumps
        drive.followTrajectory(trialRing3);
        wg.lift();
        sleep(500);
        wg.stop();
        drive.followTrajectory(trialRing4);*/
    }
    public void initialize(){
        drive = new SampleMecanumDrive(hardwareMap);
        wg = new WobbleGoal(hardwareMap);
        out=new Shooter2(hardwareMap);
        intake = new Intake(hardwareMap);

        createTrajectories();


    }
    public void createTrajectories(){
        drive.setPoseEstimate(trialStartPose);
        //Goes to powershot left and avoids hitting rings
        trialRing1 = drive.trajectoryBuilder(trialStartPose)
                .splineToConstantHeading(new Vector2d(-25,-22), 0)
                .splineToLinearHeading(new Pose2d(-5,-23, Math.toRadians(180)), 0)
                .build();
        //Strafes left to powershot 2
        goToPs2 = drive.trajectoryBuilder(trialRing1.end())
                .strafeLeft(8)
                .build();
        //Strafes left to powershot 3
        goToPs3 = drive.trajectoryBuilder(goToPs2.end())
                .strafeLeft(7)
                .build();

        //Goes to intake position, goes forward then strafes left to shooting position
        trialRing2 = drive.trajectoryBuilder(goToPs3.end())
                .splineToLinearHeading(new Pose2d(intakePre,Math.toRadians(180)),0)
                .splineToLinearHeading(new Pose2d(intakePost,Math.toRadians(180)),0)
                .splineToLinearHeading(new Pose2d(intakePost.getX(),intakePost.getY()-15,Math.toRadians(180)),0)
                .build();
        //Goes to wobble goal position, releases, comes back for second
        trialRing3 = drive.trajectoryBuilder(trialRing2.end())
                .splineToLinearHeading(new Pose2d(30,-53,Math.toRadians(180)),0)
                .addSpatialMarker(new Vector2d(30,-53),()->{
                    wg.release();
                })
                .addTemporalMarker(4,()->{
                    wg.lower();
                })
                .addTemporalMarker(4.75,()->{
                    wg.stop();
                })
                .splineToLinearHeading(new Pose2d(-40,-60,Math.toRadians(180)),0)
                .addSpatialMarker(new Vector2d(-40,-60),()->{
                    wg.grab();

                })
                .build();
        //Go to wobble goal position with wobble goal in hand releases and parks on line
        trialRing4 = drive.trajectoryBuilder(trialRing3.end())
                .splineToLinearHeading(new Pose2d(30,-53,Math.toRadians(180)),0)
                .addSpatialMarker(new Vector2d(-40,-60),()->{
                    wg.release();

                })
                .splineToLinearHeading(new Pose2d(12,-53,Math.toRadians(180)),0)
                .build();




    }
}


