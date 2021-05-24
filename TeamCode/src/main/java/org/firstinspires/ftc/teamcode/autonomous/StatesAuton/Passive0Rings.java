package org.firstinspires.ftc.teamcode.autonomous.StatesAuton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.Outtake;
import org.firstinspires.ftc.teamcode.utils.RRQuickStart.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.WobbleGoal;

public class Passive0Rings extends LinearOpMode {
    private WobbleGoal wg;
    private Pose2d startPose = new Pose2d(-60.0,-48.0,0.0);
    private Vector2d wgAVector = new Vector2d(0,-60);
    private Vector2d backup = new Vector2d(-12,-60);
    private Vector2d finish = new Vector2d(12,-36);
    private SampleMecanumDrive drive;
    private Trajectory startToWGA;
    private Trajectory WGAbackup;
    private Trajectory wgAToFinish;
    @Override
    public void runOpMode(){
        initialize();
        wg.grab();
        sleep(250);
        wg.lift();
        sleep(250);
        wg.stop();
        waitForStart();
        drive.followTrajectory(startToWGA);
        wg.release();
        wg.stop();
        drive.followTrajectory(wgAToFinish);


    }
    public void initialize(){
        wg = new WobbleGoal(hardwareMap);

        drive = new SampleMecanumDrive(hardwareMap);
        createTrajectories();


    }
    public void createTrajectories(){
        drive.setPoseEstimate(startPose);
        startToWGA = drive.trajectoryBuilder(startPose)
                .splineTo(wgAVector,Math.toRadians(0))
                .build();
        WGAbackup = drive.trajectoryBuilder(startToWGA.end())
                .splineTo(backup,Math.toRadians(0))
                .build();
        wgAToFinish = drive.trajectoryBuilder(WGAbackup.end())
                .splineTo(finish,Math.toRadians(0))
                .build();
        /*startToWGA2 =drive.trajectoryBuilder(startToWGA1.end())
                .strafeTo(wgAVector2)
                .build();*/
    }
}
