package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.Outtake;
import org.firstinspires.ftc.teamcode.utils.RRQuickStart.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.WobbleGoal;

@Autonomous(name="FebRR0RingsPS")
public class FebRR0RingsPS extends LinearOpMode {
    private WobbleGoal wg;
    private Outtake out;
    private Pose2d startPose = new Pose2d(-60.0,-36.0,0.0);
    private Vector2d wgAVector = new Vector2d(0,-50);
    private Vector2d shootingVectorA = new Vector2d(0,-26);
    private Vector2d shootingVectorB = new Vector2d(0,-36+7);
    private Vector2d shootingVectorC = new Vector2d(0,-36+14);
    private Vector2d wg2Pos = new Vector2d(-50,-32);
    private Vector2d finish = new Vector2d(12,-24);
    private SampleMecanumDrive drive;
    private Trajectory startToWGA;
    private Trajectory wgAToShootingA;
    private Trajectory wgAToShootingB;
    private Trajectory wgAToShootingC;
    private Trajectory shootingToWG2;
    private Trajectory shootingCToShootingA;
    private Trajectory wg2TowgA;
    private Trajectory wgAToFinish;
    @Override
    public void runOpMode(){
        initialize();
        wg.grab();
        sleep(2000);
        wg.lift();
        sleep(500);
        wg.stop();
        waitForStart();
        drive.followTrajectory(startToWGA);
        wg.release();
        wg.stop();
        out.spin();
        sleep(1000);
        drive.followTrajectory(wgAToShootingA);
        Trajectory[] arr = {wgAToShootingB, wgAToShootingC,shootingCToShootingA};
        //drive.turn(Math.toRadians(180));
        sleep(500);
        for(int i=0;i<3;i++) {
            out.pull();
            sleep(1500);
            out.push();
            drive.followTrajectory(arr[i]);



        }
        out.stop();
        wg.lower();
        sleep(500);
        wg.stop();
        drive.followTrajectory(shootingToWG2);
        wg.grab();
        sleep(1000);
        wg.lift();
        sleep(500);
        wg.stop();
        drive.followTrajectory(wg2TowgA);
        wg.release();
        sleep(250);
        drive.followTrajectory(wgAToFinish);

    }
    public void initialize(){
        drive = new SampleMecanumDrive(hardwareMap);
        wg = new WobbleGoal(hardwareMap);
        out=new Outtake(hardwareMap);
        createTrajectories();


    }
    public void createTrajectories(){
        drive.setPoseEstimate(startPose);
        startToWGA = drive.trajectoryBuilder(startPose)
                .splineTo(wgAVector,Math.toRadians(45))
                .build();
        wgAToShootingA = drive.trajectoryBuilder(startToWGA.end())
                .splineTo(shootingVectorA,Math.toRadians(180))
                .build();
        wgAToShootingB = drive.trajectoryBuilder(wgAToShootingA.end())
                .strafeRight(5)
                .build();
        wgAToShootingC = drive.trajectoryBuilder(wgAToShootingB.end())
                .strafeRight(7.5)
                .build();
        shootingCToShootingA = drive.trajectoryBuilder(wgAToShootingC.end())
                .splineTo(shootingVectorA,Math.toRadians(180))
                .build();
        shootingToWG2 = drive.trajectoryBuilder(shootingCToShootingA.end())
                .splineTo(wg2Pos,Math.toRadians(180))
                .build();
        wg2TowgA = drive.trajectoryBuilder(shootingToWG2.end())
                .splineTo(wgAVector,Math.toRadians(45))
                .build();
        wgAToFinish = drive.trajectoryBuilder(wg2TowgA.end())
                .splineTo(finish,Math.toRadians(0))
                .build();
        /*startToWGA2 =drive.trajectoryBuilder(startToWGA1.end())
                .strafeTo(wgAVector2)
                .build();*/
    }
}


