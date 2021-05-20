package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.RRQuickStart.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.WobbleGoal;

public class Passive4Rings extends LinearOpMode {
    private WobbleGoal wg;
    private Pose2d startPose = new Pose2d(-60.0,-48.0,0.0);
    private Vector2d wgCVector = new Vector2d(48,-60);
    private Vector2d finish = new Vector2d(12,-60);
    private SampleMecanumDrive drive;
    private Trajectory startToWGC;
    private Trajectory wgCToFinish;
    @Override
    public void runOpMode(){
        initialize();
        wg.grab();
        sleep(2000);
        wg.lift();
        sleep(500);
        wg.stop();
        waitForStart();
        drive.followTrajectory(startToWGC);
        wg.release();
        wg.stop();
        drive.followTrajectory(wgCToFinish);


    }
    public void initialize(){
        drive = new SampleMecanumDrive(hardwareMap);
        createTrajectories();


    }
    public void createTrajectories(){
        drive.setPoseEstimate(startPose);
        startToWGC = drive.trajectoryBuilder(startPose)
                .splineTo(wgCVector,Math.toRadians(45))
                .build();
        wgCToFinish = drive.trajectoryBuilder(startToWGC.end())
                .splineTo(finish,Math.toRadians(0))
                .build();
        /*startToWGA2 =drive.trajectoryBuilder(startToWGA1.end())
                .strafeTo(wgAVector2)
                .build();*/
    }
}
