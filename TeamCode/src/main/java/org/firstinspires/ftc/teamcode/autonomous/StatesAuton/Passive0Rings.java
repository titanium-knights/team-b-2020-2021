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
    private Vector2d finish = new Vector2d(12,-60);
    private SampleMecanumDrive drive;
    private Trajectory startToWGA;
    private Trajectory wgAToFinish;
    @Override
    public void runOpMode(){
        initialize();
        wg.grab();
        //TODO: We're using a servo to grab and lift so we only need 250 ms for both sleep
        sleep(2000);
        wg.lift();
        sleep(500);
        wg.stop();
        waitForStart();
        drive.followTrajectory(startToWGA);
        wg.release();
        wg.stop();
        drive.followTrajectory(wgAToFinish);


    }
    public void initialize(){
        //TODO initialize wobbleGoal object

        drive = new SampleMecanumDrive(hardwareMap);
        createTrajectories();


    }
    public void createTrajectories(){
        drive.setPoseEstimate(startPose);
        startToWGA = drive.trajectoryBuilder(startPose)
                .splineTo(wgAVector,Math.toRadians(45))
                .build();
        //TODO Currently it will drop the wobble goal and then push the wobble goal out of the box
        wgAToFinish = drive.trajectoryBuilder(startToWGA.end())
                .splineTo(finish,Math.toRadians(0))
                .build();
        /*startToWGA2 =drive.trajectoryBuilder(startToWGA1.end())
                .strafeTo(wgAVector2)
                .build();*/
    }
}
