package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.Outtake;
import org.firstinspires.ftc.teamcode.utils.RRQuickStart.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.WobbleGoal;

@Autonomous(name="JanuaryRR4Rings")
public class JanuaryRR4Rings extends LinearOpMode {
    private WobbleGoal wg;
    private Outtake out;
    private Pose2d startPose = new Pose2d(-60.0,-36.0,0.0);
    private Vector2d wgAVector = new Vector2d(0,-50);
    private Vector2d shootingVector = new Vector2d(0,-36);
    private SampleMecanumDrive drive;
    private Trajectory startToWGA;
    private Trajectory wgAToShooting;
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
        sleep(500);
        drive.followTrajectory(wgAToShooting);
        //drive.turn(Math.toRadians(180));
        for(int i=0;i<3;i++) {
            out.push();
            sleep(1000);
            out.pull();
            if(i!=2) {
                sleep(1000);
            }
        }
        out.stop();


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
        wgAToShooting = drive.trajectoryBuilder(startToWGA.end())
                .splineTo(shootingVector,Math.toRadians(180))
                .build();
        /*startToWGA2 =drive.trajectoryBuilder(startToWGA1.end())
                .strafeTo(wgAVector2)
                .build();*/
    }
}
