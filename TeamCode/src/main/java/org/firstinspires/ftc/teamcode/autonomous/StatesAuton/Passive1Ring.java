package org.firstinspires.ftc.teamcode.autonomous.StatesAuton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.DualShooterNoPID;
import org.firstinspires.ftc.teamcode.utils.RRQuickStart.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.WobbleGoal;

public class Passive1Ring extends LinearOpMode {
    private WobbleGoal wg;
    private Pose2d startPose = new Pose2d(-60.0,-48.0,0.0);
    private Vector2d wgBVector = new Vector2d(36,-60);
    private Vector2d finish = new Vector2d(12,-60);
    private SampleMecanumDrive drive;
    private Trajectory startToWGB;
    private Trajectory wgBToFinish;
    private DualShooterNoPID outtake;

    @Override
    public void runOpMode(){
        initialize();
        wg.grab();
        sleep(250);
        wg.lift();
        sleep(250);
        wg.stop();
        waitForStart();
        drive.followTrajectory(startToWGB);
        wg.release();
        wg.stop();
        //Turn to face the high goal
        drive.turn(Math.toRadians(195));
        outtake.spinHighGoal();
        sleep(500);
        for(int i=0;i<3;i++) {
            outtake.pull();
            sleep(125);
            outtake.push();
            if(i!=2){
                sleep(125);
            }
        }
        drive.followTrajectory(wgBToFinish);


    }
    public void initialize(){
        wg = new WobbleGoal(hardwareMap);
        outtake = new DualShooterNoPID(hardwareMap);

        drive = new SampleMecanumDrive(hardwareMap);
        createTrajectories();


    }
    public void createTrajectories(){
        drive.setPoseEstimate(startPose);

        startToWGB = drive.trajectoryBuilder(startPose)
                .splineTo(wgBVector,Math.toRadians(90))
                .build();
        wgBToFinish = drive.trajectoryBuilder(startToWGB.end())
                .splineTo(finish,Math.toRadians(180))
                .build();
        /*startToWGA2 =drive.trajectoryBuilder(startToWGA1.end())
                .strafeTo(wgAVector2)
                .build();*/
    }
}
