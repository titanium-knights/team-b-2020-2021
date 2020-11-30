package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.utils.CONFIG;
import org.firstinspires.ftc.teamcode.utils.DriveEncoder;
import org.firstinspires.ftc.teamcode.utils.IMU;
import org.firstinspires.ftc.teamcode.utils.Intake;
import org.firstinspires.ftc.teamcode.utils.MecDrive2;
import org.firstinspires.ftc.teamcode.utils.Outtake;
import org.firstinspires.ftc.teamcode.utils.Pusher;
import org.firstinspires.ftc.teamcode.utils.RRQuickStart.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.RingAmount;
import org.firstinspires.ftc.teamcode.utils.Vector2D;
import org.firstinspires.ftc.teamcode.utils.WobbleGoal;

public class DecemberRR extends LinearOpMode {
    private Trajectory start2Zone;
    private Trajectory zone2ShootingPt;
    private Trajectory shootingPt2Intake2shootingPt;
    private Trajectory shootingPt2Start;
    private Trajectory start2ZoneWG2;
    private Trajectory zone2Line;
    long[] stackPredictions = new long[3];
    private IMU imu;
    private SampleMecanumDrive drive;
    private Pusher pusher;
    private WobbleGoal wg;
    private Intake intake;
    private Outtake out;
    private DistanceSensor front;
    private DistanceSensor left;
    private final double P_COEF = 0.04;
    private double error;
    Pose2d startPosition = new Pose2d(-60, -48, Math.toRadians(180));
    RingAmount.Rings stack = RingAmount.Rings.ZERO;
    public void initialize(){
        imu = new IMU(hardwareMap);
        imu.initializeIMU();
        drive = new SampleMecanumDrive(hardwareMap);
        pusher = new Pusher(hardwareMap);
        wg = new WobbleGoal(hardwareMap);
        intake = new Intake(hardwareMap);
        out = new Outtake(hardwareMap);

        wg.grab();
        wg.lift();
        sleep(200);
        wg.stopElevator();
    }
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        createTrajs(stack);
        switch (stack){
            case ZERO:
                drive.followTrajectory(start2Zone);
                out.spin();
                drive.followTrajectory(zone2ShootingPt);
                drive.followTrajectory(shootingPt2Start);
                drive.followTrajectory(start2ZoneWG2);
                drive.followTrajectory(zone2Line);
                break;
            case ONE:
                drive.followTrajectory(start2Zone);
                out.spin();
                drive.followTrajectory(zone2ShootingPt);
                shoot3Rings();
                intake.spin();
                drive.followTrajectory(shootingPt2Intake2shootingPt);
                intake.stop();
                shoot1Ring();
                drive.followTrajectory(shootingPt2Start);
                drive.followTrajectory(start2ZoneWG2);
                drive.followTrajectory(zone2Line);
                break;
            case FOUR:
                drive.followTrajectory(start2Zone);
                out.spin();
                drive.followTrajectory(zone2ShootingPt);
                shoot3Rings();
                intake.spin();
                drive.followTrajectory(shootingPt2Intake2shootingPt);
                intake.stop();
                shoot3Rings();
                drive.followTrajectory(shootingPt2Start);
                drive.followTrajectory(start2ZoneWG2);
                drive.followTrajectory(zone2Line);
                break;
        }
    }
    public void createTrajs(RingAmount.Rings stack){
        Vector2d zone;
        Vector2d line;
        Vector2d shootingPt = new Vector2d(0,-36);
        Vector2d intakePt = new Vector2d(-24,-36);
        Vector2d starting1 = new Vector2d(-36,-14);
        Vector2d starting2 = new Vector2d(-60,-14);
        if(stack == RingAmount.Rings.ZERO){
            zone = new Vector2d(12,-50);
            line = new Vector2d(12,-40);
        }
        else if(stack == RingAmount.Rings.ONE){
            zone = new Vector2d(36,-36);
            line = new Vector2d(12,-36);
        }
        else{
            zone = new Vector2d(55,-60);
            line = new Vector2d(12,-60);
        }
        start2Zone = drive.trajectoryBuilder(startPosition)
                .splineToConstantHeading(zone,0)
                .build();
        zone2ShootingPt = drive.trajectoryBuilder(start2Zone.end())
                .splineToConstantHeading(shootingPt,0)
                .build();
        shootingPt2Intake2shootingPt = drive.trajectoryBuilder(zone2ShootingPt.end())
                .splineTo(intakePt,0)
                .splineTo(shootingPt,0)
                .build();
        shootingPt2Start = drive.trajectoryBuilder(shootingPt2Intake2shootingPt.end())
                .splineToConstantHeading(starting1,0)
                .splineToConstantHeading(starting2,0)
                .build();
        start2ZoneWG2 = drive.trajectoryBuilder(shootingPt2Start.end())
                .splineToConstantHeading(zone,0)
                .build();
        zone2Line = drive.trajectoryBuilder(start2Zone.end())
                .splineTo(line,0)
                .build();
    }
    public void shoot3Rings(){
        for(int i=0;i<3;i++) {
            pusher.push();
            sleep(250);
            pusher.pull();
            if(i!=2) {
                sleep(250);
            }
        }
    }
    public void shoot1Ring(){
        pusher.push();
        sleep(250);
    }
}
