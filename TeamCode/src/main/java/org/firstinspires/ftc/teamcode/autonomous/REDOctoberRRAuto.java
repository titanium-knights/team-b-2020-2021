package org.firstinspires.ftc.teamcode.autonomous;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.CONFIG;
import org.firstinspires.ftc.teamcode.utils.IMU;
import org.firstinspires.ftc.teamcode.utils.Intake;
import org.firstinspires.ftc.teamcode.utils.LaunchMath;
import org.firstinspires.ftc.teamcode.utils.Outtake;
import org.firstinspires.ftc.teamcode.utils.Pusher;
import org.firstinspires.ftc.teamcode.utils.RRQuickStart.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.RingAmount;
import org.firstinspires.ftc.teamcode.utils.RingDetector;
import org.firstinspires.ftc.teamcode.utils.WobbleGoal;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
@Autonomous(name = "Red Auto", group="Production")
public class REDOctoberRRAuto extends LinearOpMode {
    private SampleMecanumDrive drive;
    private Intake intake;
    private Outtake outtake;
    private LaunchMath lm;
    private IMU imu;
    private DistanceSensor back;
    private DistanceSensor right;
    private DistanceSensor left;
    private OpenCvCamera phoneCam;
    private RingDetector detector;
    private WobbleGoal wg;
    private RingAmount.Rings state;
    private Pose2d startPose;
    private Pusher pusher;
    private Vector2d leftVector = new Vector2d(-12,-6);
    private Vector2d centerVector = new Vector2d(-12,-12);
    private Vector2d rightVector = new Vector2d(-12,-18);
    private Vector2d zone;
    Trajectory startToPowerShotRight;
    Trajectory powerShotRightToCenter;
    Trajectory powerShotCenterToLeft;
    Trajectory psLeftToWgZone;
    Trajectory wGZoneToWg2;
    Trajectory wg2ToZone;
    Trajectory zone2Line;
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        state = detector.getState();
        createTrajs(state);
        sleep(1000);
        wg.grab();
        wg.lift();
        waitForStart();
        sleep(500);
        wg.stop();
        while(opModeIsActive()){
            drive.followTrajectory(startToPowerShotRight);
            outtake.setFlywheelSpeed(lm.getLinearVelocity());
            shootRing();
            drive.followTrajectory(powerShotRightToCenter);
            outtake.setFlywheelSpeed(lm.getLinearVelocity());
            shootRing();
            drive.followTrajectory(powerShotCenterToLeft);
            shootRing();
            drive.followTrajectory(psLeftToWgZone);
            wg.release();
            drive.followTrajectory(wGZoneToWg2);
            wg.grab();
            wg.lift();
            drive.followTrajectory(wg2ToZone);
            wg.release();
            drive.followTrajectory(zone2Line);
            break;
        }
    }
    public void shootRing(){
        pusher.push();
        sleep(500);
        pusher.pull();
    }
    public void initialize(){
        drive = new SampleMecanumDrive(hardwareMap);
        lm = new LaunchMath(60);
        intake = new Intake(hardwareMap);
        pusher = new Pusher(hardwareMap);
        outtake = new Outtake(hardwareMap);
        wg = new WobbleGoal(hardwareMap);
        back = hardwareMap.get(DistanceSensor.class, CONFIG.BACKDIST);
        right = hardwareMap.get(DistanceSensor.class,CONFIG.RIGHTDIST);
        imu = new IMU(hardwareMap);
        telemetry.addLine("Getting Distance Sensor Readouts in 1 sec");
        telemetry.update();
        sleep(1000);
        double rightD= right.getDistance(DistanceUnit.INCH);
        double backD = back.getDistance(DistanceUnit.INCH);
        telemetry.addData("Right Readout",rightD);
        telemetry.addData("Back Readout",backD);
        telemetry.update();
        startPose = new Pose2d(-72+backD+CONFIG.centerOfRobit2BackDistance,-72+rightD+CONFIG.centerOfRobit2LeftDistance,Math.toRadians(-90.0));
        drive.setPoseEstimate(startPose);
        telemetry.addLine("Starting camera init");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        detector = new RingDetector(telemetry);
        phoneCam.setPipeline(detector);
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });
        FtcDashboard.getInstance().startCameraStream(phoneCam,15);
        telemetry.addLine("Finished Initialization");
        telemetry.update();
    }
    public void createTrajs(RingAmount.Rings state){
        startToPowerShotRight = drive.trajectoryBuilder(startPose)
                .splineTo(rightVector, Math.toRadians(-90.0))
                .build();
        powerShotRightToCenter = drive.trajectoryBuilder(startToPowerShotRight.end())
                .splineTo(centerVector, Math.toRadians(-90.0))
                .build();
        powerShotCenterToLeft = drive.trajectoryBuilder(powerShotRightToCenter.end())
                .splineTo(leftVector, Math.toRadians(-90.0))
                .build();
        switch(state){
            case ZERO:
                //position A
                zone = new Vector2d(0,-50);
                psLeftToWgZone = drive.trajectoryBuilder(powerShotCenterToLeft.end())
                        .splineTo(zone, Math.toRadians(180.0))
                        .build();
                break;
            case ONE:
                //position
                zone = new Vector2d(33,-36);
                psLeftToWgZone = drive.trajectoryBuilder(powerShotCenterToLeft.end())
                        .splineTo(zone, Math.toRadians(180.0))
                        .build();
                break;
            case FOUR:
                //position C
                zone = new Vector2d(50,-50);
                psLeftToWgZone = drive.trajectoryBuilder(powerShotCenterToLeft.end())
                        .splineTo(zone, Math.toRadians(180.0))
                        .build();
                break;
            default:
                zone = new Vector2d(0,-50);
                psLeftToWgZone = drive.trajectoryBuilder(powerShotCenterToLeft.end())
                        .splineTo(zone, Math.toRadians(180.0))
                        .build();
                break;
        }
        wGZoneToWg2 = drive.trajectoryBuilder(psLeftToWgZone.end())
                .splineTo(new Vector2d(-55,-55), Math.toRadians(-90.0))
                .build();
        wg2ToZone  = drive.trajectoryBuilder(wGZoneToWg2.end())
                .splineTo(zone,Math.toRadians(180.0))
                .build();
        zone2Line = drive.trajectoryBuilder(wg2ToZone.end())
                .splineTo(new Vector2d(12,-24),Math.toRadians(90))
                .build();
    }
}
