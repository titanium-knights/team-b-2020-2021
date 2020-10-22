package org.firstinspires.ftc.teamcode.autonomous;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.CONFIG;
import org.firstinspires.ftc.teamcode.utils.IMU;
import org.firstinspires.ftc.teamcode.utils.Intake;
import org.firstinspires.ftc.teamcode.utils.LaunchMath;
import org.firstinspires.ftc.teamcode.utils.Outtake;
import org.firstinspires.ftc.teamcode.utils.RRQuickStart.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.RingAmount;
import org.firstinspires.ftc.teamcode.utils.RingDetector;
import org.firstinspires.ftc.teamcode.utils.WobbleGoal;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class REDOctoberRRAuto extends LinearOpMode {
    SampleMecanumDrive drive;
    Intake intake;
    Outtake outtake;
    LaunchMath lm;
    IMU imu;
    DistanceSensor back;
    DistanceSensor right;
    DistanceSensor left;
    OpenCvCamera phoneCam;
    RingDetector detector;
    WobbleGoal wg;
    RingAmount.Rings state;
    Pose2d startPose;
    Pose2d rightPS = new Pose2d();
    Pose2d leftPS;
    Pose2d centerPS;
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        Trajectory startToPowerShot = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(0,-24), Math.toRadians(-90.0))
                .build();
        Trajectory powerShotToA = drive.trajectoryBuilder(startToPowerShot.end())
                .splineTo(new Vector2d(0,-50), Math.toRadians(180.0))
                .build();
        Trajectory powerShotToB = drive.trajectoryBuilder(startToPowerShot.end())
                .splineTo(new Vector2d(33,-36), Math.toRadians(180.0))
                .build();
        Trajectory powerShotToC = drive.trajectoryBuilder(startToPowerShot.end())
                .splineTo(new Vector2d(50,-50), Math.toRadians(180.0))
                .build();
        Pose2d endOfABorC;
        state = detector.getState();
        waitForStart();
        wg.grab();
        wg.lift();
        sleep(500);
        wg.stop();
        while(opModeIsActive()){
            drive.followTrajectory(startToPowerShot);
            outtake.setFlywheelSpeed(lm.getLinearVelocity());
            switch(state){
                case ZERO:
                    //position A
                    drive.followTrajectory(powerShotToA);
                    endOfABorC = powerShotToA.end();
                    break;
                case ONE:
                    //position B
                    drive.followTrajectory(powerShotToB);
                    endOfABorC = powerShotToB.end();
                    break;
                case FOUR:
                    //position C
                    drive.followTrajectory(powerShotToC);
                    endOfABorC = powerShotToC.end();
                    break;
            }
            wg.release();
            Trajectory parkOnLine = drive.trajectoryBuilder(startToPowerShot.end())
                    .splineTo(new Vector2d(0,-24), Math.toRadians(180.0))
                    .build();
            drive.followTrajectory(parkOnLine);
            break;
        }
    }
    public void initialize(){
        drive = new SampleMecanumDrive(hardwareMap);
        lm = new LaunchMath(60);
        intake = new Intake(hardwareMap);
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
        startPose = new Pose2d(-72+backD,-72+rightD,Math.toRadians(-90.0));
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

}
