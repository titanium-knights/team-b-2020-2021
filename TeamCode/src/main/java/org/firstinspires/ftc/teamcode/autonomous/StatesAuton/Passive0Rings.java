package org.firstinspires.ftc.teamcode.autonomous.StatesAuton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.vision.UGContourRingDetector;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utils.DualShooterNoPID;
import org.firstinspires.ftc.teamcode.utils.Outtake;
import org.firstinspires.ftc.teamcode.utils.RRQuickStart.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.WobbleGoal;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;
import java.util.Collections;

public class Passive0Rings extends LinearOpMode {
    //Computer Vision start
    //########################
    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution
    private static final int HORIZON = 100; // horizon value to tune
    private static final boolean DEBUG = true; // if debug is wanted, change to true
    private static final boolean USING_WEBCAM = true; // change to true if using webcam
    private static final String WEBCAM_NAME = "webcam"; // insert webcam name from configuration if using webcam
    private UGContourRingPipeline pipeline;
    private OpenCvCamera camera;
    boolean cameraSetup = false;
    ArrayList<Integer> stackReadings = new ArrayList<Integer>();
    int finalStackPrediction;
    //########################
    //Computer Vision end


    private WobbleGoal wg;
    private Pose2d startPose = new Pose2d(-60.0,-48.0,0.0);
    private Vector2d wgAVector = new Vector2d(0,-60);
    private Vector2d backup = new Vector2d(-12,-60);
    private Vector2d finish = new Vector2d(12,-36);
    private SampleMecanumDrive drive;
    private Trajectory startToWGA;
    private Trajectory WGAbackup;
    private Trajectory wgAToFinish;
    private DualShooterNoPID outtake;
    private UGContourRingDetector detector;
    @Override
    public void runOpMode(){
        initialize();
        wg.grab();
        sleep(250);
        wg.lift();
        sleep(250);
        wg.stop();
        waitForStart();
        int finalStackPrediction = finalHeightPrediction();
        drive.followTrajectory(startToWGA);
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
        drive.followTrajectory(wgAToFinish);


    }
    public void initialize(){
        wg = new WobbleGoal(hardwareMap);
        outtake = new DualShooterNoPID(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        computerVision();
        createTrajectories();
        while(opModeIsActive() && !isStopRequested()&&!isStarted()){
            stackReadings.add(getStack());
        }
    }
    public void computerVision(){
        int cameraMonitorViewId = this
                .hardwareMap
                .appContext
                .getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );
        if (USING_WEBCAM) {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createWebcam(hardwareMap.get(WebcamName.class, WEBCAM_NAME), cameraMonitorViewId);
        } else {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        }

        camera.setPipeline(pipeline = new UGContourRingPipeline(telemetry, DEBUG));

        UGContourRingPipeline.Config.setCAMERA_WIDTH(CAMERA_WIDTH);

        UGContourRingPipeline.Config.setHORIZON(HORIZON);

        camera.openCameraDeviceAsync(() -> camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT));
        cameraSetup = true;
    }
    public int getStack(){
        if(!cameraSetup){
            telemetry.addData("ERROR:", "TRIED TO ACCESS CAMERA B4 INITIALIZATION");
            return -1;
        }
        else{
            switch (pipeline.getHeight()){
                case ZERO:
                    return 0;
                case ONE:
                    return 1;
                case FOUR:
                    return 4;
            }
            return -1;
        }
    }
    public int finalHeightPrediction(){
        if(stackReadings.size()==0){
            return getStack();
        }
        else{
            return Collections.max(stackReadings);
        }
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
