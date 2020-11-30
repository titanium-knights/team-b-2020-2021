package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.CONFIG;
import org.firstinspires.ftc.teamcode.utils.DriveEncoder;
import org.firstinspires.ftc.teamcode.utils.IMU;
import org.firstinspires.ftc.teamcode.utils.Intake;
import org.firstinspires.ftc.teamcode.utils.MecDrive2;
import org.firstinspires.ftc.teamcode.utils.Outtake;
import org.firstinspires.ftc.teamcode.utils.Pusher;
import org.firstinspires.ftc.teamcode.utils.RingAmount;
import org.firstinspires.ftc.teamcode.utils.WobbleGoal;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.Arrays;

@Autonomous(name = "December Auto")
public class DecemberJank2WG extends LinearOpMode {
    //BEGIN CV STUFF
    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution

    private static final int HORIZON = 100; // horizon value to tune

    private static final boolean DEBUG = false; // if debug is wanted, change to true
    private static final boolean USING_WEBCAM = false; // change to true if using webcam
    private static final String WEBCAM_NAME = ""; // insert webcam name from configuration if using webcam

    private UGContourRingPipeline pipeline;
    private OpenCvCamera camera;

    //END CV STUFF
    long[] stackPredictions = new long[3];
    private IMU imu;
    private MecDrive2 drive;
    private Pusher pusher;
    private WobbleGoal wg;
    private Intake intake;
    private Outtake out;
    private DriveEncoder encoders;
    private DistanceSensor front;
    private DistanceSensor left;
    private final double P_COEF = 0.04;
    private double error;
    RingAmount.Rings stack = RingAmount.Rings.ZERO;

    public void initialize(){
        imu = new IMU(hardwareMap);
        imu.initializeIMU();
        drive = new MecDrive2(hardwareMap);
        encoders = new DriveEncoder(drive,telemetry);
        pusher = new Pusher(hardwareMap);
        wg = new WobbleGoal(hardwareMap);
        intake = new Intake(hardwareMap);
        out = new Outtake(hardwareMap);
        left = hardwareMap.get(DistanceSensor.class, CONFIG.LEFTDIST);
        front = hardwareMap.get(DistanceSensor.class, CONFIG.FRONTDIST);
        wg.grab();
        wg.lift();
        sleep(200);
        wg.stopElevator();
        CVDetection();

    }
    public void initloop(){
        String height = "[HEIGHT]" + " " + pipeline.getHeight();
        telemetry.addData("[Ring Stack] >>", height);
        UGContourRingPipeline.Height heightA = pipeline.getHeight();
        if(heightA == UGContourRingPipeline.Height.ZERO) {
            stackPredictions[0] += 1;
        }
        else if(heightA == UGContourRingPipeline.Height.ONE) {
            stackPredictions[1] += 1;
        }
        else{
            stackPredictions[2]+=1;
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        telemetry.addLine("Scanning Rings");
        telemetry.update();

        while(!isStarted() && !isStopRequested()){
            initloop();
        }
        waitForStart();
        determineStackHeight();
        switch (stack){
            case ZERO:
                driveToShooting();
                shoot3Rings();
                break;
            case ONE:
                break;
            case FOUR:
                driveToShooting();
                shoot3Rings();
                intakeRings();
                driveToShooting();
                shoot3Rings();
                goToWGPos4();
                break;
        }

    }
    public void driveToShooting(){
        error = 60-front.getDistance(DistanceUnit.INCH);
        while(Math.abs(error)>2){
            drive.backwardWithPower(P_COEF*error);
        }
        drive.stop();
        out.spin();
        error = 24-left.getDistance(DistanceUnit.INCH);
        while(Math.abs(error)>2){
            drive.strafeRightWithPower(P_COEF*error);
        }
        drive.stop();

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
    public void intakeRings(){
        intake.spin();
        drive.forwardWithPower(.5);
        sleep(500);
        drive.stop();

    }
    public void goToWGPos4(){
        encoders.encoderDrive(0.5,-58,-58,30);
        drive.stop();
    }
    public void CVDetection(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory
                .getInstance()
                .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        camera.setPipeline(pipeline = new UGContourRingPipeline(telemetry, DEBUG));
        UGContourRingPipeline.Config.setCAMERA_WIDTH(CAMERA_WIDTH);

        UGContourRingPipeline.Config.setHORIZON(HORIZON);

        camera.openCameraDeviceAsync(() -> camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT));

    }
    public void determineStackHeight(){
        long max = Arrays.stream(stackPredictions).max().getAsLong();
        int index = findIndex(stackPredictions,max);
        if(index==0){
            stack= RingAmount.Rings.ZERO;
        }
        else if(index==1){
            stack= RingAmount.Rings.ONE;
        }
        else{
            stack= RingAmount.Rings.FOUR;
        }
    }
    public static int findIndex(long[] arr, long t)
    {

        int index = Arrays.binarySearch(arr, t);
        return (index < 0) ? -1 : index;
    }
}
