package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.utils.CONFIG;
import org.firstinspires.ftc.teamcode.utils.Contours;
import org.firstinspires.ftc.teamcode.utils.DriveEncoder;
import org.firstinspires.ftc.teamcode.utils.IMU;
import org.firstinspires.ftc.teamcode.utils.Intake;
import org.firstinspires.ftc.teamcode.utils.MecDrive2;
import org.firstinspires.ftc.teamcode.utils.Outtake;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;
import org.firstinspires.ftc.teamcode.utils.Pusher;
import org.firstinspires.ftc.teamcode.utils.RRQuickStart.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.RingAmount;
import org.firstinspires.ftc.teamcode.utils.Vector2D;
import org.firstinspires.ftc.teamcode.utils.WobbleGoal;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class JanuaryRRAuto extends LinearOpMode {
    private OpenCvCamera phoneCam;
    private Contours detector;
    private Trajectory start2Zone;
    private Trajectory zone2ShootingPt;
    private Trajectory shootingPt2Intake2shootingPt;
    private Trajectory shootingPt2Start;
    private Trajectory start2ZoneWG2;
    private Trajectory zone2Line;
    long[] stackPredictions = {0L,0L,0L};
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
    public void initializeCV(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        detector = new Contours(telemetry);
        phoneCam.setPipeline(detector);
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });
        //FtcDashboard.getInstance().startCameraStream(phoneCam,15);
        telemetry.addLine("Initialized CV");
    }
    public void initialize(){
        imu = new IMU(hardwareMap);
        imu.initializeIMU();
        drive = new SampleMecanumDrive(hardwareMap);
        pusher = new Pusher(hardwareMap);
        wg = new WobbleGoal(hardwareMap);
        intake = new Intake(hardwareMap);
        out = new Outtake(hardwareMap);
        initializeCV();
        wg.grab();
        wg.lift();
        sleep(200);
        wg.stopElevator();
        initLoop();
    }
    public void initLoop(){
        while(!isStarted() && !isStopRequested()){
            RingAmount.Rings predictionOnIteration = detector.getStack();
            switch(predictionOnIteration){
                case ZERO:
                    stackPredictions[0]+=1;
                    break;
                case ONE:
                    stackPredictions[1]+=1;
                    break;
                case FOUR:
                    stackPredictions[2]+=1;
                    break;
            }
        }
    }
    public void determineStack(){
        long max = -1;
        int maxIndex = 0;
        stack = RingAmount.Rings.ZERO;
        for(int i =0; i<3;i++){
            if(stackPredictions[i]>max){
                max = stackPredictions[i];
                maxIndex = i;
            }
        }
        if(maxIndex==2){
            stack = RingAmount.Rings.FOUR;
        }
        else if(maxIndex==1){
            stack = RingAmount.Rings.ONE;
        }
        else{
            stack = RingAmount.Rings.ZERO;
        }
    }
    public void setPoseEstimate(){
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        determineStack();
        createTrajs(stack);
        setPoseEstimate();
        switch (stack){
            case ZERO:
                drive.followTrajectory(start2Zone);
                setPoseEstimate();
                out.spin();
                setPoseEstimate();
                drive.followTrajectory(zone2ShootingPt);
                setPoseEstimate();
                shoot3Rings();
                setPoseEstimate();
                drive.followTrajectory(shootingPt2Start);
                setPoseEstimate();
                drive.followTrajectory(start2ZoneWG2);
                setPoseEstimate();
                drive.followTrajectory(zone2Line);
                setPoseEstimate();
                break;
            case ONE:
                drive.followTrajectory(start2Zone);
                setPoseEstimate();
                out.spin();
                setPoseEstimate();
                drive.followTrajectory(zone2ShootingPt);
                setPoseEstimate();
                shoot3Rings();
                setPoseEstimate();
                intake.spin();
                setPoseEstimate();
                drive.followTrajectory(shootingPt2Intake2shootingPt);
                setPoseEstimate();
                intake.stop();
                setPoseEstimate();
                shoot1Ring();
                setPoseEstimate();
                drive.followTrajectory(shootingPt2Start);
                setPoseEstimate();
                drive.followTrajectory(start2ZoneWG2);
                setPoseEstimate();
                drive.followTrajectory(zone2Line);
                setPoseEstimate();
                break;
            case FOUR:
                drive.followTrajectory(start2Zone);
                setPoseEstimate();
                out.spin();
                setPoseEstimate();
                drive.followTrajectory(zone2ShootingPt);
                setPoseEstimate();
                shoot3Rings();
                setPoseEstimate();
                intake.spin();
                setPoseEstimate();
                drive.followTrajectory(shootingPt2Intake2shootingPt);
                setPoseEstimate();
                intake.stop();
                setPoseEstimate();
                shoot3Rings();
                setPoseEstimate();
                drive.followTrajectory(shootingPt2Start);
                setPoseEstimate();
                drive.followTrajectory(start2ZoneWG2);
                setPoseEstimate();
                drive.followTrajectory(zone2Line);
                setPoseEstimate();
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
