package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.CONFIG;
import org.firstinspires.ftc.teamcode.utils.IMU;
import org.firstinspires.ftc.teamcode.utils.MecDrive;
import org.firstinspires.ftc.teamcode.utils.Outtake;
import org.firstinspires.ftc.teamcode.utils.RingAmount;
import org.firstinspires.ftc.teamcode.utils.RingDetector;
import org.firstinspires.ftc.teamcode.utils.WobbleGoal;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
@Disabled
@Autonomous (name = "November Auto")
public class NovemberAuto extends LinearOpMode {
    MecDrive drive;
    DistanceSensor front;
    DistanceSensor left;
    //DistanceSensor right;
    IMU imu;
    WobbleGoal wg;
    Outtake out;
    private OpenCvCamera phoneCam;
    private RingDetector detector;
    private RingAmount.Rings state;
    public void initialize() {
        front = hardwareMap.get(DistanceSensor.class, CONFIG.FRONTDIST);
        left = hardwareMap.get(DistanceSensor.class, CONFIG.LEFTDIST);
        out = new Outtake(hardwareMap);
        //right = hardwareMap.get(DistanceSensor.class, CONFIG.RIGHTDIST);
        imu = new IMU(hardwareMap);
        drive = new MecDrive(hardwareMap,false);
        wg = new WobbleGoal(hardwareMap);

        wg.grab();
        wg.lift();
        sleep(200);
        wg.stopElevator();

        /*telemetry.addLine("Starting camera init");
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
        state = detector.getState();*/
        state = RingAmount.Rings.ZERO;
        telemetry.addData("State:",state);
        telemetry.addLine("Finished Initialization");
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        out.spin();
        switch (state){
            case ZERO:
                doZeroRingsAuto();
                break;
            case ONE:
                doOneRingAuto();
                break;
            case FOUR:
                doFourRingsAuto();
                break;
        }
        drive.stop();
    }

    public void doZeroRingsAuto(){
        driveForwards();
        shootThreeRings();
        driveToZero();
        releaseWG();
        sleep(1000);
        driveBackToStart();
        driveBackToOne();
        releaseWG();
       // parkOnLine();
        drive.forwardWithPower(.5);
        sleep(500);
        drive.stop();

        //parkOnLineFromZero();
    }
    public void doOneRingAuto(){
        driveForwards();
        shootThreeRings();
        driveToOne();
        releaseWG();
        sleep(1000);

        parkOnLine();
    }
    public void doFourRingsAuto(){
        driveForwards();
        shootThreeRings();
        driveToFour();
        releaseWG();
        parkOnLine();
    }
    public void driveForwards(){
        while(getD(front)<60){
            if(getD(front)<20){
                drive.forwardWithPower(-.5);
            }
            else if(getD(front)<40){
                drive.forwardWithPower(-.5);
            }
            else{
                drive.forwardWithPower(-.4);
            }
        }
        drive.stop();

    }
    public void driveBackToOne(){
        drive.forwardWithPower(-.5);
        sleep(2500);
        drive.stop();
    }
    public void shootThreePowerShots(){
        while(getD(left)<45){
            if(getD(left)<15){
                drive.strafeRightWithPower(1);
            }
            else if(getD(left)<30){
                drive.strafeRightWithPower(.5);
            }
            else{
                drive.strafeRightWithPower(.5);
            }
        }
        drive.stop();
        shoot();
        while(getD(left)<52.5){
            drive.strafeRightWithPower(.5);
        }
        drive.stop();
        shoot();
        while(getD(left)<60){
            drive.strafeRightWithPower(.5);
        }
        drive.stop();
        shoot();
        out.stop();
        sleep(1000);
    }
   public void driveBackToStart(){
        drive.forwardWithPower(0.3);
        sleep(1000);
        while(getD(front)>10){
            drive.forwardWithPower(0.5);
        }
        drive.stop();
        wg.lower();
        wg.release();
        sleep(500);
        wg.stopElevator();
        while(getD(left)<26){
            drive.strafeRightWithPower(0.5);
        }
        drive.stop();
        wg.grab();
        sleep(250);
        drive.stop();
   }
   public void driveToZero(){
        //drive.gyroTurn(.5,180);
        /*while(getD(right)>18){
            if(getD(right)>30){
                drive.strafeRightWithPower(.8);
            }
            else if(getD(right)>24){
                drive.strafeRightWithPower(.4);
            }
        }*/
        while(getD(left)>5){
            drive.strafeRightWithPower(-0.5);
       }
        drive.stop();
        drive.forwardWithPower(-0.5);
        sleep(500);
        drive.stop();
    }
    public void driveToOne(){
        //Implement: Turn 180 degrees at .5 power
        while(getD(front)>36){
            drive.forwardWithPower(0.5);
        }
        drive.stop();
    }
    public void driveToFour(){
        //Implement: Turn 180 degrees at .5 power
        /*while(getD(right)>18){
            if(getD(right)>30){
                drive.strafeRightWithPower(.8);
            }
            else if(getD(right)>24){
                drive.strafeRightWithPower(.4);
            }
        }*/
        drive.strafeRightWithPower(0.5);
        sleep(1000);
        while(getD(front)>10){
            if(getD(front)>40){
                drive.forwardWithPower(1);
            }
            else if (getD(front)>25){
                drive.forwardWithPower(0.5);
            }
            else{
                drive.forwardWithPower(0.3);
            }
        }
        drive.stop();
    }
    public void parkOnLineFromZero(){
        drive.strafeLeftWithPower(0.5);
        sleep(500);
        drive.stop();
        drive.backwardWithPower(0.5);
        sleep(500);
        drive.stop();
    }




    public double getD(DistanceSensor sensor){
        return sensor.getDistance(DistanceUnit.INCH);
    }
    public void shoot(){
        out.push();
        sleep(200);
        out.pull();
    }
    public void shootThreeRings(){
        shoot();
        shoot();
        shoot();
    }
    public void releaseWG(){
        wg.release();
    }
    public void parkOnLine(){
        drive.strafeLeftWithPower(0.5);
        sleep(500);
        drive.stop();
        while(getD(front)<48){
            drive.backwardWithPower(0.5);
        }
        drive.stop();
    }
}
