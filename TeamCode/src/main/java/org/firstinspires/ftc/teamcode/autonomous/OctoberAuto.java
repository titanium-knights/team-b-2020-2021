package org.firstinspires.ftc.teamcode.autonomous;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.CONFIG;
import org.firstinspires.ftc.teamcode.utils.IMU;
import org.firstinspires.ftc.teamcode.utils.LaunchMath;
import org.firstinspires.ftc.teamcode.utils.MecDrive;
import org.firstinspires.ftc.teamcode.utils.MecDriveWLib;
import org.firstinspires.ftc.teamcode.utils.Outtake;
import org.firstinspires.ftc.teamcode.utils.RingAmount;
import org.firstinspires.ftc.teamcode.utils.RingDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class OctoberAuto extends LinearOpMode {
    MecDrive drive;
    Outtake outtake;
    DistanceSensor back;
    IMU imu;
    LaunchMath calc;
    DistanceSensor left;
    DistanceSensor front;
    OpenCvCamera phoneCam;
    RingDetector detector;
    RingAmount.Rings state;
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        driveToPowerShotZone();
        hitPowerShots();


    }
    public void hitPowerShots(){
        drive.turnRightXDegrees(imu,180);
        outtake.setFlywheelSpeed(calc.getLinearVelocity());
        //Once it is at that speed
        //outtake.pushRing();
        //while(left.getDistance(DistanceUnit.INCH)<)
    }
    public void driveToPowerShotZone(){
        while(left.getDistance(DistanceUnit.INCH)<48){
            drive.strafeRightWithPower(1);
        }
        while(left.getDistance(DistanceUnit.INCH)>48){
            drive.strafeLeftWithPower(.25);
        }
        while(back.getDistance(DistanceUnit.INCH)<48){
            drive.forwardWithPower(1);
        }
        while(back.getDistance(DistanceUnit.INCH)>48){
            drive.backwardWithPower(.25);
        }
        drive.stop();
    }

    public void initialize(){
        imu = new IMU(hardwareMap);
        calc = new LaunchMath(48);
        drive = new MecDrive(hardwareMap,false);
        outtake = new Outtake(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
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
        back = hardwareMap.get(DistanceSensor.class, CONFIG.BACKDIST);
        front = hardwareMap.get(DistanceSensor.class, CONFIG.FRONTDIST);
        left = hardwareMap.get(DistanceSensor.class, CONFIG.LEFTDIST);
        state = detector.getState();
        telemetry.addLine("Ready to Rumble");
        telemetry.update();

    }
}
