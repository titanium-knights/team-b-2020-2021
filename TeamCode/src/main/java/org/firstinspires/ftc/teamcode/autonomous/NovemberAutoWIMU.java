package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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

@Autonomous (name = "November Auto IMU")
public class NovemberAutoWIMU extends LinearOpMode {
    MecDrive drive;
    DistanceSensor front;
    DistanceSensor left;
    //DistanceSensor right;
    BNO055IMU imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;
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
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
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
        shootThreePowerShots();
        //driveToZero();
        releaseWG();

        //parkOnLineFromZero();
    }
    public void doOneRingAuto(){
        driveForwards();
        shootThreePowerShots();
        driveToOne();
        releaseWG();
        parkOnLine();
    }
    public void doFourRingsAuto(){
        driveForwards();
        shootThreePowerShots();
        //driveToFour();
        releaseWG();
        parkOnLine();
    }
    public void driveForwards(){
        while(getD(front)<60){
            correction = checkDirection();
            if(getD(front)<20){
                power = -.5;
                drive.setPowerToLeftDrive(power-correction);
                drive.setPowerToRightDrive(power+correction);
            }
            else if(getD(front)<40){
                power = -.5;
                drive.setPowerToLeftDrive(power-correction);
                drive.setPowerToRightDrive(power+correction);
            }
            else{
                power = -.4;
                drive.forwardWithPower(-.4);
            }
        }
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
    }

    /* public void driveToZero(){
         drive.gyroTurn(.5,180);
         while(getD(right)>18){
             if(getD(right)>30){
                 drive.strafeRightWithPower(.8);
             }
             else if(getD(right)>24){
                 drive.strafeRightWithPower(.4);
             }
         }
         drive.forwardWithPower(0.5);
         sleep(500);
         drive.stop();
     }*/
    public void driveToOne(){
        drive.gyroTurn(.5,180);
        while(getD(front)>36){
            drive.forwardWithPower(0.5);
        }
        drive.stop();
    }
    /*public void driveToFour(){
        drive.gyroTurn(.5,180);
        while(getD(right)>18){
            if(getD(right)>30){
                drive.strafeRightWithPower(.8);
            }
            else if(getD(right)>24){
                drive.strafeRightWithPower(.4);
            }
        }
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
    }*/
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
    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
    private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }
    private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        drive.setPowerToLeftDrive(leftPower);
        drive.setPowerToRightDrive(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        drive.stop();
        // reset angle tracking on new heading.
        resetAngle();
    }
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

}
