package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.Contours;
import org.firstinspires.ftc.teamcode.utils.MecDrive2;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
@Disabled
@Autonomous(name = "GoToRing Test")
@Config
public class GoToRing extends LinearOpMode {

    public static final double kP = 1/10.0;
    public static final double kI = 1/10.0;
    public static final double kD = 1/10.0;

    OpenCvCamera phoneCam;
    Contours d;
    MecDrive2 drive;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecDrive2(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        d = new Contours(telemetry);
        phoneCam.setPipeline(d);
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });
        //FtcDashboard.getInstance().startCameraStream(phoneCam,15);
        telemetry.addLine("Waiting for start");
        telemetry.update();


        waitForStart();
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive()) {
            double sumError = d.getError();
            double previousError = 0;
            double previousTimeStamp = 0;
            double currentTimeStamp;
            timer.reset();
            while(Math.abs(d.getError())<5) {
                //Positive error --> robot needs to turn cw
                //Negative error --> robot needs to turn ccw
                double error = d.getError();
                //Math for proportional
                double P = kP * error;

                //Math for Integral
                sumError+=error;
                double I = kI * sumError;

                //Math for derivative
                currentTimeStamp = timer.milliseconds();
                double deltaTime = currentTimeStamp-previousTimeStamp;
                double deltaError = error - previousError;
                double instantaneous = deltaError/deltaTime;
                double D = kD*instantaneous;

                drive.turnRightWithPower(P+I+D);

                //Resetting for next iter
                previousError=error;
                previousTimeStamp = currentTimeStamp;

            }
        }
        phoneCam.stopStreaming();
    }
}


