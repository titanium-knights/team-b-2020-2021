package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.ComputerVisionRectDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
@Disabled

@TeleOp
public class RectangleComputerVisionTester extends OpMode {
    OpenCvCamera camera;
    ComputerVisionRectDetector detector;
    @Override
    public void init() {
        int cameraMonitorViewID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id",hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK,cameraMonitorViewID);
        detector = new ComputerVisionRectDetector();

        camera.setPipeline(detector);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT );
            }
        });

        FtcDashboard.getInstance().startCameraStream(camera,15);


    }

    @Override
    public void loop() {
        telemetry.addData("Stack: ",detector.getStack());
        telemetry.update();
    }
}
