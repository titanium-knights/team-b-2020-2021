package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.vision.UGContourRingDetector;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class RingScanner{
    private static final int CAMERA_WIDTH = 1920; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 1080; // height of wanted camera resolution

    private static final int HORIZON = 100; // horizon value to tune

    private static final boolean DEBUG = false; // if debug is wanted, change to true

    private static final boolean USING_WEBCAM = false; // change to true if using webcam
    private static final String WEBCAM_NAME = CONFIG.WEBCAM; // insert webcam name from configuration if using webcam

    private UGContourRingPipeline pipeline;
    private OpenCvCamera camera;

    public RingScanner(HardwareMap hardwareMap, Telemetry tm){
        Telemetry telemetry = new MultipleTelemetry(tm, FtcDashboard.getInstance().getTelemetry());
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory
                .getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, WEBCAM_NAME), cameraMonitorViewId);
        camera.setPipeline(pipeline = new UGContourRingPipeline(telemetry, DEBUG));
        UGContourRingPipeline.Config.setCAMERA_WIDTH(CAMERA_WIDTH);

        UGContourRingPipeline.Config.setHORIZON(HORIZON);

        camera.openCameraDeviceAsync(() -> camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT));
        FtcDashboard.getInstance().startCameraStream(camera,30);
    }

    public int getHeight(){
        if(pipeline.getHeight()== UGContourRingPipeline.Height.ZERO){
            return 0;
        }
        else if(pipeline.getHeight()== UGContourRingPipeline.Height.ONE){
            return 1;
        }
        else{
            return 4;
        }
    }
}