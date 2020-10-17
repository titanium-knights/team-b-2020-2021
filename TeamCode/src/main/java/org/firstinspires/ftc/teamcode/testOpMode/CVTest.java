package org.firstinspires.ftc.teamcode.testOpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.RingAmount;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

/*
 * This version of the internal camera example uses EasyOpenCV's interface to the
 * original Android camera API
@TeleOp
 */
public class CVTest extends LinearOpMode {
    OpenCvCamera phoneCam;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        RingPipeline pipe = new RingPipeline();
        // OR...  Do Not Activate the Camera Monitor View
        //phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        phoneCam.setPipeline(pipe);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();


        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Frame Count", phoneCam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", phoneCam.getFps()));
            telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
            telemetry.addData("Amount of Rings", pipe.rings.toString());
            telemetry.update();

            if(gamepad1.a) {
                phoneCam.stopStreaming();
                //phoneCam.closeCameraDevice();
            }
            sleep(100);
        }
    }
    class RingPipeline extends OpenCvPipeline {
        public RingAmount.Rings rings = RingAmount.Rings.ZERO;

        boolean viewportPaused = false;
        private Mat image = new Mat();
        private Mat subArr = new Mat();
        @Override
        public Mat processFrame(Mat input) {

            input.copyTo(image);
            if (image.empty()) {
                return input;
            }
            Imgproc.cvtColor(image, image, Imgproc.COLOR_BGR2YCrCb);
            Imgproc.rectangle(
                    input,
                    new Point(1675, 1420),
                    new Point(2335, 1775),
                    new Scalar(255, 0, 0),
                    2
            );
            Imgproc.blur(image, image, new Size(5, 5));
            Imgproc.threshold(image, image, 100, 255, Imgproc.THRESH_BINARY);
            subArr = image.submat(1420, 1775, 1950, 2060);

            double summedSubArea = Core.sumElems(subArr).val[2];
            if (summedSubArea > 9000000) {
                rings = RingAmount.Rings.ZERO;
            }
            else if (summedSubArea > 650000) {
                rings = RingAmount.Rings.ONE;
            }
            else {
                rings = RingAmount.Rings.FOUR;
            }
            return input;
        }


        @Override
        public void onViewportTapped() {
            viewportPaused = !viewportPaused;
            if(viewportPaused) {
                phoneCam.pauseViewport();
            }
            else {
                phoneCam.resumeViewport();
            }
        }
    }
}