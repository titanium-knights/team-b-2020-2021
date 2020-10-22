package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class RingDetector extends OpenCvPipeline {
    private Telemetry telemetry;
    private int i=0;
    private double summedSubArea;
    Mat image = new Mat();
    private static int widthMultiplier = 320;
    private static int heightMultiplier = 240;
    static final Rect ROI = new Rect(new Point((int)(0.4695767196*heightMultiplier), (int)(0.4154265873*widthMultiplier)),
            new Point((int)(0.5869708995*heightMultiplier), (int)(0.5791170635*widthMultiplier)));
    private RingAmount.Rings rings = RingAmount.Rings.ZERO;
    public RingDetector(Telemetry t){
        this.telemetry = t;
    }
    @Override
    public Mat processFrame(Mat input) {
        if (input.empty()) {
            return null;
        }
        Imgproc.cvtColor(input, input, Imgproc.COLOR_BGR2YCrCb);



        Imgproc.blur(input, input, new Size(5, 5));
        //Imgproc.threshold(image, image, 100, 255, Imgproc.THRESH_BINARY);

        Imgproc.rectangle(
                input,
                ROI,
                new Scalar(240, 0, 0),
                2
        );

        Mat subArr = input.submat(ROI);

        summedSubArea = Core.sumElems(subArr).val[2];

        if (summedSubArea > 195000) {
            rings = RingAmount.Rings.FOUR;
        }
        else if (summedSubArea > 170000) {
            rings = RingAmount.Rings.ONE;
        }
        else {
            rings = RingAmount.Rings.ZERO;
        }
        subArr.release();
        telemetry.addData("Summed",summedSubArea);
        telemetry.addData("State",rings.toString());
        telemetry.addData("Iteration",i);
        telemetry.update();
        i++;
        return input;
    }
    public RingAmount.Rings getState(){
        return rings;
    }
}
