package org.firstinspires.ftc.teamcode.utils;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ComputerVisionRectDetector extends OpenCvPipeline {
    public ComputerVisionRectDetector(){}
    public enum Rings{
        ZERO,
        ONE,
        FOUR
    }

    Rings stack = Rings.ZERO;
    Point coordinate1 = new Point((1700.0/4000.0)*320,(1900.0/3000.0)*240);
    Point coordinate2 = new Point((2250.0/4000.0)*320,(1400/3000.0)*240);
    static final Scalar BLUE = new Scalar(0,0,255);
    Mat YCrCb = new Mat();
    Mat Cr = new Mat();
    Mat croppedRegion = new Mat();
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input,YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb,Cr,1);
        croppedRegion = Cr.submat(new Rect(coordinate1,coordinate2));
        int avg = (int) Core.mean(croppedRegion).val[0];
        Imgproc.rectangle(input, coordinate1,coordinate2,BLUE,2);
        if(avg >142){
            stack = Rings.FOUR;
        }
        else if (avg>134){
            stack = Rings.ONE;
        }
        else{
            stack = Rings.ZERO;
        }

        return input;
    }
    public Rings getStack(){
        return stack;
    }
}