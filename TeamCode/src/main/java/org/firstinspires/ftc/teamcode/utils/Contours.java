package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class Contours extends OpenCvPipeline {
    private Telemetry telemetry;
    private Mat mat = new Mat();
    private Mat rect = new Mat();
    private RingAmount.Rings stack = RingAmount.Rings.ZERO;
    private Scalar lowerOrange = new Scalar(0.0, 141.0, 0.0);
    private Scalar upperOrange = new Scalar(255.0, 230.0, 95.0);
    private int CAMERA_WIDTH = 320;
    private int HORIZON = (int)((100.0 / 320.0) * CAMERA_WIDTH);
    private int MIN_WIDTH = (int) ((30 / 320.0) * CAMERA_WIDTH);
    private final double BOUND_RATIO = 0.7;
    private double midRing = 0;
    private double midActual =0;
    public Contours(Telemetry telemetry){this.telemetry=telemetry;}
    @Override
    public Mat processFrame(Mat input) {
        rect.release();
        rect = new Mat();
        input = input.submat(0,input.rows(), input.cols()/2-100, input.cols()/2+100);
        Imgproc.cvtColor(input,mat,Imgproc.COLOR_RGB2YCrCb);
        Mat mask = new Mat(mat.rows(),mat.cols(), CvType.CV_8UC1);
        Core.inRange(mat,lowerOrange,upperOrange,mask);
        Core.bitwise_and(input,input,rect,mask);
        Imgproc.GaussianBlur(mask,mask,new Size(5,15),0);
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);
        Imgproc.drawContours(rect, contours, -1, new Scalar(0.0, 255.0, 0.0), 3);

        int maxWidth = 0;
        int height =0;
        Rect maxRect = new Rect();
        for (MatOfPoint c: contours) {
            Mat copy = new MatOfPoint2f(c.toArray());
            Rect ret = Imgproc.boundingRect(copy);

            int w = ret.width;
            // checking if the rectangle is below the horizon
            if (w > maxWidth ) {
                maxWidth = w;
                maxRect = ret;
                height = ret.height;
            }
            copy.release();
            c.release(); // releasing the buffer of the contour, since after use, it is no longer needed
        }
        /**drawing widest bounding rectangle to ret in blue**/
        telemetry.addData("Max Width",maxWidth);
        telemetry.addData("height",height);
        if(maxWidth>27 || height >20) {
            Imgproc.rectangle(rect, maxRect, new Scalar(0.0, 0.0, 255.0), 2);


            /** drawing a red line to show the horizon (any above the horizon is not checked to be a ring stack **/
            Imgproc.line(
                    rect,
                    new Point(rect.width() / 2.0, 0),
                    new Point(rect.width() / 2.0, rect.height()),
                    new Scalar(255.0, .0, 255.0)
            );
            midRing = maxRect.x + (maxRect.width) / 2.0;
            midActual = rect.width()/2.0;
            telemetry.addData("Ring Mid:", midRing);
            telemetry.addData("Frame Mid", midActual);
            Imgproc.line(
                    rect,
                    new Point(midRing, 0),
                    new Point(midRing, rect.height()),
                    new Scalar(0, 153, 255)
            );
        }
        //Debugging (can remove)
        if (maxWidth >= MIN_WIDTH) {
            double aspectRatio = (double) maxRect.height/ maxRect.width;


            /** checks if aspectRatio is greater than BOUND_RATIO
             * to determine whether stack is ONE or FOUR
             */
            if (aspectRatio > BOUND_RATIO)
                stack = RingAmount.Rings.ZERO ;// height variable is now FOUR
            else
                stack= RingAmount.Rings.ONE ;// height variable is now ONE
        }
        else{
            stack= RingAmount.Rings.FOUR;
        }
        telemetry.addData("stack",stack);
        telemetry.update();
        mat.release();
        mask.release();
        hierarchy.release();
        return rect;

    }


    public double getMidRing(){
        return midRing;
    }
    public double getMidActual(){
        return midActual;
    }
    public double getError(){
        return midRing-midActual;
    }
    public RingAmount.Rings getStack(){
        return stack;
    }
}
