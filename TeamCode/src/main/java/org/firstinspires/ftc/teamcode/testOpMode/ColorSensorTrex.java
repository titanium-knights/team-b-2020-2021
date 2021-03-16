package org.firstinspires.ftc.teamcode.testOpMode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utils.trex.MDrive;

public class ColorSensorTrex extends LinearOpMode {
  //Creating a variable named csensor of type ColorSensor
    ColorSensor csensor;
    //Create a MDrive variable using the MDrive class we wrote a while ago
    MDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        //Finds the colorsensor on the robot that is named "sensor_color"
        csensor = hardwareMap.get(ColorSensor.class,"sensor_color");
        //Color sensors have a bright led next to the sensor
        //Since we are putting the color sensor at the bottom of the robot we need to illuminate it a bit.
        csensor.enableLed(true);
        drive = new MDrive(hardwareMap);
        waitForStart();

        float saturation;
        do{
          //Creates a hsv array that holds the array {hue, saturation, value}
            float[] hsv = new float[3];
            //Hsv conversion
            Color.RGBToHSV(csensor.red(),csensor.green(),csensor.blue(),hsv);
            //To get saturation we need to get the second element of the array (index 1)
            saturation = hsv[1];
            //While there is very little color (gray), keep driving forward, otherwise exit the while and stop
            drive.driveForwardsWithPower(0.5);
        }
        while(saturation<40);

        drive.stop();
    }
}
