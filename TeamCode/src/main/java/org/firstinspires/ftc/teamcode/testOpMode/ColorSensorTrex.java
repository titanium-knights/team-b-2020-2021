package org.firstinspires.ftc.teamcode.testOpMode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utils.trex.MDrive;

public class ColorSensorTrex extends LinearOpMode {
    ColorSensor csensor;
    MDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        csensor = hardwareMap.get(ColorSensor.class,"sensor_color");
        csensor.enableLed(true);
        drive = new MDrive(hardwareMap);
        waitForStart();
        float saturation = 0.0f;
        do{
            float[] hsv = new float[3];
            //Hsv conversion
            Color.RGBToHSV(csensor.red(),csensor.green(),csensor.blue(),hsv);
            saturation = hsv[1];
            drive.driveForwardsWithPower(0.5);
        }
        while(saturation<40);

        drive.stop();
    }
}
