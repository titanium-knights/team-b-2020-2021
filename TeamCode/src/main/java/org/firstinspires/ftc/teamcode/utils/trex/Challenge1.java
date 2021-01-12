package org.firstinspires.ftc.teamcode.utils.trex;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//12/16/2020
//Ari
public class Challenge1 extends LinearOpMode {
    MDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MDrive(hardwareMap);
        waitForStart();
        //Put your code here
        //travel in a rectangle
    }


}
