package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.RRQuickStart.drive.SampleMecanumDrive;

public class OneRingAutonomous extends LinearOpMode {
    //Declare Trajectories

    SampleMecanumDrive drive;
    public void initialize(){
       //Initialize all the objects, etc

    }


    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
    }

}