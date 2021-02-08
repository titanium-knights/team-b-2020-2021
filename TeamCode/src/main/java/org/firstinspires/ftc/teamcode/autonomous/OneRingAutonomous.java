package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.Intake;
import org.firstinspires.ftc.teamcode.utils.RRQuickStart.drive.SampleMecanumDrive;

public class OneRingAutonomous extends LinearOpMode {
    //Declare Trajectories
    Intake intake;
    SampleMecanumDrive drive;


    public void initialize(){
       //Initialize all the objects, etc
        intake = new Intake(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);

    }


    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();

    }

}