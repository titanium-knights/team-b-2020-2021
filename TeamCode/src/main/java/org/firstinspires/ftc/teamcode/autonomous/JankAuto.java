package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.CONFIG;
import org.firstinspires.ftc.teamcode.utils.Intake;
import org.firstinspires.ftc.teamcode.utils.MecDrive;
import org.firstinspires.ftc.teamcode.utils.Outtake;
@Autonomous(name="jank pl forgive :)")
public class JankAuto extends LinearOpMode {
    MecDrive drive;
    Intake intake;
    Outtake out;
    DistanceSensor right;
    DistanceSensor back;
    Servo pusher;
    @Override
    public void runOpMode() throws InterruptedException {
        initial();
        waitForStart();
        out.spin();
        driveForwardToPowerShotLine();
        drive.stop();
        driveLeftToPowershotR();
        drive.stop();
        shoot();
        driveLeftTPowershotC();
        drive.stop();
        shoot();
        driveLeftTPowershotL();
        drive.stop();
        shoot();
        driveToLine();
    }
    public void initial(){
        drive = new MecDrive(hardwareMap,false);
        intake = new Intake(hardwareMap);
        out = new Outtake(hardwareMap);
        pusher = hardwareMap.get(Servo.class,CONFIG.PUSH);
        right = hardwareMap.get(DistanceSensor.class, CONFIG.LEFTDIST);
        back = hardwareMap.get(DistanceSensor.class,CONFIG.FRONTDIST);

    }
    public void driveForwardToPowerShotLine(){
         while(back.getDistance(DistanceUnit.INCH)<15){
             drive.backwardWithPower(1);
         }
        while(back.getDistance(DistanceUnit.INCH)<24){
            drive.backwardWithPower(0.8);
        }
        while(back.getDistance(DistanceUnit.INCH)<48){
            drive.backwardWithPower(0.5);
        }

    }
    public void driveLeftToPowershotR() {
        while (right.getDistance(DistanceUnit.INCH) < 54){
            drive.strafeRightWithPower(0.5);
        }

    }
    public void driveLeftTPowershotC(){
        while (right.getDistance(DistanceUnit.INCH) < 60){
            drive.strafeRightWithPower(0.5);
        }
    }
    public void driveLeftTPowershotL(){
        while (right.getDistance(DistanceUnit.INCH) < 66){
            drive.strafeRightWithPower(0.5);
        }
    }
    public void shoot(){
        pusher.setPosition(0.7);
        pusher.setPosition(0);
    }
    public void driveToLine(){
        while(back.getDistance(DistanceUnit.INCH)<72){
            drive.forwardWithPower(0.5);
        }
    }
}
