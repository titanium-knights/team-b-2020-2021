package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public class WobbleGoal {
    Servo arm;
    Servo grabber;
    public static double armPositionLift = 0.65;
    public static double armPositionLower = 0.92;
    public static double grabPositionGrab =0.75;
    public static double grabPositionRelease =0.35;
    public static double initializationVal = 0.25;
    public WobbleGoal(HardwareMap hmap) {
        arm = hmap.get(Servo.class,"wgarm");
        grabber = hmap.get(Servo.class,"wggrabber");
    }

    public void grab() {
        //grabber.setPosition(0.21);
        grabber.setPosition(grabPositionGrab);
    }

    public void release() {
        grabber.setPosition(grabPositionRelease);
    }

    public void lift() {
        arm.setPosition(armPositionLift);
    }

    public void lower() {
        arm.setPosition(armPositionLower);
    }

    public void allBackArm(){
        arm.setPosition(initializationVal);
    }
    public void stopServo(){
        //grabber.setPower(0);
    }

    public void setElevatorPower(double power){
        //arm.setPower(power);
    }
    public void stopElevator(){
        int a=0;
    }
    public void stop(){
        stopElevator();
    }
    public void setServoPos(double pos){
        grabber.setPosition(pos);
    }
}

