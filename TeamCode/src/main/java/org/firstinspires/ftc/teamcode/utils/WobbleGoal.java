package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WobbleGoal {
    DcMotor elevator;
    Servo grabber;

    public WobbleGoal(HardwareMap hmap) {
        elevator = hmap.dcMotor.get("elevator");
        grabber = hmap.get(Servo.class,"grabber");
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void grab() {
        grabber.setPosition(0.21);
    }

    public void release() {
        grabber.setPosition(0.63);
    }

    public void lift() {
        elevator.setPower(-1);
    }

    public void lower() {
        elevator.setPower(1);
    }

    public void stopElevator() {
        elevator.setPower(0);
    }

    public void stopServo(){
        //grabber.setPower(0);
    }
    public void stop(){
        elevator.setPower(0);
        //grabber.setPower(0);
    }
    public void setElevatorPower(double power){
        elevator.setPower(power);
    }
    public void setServoPos(double pos){
        grabber.setPosition(pos);
    }
}
