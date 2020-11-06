package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WobbleGoal {
    DcMotor elevator;
    CRServo grabber;

    public WobbleGoal(HardwareMap hmap) {
        elevator = hmap.dcMotor.get("elevator");
        grabber = hmap.get(CRServo.class,"grabber");
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void grab() {
        grabber.setPower(1);
    }

    public void release() {
        grabber.setPower(-1);
    }

    public void lift() {
        elevator.setPower(0.3);
    }

    public void lower() {
        elevator.setPower(-0.3);
    }

    public void stopElevator() {
        elevator.setPower(0);
    }

    public void stopServo(){
        grabber.setPower(0);
    }
}
