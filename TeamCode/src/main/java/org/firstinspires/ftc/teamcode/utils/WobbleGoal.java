package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WobbleGoal {
    DcMotor elevator;
    Servo grabber;

    public WobbleGoal(HardwareMap hmap) {
        elevator = hmap.dcMotor.get("elevator");
        grabber = hmap.servo.get("grabber");
    }

    public void grab() {
        grabber.setPosition(1);
    }

    public void release() {
        grabber.setPosition(0);
    }

    public void lift() {
        elevator.setPower(0.5);
    }

    public void lower() {
        elevator.setPower(-0.5);
    }

    public void stop() {
        elevator.setPower(0);
    }
}
