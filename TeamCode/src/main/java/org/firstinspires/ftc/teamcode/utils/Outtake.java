package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake {
    private HardwareMap hardwareMap;

    DcMotor flywheel;
    DcMotor tilts;

    Servo pusher;

    public Outtake(HardwareMap hmap) {
        flywheel = hmap.dcMotor.get("flywheel");
        tilts = hmap.dcMotor.get("tilt");

        pusher = hmap.servo.get("pusher");
    }

    public void pushRing() {
        pusher.setPosition(0.25);
        pusher.setPosition(0);
    }

    public void setFlywheelSpeed(double x) {
        flywheel.setPower(x);
    }

    public void spin() {
        flywheel.setPower(0.5);
    }
}
