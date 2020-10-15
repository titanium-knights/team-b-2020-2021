package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake {

    DcMotor flywheel;
    DcMotor shooter;
    Servo push;

    public Outtake(HardwareMap hmap) {
        flywheel = hmap.dcMotor.get("flywheel");
        shooter = hmap.dcMotor.get("shooter");
        push = hmap.servo.get("push");

    }

    public void pushRing() {
        push.setPosition(.25);
        push.setPosition(0);
    }

    public void setFlywheelSpeed(double x) {
        flywheel.setPower(x);
    }

    public void spin() {
        flywheel.setPower(.5);
    }

}
