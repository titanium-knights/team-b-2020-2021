package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake {

    DcMotor shooter;
    //DcMotor angle;
    Servo push;

    public Outtake(HardwareMap hmap) {
        shooter = hmap.dcMotor.get(CONFIG.SHOOTER);
        //angle = hmap.dcMotor.get(CONFIG.ANGLE);
        push = hmap.servo.get(CONFIG.PUSH);
    }

    public void pushRing() {
        push.setPosition(.25);
        push.setPosition(0);
    }

    public void setFlywheelSpeed(double x) {
        shooter.setPower(x);
    }

    public void spin() {
        shooter.setPower(.5);
    }

}
