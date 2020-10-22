package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    DcMotor intakeMotor;

    public Intake(HardwareMap hmap) {
        this.intakeMotor = hmap.dcMotor.get("intakeMotor");
    }

    public void spin() {
        intakeMotor.setPower(1);
    }

    public void stop() {
        intakeMotor.setPower(0);
    }

    public void setSpeed(double val) {
        intakeMotor.setPower(val);
    }

    public void reverse() {
        intakeMotor.setPower(-1);
    }
}