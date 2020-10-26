package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    DcMotor intakeMotor;

    public Intake(HardwareMap hmap) {
        this.intakeMotor = hmap.dcMotor.get(CONFIG.INTAKE);
    }

    public void spin() {
        intakeMotor.setPower(1);
    }

    public void stop() {
        intakeMotor.setPower(0);
    }

    void setSpeed(double val) {
        intakeMotor.setPower(val);
    }

    void reverse() {
        intakeMotor.setPower(-1);
    }
}
