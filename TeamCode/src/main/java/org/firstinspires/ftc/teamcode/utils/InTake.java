package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class InTake {

    DcMotor intakeMotor;

    public InTake(HardwareMap hmap) {
        this.intakeMotor = hmap.dcMotor.get("intakeMotor");
    }

    void spin() {
        intakeMotor.setPower(1);
    }

    void stop() {
        intakeMotor.setPower(0);
    }

    void setSpeed(double val) {
        intakeMotor.setPower(val);
    }

    void reverse() {
        intakeMotor.setPower(-1);
    }
}
