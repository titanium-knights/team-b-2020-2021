package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    DcMotor intakeMotor;
    DcMotor bottomRoller;
    public Intake(HardwareMap hmap) {
        this.intakeMotor = hmap.dcMotor.get(CONFIG.INTAKE);
        this.bottomRoller = hmap.dcMotor.get(CONFIG.BOTTOMROLLER);
    }

    public void spin() {
        intakeMotor.setPower(1);
    }
    public void spinBoth() {
        spin();
        spinBottom();

    }
    public void spinBottom(){
        bottomRoller.setPower(1);
    }
    public void stopUpper(){
        intakeMotor.setPower(0);
    }
    public void stopBottom(){
        bottomRoller.setPower(0);
    }
    public void stop() {
        intakeMotor.setPower(0);
        bottomRoller.setPower(0);
    }


    void setSpeed(double val) {
        intakeMotor.setPower(val);
    }

    void reverse() {
        intakeMotor.setPower(-1);
    }
}
