package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake {

    DcMotorEx shooter;
    Servo pusher;
    public Outtake(HardwareMap hmap) {
        shooter = hmap.get(DcMotorEx.class, CONFIG.SHOOTER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pusher = hmap.get(Servo.class, CONFIG.PUSH);

    }
    public void setPIDF(){
        setPIDF(DashboardVars.shooter.p, DashboardVars.shooter.i, DashboardVars.shooter.d, DashboardVars.shooter.f);
    }
    public void setPIDF(double p, double i, double d, double f){
        shooter.setVelocityPIDFCoefficients(p,i,d,f);
    }
    public void setVelo(double v){
        shooter.setVelocity(v);
    }
    public void setFlywheelSpeed(double x) {
        shooter.setPower(x);
    }

    public void spin() {
        shooter.setPower(-1);
    }
    public void stop(){
        shooter.setPower(0);
    }
    public void push(){
        pusher.setPosition(0.25);
    }
    public void pull(){
        pusher.setPosition(.7);
    }

}
