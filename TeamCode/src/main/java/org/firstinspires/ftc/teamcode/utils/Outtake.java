package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake {

    DcMotorEx shooter;
    Servo pusher;
    final double TICKS_PER_ROTATION = 28;
    public Outtake(HardwareMap hmap) {
        shooter = hmap.get(DcMotorEx.class, CONFIG.SHOOTER);
        //shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //RUE
        pusher = hmap.get(Servo.class, CONFIG.PUSH);

    }
    public void setPIDF(){
        setPIDF(DashboardVars.shooter.p, DashboardVars.shooter.i, DashboardVars.shooter.d, DashboardVars.shooter.f);
    }
    public void setPIDF(double p, double i, double d, double f){
        shooter.setVelocityPIDFCoefficients(p,i,d,f);
    }

    public void setVelocityTPS(double ticksPSec){
        shooter.setVelocity(ticksPSec);
    }
    public void setFlywheelSpeed(double a){
        setVelocityRPM(a);
    }
    public void setVelocityRPM(double rpm){
        double tps = (rpm / 60.0) * TICKS_PER_ROTATION;
        shooter.setVelocity(tps);
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
        pusher.setPosition(.85);
    }
    public void spinWPower(double pow){
        shooter.setPower(pow);
    }

}
