package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.FlwheelPID.TuningController;
import org.firstinspires.ftc.teamcode.utils.FlwheelPID.VelocityPIDFController;

@Config
public class DualShooterNoPID {
    public static double SHOOTER_POWER=0;
    public static double HIGHGOALPOWER=0.55;
    public static double POWERSHOTPOWER=0.49;

    DcMotorEx shooter1, shooter2;
    Servo pusher;


    public DualShooterNoPID(HardwareMap hm){
        pusher = hm.get(Servo.class, CONFIG.PUSH);
        shooter1=hm.get(DcMotorEx.class,CONFIG.SHOOTER);
        shooter2=hm.get(DcMotorEx.class,CONFIG.SHOOTER2);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void spinHighGoal(){
        spinWPower(HIGHGOALPOWER);
    }
    public void spinPowerShot(){
        spinWPower(POWERSHOTPOWER);
    }
    public void spinPowershot(){
        spinPowerShot();
    }
    public void stop(){
        spinWPower(0);
    }
    public void spinWPower(double p){
        shooter1.setPower(p);
        shooter2.setPower(p);
    }
    public void push(){
        pusher.setPosition(0.25);
    }
    public void pull(){
        pusher.setPosition(.85);
    }
}