package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class Shooter2{
    // Copy your PIDF Coefficients here
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(85, 0, 10.5, 15.5);
    DcMotorEx shooter;

    Servo pusher;
    public Shooter2(HardwareMap hm){
        shooter = hm.get(DcMotorEx.class, CONFIG.SHOOTER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        for (LynxModule module : hm.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        // RUE limits max motor speed to 85% by default
        // Raise that limit to 100%
        MotorConfigurationType motorConfigurationType = shooter.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        shooter.setMotorType(motorConfigurationType);
        // Turn on RUN_USING_ENCODER
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set PIDF Coefficients with voltage compensated feedforward value
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                MOTOR_VELO_PID.p, MOTOR_VELO_PID.i, MOTOR_VELO_PID.d,
                MOTOR_VELO_PID.f * 12 / hm.voltageSensor.iterator().next().getVoltage()
        ));
        pusher = hm.get(Servo.class, CONFIG.PUSH);
    }
    public void spin() {
        shooter.setPower(1);
    }
    public void spinPowershot(){shooter.setPower(0.7);}
    public void spinHighGoal(){spin();}
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