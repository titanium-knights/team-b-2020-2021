package org.firstinspires.ftc.teamcode.testOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.utils.CONFIG;

public class TempShooter {

    DcMotorEx shooter,shooter2;
    Servo pusher;
    public TempShooter(HardwareMap hmap) {

        shooter = hmap.get(DcMotorEx.class, CONFIG.SHOOTER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2 = hmap.get(DcMotorEx.class, CONFIG.SHOOTER2);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        //shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //RUE
        pusher = hmap.get(Servo.class, CONFIG.PUSH);
        MotorConfigurationType motorConfigurationType = shooter.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        shooter.setMotorType(motorConfigurationType);

        MotorConfigurationType motorConfigurationType2 = shooter2.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        shooter2.setMotorType(motorConfigurationType);

    }

    public void spin() {
        shooter.setPower(1);
        shooter2.setPower(1);
    }
    public void stop(){
        shooter.setPower(0);
        shooter2.setPower(0);
    }
    public void spinPowershot(){spinWPower(0.4);}
    public void spinHighGoal(){spinWPower(0.58);}
    public void push(){
        pusher.setPosition(0.25);
    }
    public void pull(){
        pusher.setPosition(.85);
    }
    public void spinWPower(double pow){
        shooter.setPower(pow);
        shooter2.setPower(pow);
    }
    public DcMotorEx getShooter(){
        return shooter;
    }
    public DcMotorEx getShooter2(){
        return shooter2;
    }

}
