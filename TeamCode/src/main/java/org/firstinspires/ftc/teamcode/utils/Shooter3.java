package org.firstinspires.ftc.teamcode.utils;

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

@Disabled
public class Shooter3 {
    // Copy your PID Coefficients here
    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(5e-7, 0, 0.0019);
    public static double SHOOTER_POWER=0;
    public static double HIGHGOALPOWER=0.5;
    public static double POWERSHOTPOWER=0.4;
    // Copy your feedforward gains here
    public static double kV = 0.00045;
    public static double kA = 0.00035;
    public static double kStatic = 0;

    // Timer for calculating desired acceleration
    // Necessary for kA to have an affect
    private final ElapsedTime veloTimer = new ElapsedTime();
    private double lastTargetVelo = 0.0;

    // Our velocity controller
    private final VelocityPIDFController veloController = new VelocityPIDFController(MOTOR_VELO_PID, kV, kA, kStatic);
    DcMotorEx shooter1, shooter2;
    Servo pusher;


    public Shooter3(HardwareMap hm){
        pusher = hm.get(Servo.class, CONFIG.PUSH);
        shooter1=hm.get(DcMotorEx.class,CONFIG.SHOOTER);
        shooter2=hm.get(DcMotorEx.class,CONFIG.SHOOTER2);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        for (LynxModule module : hm.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }
    public void update(){
        double targetVelo = TuningController.rpmToTicksPerSecond(SHOOTER_POWER*TuningController.MOTOR_MAX_RPM);

        veloController.setTargetVelocity(targetVelo);
        veloController.setTargetAcceleration((targetVelo - lastTargetVelo) / veloTimer.seconds());
        veloTimer.reset();

        lastTargetVelo = targetVelo;

        // Get the velocity from the motor with the encoder
        double motorPos = shooter1.getCurrentPosition();
        double motorVelo = shooter1.getVelocity();

        // Update the controller and set the power for each motor
        double power = veloController.update(motorPos, motorVelo);
        shooter1.setPower(power);
        shooter2.setPower(power);
    }
    public void spinHighGoal(){
        SHOOTER_POWER=HIGHGOALPOWER;
    }
    public void spinPowerShot(){
        SHOOTER_POWER=POWERSHOTPOWER;
    }
    public void spinPowershot(){
        spinPowerShot();
    }
    public void stop(){
        SHOOTER_POWER=0;
    }
    public void spinWPower(double p){
        SHOOTER_POWER=p;
    }
    public void push(){
        pusher.setPosition(0.25);
    }
    public void pull(){
        pusher.setPosition(.85);
    }
}