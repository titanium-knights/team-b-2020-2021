package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.CONFIG;
import org.firstinspires.ftc.teamcode.utils.IMU;
import org.firstinspires.ftc.teamcode.utils.Intake;
import org.firstinspires.ftc.teamcode.utils.MecDrive;
import org.firstinspires.ftc.teamcode.utils.Outtake;
import org.firstinspires.ftc.teamcode.utils.WobbleGoal;

@TeleOp(name="Single Gamepad Robo Centric")
public class RoboCentricSingleGP extends OpMode {
    MecDrive drive;
    Intake intake;
    IMU imu;
    WobbleGoal wg;
    Outtake out;
    Servo servo;
    @Override
    public void init() {
        drive = new MecDrive(hardwareMap,false);
        intake = new Intake(hardwareMap);
        wg = new WobbleGoal(hardwareMap);
        out = new Outtake(hardwareMap);
        imu = new IMU(hardwareMap);
        imu.initializeIMU();
        servo = hardwareMap.get(Servo.class, CONFIG.PUSH);
    }

    @Override
    public void loop() {
        drive.teleopTank(gamepad1,1);
        if(gamepad1.a){
            intake.spin();
        }
        else{
            intake.stop();
        }

        if(gamepad1.left_bumper){
            wg.lift();
        }
        else if(gamepad1.right_bumper){
            wg.lower();
        }
        else{
            wg.stopElevator();
        }

        if(gamepad1.dpad_up){
            wg.grab();
        }
        else if(gamepad1.dpad_down){
            wg.release();
        }


        if(gamepad1.y){
            out.spin();
        }
        if(gamepad1.b){
            out.stop();
        }
        if(gamepad1.dpad_left){
            servo.setPosition(0.5);
        }
        else if(gamepad1.dpad_right){
            servo.setPosition(1);
        }
        telemetry.addData("leftx",gamepad1.left_stick_x);
        telemetry.addData("leftY",gamepad1.left_stick_y);
        telemetry.addData("rightx",gamepad1.right_stick_y);
        telemetry.addData("righty",gamepad1.right_stick_y);
        telemetry.addData("imu",imu.getZAngle());
        telemetry.update();
    }
}

