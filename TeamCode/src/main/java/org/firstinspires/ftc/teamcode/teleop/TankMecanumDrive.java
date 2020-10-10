package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utils.MecanumDrive;
@TeleOp(name = "Simple Mecanum", group="TeleOp")
public class TankMecanumDrive extends OpMode {
    MecanumDrive drive;
    @Override
    public void init(){
        drive = new MecanumDrive(hardwareMap,false);
    }
    @Override
    public void loop(){
        drive.teleopTank(gamepad1);
    }

}
