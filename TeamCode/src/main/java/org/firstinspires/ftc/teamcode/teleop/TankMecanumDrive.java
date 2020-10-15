package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.MecDrive;
@TeleOp(name = "Simple Mecanum", group="TeleOp")
public class TankMecanumDrive extends OpMode {
    MecDrive drive;
    @Override
    public void init(){
        drive = new MecDrive(hardwareMap,false);
    }
    @Override
    public void loop(){
        drive.teleopTank(gamepad1);
    }

}
