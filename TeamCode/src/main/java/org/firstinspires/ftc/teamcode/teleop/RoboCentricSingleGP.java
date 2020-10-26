package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.Intake;
import org.firstinspires.ftc.teamcode.utils.MecDrive;
import org.firstinspires.ftc.teamcode.utils.Outtake;
import org.firstinspires.ftc.teamcode.utils.WobbleGoal;

@TeleOp(name="Single Gamepad Robo Centric")
public class RoboCentricSingleGP extends OpMode {
    MecDrive drive;
    Intake intake;
    WobbleGoal wg;
    Outtake out;
    @Override
    public void init() {
        drive = new MecDrive(hardwareMap,false);
        intake = new Intake(hardwareMap);
        wg = new WobbleGoal(hardwareMap);
        out = new Outtake(hardwareMap);

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

        if(gamepad1.dpad_up){
            wg.grab();
        }
        else if(gamepad1.dpad_down){
            wg.release();
        }
        else{
            wg.stop();
        }
        if(gamepad1.y){
            out.spin();
        }
        if(gamepad1.b){
            out.stop();
        }
    }
}
