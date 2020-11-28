package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utils.ButtonToggler;
import org.firstinspires.ftc.teamcode.utils.IMU;
import org.firstinspires.ftc.teamcode.utils.Intake;
import org.firstinspires.ftc.teamcode.utils.MecDrive2;
import org.firstinspires.ftc.teamcode.utils.Outtake;
import org.firstinspires.ftc.teamcode.utils.Pusher;
import org.firstinspires.ftc.teamcode.utils.WobbleGoal;

public class DecemberTele extends OpMode {
    MecDrive2 drive;
    Intake intake;
    WobbleGoal wg;
    Outtake out;
    IMU imu;
    Pusher pusher;
    ButtonToggler btA;
    @Override
    public void init(){
        drive= new MecDrive2(hardwareMap);
        intake = new Intake(hardwareMap);
        out  =new Outtake(hardwareMap);
        imu = new IMU(hardwareMap);
        wg=new WobbleGoal(hardwareMap);
        pusher = new Pusher(hardwareMap);
        btA = new ButtonToggler();
    }

    @Override
    public void loop(){
        btA.ifRelease(gamepad1.y);
        btA.update(gamepad1.y);
        drive.teleOp(gamepad1);
        if(btA.getMode()){
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
            pusher.pull();
        }
        else if(gamepad1.dpad_right){
            pusher.push();
        }
        telemetry.addData("leftx",gamepad1.left_stick_x);
        telemetry.addData("leftY",gamepad1.left_stick_y);
        telemetry.addData("rightx",gamepad1.right_stick_y);
        telemetry.addData("righty",gamepad1.right_stick_y);
        telemetry.addData("imu",imu.getZAngle());
        telemetry.update();
    }

}
