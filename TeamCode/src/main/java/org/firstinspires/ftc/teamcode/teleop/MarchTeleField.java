package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.ButtonToggler;
import org.firstinspires.ftc.teamcode.utils.IMU;
import org.firstinspires.ftc.teamcode.utils.Intake;
import org.firstinspires.ftc.teamcode.utils.MecDrive2;
import org.firstinspires.ftc.teamcode.utils.Outtake;
import org.firstinspires.ftc.teamcode.utils.Pusher;
import org.firstinspires.ftc.teamcode.utils.Shooter2;
import org.firstinspires.ftc.teamcode.utils.WobbleGoal;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "MarchTeleField")
public class MarchTeleField extends OpMode {
    boolean hgOrPower= true; //True for highgoal false for powershot
    MecDrive2 drive;
    Intake intake;
    WobbleGoal wg;
    Shooter2 out;
    IMU imu;
    Pusher pusher;
    ButtonToggler btA;
    ButtonToggler btY;
    ButtonToggler btB;
    ButtonToggler btX;
    long set1, set2, set3;
    ElapsedTime time = new ElapsedTime();
    boolean flickerInAction = false;
    @Override
    public void init(){
        drive= new MecDrive2(hardwareMap);
        intake = new Intake(hardwareMap);
        out  =new Shooter2(hardwareMap);
        imu = new IMU(hardwareMap);
        imu.initializeIMU();

        wg=new WobbleGoal(hardwareMap);
        pusher = new Pusher(hardwareMap);
        btY = new ButtonToggler();
        btA = new ButtonToggler();
        btB = new ButtonToggler();
        btX=new ButtonToggler();
    }

    @Override
    public void loop(){
        btA.ifRelease(gamepad1.a);
        btA.update(gamepad1.a);

        btX.ifRelease(gamepad1.x);
        btX.update(gamepad1.x);

        btB.ifRelease(gamepad1.b);
        btB.update(gamepad1.b);

        btY.ifRelease(gamepad1.y);
        btY.update(gamepad1.y);

        /*if(!btB.getMode()){
            drive.teleOpRobotCentric(gamepad1,1);
        }
        else{
            drive.teleOpRobotCentric(gamepad1,0.5);
        }*/
        drive.teleOpFieldCentric(gamepad1,imu);

        if(btB.getMode()){
            hgOrPower = false;
        }
        else{
            hgOrPower = true;
        }

        if(btA.getMode()){
            intake.spinBoth();
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


        if(btX.getMode()){
            wg.grab();
        }
        else {
            wg.release();
        }

        if(btY.getMode()){
            if(hgOrPower){
                out.spinHighGoal();
            }
            else{
                out.spinPowershot();
            }
        }
        else{
            out.stop();
        }
        if(gamepad1.dpad_left){
            telemetry.addData("Dpad left pressed",true);
            telemetry.update();
            out.pull();
        }
        else if(gamepad1.dpad_right){
            telemetry.addData("Dpad right pressed",true);
            telemetry.update();
            out.push();
        }
        telemetry.addData("leftX",gamepad1.left_stick_x);
        telemetry.addData("leftY",-gamepad1.left_stick_y);
        telemetry.addData("rightX",gamepad1.right_stick_x);
        telemetry.addData("rightY",gamepad1.right_stick_y);
        //telemetry.addData("imu",imu.getZAngle());

        if(gamepad1.dpad_up && !flickerInAction){
            //first time press flickerInAction
            //need to start the time
            flickerInAction = true;
            time.reset();
            out.pull();
        }
        else if(gamepad1.dpad_up){
            //If still held down from previous cycle, simply ignore maintain the true state
            flickerInAction=true;
        }
        else{
            flickerInAction = false;
        }
        if(flickerInAction && time.milliseconds() >= 150){
            out.pull();
            flickerInAction = false;
            time.reset();
        }
        telemetry.update();
    }
}
