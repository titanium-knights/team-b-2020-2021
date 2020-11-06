package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.IMU;
import org.firstinspires.ftc.teamcode.utils.Intake;
import org.firstinspires.ftc.teamcode.utils.Outtake;
import org.firstinspires.ftc.teamcode.utils.WobbleGoal;

@TeleOp(name = "FieldCentricSingle")
public class FieldCentricSingleGP extends OpMode {
    MecanumDrive drive;
    Motor fl; //front left
    Motor fr; //front right
    Motor bl; //back left
    Motor br; //back right

    Intake intake;
    Outtake outtake;
    WobbleGoal wg;
    IMU imu;
    @Override
    public void init() {
        fl = new Motor(hardwareMap,"fl");
        fr = new Motor(hardwareMap,"fr");
        bl = new Motor(hardwareMap,"bl");
        br = new Motor(hardwareMap,"br");

        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        drive = new MecanumDrive(fl,fr,bl,br);
        imu = new IMU(hardwareMap);
        imu.initializeIMU();
        wg = new WobbleGoal(hardwareMap);
    }

    @Override
    public void loop() {
        double angle = imu.getZAngle();
        drive.driveFieldCentric(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x,angle);
        if(gamepad1.a){
            intake.spin();
        }
        else{
            intake.stop();
        }

        /*if(gamepad1.dpad_up){
            wg.grab();
        }
        else if(gamepad1.dpad_down){
            wg.release();
        }
        else{
            wg.stop();
        }*/

        if(Math.abs(gamepad1.left_trigger)>0.3){
            wg.lift();
        }
        else if(Math.abs(gamepad1.right_trigger)>0.3){
            wg.lower();
        }
        else{
            wg.stop();
        }

        if(gamepad1.y){
            outtake.spin();
        }
        if(gamepad1.b){
            outtake.stop();
        }
    }
}
