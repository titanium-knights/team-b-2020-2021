package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utils.ButtonToggler;
import org.firstinspires.ftc.teamcode.utils.CONFIG;
import org.firstinspires.ftc.teamcode.utils.IMU;
import org.firstinspires.ftc.teamcode.utils.Intake;
import org.firstinspires.ftc.teamcode.utils.LaunchMath;
import org.firstinspires.ftc.teamcode.utils.MecDriveWLib;
import org.firstinspires.ftc.teamcode.utils.Outtake;
import org.firstinspires.ftc.teamcode.utils.Pusher;

@TeleOp(name = "Field Centric Mecanum", group="TeleOp")
public class FieldCentricMecanumDrive extends OpMode {
    MecanumDrive drive;
    MecDriveWLib driveLib;
    IMU imu;
    Motor[] arr = new Motor[4];
    ButtonToggler buttonA;
    LaunchMath lm;
    ButtonToggler buttonA2;
    ButtonToggler buttonX;
    ButtonToggler buttonY;
    Intake intake;
    Pusher pusher;
    Outtake out;
    @Override
    public void init(){
        imu = new IMU(hardwareMap);
        lm = new LaunchMath(84.61433584517874/39.37);
        arr[0] = (Motor) hardwareMap.get(DcMotor.class,CONFIG.FRONTLEFT);
        arr[1] = (Motor) hardwareMap.get(DcMotor.class,CONFIG.FRONTRIGHT);
        arr[2] = (Motor) hardwareMap.get(DcMotor.class,CONFIG.BACKLEFT);
        arr[3] = (Motor) hardwareMap.get(DcMotor.class,CONFIG.BACKRIGHT);
        drive = new MecanumDrive(arr[0],arr[1],arr[2],arr[3]);
        telemetry.addData("IMU Calibration",imu.initializeIMU());
        buttonA = new ButtonToggler();
        buttonX = new ButtonToggler();
        buttonY = new ButtonToggler();
        pusher = new Pusher(hardwareMap);
        out.setPIDF();
    }
    @Override
    public void loop(){
        buttonA.ifRelease(gamepad1.a);
        buttonA.update(gamepad1.a);
        buttonY.ifRelease(gamepad2.y);
        buttonY.update(gamepad2.y);
        buttonX.ifRelease(gamepad2.x);
        buttonX.update(gamepad2.x);
        drive.driveFieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x,imu.getZAngle());
        if(gamepad2.a){
            intake.spin();
        }
        else if(buttonA.getMode()){
            intake.spin();
        }
        else{
            intake.stop();
        }
        if(buttonY.getMode()){
            out.spin();
        }
        if(gamepad2.dpad_up){
            pusher.push();
        }
        if(gamepad2.dpad_down){
            pusher.pull();
        }
        if(gamepad2.right_bumper){
            out.setFlywheelSpeed(lm.MSToTPS(lm.getLinearVelocity()));
        }
    }

}
