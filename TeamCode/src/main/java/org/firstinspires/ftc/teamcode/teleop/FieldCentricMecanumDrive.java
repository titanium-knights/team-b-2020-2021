package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utils.IMU;
import org.firstinspires.ftc.teamcode.utils.MecanumDrive;
@TeleOp(name = "Field Centric Mecanum", group="TeleOp")
public class FieldCentricMecanumDrive extends OpMode {
    MecanumDrive drive;
    IMU imu;
    @Override
    public void init(){
        imu = new IMU(hardwareMap);
        drive = new MecanumDrive(hardwareMap,false);
        telemetry.addData("IMU Calibration",imu.initializeIMU());
    }
    @Override
    public void loop(){
        drive.fieldCentricDrive(gamepad1,imu.getZAngle());
    }

}
