package org.firstinspires.ftc.teamcode.testOpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utils.IMU;
import org.firstinspires.ftc.teamcode.utils.MecDrive2;

public class TestOO extends OpMode {
    MecDrive2 drive;
    IMU imu;
    public void init(){
        drive = new MecDrive2(hardwareMap);
        imu = new IMU(hardwareMap);
        imu.initializeIMU();
    }
    public void loop(){
        drive.teleOpFieldCentric(gamepad1,imu);
    }
}
