package org.firstinspires.ftc.teamcode.teleop;

/*package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
 import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.IMU;
import org.firstinspires.ftc.teamcode.utils.MecDrive;
import org.firstinspires.ftc.teamcode.utils.MecDriveWLib;

@TeleOp(name = "Field Centric Mecanum", group="TeleOp")
public class FieldCentricMecanumDrive extends OpMode {
    MecanumDrive drive;
    MecDriveWLib driveLib;
    IMU imu;
    @Override
    public void init(){
        imu = new IMU(hardwareMap);
        driveLib = new MecDriveWLib(hardwareMap,false);
        Motor[] arr = driveLib.getMotors();
        drive = new MecanumDrive(arr[0],arr[1],arr[2],arr[3]);
        telemetry.addData("IMU Calibration",imu.initializeIMU());

    }
    @Override
    public void loop(){
        drive.driveFieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x,imu.getZAngle());
    }

}*/
public class FieldCentricMecanumDrive{

}
