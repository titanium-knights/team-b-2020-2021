package org.firstinspires.ftc.teamcode.testOpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.IMU;

@TeleOp(name = "IMU Read Out")
public class imuRead extends OpMode {
    IMU imu;
    @Override
    public void init() {
        imu = new IMU(hardwareMap);
        imu.initializeIMU();
    }

    @Override
    public void loop() {
        telemetry.addData("Z angle", imu.getZAngle());
        telemetry.addData("y angle", imu.getYAngle());
        telemetry.addData("x angle", imu.getXAngle());
    }
}
