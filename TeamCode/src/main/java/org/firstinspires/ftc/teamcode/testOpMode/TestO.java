package org.firstinspires.ftc.teamcode.testOpMode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utils.CONFIG;

import java.util.Arrays;

public class TestO extends OpMode {
    DcMotorEx[] motors=new DcMotorEx[4];
    BNO055IMU imu;
    @Override
    public void init(){
        motors[0] = hardwareMap.get(DcMotorEx.class, CONFIG.FRONTLEFT);
        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[1] = hardwareMap.get(DcMotorEx.class,CONFIG.FRONTRIGHT);
        motors[2] = hardwareMap.get(DcMotorEx.class,CONFIG.BACKLEFT);
        motors[2].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[3] = hardwareMap.get(DcMotorEx.class,CONFIG.BACKRIGHT);
        for(DcMotorEx motor: motors){
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }
        imu = hardwareMap.get(BNO055IMU.class,"imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
    }
    @Override
    public void loop(){
        double angle = Math.toRadians(-imu.getAngularOrientation().firstAngle);
        double inputY = -gamepad1.left_stick_y;
        double inputX = gamepad1.left_stick_x;
        double rot = gamepad1.right_stick_x;
        double x = Math.cos(angle) * inputX - Math.sin(angle) * inputY;
        double y = Math.sin(angle) * inputX + Math.cos(angle) * inputY;
        double[] power={
                y+x+rot,//fl
                y-x-rot,//fr
                y-x+rot,//bl
                y+x-rot//br
        };
        double max = Math.max(Math.abs(Arrays.stream(power).max().getAsDouble()),Math.abs(Arrays.stream(power).min().getAsDouble()));
        if(max>1){
            for(int i=0;i<4;i++){
                power[i]/=max;
            }
        }

        int i=0;
        for(DcMotorEx motor:motors){
            motor.setPower(power[i]);
            i++;
        }
    }
}
