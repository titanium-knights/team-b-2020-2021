package org.firstinspires.ftc.teamcode.testOpMode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.CONFIG;
import org.firstinspires.ftc.teamcode.utils.IMU;

import java.util.Arrays;

@Disabled
public class DSLocalizationTest extends LinearOpMode {
    DistanceSensor front;
    DistanceSensor left;
    DistanceSensor right;
    DistanceSensor back;
    Motor fl,fr,bl,br;
    IMU imu;
    MecanumDrive drive;
    double frontRawVal,leftRawVal,backRawVal,rightRawVal,frontVal,leftVal,backVal,rightVal;
    @Override
    public void runOpMode() throws InterruptedException {
        front = hardwareMap.get(DistanceSensor.class, CONFIG.FRONTDIST);
        left = hardwareMap.get(DistanceSensor.class, CONFIG.LEFTDIST);
        right = hardwareMap.get(DistanceSensor.class, CONFIG.RIGHTDIST);
        back = hardwareMap.get(DistanceSensor.class, CONFIG.BACKDIST);
        fl = (Motor) hardwareMap.get(DcMotorEx.class, CONFIG.FRONTLEFT);
        fr = (Motor) hardwareMap.get(DcMotorEx.class, CONFIG.FRONTRIGHT);
        bl = (Motor) hardwareMap.get(DcMotorEx.class, CONFIG.BACKLEFT);
        br = (Motor) hardwareMap.get(DcMotorEx.class, CONFIG.BACKRIGHT);
        drive = new MecanumDrive(fl,fr,bl,br);
        imu.initializeIMU();
        waitForStart();
        while(opModeIsActive()){
            leftRawVal = left.getDistance(DistanceUnit.INCH);
            boolean leftBad = leftRawVal == DistanceSensor.distanceOutOfRange;
            rightRawVal = right.getDistance(DistanceUnit.INCH);
            boolean rightBad = rightRawVal == DistanceSensor.distanceOutOfRange;
            backRawVal = back.getDistance(DistanceUnit.INCH);
            boolean backBad = backRawVal == DistanceSensor.distanceOutOfRange;
            frontRawVal = front.getDistance(DistanceUnit.INCH);
            boolean frontBad = frontRawVal == DistanceSensor.distanceOutOfRange;

            frontVal= Math.sin(Math.toRadians(imu.getZAngle())) * frontRawVal;
            rightVal = Math.sin(Math.toRadians(imu.getZAngle())) *rightRawVal;
            backVal= Math.sin(Math.toRadians(imu.getZAngle())) *backRawVal;
            leftVal = Math.sin(Math.toRadians(imu.getZAngle())) *leftRawVal;

        }
    }
}
