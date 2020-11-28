package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveEncoder {
    MecDrive2 drive;
    Telemetry telemetry;
    final double TICKS_PER_REV = 537.6;
    final double DRIVE_GEAR_REDUCTION = 1;
    final double WHEEL_DIAMETER_INCHES =4*Math.PI;
    final double COUNTS_PER_INCH = (TICKS_PER_REV*DRIVE_GEAR_REDUCTION)/(WHEEL_DIAMETER_INCHES*Math.PI);
    ElapsedTime timer = new ElapsedTime();

    public DriveEncoder(MecDrive2 drive, Telemetry telemetry){
        this.drive = drive;
        this.telemetry = telemetry;
        timer.startTime();
    }

    /**
     *
     * @param power Power for drive train to move
     * @param left Amount of inches that left drive needs to travel
     * @param right Amount of inches that right drive needs to travel
     * @param timeout Level of precision
     */
    public void encoderDrive(double power,double left, double right,double timeout){
        int[] prevEncoderValues = drive.getEncoderVals();
        int[] newTargets= new int[4];
        double powerAbs = Math.abs(power);
        double[] powerArr = {powerAbs,powerAbs,powerAbs,powerAbs};
        if(!Thread.currentThread().interrupted()){
            newTargets[0]=prevEncoderValues[0]+(int)(left*COUNTS_PER_INCH);
            newTargets[1]=prevEncoderValues[1]+(int)(right*COUNTS_PER_INCH);
            newTargets[2]=prevEncoderValues[2]+(int)(left*COUNTS_PER_INCH);
            newTargets[3]=prevEncoderValues[3]+(int)(right*COUNTS_PER_INCH);
            drive.setTargetPosition(newTargets);
            drive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            timer.reset();
            drive.setRawPowers(powerArr);
            while(!Thread.currentThread().isInterrupted() && (timer.seconds()<timeout) && (drive.allMotorsBusy())){
                telemetry.addData("Path 1", "Running to %7d : %7d", newTargets[0], newTargets[1]);
                telemetry.addData("Path 2", "Running to %7d : %7d", newTargets[2], newTargets[3]);
                telemetry.update();
            }
            drive.stop();
            drive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            drive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    /**
     *
     * @param power
     * @param inchesForward
     * @param timeout maximum amount of seconds for movement to occur
     */
    public void encoderStraight(double power, double inchesForward,double timeout){
        int[] prevEncoderValues = drive.getEncoderVals();
        encoderDrive(power, inchesForward, inchesForward,timeout);
    }


}
