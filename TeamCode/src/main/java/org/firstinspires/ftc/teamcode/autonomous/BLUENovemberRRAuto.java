package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.utils.IMU;
import org.firstinspires.ftc.teamcode.utils.Intake;
import org.firstinspires.ftc.teamcode.utils.LaunchMath;
import org.firstinspires.ftc.teamcode.utils.Outtake;
import org.firstinspires.ftc.teamcode.utils.RingAmount;
import org.firstinspires.ftc.teamcode.utils.RingDetector;
import org.firstinspires.ftc.teamcode.utils.WobbleGoal;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;
@Disabled
@Autonomous(name= "BlueAuto",group="auton")
public class BLUENovemberRRAuto extends LinearOpMode {
    private Intake intake;
    private Outtake outtake;
    private RingDetector detector;
    private IMU imu;
    private LaunchMath lm;
    private DistanceSensor left;
    private DistanceSensor front;
    private OpenCvCamera phoneCam;
    private WobbleGoal wg;
    private RingAmount.Rings state;
    private Pose2d startPose;
    private Vector2d leftVector = new Vector2d(-12,6);
    

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();

    }
    public void initialize(){

    }
}
