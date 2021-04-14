package org.firstinspires.ftc.teamcode.teleop;

import android.widget.Switch;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.ButtonToggler;
import org.firstinspires.ftc.teamcode.utils.IMU;
import org.firstinspires.ftc.teamcode.utils.Intake;
import org.firstinspires.ftc.teamcode.utils.MecDrive;
import org.firstinspires.ftc.teamcode.utils.MecDrive2;
import org.firstinspires.ftc.teamcode.utils.Outtake;
import org.firstinspires.ftc.teamcode.utils.Pusher;
import org.firstinspires.ftc.teamcode.utils.RRQuickStart.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Shooter2;
import org.firstinspires.ftc.teamcode.utils.WobbleGoal;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp
public class MarchRRTeleop extends OpMode {
    enum Mode{
        DRIVER_CONTROL,
        AUTOMATED_MOVEMENT,
        IN_PROGESS
    }
    Mode currentMode = Mode.DRIVER_CONTROL;
    Pose2d startingPose = new Pose2d(-60,-36,0);//REPLACE WITH POSESTORAGE LATER
    Vector2d shootingPosition = new Vector2d(-24,-53);
    PIDFController hController = new PIDFController(SampleMecanumDrive.HEADING_PID);
    Pose2d shootingPose= new Pose2d(-24,-53,Math.toRadians(180));
    SampleMecanumDrive rrDrive;
    MecDrive2 drive;
    Outtake out;
    WobbleGoal wg;
    Intake in;
    IMU imu;
    boolean previousLBState = false;
    @Override
    public void init() {

        drive = new MecDrive2(hardwareMap);
        out = new Outtake(hardwareMap);
        wg = new WobbleGoal(hardwareMap);
        in = new Intake(hardwareMap);
        imu = new IMU(hardwareMap);
        imu.initializeIMU();
        rrDrive = new SampleMecanumDrive(hardwareMap);
        //rrDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rrDrive.getLocalizer().setPoseEstimate(startingPose);
        hController.setInputBounds(-Math.PI,Math.PI);
    }

    @Override
    public void loop() {
        Pose2d currentPose = rrDrive.getLocalizer().getPoseEstimate();
        switch (currentMode){
            case DRIVER_CONTROL:
                drive.teleOpFieldCentric(gamepad1,imu);
                break;
            case AUTOMATED_MOVEMENT:
                Trajectory t = rrDrive.trajectoryBuilder(currentPose)
                        .lineToSplineHeading(shootingPose)
                        .build();
                rrDrive.followTrajectoryAsync(t);
                currentMode = Mode.IN_PROGESS;
            case IN_PROGESS:
                break;
        }
        if((currentMode==Mode.AUTOMATED_MOVEMENT||currentMode==Mode.IN_PROGESS) && (previousLBState && !gamepad1.left_bumper)){
            currentMode = Mode.DRIVER_CONTROL;
        }
        else if((currentMode==Mode.DRIVER_CONTROL) && (previousLBState && !gamepad1.left_bumper)){
            currentMode=Mode.AUTOMATED_MOVEMENT;
        }
        previousLBState = gamepad1.left_bumper;
        rrDrive.update();
        telemetry.addData("Current Mode",currentMode);
        telemetry.addData("current pose",currentPose);
        telemetry.addData("prev state",previousLBState);
        telemetry.update();
    }
}