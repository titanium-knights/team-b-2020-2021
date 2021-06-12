package org.firstinspires.ftc.teamcode.teleop.StatesTeleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.testOpMode.TempShooter;
import org.firstinspires.ftc.teamcode.utils.ButtonToggler;
import org.firstinspires.ftc.teamcode.utils.DualShooterNoPID;
import org.firstinspires.ftc.teamcode.utils.IMU;
import org.firstinspires.ftc.teamcode.utils.Intake;
import org.firstinspires.ftc.teamcode.utils.MecDrive2;
import org.firstinspires.ftc.teamcode.utils.Outtake;
import org.firstinspires.ftc.teamcode.utils.Pusher;
import org.firstinspires.ftc.teamcode.utils.RRQuickStart.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Shooter2;
import org.firstinspires.ftc.teamcode.utils.Shooter3;
import org.firstinspires.ftc.teamcode.utils.WobbleGoal;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp
public class StatesTeleop extends OpMode {
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
    public static double power=0.58;
    boolean hgOrPower= true; //True for high goal false for power shot
    MecDrive2 drive;
    Intake intake;
    //WobbleGoal wg;
    DualShooterNoPID out;
    WobbleGoal wg;
    IMU imu;
    ButtonToggler btA;
    ButtonToggler btY;
    ButtonToggler btB;
    ButtonToggler btX;
    ButtonToggler btLB;
    boolean previousRBState = false;
    boolean previousLBState = false;


    long set1, set2, set3;
    ElapsedTime time = new ElapsedTime();
    ElapsedTime finishFlickerTime = new ElapsedTime();
    boolean flickerInAction = false;
    @Override
    public void init(){
        drive= new MecDrive2(hardwareMap);
        intake = new Intake(hardwareMap);
        out  =new DualShooterNoPID(hardwareMap);
        imu = new IMU(hardwareMap);
        imu.initializeIMU();
        rrDrive = new SampleMecanumDrive(hardwareMap);
        //rrDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rrDrive.getLocalizer().setPoseEstimate(startingPose);
        wg=new WobbleGoal(hardwareMap);
        btY = new ButtonToggler();
        btA = new ButtonToggler();
        btB = new ButtonToggler();
        btX=new ButtonToggler();
        btLB=new ButtonToggler();
        finishFlickerTime.startTime();
        wg.allBackArm();
    }

    @Override
    public void loop(){
        btA.ifRelease(gamepad1.a);
        btA.update(gamepad1.a);

        btX.ifRelease(gamepad1.x);
        btX.update(gamepad1.x);

        btB.ifRelease(gamepad1.b);
        btB.update(gamepad1.b);

        btY.ifRelease(gamepad1.y);
        btY.update(gamepad1.y);

        btLB.ifRelease(gamepad1.left_bumper);
        btLB.update(gamepad1.left_bumper);

        Pose2d currentPose = rrDrive.getLocalizer().getPoseEstimate();
        switch (currentMode){
            case DRIVER_CONTROL:
                Vector2d input = new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                ).rotated(-currentPose.getHeading());

                rrDrive.setWeightedDrivePower(
                        new Pose2d(
                                input.getX(),
                                input.getY(),
                                -gamepad1.right_stick_x
                        )
                );
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
        if(btLB.getMode()){
            if (currentMode == Mode.DRIVER_CONTROL)   {
                currentMode = Mode.AUTOMATED_MOVEMENT;
            }
        }
        else{
            currentMode=Mode.DRIVER_CONTROL;
        }

        rrDrive.update();
        telemetry.addData("Current Mode",currentMode);
        telemetry.addData("current pose",currentPose);
        telemetry.addData("prev state",btLB.getMode());

        if(btB.getMode()){
            hgOrPower = false;

        }
        else{
            hgOrPower = true;
        }

        if(btA.getMode()){
            intake.spinBoth();
        }
        else{
            intake.stop();
        }
        if(btY.getMode()){
            if(hgOrPower){
                out.spinHighGoal();
                telemetry.addData("Mode","High Goal");
            }
            else{
                out.spinPowershot();
                telemetry.addData("Mode","Powershot");
            }
        }
        else{
            out.stop();
        }
        if(gamepad1.dpad_left){
            telemetry.addData("Dpad left pressed",true);
            telemetry.update();
            out.pull();
        }
        else if(gamepad1.dpad_right){
            telemetry.addData("Dpad right pressed",true);
            telemetry.update();
            out.push();
        }

        if(gamepad1.left_trigger>0.2){
            wg.lift();
        }
        else if(gamepad1.right_trigger>0.2){
            wg.lower();
        }
        if(btX.getMode()){
            wg.grab();
        }
        else {
            wg.release();
        }

        telemetry.addData("leftX",gamepad1.left_stick_x);
        telemetry.addData("leftY",-gamepad1.left_stick_y);
        telemetry.addData("rightX",gamepad1.right_stick_x);
        telemetry.addData("rightY",gamepad1.right_stick_y);
        telemetry.update();
        telemetry.addData("imu",imu.getZAngle());
        if(previousRBState && !gamepad1.right_bumper){
            //RB was just released. Start flick
            flickerInAction = true;
            time.reset();
            out.pull();
        }
        if(flickerInAction && time.time()>0.25){
            out.push();
            flickerInAction=false;
        }
        previousRBState = gamepad1.right_bumper;

    }
}
