package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.testOpMode.TempShooter;
import org.firstinspires.ftc.teamcode.utils.ButtonToggler;
import org.firstinspires.ftc.teamcode.utils.IMU;
import org.firstinspires.ftc.teamcode.utils.Intake;
import org.firstinspires.ftc.teamcode.utils.MecDrive2;
import org.firstinspires.ftc.teamcode.utils.Outtake;
import org.firstinspires.ftc.teamcode.utils.Pusher;
import org.firstinspires.ftc.teamcode.utils.Shooter2;
import org.firstinspires.ftc.teamcode.utils.WobbleGoal;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp
public class TempAprilTeleField extends OpMode {
    public static double power=0.58;
    boolean hgOrPower= true; //True for high goal false for power shot
    MecDrive2 drive;
    Intake intake;
    //WobbleGoal wg;
    TempShooter out;
    IMU imu;
    Pusher pusher;
    ButtonToggler btA;
    ButtonToggler btY;
    ButtonToggler btB;
    ButtonToggler btX;
    boolean previousRBState = false;
    long set1, set2, set3;
    ElapsedTime time = new ElapsedTime();
    ElapsedTime finishFlickerTime = new ElapsedTime();
    boolean flickerInAction = false;
    FtcDashboard dashboard;
    @Override
    public void init(){
        drive= new MecDrive2(hardwareMap);
        intake = new Intake(hardwareMap);
        out  =new TempShooter(hardwareMap);
        imu = new IMU(hardwareMap);
        imu.initializeIMU();

        //wg=new WobbleGoal(hardwareMap);
        pusher = new Pusher(hardwareMap);
        btY = new ButtonToggler();
        btA = new ButtonToggler();
        btB = new ButtonToggler();
        btX=new ButtonToggler();
        finishFlickerTime.startTime();
        dashboard= FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
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


        drive.teleOpFieldCentric(gamepad1,imu);

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
    /*
        if(gamepad1.left_trigger>0.2){
            wg.setElevatorPower(gamepad1.left_trigger);
        }
        else if(gamepad1.right_trigger>0.2){
            wg.setElevatorPower(-gamepad1.right_trigger);
        }
        else{
            wg.stopElevator();
        }


        if(btX.getMode()){
            wg.grab();
        }
        else {
            wg.release();
        }
*/
        if(btY.getMode()){
            if(hgOrPower){
                //out.spinHighGoal();
                telemetry.addData("Mode","High Goal");
                out.spinWPower(power);
            }
            else{
                //out.spinPowershot();
                telemetry.addData("Mode","Powershot");
                out.spinWPower(power);
            }
        }
        else{
            out.stop();
        }
        if(gamepad1.dpad_left){
            telemetry.addData("Dpad left pressed",true);
            out.pull();
        }
        else if(gamepad1.dpad_right){
            telemetry.addData("Dpad right pressed",true);
            out.push();
        }
        telemetry.addData("leftX",gamepad1.left_stick_x);
        telemetry.addData("leftY",-gamepad1.left_stick_y);
        telemetry.addData("rightX",gamepad1.right_stick_x);
        telemetry.addData("rightY",gamepad1.right_stick_y);
        telemetry.addData("Velocity",out.getShooter().getVelocity());
        telemetry.addData("VelocityRPM",tickPSecToRPM(out.getShooter().getVelocity()));
        telemetry.update();
        //telemetry.addData("imu",imu.getZAngle());
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
    public double tickPSecToRPM(double tps){
        return (tps*1*60)/(28.0);
    }
}
