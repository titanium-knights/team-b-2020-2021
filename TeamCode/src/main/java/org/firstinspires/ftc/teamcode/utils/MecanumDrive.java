package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

//import org.firstinspires.ftc.teamcode.utils.odometry.GlobalPosition;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class MecanumDrive {
    private String flName = CONFIG.FRONTLEFT;
    private String frName = CONFIG.FRONTRIGHT;
    private String blName = CONFIG.BACKLEFT;
    private String brName = CONFIG.BACKRIGHT;
    //GlobalPosition gps;
    private DcMotor fl, fr,bl,br;

    /**
     * Function to initialize Mecanum Drive
     * @param hardwareMap Initializes the motors based on the name from the config file
     * @param usingEncoder If true, the wheels will be set to run using encoder and vice versa
     */
    public MecanumDrive(HardwareMap hardwareMap, boolean usingEncoder){
        //gps = new GlobalPosition(hardwareMap,75);
        //GlobalPosition gps = new GlobalPosition(hardwareMap,75);
        //Thread pos = new Thread(gps);
        //pos.start();

        this.fl = hardwareMap.get(DcMotor.class, flName);
        this.fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.fr = hardwareMap.get(DcMotor.class, frName);
        this.fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.fr.setDirection(DcMotorSimple.Direction.REVERSE);

        this.bl = hardwareMap.get(DcMotor.class, blName);
        this.bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.br = hardwareMap.get(DcMotor.class, brName);
        this.br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.br.setDirection(DcMotorSimple.Direction.REVERSE);
        if(usingEncoder){
            this.fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else{
            this.fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void setRunMode(DcMotor.RunMode runMode){
        this.fl.setMode(runMode);
        this.fr.setMode(runMode);
        this.bl.setMode(runMode);
        this.br.setMode(runMode);
    }

    public void setPower(double fl, double fr, double bl, double br){
        this.fl.setPower(fl);
        this.fr.setPower(fr);
        this.bl.setPower(bl);
        this.br.setPower(br);
    }
    public void forwardWithPower(double power){
        if(Math.abs(power)>1){
            power = power>0 ? 1 : -1;
        }
        setPower(power,power,power,power);
    }
    public void backwardWithPower(double power){
        forwardWithPower(-power);
    }
    public void strafeLeftWithPower(double power){
        if(Math.abs(power)>1){
            power = power>0 ? 1 : -1;
        }
        setPower(-power,power,power,-power);
    }
    public void strafeRightWithPower(double power){
        strafeLeftWithPower(-power);
    }
    public double[] calculateSpeedsJoysticks(double leftX,double leftY, double rightX){
        double wheelPower, stickAngleRadians, rightx, flPower, frPower, blPower,brPower, sinAngle,cosAngle,factor;
        wheelPower = Math.hypot(leftX,leftY);
        stickAngleRadians= Math.atan2(leftY,leftX);
        stickAngleRadians-=(Math.PI/4.0);
        sinAngle=Math.sin(stickAngleRadians);
        cosAngle= Math.cos(stickAngleRadians);
        factor = 1 / Math.max(Math.abs(sinAngle), Math.abs(cosAngle));
        rightx = rightX*.5;
        flPower = wheelPower * cosAngle * factor + rightX;
        frPower = wheelPower * sinAngle * factor - rightX;
        blPower = wheelPower * sinAngle * factor + rightX;
        brPower = wheelPower * cosAngle * factor - rightX;
        double[] wheelPowers = {flPower,frPower,blPower,brPower};
        return  wheelPowers;
    }

    public void joystickTeleop(Gamepad gamepad1, double speedEnhancer){
        double motorMax=1;
        double X1,X2,Y1,Y2;
        double LF,RF,LR,RR;
        LF=RF=LR=RR=0;
        Y1 = -gamepad1.right_stick_y * speedEnhancer; // invert so up is positive
        X1 = gamepad1.right_stick_x * speedEnhancer;
        Y2 = -gamepad1.left_stick_y * speedEnhancer; // Y2 is not used at present
        X2 = gamepad1.left_stick_x * speedEnhancer;
        // Forward/back movement
        LF += Y1; RF += Y1; LR += Y1; RR += Y1;

        // Side to side movement
        LF += X1; RF -= X1; LR -= X1; RR += X1;

        // Rotation movement
        LF += X2; RF -= X2; LR += X2; RR -= X2;
        LF = Math.max(-motorMax, Math.min(LF, motorMax));
        RF = Math.max(-motorMax, Math.min(RF, motorMax));        // Send values to the motors

        LR = Math.max(-motorMax, Math.min(LR, motorMax));
        RR = Math.max(-motorMax, Math.min(RR, motorMax));

        this.fl.setPower(LF);
        this.fr.setPower(RF);
        this.bl.setPower(LR);
        this.br.setPower(RR);
    }
    public void teleopTank(Gamepad g1){
        //default value of 1 for speedModifier()
        teleopTank(g1,1);
    }
    public void teleopTank(Gamepad g1,double speedModifier){
        double right = g1.right_stick_y*speedModifier;
        double left = g1.left_stick_y*speedModifier;
        double strafeLeft =g1.left_trigger;
        double strafeRight = g1.right_trigger;
        if(Math.abs(left)>0.1 || Math.abs(right)>0.1){
            //significant movement in joystick y direction
            this.fl.setPower(left);
            this.bl.setPower(left);
            this.fr.setPower(right);
            this.br.setPower(right);
            //This is tank drive. Left joystick controls left wheels & Right joystick controls right wheels
        }
        else{
            //No significant joystick movements
            stop();
        }
        if(strafeRight>0.1){
            //Strafe Right means means we want front left to go forwards and back right to go forwards while all others go backwards
            this.fl.setPower(-strafeRight);
            this.bl.setPower(strafeRight);
            this.fr.setPower(strafeRight);
            this.br.setPower(-strafeRight);
        }
        else if(strafeLeft>0.1){
            //Strafe Right means means we want front left to go forwards and back right to go forwards while all others go backwards
            this.fl.setPower(strafeLeft);
            this.bl.setPower(-strafeLeft);
            this.fr.setPower(-strafeLeft);
            this.br.setPower(strafeLeft);
        }

    }
    public void fieldCentricDrive(Gamepad gamepad1, double angle){
        double forward = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        angle = Math.toRadians(angle);
        double x = forward*Math.cos(angle) + strafe*Math.sin(angle);
        double y = -forward*Math.sin(angle)+strafe*Math.cos(angle);
        XYCorrection(x,y,turn);
    }
    public void stop(){
        setPower(0.0,0.0,0.0,0.0);
    }
    public void XYCorrection(double x, double y, double turn){
        double flPower = y + turn + x;
        double blPower = y + turn - x;
        double frPower = y - turn - x;
        double brPower = y - turn + x;
        List<Double> list = Arrays.asList(a(flPower), a(blPower), a(frPower), a(brPower));
        Double max = Collections.max(list);
        if (max > 1.0) {
            flPower /= max;
            blPower /= max;
            frPower /= max;
            brPower /= max;
        }
        this.fl.setPower(flPower);
        this.bl.setPower(blPower);
        this.fr.setPower(frPower);
        this.br.setPower(brPower);

    }
    public double a(double b){
        return Math.abs(b);
    }   
}
