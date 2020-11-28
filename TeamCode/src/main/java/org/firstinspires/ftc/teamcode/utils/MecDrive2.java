package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;

public class MecDrive2 {
    public DcMotorEx[] motors = new DcMotorEx[4];
    //FL,FR,BL,BR


    /**
     * @param hmap reference to OpMode's HardwareMap
     */
    public MecDrive2(HardwareMap hmap){
        motors[0] = hmap.get(DcMotorEx.class,CONFIG.FRONTLEFT);
        motors[1] = hmap.get(DcMotorEx.class,CONFIG.FRONTRIGHT);
        motors[2] = hmap.get(DcMotorEx.class,CONFIG.BACKLEFT);
        motors[3] = hmap.get(DcMotorEx.class,CONFIG.BACKRIGHT);
        for(DcMotorEx motor: motors){
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    public void setRawPowers(double fl, double fr, double bl, double br){
        double[] powers= {fl,fr,bl,br};
        setRawPowers(powers);
    }
    public void setRawPowers(double[] powers){
        int i=0;
        for(DcMotorEx motor:motors){
            motor.setPower(powers[i]);
            i++;
        }
    }
    public void setPower(double x, double y, double rot){
        double[] power={
                y-x-rot,//fl
                y+x+rot,//fr
                y+x-rot,//bl
                y-x+rot//br
        };
        double max = Math.max(Math.abs(Arrays.stream(power).max().getAsDouble()),Math.abs(Arrays.stream(power).min().getAsDouble()));
        if(max>1){
            for(int i=0;i<4;i++){
                power[i]/=max;
            }
        }
        setRawPowers(power);
    }
    public void setPower(Vector2D v){
        setPower(v.x,v.y,0);
    }
    public void setPower(Pose2D p){
        setPower(p.x,p.y,p.rot);
    }
    public int[] getEncoderVals(){
        return new int[]{motors[0].getCurrentPosition(), motors[1].getCurrentPosition(), motors[2].getCurrentPosition(), motors[3].getCurrentPosition()};
    }
    public void setTargetPosition(int[] pos){
        for(int i=0;i<4;i++){
            motors[i].setTargetPosition(pos[i]);
        }
    }
    public void setMode(DcMotorEx.RunMode mode){
        for(DcMotorEx motor: motors){
            motor.setMode(mode);
        }
    }
    public void teleOp(Gamepad g1){
        setPower(g1.left_stick_x,-g1.left_stick_y,g1.right_stick_x);
    }
    public boolean allMotorsBusy(){
        boolean a= true;
        for(DcMotorEx motor: motors){
            a=a&&motor.isBusy();
        }
        return a;
    }
    public void stop(){
        setRawPowers(0,0,0,0);
    }
}
