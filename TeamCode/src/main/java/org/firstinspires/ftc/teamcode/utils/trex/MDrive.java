package org.firstinspires.ftc.teamcode.utils.trex;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;

public class MDrive {
    //Declaring 4 variables indicated that there are 4 drive motors
    //fl= front left
    //fr = front right
    //bl = back left
    //br = back right
    DcMotor fl, fr, bl, br;


    public MDrive(HardwareMap hmap){
        fl = hmap.dcMotor.get("fl");
        fr = hmap.dcMotor.get("fr");
        bl = hmap.dcMotor.get("bl");
        br = hmap.dcMotor.get("br");
    }

    //[flPower, frPower, blPower, brPower]
    double[] powerArr = new double[4];

    public void driveXYRot(double x, double y, double rot){
        powerArr[0] = x+y+rot; //fl power is x+y+rot
        powerArr[1] = y-x-rot; //Fr POwer is y-x-rot
        powerArr[2] = y-x+rot;//blPower is y-x+rot
        powerArr[3] = y+x-rot; //brPower is y+x-rot
        double max = Math.max(Math.abs(Arrays.stream(powerArr).max().getAsDouble()),Math.abs(Arrays.stream(powerArr).min().getAsDouble()));
        //Finds either the lowest number or the higheest number and takes absolute value
        if(max>1){
            //We have to normalize
            for(int i=0;i<4;i++){
                powerArr[i]=powerArr[i]/max;
            }
        }
        fl.setPower(powerArr[0]);
        fr.setPower(powerArr[1]);
        bl.setPower(powerArr[2]);
        br.setPower(powerArr[3]);
    }

    /*
    driveForwardWithPower
    driveBackwardWithPower
    strafeLeftWithPower
    strafeRightWithPower

    turnLeftWithPower
    turnRightWithPower
    stop

     */
    public void driveForwardsWithPower(double power){
        driveXYRot(0,power,0);
    }
    public void driveBackwardsWithPower(double power){
        driveXYRot(0,power,0);

    }
    public void strafeLeftWithPower(double power){
        driveXYRot(-power,0,0);
    }
    public void strafeRightWithPower(double power){
        driveXYRot(power,0,0);
    }
    public void turnLeftWithPower(double power){
        driveXYRot(0,0,-power);
    }
    public void turnRightWithPower(double power){
        driveXYRot(0,0, power);
    }
    public void stop(){
        driveXYRot(0,0,0);
    }
}
