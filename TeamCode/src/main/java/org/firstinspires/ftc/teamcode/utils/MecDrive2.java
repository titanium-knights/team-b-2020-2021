package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;

public class MecDrive2 {
    public DcMotorEx[] motors = new DcMotorEx[4];
    //FL,FR,BL,BR


    /**
     * @param hmap reference to OpMode's HardwareMap
     */
    public MecDrive2(HardwareMap hmap){
        motors[0] = hmap.get(DcMotorEx.class,CONFIG.FRONTLEFT);
        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[1] = hmap.get(DcMotorEx.class,CONFIG.FRONTRIGHT);
        motors[2] = hmap.get(DcMotorEx.class,CONFIG.BACKLEFT);
        motors[2].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[3] = hmap.get(DcMotorEx.class,CONFIG.BACKRIGHT);
        for(DcMotorEx motor: motors){
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            //motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            //motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    /**
     * Sets raw power values to all four wheels
     * @param fl power applied to front left motor
     * @param fr power applied to front right motor
     * @param bl power applied to back left motor
     * @param br power applied to back right motor
     */
    public void setRawPowers(double fl, double fr, double bl, double br){
        double[] powers= {fl,fr,bl,br};
        setRawPowers(powers);
    }

    /**
     * Sets raw power values to all four wheels via an array
     * @param powers array of powers with the following indices [fl,fr,bl,br]
     */
    public void setRawPowers(double[] powers){
        int i=0;
        for(DcMotorEx motor:motors){
            motor.setPower(powers[i]);
            i++;
        }
    }

    /**
     * Uses Mecanum kinematic equations to find power for all 4 motors based on x y and rotational powers required
     * @param x power in the horizontal or x direction
     * @param y power in the vertical or y direction
     * @param rot rotational power
     */
    public void setPower(double x, double y, double rot, double speedModifier){
        double[] power={
                y+x+rot,//fl
                y-x-rot,//fr
                y-x+rot,//bl
                y+x-rot//br
        };
        for(int i=0;i<4;i++){
            power[i]*=speedModifier;
        }
        double max = Math.max(Math.abs(Arrays.stream(power).max().getAsDouble()),Math.abs(Arrays.stream(power).min().getAsDouble()));
        if(max>1){
            for(int i=0;i<4;i++){
                power[i]/=max;
            }
        }

        setRawPowers(power);
    }
    public void setPower(double x, double y, double rot){
        setPower(x,y,rot,1);
    }

    /**
     * A method to get a double array of length 4 of powers to all 4 motors
     * @param x power in the x direction
     * @param y power in the y direction
     * @param rot rotational power
     * @return double array of powers for all 4 motors.
     */
    public double[] getPowerArr(double x, double y, double rot){//1 0 0
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
        return power;
    }
    /**
     * Sets Power using the setPower(double x, double y, double rot) method but takes in x and y powers through a vector data type. assumes rot is 0
     * @param v Vector2D of x and y powers
     */
    public void setPower(Vector2D v){
        setPower(v.x,v.y,0);
    }
    /**
     * Sets Power using the setPower(double x, double y, double rot) method but takes in x and y powers through a Pose data type
     * @param p Pose2D of x, y,and Rot powers
     */
    public void setPower(Pose2D p){
        setPower(p.x,p.y,p.rot);
    }

    /**
     * Get an array of all encoder values as an array
     * @return Encoder values in form [fl,fr,bl,br]
     */
    public int[] getEncoderVals(){
        return new int[]{motors[0].getCurrentPosition(), motors[1].getCurrentPosition(), motors[2].getCurrentPosition(), motors[3].getCurrentPosition()};
    }

    /**
     * Sets position of all drive encoders based on input array
     * @param pos array of all target encoder positions
     */
    public void setTargetPosition(int[] pos){
        for(int i=0;i<4;i++){
            motors[i].setTargetPosition(pos[i]);
        }
    }

    /**
     * Sets modes for all Drive Motors at once
     * @param mode The mode that all Drive Motors need to be set to
     */
    public void setMode(DcMotorEx.RunMode mode){
        for(DcMotorEx motor: motors){
            motor.setMode(mode);
        }
    }

    /**
     * Easy teleop function which only needs a gamepad to be passed
     * @param g1 gamepad1 or gamepad that controls driving.
     */
    public void teleOpRobotCentric(Gamepad g1){
        setPower(g1.left_stick_x,-g1.left_stick_y,g1.right_stick_x);
    }
    public void teleOpRobotCentric(Gamepad g1, Telemetry telemetry){
        setPower(g1.left_stick_x,-g1.left_stick_y,g1.right_stick_x);
        telemetry.addData("Called setPower","");
        telemetry.update();
    }
    public void teleOpRobotCentric(Gamepad g1, double speedModifier){
        setPower(g1.left_stick_x,-g1.left_stick_y,g1.right_stick_x, speedModifier);
    }

    public void teleOpFieldCentric(Gamepad g1, IMU imu){
        double angle = Math.toRadians(imu.getZAngle());
        double inputY = -g1.left_stick_y;
        double inputX = g1.left_stick_x;
        double rot = g1.right_stick_x;
        double x = Math.cos(angle) * inputX - Math.sin(angle) * inputY;
        double y = Math.sin(angle) * inputX + Math.cos(angle) * inputY;
        setPower(x,y,rot);
    }

    public void teleOpFieldCentricDeg(Gamepad g1, double deg){
        double angle = Math.toRadians(deg);
        double inputY = -g1.left_stick_y;
        double inputX = g1.left_stick_x;
        double rot = g1.right_stick_x;
        double x = Math.cos(angle) * inputX - Math.sin(angle) * inputY;
        double y = Math.sin(angle) * inputX + Math.cos(angle) * inputY;
        setPower(x,y,rot);
    }
    public void teleOpFieldCentricRad(Gamepad g1, double rad){
        double angle = rad;
        double inputY = -g1.left_stick_y;
        double inputX = g1.left_stick_x;
        double rot = g1.right_stick_x;
        double x = Math.cos(angle) * inputX - Math.sin(angle) * inputY;
        double y = Math.sin(angle) * inputX + Math.cos(angle) * inputY;
        setPower(x,y,rot);
    }
    /**
     * Method to check if all motors are busy at a given point
     * @return A boolean if all motors are busy
     */
    public boolean allMotorsBusy(){
        boolean a= true;
        for(DcMotorEx motor: motors){
            a=a&&motor.isBusy();
        }
        return a;
    }

    /**
     * Stops all motors
     */
    public void stop(){
        setRawPowers(0,0,0,0);
    }

    /**
     * Sets power to dt such that it will turn right
     * @param power power that dt should turn right at
     */
    public void turnRightWithPower(double power){
        setPower(new Pose2D(0,0,power));
    }
    /**
     * Sets power to dt such that it will turn left
     * @param power power that dt should turn left at
     */
    public void turnLeftWithPower(double power){
        setPower(new Pose2D(0,0,-power));
    }
    /**
     * Sets power to dt such that it will strafe right
     * @param power power that dt should strafe right at
     */
    public void strafeRightWithPower(double power){
        setPower(new Pose2D(-power,0,0));
    }
    /**
     * Sets power to dt such that it will strafe left
     * @param power power that dt should strafe left at
     */
    public void strafeLeftWithPower(double power){
        setPower(new Pose2D(power,0,0));
    }
    /**
     * Sets power to dt such that it will drive forward
     * @param power power that dt should drive forward at
     */
    public void forwardWithPower(double power){
        setPower(new Pose2D(0,power,0));
    }
    /**
     * Sets power to dt such that it will drive backwards
     * @param power power that dt should drive backwards at
     */
    public void backwardWithPower(double power){
        setPower(new Pose2D(0,-power,0));
    }

    public void gyroTurn(double angle, double marginOfError, IMU imu){
        double target = angle+imu.getZAngle();
        double error = target - imu.getZAngle();
        double Kp = 0.04;
        double leftPow;
        double rightPow;

        while((Math.abs(error)>marginOfError) && !Thread.currentThread().interrupted())
        {
            error = imu.getZAngle() - target;
            leftPow = error * Kp;
            rightPow = -error * Kp;
            setRawPowers(leftPow, rightPow,leftPow, rightPow);
        }
        stop();
    }
}
