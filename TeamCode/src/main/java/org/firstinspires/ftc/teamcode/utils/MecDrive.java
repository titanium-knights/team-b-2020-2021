package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//import org.firstinspires.ftc.teamcode.utils.odometry.GlobalPosition;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class MecDrive {
    private double HEADING_THRESHOLD = 1;
    private double P_TURN_COEFF = 0.1;
    private String flName = CONFIG.FRONTLEFT;
    private String frName = CONFIG.FRONTRIGHT;
    private String blName = CONFIG.BACKLEFT;
    private String brName = CONFIG.BACKRIGHT;
    private IMU imu;
    //GlobalPosition gps;
    private DcMotor fl, fr,bl,br;

    /**
     * Function to initialize Mecanum Drive
     * @param hardwareMap Initializes the motors based on the name from the config file
     * @param usingEncoder If true, the wheels will be set to run using encoder and vice versa
     */
    public MecDrive(HardwareMap hardwareMap, boolean usingEncoder){
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
    public void setIMU(IMU imu){
        this.imu = imu;
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
    public void setPowerToLeftDrive(double power){
        this.fl.setPower(power);
        this.bl.setPower(power);
    }
    public void setPowerToRightDrive(double power){
        this.fr.setPower(power);
        this.br.setPower(power);
    }
    public void forwardWithPower(double power){
        if(Math.abs(power)>1){
            power = power>0 ? 1 : -1;
        }
        setPower(-power,-power,-power,-power);
    }
    public void backwardWithPower(double power){
        forwardWithPower(-power);
    }
    public void strafeLeftWithPower(double power){
        if(Math.abs(power)>1){
            power = power>0 ? 1 : -1;
        }
        setPower(power,-power,-power,power);
    }
    public DcMotor[] getMotors(){
        DcMotor[] arr = new DcMotor[4];
        arr[0] = this.fl;
        arr[1] = this.fr;
        arr[2] = this.bl;
        arr[3] = this.br;
        return arr;
    }
    public void turnRightXDegrees(IMU imu, double deg){
        double initAngle = imu.getZAngle();
        double setPoint = initAngle+180;
        //if(setPoint<)
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
    public void turnRightWithPower(double p){
        fl.setPower(p);
        bl.setPower(p);
        fr.setPower(-p);
        br.setPower(-p);
    }

    public void gyroTurn (double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (!onHeading(speed, angle, P_TURN_COEFF)) {
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while ((holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
        }

        // Stop all motion;
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        fl.setPower(leftSpeed);
        bl.setPower(leftSpeed);
        fr.setPower(rightSpeed);
        br.setPower(rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - imu.getZAngle();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
}
