package org.firstinspires.ftc.teamcode.testOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.CONFIG;

@Disabled
@TeleOp(name= "motor finder")
public class MotorFinder extends OpMode {
    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;
    @Override
    public void init() {
        fl = hardwareMap.get(DcMotorEx.class,CONFIG.FRONTLEFT);
        fr = hardwareMap.get(DcMotorEx.class,CONFIG.FRONTRIGHT);
        bl = hardwareMap.get(DcMotorEx.class,CONFIG.BACKLEFT);
        br = hardwareMap.get(DcMotorEx.class,CONFIG.BACKRIGHT);
    }

    @Override
    public void loop() {
        if(gamepad1.dpad_down){
            br.setPower(0.5);
        }
        else{
            br.setPower(0);
        }

        if(gamepad1.dpad_up){
            fr.setPower(0.5);
        }
        else{
            fr.setPower(0);
        }

        if(gamepad1.dpad_left){
            fl.setPower(0.5);
        }
        else{
            fl.setPower(0);
        }

        if(gamepad1.dpad_right){
            bl.setPower(0.5);
        }
        else{
            bl.setPower(0);
        }
    }
}
