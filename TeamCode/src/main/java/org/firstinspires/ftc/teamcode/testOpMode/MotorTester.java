package org.firstinspires.ftc.teamcode.testOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utils.ButtonToggler;

@Disabled
@TeleOp(name = "19.2 full speed", group="test")
public class MotorTester extends LinearOpMode {
    DcMotor motor;
    ButtonToggler buttonA;
    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotor.class,"shooter");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        buttonA = new ButtonToggler();
        waitForStart();
        while(opModeIsActive()) {
            buttonA.ifRelease(gamepad1.y);
            buttonA.update(gamepad1.y);
            if(buttonA.getMode()){
                motor.setPower(1);
            }
            else{
                motor.setPower(0);
            }
        }

    }
}
