package org.firstinspires.ftc.teamcode.testOpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.ButtonToggler;
import org.firstinspires.ftc.teamcode.utils.CONFIG;
import org.firstinspires.ftc.teamcode.utils.Intake;
import org.firstinspires.ftc.teamcode.utils.MecDrive;
import org.firstinspires.ftc.teamcode.utils.Outtake;

@TeleOp(name="Intake and Shooter test",group = "test")
public class IntakeShooterServoTester extends OpMode {
    Intake intake;
    ButtonToggler buttonA;
    ButtonToggler buttonY;
    ButtonToggler buttonX;
    Servo servo;
    Outtake outtake;
    @Override
    public void init() {
        intake = new Intake(hardwareMap);
        servo = hardwareMap.get(Servo.class, CONFIG.PUSH);
        buttonA = new ButtonToggler();
        buttonY = new ButtonToggler();
        buttonX = new ButtonToggler();
        //servo.setDirection(CRServo.Direction.REVERSE);
        outtake = new Outtake(hardwareMap);
    }

    @Override
    public void loop() {
        buttonA.ifRelease(gamepad1.a);
        buttonA.update(gamepad1.a);
        buttonY.ifRelease(gamepad1.y);
        buttonY.update(gamepad1.y);
        buttonX.ifRelease(gamepad1.x);
        buttonX.update(gamepad1.x);
        if(gamepad1.a){
            intake.spin();
        }
        else{
            intake.stop();
        }
        if(buttonY.getMode()){
            outtake.spin();
        }
        else{
            outtake.stop();
        }
        if(gamepad1.dpad_up){
            servo.setPosition(0);
        }
        else if(gamepad1.dpad_down){
            servo.setPosition(1);
        }
        else if(gamepad1.dpad_right){
            //servo.setPosition(0);
        }
        //telemetry.addData("Servo Pos",servo.getPosition());
        telemetry.update();
    }
}
