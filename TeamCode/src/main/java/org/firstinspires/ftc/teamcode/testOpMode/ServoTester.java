package org.firstinspires.ftc.teamcode.testOpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.CONFIG;
@TeleOp(name="servo test")
public class ServoTester extends OpMode {
    Servo servo;
    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, CONFIG.PUSH);
    }

    @Override
    public void loop() {
        if(gamepad1.x){
            servo.setPosition(0.4);
        }
        else if(gamepad1.y){
            servo.setPosition(1);
        }
    }
}
