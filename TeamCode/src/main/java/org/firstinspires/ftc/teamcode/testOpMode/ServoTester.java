package org.firstinspires.ftc.teamcode.testOpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.CONFIG;
@TeleOp(name="servo test")
public class ServoTester extends LinearOpMode {
    Servo servo;
    double pos;

    public void initi() {
        servo = hardwareMap.get(Servo.class, CONFIG.PUSH);
        pos =  servo.getPosition();
    }

    @Override
    public void runOpMode() {
        initi();
        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {
            if (gamepad1.dpad_up) {
                pos += .1;
                servo.setPosition(pos);

                sleep(150);
            } else if (gamepad1.dpad_down) {
                pos -= .1;
                servo.setPosition(pos);

                sleep(150);
            }
            telemetry.addData("Position", pos);
            telemetry.update();
        }
    }
}
