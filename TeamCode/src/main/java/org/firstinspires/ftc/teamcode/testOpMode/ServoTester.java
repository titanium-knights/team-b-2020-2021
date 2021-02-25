package org.firstinspires.ftc.teamcode.testOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.CONFIG;
@Config
@TeleOp(name="servo test")
public class ServoTester extends LinearOpMode {
    public static double pos = 0;
    Servo servo;
    //double pos;

    public void initi() {
        servo = hardwareMap.get(Servo.class, CONFIG.PUSH);
        pos =  servo.getPosition();
    }

    @Override
    public void runOpMode() {
        initi();
        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {
            servo.setPosition(pos);
            telemetry.addData("Position", pos);
            telemetry.update();
        }
    }
}
