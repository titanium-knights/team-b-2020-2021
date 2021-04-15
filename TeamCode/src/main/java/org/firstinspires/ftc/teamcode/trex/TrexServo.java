package org.firstinspires.ftc.teamcode.trex;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class TrexServo extends LinearOpMode {
    Servo normalServo;
    CRServo continousRotationServo;

    @Override
    public void runOpMode() throws InterruptedException {
        //There are 2 types servos: continuous rotation and normal servo.

        //Normal Servo
        normalServo = hardwareMap.get(Servo.class, "servo1");
        //normal servos

        //Start off at 0 degrees
        normalServo.setPosition(0);
        sleep(250);//we need to wait 250 milliseconds to get to that position otherwise it might just get nullified
        normalServo.setPosition(1);
        sleep(250);//we need to wait 250 milliseconds to get to that position otherwise it might just get nullified


        normalServo.setPosition(0.5);
        sleep(250);//we need to wait 250 milliseconds to get to that position otherwise it might just get nullified


        normalServo.setPosition(0.25);
        sleep(250);//we need to wait 250 milliseconds to get to that position otherwise it might just get nullified

        normalServo.setPosition(0.75);
        sleep(250);//we need to wait 250 milliseconds to get to that position otherwise it might just get nullified

        telemetry.addData("Servo is currently at: ", normalServo.getPosition());
        telemetry.update();



        //CR Servo
        continousRotationServo = hardwareMap.get(CRServo.class, "servo2");
        continousRotationServo.setPower(-1);
        sleep(500);
        continousRotationServo.setPower(1);
        sleep(500);
        continousRotationServo.setPower(0);
        telemetry.addData("CR Servo is currently going at this power:",continousRotationServo.getPower());
        telemetry.update();
    }

}
