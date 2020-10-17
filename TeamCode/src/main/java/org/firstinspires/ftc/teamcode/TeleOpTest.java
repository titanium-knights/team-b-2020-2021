package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Tele Op Test",  group = "Tests and Experiments")
public class TeleOpTest extends LinearOpMode {
    @Override public void runOpMode() {
        DcMotor left = hardwareMap.dcMotor.get("left");
        DcMotor right = hardwareMap.dcMotor.get("right");

        waitForStart();

        while(opModeIsActive()) {
            double stick = gamepad1.left_stick_y;
            left.setPower(stick);

            if(gamepad2.x) {
                right.setPower(1);
            }
            else {
                right.setPower(0);
            }

            if(gamepad2.y) {
                right.setPower(1);
            }
            else {
                right.setPower(0);
            }

            if(gamepad2.x && gamepad2.y) {
                right.setPower(1);
                left.setPower(1);
            }
            else {
                right.setPower(0);
                left.setPower(0);
            }
        }
    }
}
