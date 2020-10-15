package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "gab")
public class TestOpMode2 extends LinearOpMode {
    @Override public void runOpMode() {
        DcMotor left = hardwareMap.dcMotor.get("left");
        DcMotor right = hardwareMap.dcMotor.get("right");

        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            left.setPower(gamepad1.left_stick_x);
            right.setPower(gamepad1.right_stick_y);
            if (gamepad1.a) {
                left.setPower(gamepad1.left_stick_x * 2);
                right.setPower(gamepad1.right_stick_y * 2);
            }
            if (gamepad1.b) {
                if ((int) (Math.random()) < 1) {
                    left.setPower(gamepad1.left_stick_x * -1);
                }
                else {
                    right.setPower(gamepad1.right_stick_y * -1);
                }
            }
        }
    }
}