package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "test1", group = "tests")
public class TeleOpTest extends LinearOpMode {
    @Override public void runOpMode() {
        DcMotor left = hardwareMap.dcMotor.get("left");
        DcMotor right = hardwareMap.dcMotor.get("right");

        waitForStart();

        while (opModeIsActive()) {
            double stick = gamepad1.left_stick_y;
            left.setPower(stick);
            right.setPower(stick);

//            int pos = (int) (gamepad1.left_stick_x);
//            left.setTargetPosition(pos);
//            right.setTargetPosition(pos);

            if (gamepad1.x) {
                left.setPower(0);
            }

            if (gamepad1.y) {
                right.setPower(0);
            }

            if (gamepad1.x && gamepad1.y) {
                left.setPower(0);
                right.setPower(0);
                break;
            }
        }
    }
}
