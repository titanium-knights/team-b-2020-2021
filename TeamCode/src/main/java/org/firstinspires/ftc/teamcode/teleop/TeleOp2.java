package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="dual drive")
public class TeleOp2 extends LinearOpMode {
    @Override public void runOpMode() {
        DcMotor fl = hardwareMap.dcMotor.get("fl");
        DcMotor fr = hardwareMap.dcMotor.get("fr");
        DcMotor bl = hardwareMap.dcMotor.get("bl");
        DcMotor br = hardwareMap.dcMotor.get("br");

        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()) {
            //gamepad 1 for movement
            if (gamepad1.left_stick_button) {
                fl.setPower(gamepad1.left_stick_y);
                bl.setPower(gamepad1.left_stick_y);
            }
            if (gamepad1.right_stick_button) {
                fr.setPower(gamepad1.right_stick_y);
                br.setPower(gamepad1.right_stick_y);
            }

            //gamepad 2 for functions
            if (gamepad2.a) {

            }
        }

    }
}
