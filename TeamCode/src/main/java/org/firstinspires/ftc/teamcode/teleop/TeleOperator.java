package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Hardware;
import org.firstinspires.ftc.teamcode.utils.Outtake;
import org.firstinspires.ftc.teamcode.utils.WobbleGoal;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "")
public class TeleOperator extends LinearOpMode {
    @Override public void runOpMode() {
        DcMotor left = hardwareMap.dcMotor.get("left");
        DcMotor right = hardwareMap.dcMotor.get("right");

        WobbleGoal wobble = new WobbleGoal(hardwareMap);

//        Intake in = new Intake();
        Outtake out = new Outtake(hardwareMap);

        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.left_stick_button) {
                left.setPower(gamepad1.left_stick_x);
                right.setPower(gamepad1.left_stick_x);
            }

            if (gamepad1.a) {
                wobble.grab();
                wobble.lift();
            }
            if (gamepad1.b) {
                wobble.release();
                wobble.lower();
                wobble.stop();
            }

            if (gamepad1.x) {
                //intake
            }

            if (gamepad1.y) {
                //out.pushRing();
                out.setFlywheelSpeed(1);
                out.spin();
            }

        }
    }
}
