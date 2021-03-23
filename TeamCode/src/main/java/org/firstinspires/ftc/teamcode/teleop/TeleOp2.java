package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.utils.Intake;
import org.firstinspires.ftc.teamcode.utils.Outtake;
import org.firstinspires.ftc.teamcode.utils.WobbleGoal;
@Disabled

@TeleOp(name="dual drive")
public class TeleOp2 extends LinearOpMode {
    @Override public void runOpMode() {
        Intake intake = new Intake(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);
        WobbleGoal wg = new WobbleGoal(hardwareMap);

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
            while(gamepad2.a) {
                intake.spin();
            }
            intake.stop();
            if (gamepad2.b) {
                //outtake.pushRing();
                outtake.setFlywheelSpeed(gamepad2.left_stick_y);
                outtake.spin();
            }

            if (gamepad2.x) {
                wg.grab();
            }
            if (gamepad2.right_stick_button) {
                if (gamepad2.right_stick_y > 0)
                    wg.lift();
                else
                    wg.lower();
            }
            if (gamepad2.y) {
                wg.release();
                wg.stop();
            }
        }

    }
}
