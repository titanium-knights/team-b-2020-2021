package org.firstinspires.ftc.teamcode.testOpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utils.ButtonToggler;
import org.firstinspires.ftc.teamcode.utils.Intake;

@TeleOp(name="December 11 Tester")
public class December11Test extends OpMode {
    ButtonToggler btA = new ButtonToggler();
    ButtonToggler btY = new ButtonToggler();
    Intake intake;
    @Override
    public void init() {
        intake = new Intake(hardwareMap);
    }

    @Override
    public void loop() {
        btA.ifRelease(gamepad1.a);
        btA.update(gamepad1.a);
        btY.ifRelease(gamepad1.y);
        btY.update(gamepad1.y);
        if (btA.getMode()) {
            intake.spin();
        }
        else {
            intake.stopUpper();
        }

        if (btY.getMode()) {
            intake.spinBottom();
        }
        else {
            intake.stopBottom();
        }
    }
}
