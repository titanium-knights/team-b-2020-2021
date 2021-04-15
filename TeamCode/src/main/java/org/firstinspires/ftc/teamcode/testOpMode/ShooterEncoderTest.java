package org.firstinspires.ftc.teamcode.testOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.CONFIG;
import org.firstinspires.ftc.teamcode.utils.Outtake;

@TeleOp
@Config
public class ShooterEncoderTest extends OpMode {
    DcMotorEx motor,motor2;
    public static double speed=0.0;
    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, CONFIG.SHOOTER);
        motor2 = hardwareMap.get(DcMotorEx.class, CONFIG.SHOOTER2);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        telemetry.addData("motor 1pos",motor.getCurrentPosition());
        telemetry.addData("motor 2pos",motor2.getCurrentPosition());
        telemetry.update();
    }
}
