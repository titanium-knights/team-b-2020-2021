package org.firstinspires.ftc.teamcode.testOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.Outtake;

@TeleOp
@Config
public class ShooterEncoderTest extends OpMode {
    Outtake out;
    public static double speed=0.0;
    @Override
    public void init() {
        out = new Outtake(hardwareMap);
    }

    @Override
    public void loop() {
        out.setVelocityRPM(speed);
    }
}
