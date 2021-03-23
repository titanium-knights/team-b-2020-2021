package org.firstinspires.ftc.teamcode.testOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.MecDrive2;
import org.firstinspires.ftc.teamcode.utils.WobbleGoal;

@Disabled
@Config
@TeleOp(name = "WGTest")
public class WGTest extends OpMode {
    public static double position =0;
    WobbleGoal wg;
    MecDrive2 drive;
    @Override
    public void init() {
        wg = new WobbleGoal(hardwareMap);
        drive = new MecDrive2(hardwareMap);
    }

    @Override
    public void loop() {
        wg.setElevatorPower(gamepad1.left_stick_y);
        wg.setServoPos(position);
        telemetry.update();
    }
}
