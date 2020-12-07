package org.firstinspires.ftc.teamcode.utils.trex;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class TrexTeleOp extends OpMode {
    MDrive drive;
    @Override
    public void init() {
        MDrive drive = new MDrive(hardwareMap);
    }

    @Override
    public void loop() {
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double rot = gamepad1.right_stick_x;
        drive.driveXYRot(x,y,rot);
    }
}
