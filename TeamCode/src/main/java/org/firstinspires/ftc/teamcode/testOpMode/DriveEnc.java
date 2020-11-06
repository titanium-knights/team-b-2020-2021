package org.firstinspires.ftc.teamcode.testOpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Encoder Test")
public class DriveEnc extends LinearOpMode {
    DcMotorEx fl,fr,bl,br;
    @Override
    public void runOpMode() throws InterruptedException {
        fl = hardwareMap.get(DcMotorEx.class,"fl");
        fr = hardwareMap.get(DcMotorEx.class,"fr");
        bl = hardwareMap.get(DcMotorEx.class,"bl");
        br = hardwareMap.get(DcMotorEx.class,"br");
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("fl", fl.getCurrentPosition());
            telemetry.addData("fr", fr.getCurrentPosition());
            telemetry.addData("bl", bl.getCurrentPosition());
            telemetry.addData("br", br.getCurrentPosition());
            telemetry.update();
        }
    }

}

