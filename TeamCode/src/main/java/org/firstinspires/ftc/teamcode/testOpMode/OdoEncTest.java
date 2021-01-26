package org.firstinspires.ftc.teamcode.testOpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.CONFIG;
@TeleOp(name="OdoEncTest")
public class OdoEncTest extends LinearOpMode {
    DcMotorEx l;
    DcMotorEx r;
    DcMotorEx h;
    public void initi(){
        l = hardwareMap.get(DcMotorEx.class, CONFIG.LEFTVERTICAL);
        r = hardwareMap.get(DcMotorEx.class, CONFIG.RIGHTVERTICAL);
        h = hardwareMap.get(DcMotorEx.class, CONFIG.HORIZONTAL);
    }
    @Override
    public void runOpMode(){
        initi();
        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {
            telemetry.addData("left", l.getCurrentPosition());
            telemetry.addData("right", r.getCurrentPosition());
            telemetry.addData("horizontal", h.getCurrentPosition());
            telemetry.update();
        }
    }
}
