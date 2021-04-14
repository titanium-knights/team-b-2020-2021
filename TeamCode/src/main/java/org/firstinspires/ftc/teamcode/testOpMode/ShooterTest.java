package org.firstinspires.ftc.teamcode.testOpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utils.CONFIG;

import static java.lang.Thread.sleep;

@TeleOp
public class ShooterTest extends LinearOpMode {
    DcMotor m1,m2;

    public void initi() {
        m1=hardwareMap.get(DcMotor.class, CONFIG.SHOOTER);
        m2=hardwareMap.get(DcMotor.class, CONFIG.SHOOTER);
    }

    @Override
    public void runOpMode() {
        initi();
        waitForStart();
        m1.setPower(1);
        sleep(2000);
        m1.setPower(0);
        sleep(3000);
        m2.setPower(1);
        sleep(2000);
        m2.setPower(0);


    }
}
