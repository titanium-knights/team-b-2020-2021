package org.firstinspires.ftc.teamcode.testOpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.WobbleGoal;

@TeleOp
public class WgArmTester extends LinearOpMode {
    WobbleGoal wg;
    @Override
    public void runOpMode(){
        wg = new WobbleGoal(hardwareMap);
        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {
            wg.grab();
        }
    }
}
