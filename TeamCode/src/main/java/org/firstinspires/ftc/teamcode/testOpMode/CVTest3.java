package org.firstinspires.ftc.teamcode.testOpMode;

import com.arcrobotics.ftclib.vision.UGContourRingDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.RingScanner;

@TeleOp
public class CVTest3 extends LinearOpMode {
    RingScanner rs;
    @Override
    public void runOpMode(){
        rs=new RingScanner(hardwareMap,telemetry);
        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            telemetry.addData("hgt",rs.getHeight());
        }
    }

}
