package org.firstinspires.ftc.teamcode.testOpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.CONFIG;

@TeleOp(name="Test Front&Left Distance", group = "Test")
public class TestDistanceSensors extends LinearOpMode {
    DistanceSensor front;
    DistanceSensor right;
    DistanceSensor left;
    @Override
    public void runOpMode(){
        front = hardwareMap.get(DistanceSensor.class, CONFIG.FRONTDIST);
        left = hardwareMap.get(DistanceSensor.class, CONFIG.LEFTDIST);
        //right = hardwareMap.get(DistanceSensor.class,CONFIG.RIGHTDIST);
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("front", front.getDistance(DistanceUnit.INCH));
            telemetry.addData("left", left.getDistance(DistanceUnit.INCH));
            //telemetry.addData("right",right.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }

    }
}
