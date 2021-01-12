package org.firstinspires.ftc.teamcode.testOpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.StateMachine;
import org.firstinspires.ftc.teamcode.utils.PoseStorageManager;
import org.firstinspires.ftc.teamcode.utils.RRQuickStart.drive.StandardTrackingWheelLocalizer;

import java.io.IOException;

public class test extends LinearOpMode {
    DcMotor a;
    PoseStorageManager psm;
    StandardTrackingWheelLocalizer myLocalizer ;
    public test(HardwareMap hm){
        a = hm.get(DcMotor.class, "a");
    }

    @Override
    public void runOpMode() throws InterruptedException {
        myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap);
        try {
            psm = new PoseStorageManager();
        } catch (IOException e) {
            e.printStackTrace();
        }
        waitForStart();
        if(psm != null){
            try {
                psm.setPose(myLocalizer.getPoseEstimate());
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }


}
