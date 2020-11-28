package org.firstinspires.ftc.teamcode.testOpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.StateMachine;

public class test extends LinearOpMode {
    private StateMachine auto;
    DcMotor a;
    public test(HardwareMap hm){
        a = hm.get(DcMotor.class, "a");
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //auto = new StateMachineBuilder<>
    }


}
