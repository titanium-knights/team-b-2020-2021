package org.firstinspires.ftc.teamcode.autonomous.StatesAuton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class AutonomousOpMode extends LinearOpMode {
    int allianceSide =3;
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        if(allianceSide ==0){
            //Start doing blue stuff
        }





    }
    public void initialize(){
        telemetry.addLine("Are you on the red or blue alliance. Press x for blue. Press b for red");
        while(allianceSide ==3){
            if(gamepad1.x){
                allianceSide=0;
            }
            if(gamepad1.b){
                allianceSide=1;
            }
        }

    }
}
