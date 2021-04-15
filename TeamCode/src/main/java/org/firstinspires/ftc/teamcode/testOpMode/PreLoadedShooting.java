package org.firstinspires.ftc.teamcode.testOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.Outtake;
@Config
@TeleOp
public class PreLoadedShooting extends LinearOpMode {
    public static boolean testing = true;
    public static long pushTime = 1000;
    public static long pullTime = 1000;
    @Override
    public void runOpMode(){
        Outtake out = new Outtake(hardwareMap);
        waitForStart();
        out.spin();
        sleep(2000);
        while(opModeIsActive()) {
            while (testing && !isStopRequested()) {
                out.spin();
                for (int i=0;i<3;i++){
                    out.push();
                    sleep(pushTime);
                    out.pull();
                    if(i!=2){
                        sleep(pullTime);
                    }
                }
            }
            out.stop();
        }
    }
}
