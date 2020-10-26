package org.firstinspires.ftc.teamcode.testOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class test {
    DcMotor a;
    public test(HardwareMap hm){
        a = hm.get(DcMotor.class, "a");
    }
}
