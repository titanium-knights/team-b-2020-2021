package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Pusher {
     private Servo push;
     public Pusher(HardwareMap hm){
         push = hm.get(Servo.class,CONFIG.PUSH);
     }
     public void push(){
         push.setPosition(0.6);
     }
     public void pull(){
         push.setPosition(0.8);
     }

}
