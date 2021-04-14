package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.Intake;
import org.firstinspires.ftc.teamcode.utils.Outtake;
import org.firstinspires.ftc.teamcode.utils.RRQuickStart.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Shooter2;
import org.firstinspires.ftc.teamcode.utils.Shooter3;
import org.firstinspires.ftc.teamcode.utils.WobbleGoal;

/*
Initialize util objects
Create Trajectories
Scan for stack size
PowerShots
    Go to PowerShot Mid & shoot
    Turn to PowerShot left & shoot
    Turn to PowerShot right & shoot
Rings
    If 0 Ring -> ignore this area
    If 1 ring -> go to intake pos, intake, shoot
    If 4 rings -> go to intake pos, intake1, shoot, intake 3, shoot 3
Wobble Goal
    Go to Wobble Goal Dropping zone & drop
    come to start & pick up wobble goal
    Go to Wobble Goal Dropping Zone & drop
Park on line
 */

public class AutonSM extends LinearOpMode {

    enum STATE{
        GOTOPOWERSHOTL,

        WAIT250MS_05,
        WAIT250MS_1,

        FLICKRINGPSL,
        FLICKRINGPSL2,

        GOTOPOWERSHOTC,
        WAIT250MS_15,
        WAIT250MS_2,

        FLICKRINGPSC,
        FLICKRINGPSC2,

        GOTOPOWERSHOTR,
        WAIT250MS_25,
        WAIT250MS_3,

        FLICKRINGPSR,
        FLICKRINGPSR2,

        WAIT250MS,

        GOTOINTAKEPRE,
        GOTOINTAKEPOST,
        GOTOINTAKEPOSTPOST,

        SHOOT1OF1RING,
        SHOOT1OF4RINGS,

        GOTOWGADUMP,


        ONSTARTLINE


    }

    SampleMecanumDrive drive;
    Shooter3 shooter;
    Intake in;
    WobbleGoal wg;
    //###########################################################################################
    Pose2d startingPose= new Pose2d(-60.0,-36.0,Math.toRadians(180));
    Trajectory t1,t2,t3,t4,t5,t6, t7,t8,t9,t10;


    //###########################################################################################
    Pose2d psLeft = new Pose2d(-5.0,-3.5,Math.toRadians(180.0));
    Vector2d intakePre = new Vector2d(-10.0,-36.0);
    Vector2d intakePost = new Vector2d(-18.0,-36.0);
    Vector2d intakePostPost = new Vector2d(-36.0,-36.0);
    Pose2d wgPos4Ring = new Pose2d(54.0,-60.0,0.0);
    Pose2d wgPos1Ring = new Pose2d(54.0-24,-60.0+24,0.0);
    Pose2d wgPos0Ring = new Pose2d(12.0,-48.0,Math.toRadians(-90.0));
    Pose2d wgStartPose = new Pose2d(-38.0,-48.0,Math.toRadians(180.0));
    Pose2d startLine;
    public STATE currentState;
    int stackSize=0;

    ElapsedTime waitTimer= new ElapsedTime();
    @Override
    public void runOpMode(){
        initialize();
        waitForStart();
        shooter.spinPowershot();

        while(currentState != STATE.ONSTARTLINE){
            switch(currentState){
                case GOTOPOWERSHOTL:
                    drive.followTrajectoryAsync(t1);
                    currentState = STATE.WAIT250MS;
                    break;
                case WAIT250MS_05:
                    if(!drive.isBusy()){
                        currentState=STATE.WAIT250MS_1;
                        waitTimer.reset();
                    }
                    break;
                case WAIT250MS:
                    if(!drive.isBusy() && waitTimer.time()>0.25){
                        currentState=STATE.FLICKRINGPSL;
                    }
                    break;
                case FLICKRINGPSL:
                    if(!drive.isBusy()){
                        currentState=STATE.FLICKRINGPSL2;
                        waitTimer.reset();
                        shooter.pull();
                    }
                    break;
                case FLICKRINGPSL2:
                    if(!drive.isBusy() && waitTimer.time()>2.5){
                        shooter.push();
                        currentState=STATE.GOTOPOWERSHOTC;
                    }
                    break;
                case GOTOPOWERSHOTC:
                    drive.followTrajectoryAsync(t2);
                    currentState = STATE.WAIT250MS_15;
                    break;
                case WAIT250MS_15:
                    if(!drive.isBusy()){
                        currentState=STATE.WAIT250MS_2;
                        waitTimer.reset();
                    }
                    break;
                case WAIT250MS_2:
                    if(!drive.isBusy() && waitTimer.time()>0.25){
                        currentState=STATE.FLICKRINGPSC;
                    }
                    break;
                case FLICKRINGPSC:
                    if(!drive.isBusy()){
                        currentState=STATE.FLICKRINGPSC2;
                        shooter.pull();
                        waitTimer.reset();
                    }
                    break;

                case FLICKRINGPSC2:
                    if(!drive.isBusy() && waitTimer.time()>2.5){
                        shooter.push();
                        currentState = STATE.GOTOPOWERSHOTR;
                    }
                    break;
                case GOTOPOWERSHOTR:
                    drive.followTrajectoryAsync(t2);
                    currentState = STATE.WAIT250MS_25;
                    break;
                case WAIT250MS_25:
                    if(!drive.isBusy()){
                        currentState=STATE.WAIT250MS_3;
                        waitTimer.reset();
                    }
                    break;
                case WAIT250MS_3:
                    if(!drive.isBusy() && waitTimer.time()>0.25){
                        currentState=STATE.FLICKRINGPSR;
                    }
                    break;
                case FLICKRINGPSR:
                    if(!drive.isBusy()){
                        shooter.pull();
                        currentState=STATE.FLICKRINGPSR2;
                        waitTimer.reset();
                    }
                    break;

                case FLICKRINGPSR2:
                    if(!drive.isBusy() && waitTimer.time()>2.5){
                        shooter.push();
                        if(stackSize >0) {
                            currentState = STATE.GOTOINTAKEPRE;
                        }
                        else{
                            currentState = STATE.GOTOWGADUMP;
                        }

                    }
                    break;
                case GOTOINTAKEPRE:
                    if(!drive.isBusy()) {
                        in.spinBoth();
                        drive.followTrajectoryAsync(t4);
                        currentState = STATE.GOTOINTAKEPOST;
                    }
                    break;
                case GOTOINTAKEPOST:
                    if(!drive.isBusy()) {
                        drive.followTrajectoryAsync(t5);
                        if(stackSize==1){
                            currentState=STATE.SHOOT1OF1RING;
                        }
                        else {
                            currentState = STATE.SHOOT1OF4RINGS;
                        }
                    }
                    break;

            }
            shooter.update();
        }


    }
    public void flickRing(){
        shooter.pull();
        sleep(250);
        shooter.push();
        sleep(100);

    }
    public void goDoPowerShots() {
        Trajectory[] arr = {t1, t2, t3};
        for (int i = 0; i < 3; i++) {
            drive.followTrajectory(arr[i]);
            sleep(250);
            /*shooter.pull();
            sleep(250);
            shooter.push();*/
            flickRing();
        }
    }

    public void initialize(){
        drive=new SampleMecanumDrive(hardwareMap);
        shooter=new Shooter3(hardwareMap);
        in = new Intake(hardwareMap);
        wg = new WobbleGoal(hardwareMap);
        //TODO
        //find stack size

        createTrajectories();
    }
    public void createTrajectories(){
        //powerShotRight = drive.trajectoryBuilder(startingPose)
        //      .splineToLinearHeading()

        //Go to the left powershot
        t1 = drive.trajectoryBuilder(startingPose,true)
                .splineToLinearHeading(psLeft,0.0)
                .build();
        //Strafe to center powershot
        t2 = drive.trajectoryBuilder(t1.end(),true)
                .strafeLeft(7.5)
                .build();
        //Strafe to right powershot
        t3 = drive.trajectoryBuilder(t2.end(),true)
                .strafeLeft(7.5)
                .build();

        //If stacksize is 1 or 4 it means that we need to intake rings
        if(stackSize==1 || stackSize==4){
            //Go to pre intake position
            t4= drive.trajectoryBuilder(t3.end(),true)
                    .lineToConstantHeading(intakePre)
                    .build();
            //Go forward to pick up ring
            t5 = drive.trajectoryBuilder(t4.end(),true)
                    .lineToConstantHeading(intakePost)
                    .build();
            //If there are 4 rings we need to go further as there are more rings
            //Also Customize so that wg4 positioning is accurate
            if(stackSize==4){
                //At the end we want robit to end up on startLine Pose
                startLine=new Pose2d(12.0,-60.0,0.0);

                //Drive forward more so that other rings will be intaked as well
                t6 = drive.trajectoryBuilder(t5.end(), true)
                        .lineToConstantHeading(intakePostPost)
                        .build();

                //Go To the Wobble Goal C dump zone
                t7 = drive.trajectoryBuilder(t6.end(), true)
                        .splineToLinearHeading(wgPos4Ring,0.0)
                        .build();;
                //Go To the second start position to get the wobble goal
                t8 = drive.trajectoryBuilder(t7.end(), false)
                        .lineToLinearHeading(wgStartPose)
                        .build();
                //Go back to the Wobble Goal C dump zone
                t9 = drive.trajectoryBuilder(t8.end(), false)
                        .lineToLinearHeading(wgPos4Ring)
                        .build();
                //Park on line
                t10 = drive.trajectoryBuilder(t9.end(), false)
                        .lineToLinearHeading(startLine)
                        .build();
            }
            //If there is 1 ring we don't need to go further as there are no more rings
            //Also Customize so that wg1 positioning is accurate
            else{
                //At the end we want robit to end up on startLine Pose
                startLine= new Pose2d(12.0,-60.0+24,0.0);

                //Drive To Wobble Goal B dump zone
                t7 = drive.trajectoryBuilder(t5.end(), true)
                        .splineToLinearHeading(wgPos1Ring,0.0)
                        .build();

                //Go To the second start position to get the wobble goal
                t8 =drive.trajectoryBuilder(t7.end(), false)
                        .lineToLinearHeading(wgStartPose)
                        .build();

                //Go back to the Wobble Goal C dump zone
                t9 = drive.trajectoryBuilder(t8.end(), false)
                        .lineToLinearHeading(wgPos1Ring)
                        .build();

                //Park On line
                t10 = drive.trajectoryBuilder(t9.end(), false)
                        .lineToLinearHeading(startLine)
                        .build();
            }
        }
        //If there are 0 rings, no intaking should occur
        //Also Customize so that wg0 positioning is accurate
        else{
            //Go to Wobble GOal A Dump Zone
            t7 = drive.trajectoryBuilder(t3.end(), true)
                    .splineToLinearHeading(wgPos0Ring,0.0)
                    .build();

            //Go To the second start position to get the wobble goal
            t8 = drive.trajectoryBuilder(t7.end(), false)
                    .lineToLinearHeading(wgStartPose)
                    .build();

            //Go back to the Wobble Goal A dump zone
            t9 = drive.trajectoryBuilder(t8.end(), false)
                    .lineToLinearHeading(wgPos0Ring)
                    .build();

            //It should alrdy be on line at this pt
        }



    }
}
