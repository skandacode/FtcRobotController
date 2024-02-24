package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
@Config
public class Servotest extends LinearOpMode {
    ServoImplEx intakeheights0;
    Servo intakepitch1, miniturret2, depositextendo3, depositflip4, drone;
    public static double intakeheights0pos=0;
    public static double intakepitch1pos=0;
    public static double miniturret2pos=0;
    public static double depositextendo3pos=0;
    public static double depositflip4pos=0;
    public static double dronePos=0;

    @Override
    public void runOpMode() throws InterruptedException {
        intakeheights0=(ServoImplEx) hardwareMap.servo.get("intakeheights0");
        intakepitch1=hardwareMap.servo.get("intakepitch1");
        miniturret2=hardwareMap.servo.get("miniturret2");
        depositextendo3=hardwareMap.servo.get("depositextendo3");
        depositflip4=hardwareMap.servo.get("depositflip4");
        drone=hardwareMap.servo.get("drone");

        intakeheights0.setPwmRange(new PwmControl.PwmRange(510, 2490));

        waitForStart();
        while (opModeIsActive()){
            intakeheights0.setPosition(intakeheights0pos);
            intakepitch1.setPosition(intakepitch1pos);
            miniturret2.setPosition(miniturret2pos);
            depositextendo3.setPosition(depositextendo3pos);
            depositflip4.setPosition(depositflip4pos);
            drone.setPosition(dronePos);
        }
    }
}
