package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class deposittest extends LinearOpMode {
    Motor outtake1, outtake2;
    Servo miniTurret, depositFlip, depositExtendo;
    AnalogInput turretEncoder, flipEncoder;
    MotorGroup outtakeMotors;
    public static double motorPowers=0;
    public static double turretPos=0;
    public static double flipPos=0;
    public static double extendoPos=0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        outtake1=new Motor(hardwareMap, "depositfirst1", Motor.GoBILDA.RPM_1150);
        outtake2=new Motor(hardwareMap, "depositsecond2", Motor.GoBILDA.RPM_1150);
        outtake1.setInverted(true);

        outtake1.resetEncoder();
        outtake2.resetEncoder();
        outtakeMotors=new MotorGroup(outtake1, outtake2);
        outtake1.encoder.setDirection(Motor.Direction.REVERSE);
        miniTurret=hardwareMap.servo.get("miniturret2");
        depositFlip=hardwareMap.servo.get("depositflip4");
        depositExtendo=hardwareMap.servo.get("depositextendo3");


        turretEncoder=hardwareMap.analogInput.get("turretencoder");
        flipEncoder=hardwareMap.analogInput.get("flipencoder");

        waitForStart();

        while (opModeIsActive()){
            outtakeMotors.set(motorPowers);
            miniTurret.setPosition(0.475+turretPos);
            depositFlip.setPosition(flipPos);
            depositExtendo.setPosition(extendoPos);
            telemetry.addData("Motor encoder pos", outtake1.getCurrentPosition());
            telemetry.addData("flip encoder pos", flipEncoder.getVoltage());
            telemetry.addData("turret encoder pos", turretEncoder.getVoltage());
            telemetry.update();
        }
    }
}
