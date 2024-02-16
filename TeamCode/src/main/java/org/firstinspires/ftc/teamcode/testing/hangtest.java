package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@TeleOp
public class hangtest extends LinearOpMode {
    Servo hang1, hang2;
    MotorEx hangMotor;
    public static double hang1Pos=0;
    public static double hang2Pos=0;
    public static double hangMotorPower=0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        hangMotor=new MotorEx(hardwareMap, "hangmotor3", Motor.GoBILDA.RPM_435);
        hang1=hardwareMap.servo.get("hang1");
        hang2=hardwareMap.servo.get("hang2");
        hang2.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()){
            hangMotor.set(hangMotorPower);
            hang1.setPosition(hang1Pos);
            hang2.setPosition(hang2Pos);
            telemetry.addData("Motor Current", hangMotor.motorEx.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
}
