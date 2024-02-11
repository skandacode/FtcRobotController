package org.firstinspires.ftc.teamcode.testing;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Config
@TeleOp
public class IntakeTestActuallyUsingIntakeClass extends LinearOpMode {
    public static double intakeHeightsPos=0.5;
    public static double intakePitchPos=0.5;
    public static double intakePower=0;
    public static int intakePos=0;
    Intake intake=new Intake();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        intake.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            intake.setTarget(intakePos);
            intake.setServos(intakeHeightsPos, intakePitchPos);
            intake.setPower(intakePower);
            intake.update();
            telemetry.addData("Intake encoder pos", intake.getEncoderPos());
            telemetry.addData("Intake height", intake.readHeightAnalog());
            telemetry.update();
        }
    }
}
