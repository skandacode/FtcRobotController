package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Hang;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

import java.util.List;

public class Teleop extends LinearOpMode {
    MecanumDrive drive;
    Intake intake;
    Outtake outtake;
    Hang hang;
    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
        drive=new MecanumDrive(
                new Motor(hardwareMap, "frontleft", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "frontright", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "backleft", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "backright", Motor.GoBILDA.RPM_312));
        intake.init(hardwareMap);
        outtake.init(hardwareMap);
        hang.init(hardwareMap);
        intake.intakePosition(0);
        outtake.transferPosition();
        double loopTime=0.0;
        waitForStart();

        while (opModeIsActive()){
            hubs.forEach(LynxModule::clearBulkCache);
            if (gamepad1.right_bumper){
                drive.driveRobotCentric(-gamepad1.left_stick_x*0.3, gamepad1.left_stick_y*0.3, -gamepad1.right_stick_x*0.3);
            }else{
                drive.driveRobotCentric(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);
            }

            if (gamepad2.touchpad){
                intake.transferPosition();
                if (intake.canEject()){
                    intake.setPower(-1);
                }else{
                    intake.setPower(0);
                }
            }else if (gamepad2.y){
                intake.intakePosition(600);
                intake.setPower(1);
            }else if (gamepad2.b){
                intake.intakePosition(0);
                intake.setPower(1);
            }else{
                intake.intakePosition(0);
                intake.setPower(0);
            }



            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.update();
        }
    }
}