package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.subsystems.Hang;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

import java.util.List;
@TeleOp
public class Teleop extends LinearOpMode {
    MecanumDrive drive;
    Intake intake=new Intake();
    Outtake outtake=new Outtake();
    Hang hang;
    enum TransferStates{
        IDLE,
        RETRACT,
        REVERSEINTAKE,
        PUTDOWN
    }
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
        //hang.init(hardwareMap);
        intake.intakePosition(0);
        outtake.transferPosition();
        double loopTime=0.0;
        StateMachine transferMachine= new StateMachineBuilder()
                .state(TransferStates.IDLE)
                .transition(()->gamepad1.touchpad)

                .state(TransferStates.RETRACT)
                .onEnter(()-> {
                    intake.transferPosition();
                    intake.setPower(1);
                })
                .transition(()->intake.canEject())

                .state(TransferStates.REVERSEINTAKE)
                .onEnter(()-> intake.setPower(-1))
                .transitionTimed(1)

                .state(TransferStates.PUTDOWN)
                .onEnter(()->{
                    intake.intakePosition(0);
                    intake.setPower(0);
                })
                .transitionTimed(0.5, TransferStates.IDLE)
                .build();
        waitForStart();
        transferMachine.start();
        while (opModeIsActive()){
            hubs.forEach(LynxModule::clearBulkCache);
            if (gamepad1.right_bumper){
                drive.driveRobotCentric(-gamepad1.left_stick_x*0.3, gamepad1.left_stick_y*0.3, -gamepad1.right_stick_x*0.3);
            }else{
                drive.driveRobotCentric(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);
            }

            if (gamepad1.y){
                intake.intakePosition(800);
                intake.setPower(1);
            }
            if (gamepad1.a){
                intake.intakePosition(0);
                intake.setPower(1);
            }
            if (gamepad1.b){
                if (transferMachine.getState()==TransferStates.IDLE){
                    intake.setPower(0);
                    intake.intakePosition(0);
                }
            }

            if (gamepad1.dpad_up){//score deposit
                outtake.setServos(0, true, true, true);
                outtake.setTarget(300);
            }
            if (gamepad1.dpad_down){//retract
                outtake.setServos(0, false, false, false);
                outtake.setTarget(0);
            }

            transferMachine.update();
            intake.update();
            outtake.update();

            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.update();
        }
    }
}
