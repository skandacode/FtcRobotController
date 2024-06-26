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
    Hang hang=new Hang();
    enum TransferStates{
        IDLE,
        RETRACT,
        WAITING,
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
        hang.init(hardwareMap);
        intake.intakePosition(0);
        outtake.transferPosition();
        hang.retract();
        double loopTime=0.0;
        StateMachine transferMachine= new StateMachineBuilder()
                .state(TransferStates.IDLE)
                .transition(()->gamepad2.b)

                .state(TransferStates.RETRACT)
                .onEnter(()-> {
                    intake.transferPosition();
                    intake.setPower(1);
                })
                .transition(()->intake.canEject())
                .state(TransferStates.WAITING)
                .transitionTimed(0.5)

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
        outtake.setVelocity(-0.3);
        sleep(500);
        while (outtake.getVelocity()<0 && !isStopRequested()){
            outtake.setVelocity(-0.3);
        }
        outtake.setVelocity(0);
        outtake.resetEncoder();
        waitForStart();
        int currentheight=0;
        boolean prevLeftTrigger=false;
        boolean prevRightTrigger=false;

        transferMachine.start();
        while (opModeIsActive()){
            hubs.forEach(LynxModule::clearBulkCache);
            if (gamepad1.right_bumper){
                drive.driveRobotCentric(-gamepad1.left_stick_x*0.3, gamepad1.left_stick_y*0.3, -gamepad1.right_stick_x*0.3);
            }else{
                drive.driveRobotCentric(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x*0.7);
            }

            if (gamepad2.y){
                intake.intakePositionExtended(500);
                intake.setPower(1);
                transferMachine.setState(TransferStates.IDLE);
            }
            if (gamepad2.a){
                intake.intakePosition(0);
                intake.setPower(1);
                transferMachine.setState(TransferStates.IDLE);
            }
            if (gamepad2.x) {
                intake.setPower(0);
                intake.intakePosition2nd(0);
                transferMachine.setState(TransferStates.IDLE);
            }
            boolean currRightTrigger=gamepad2.right_trigger>0.5;
            boolean currLeftTrigger=gamepad2.left_trigger>0.5;

            if (currRightTrigger && !prevRightTrigger){
                if (currentheight<700) {
                    currentheight = currentheight + 70;
                }
                outtake.depositPosition(currentheight);
            }
            if (currLeftTrigger && !prevLeftTrigger){
                if (currentheight>0){
                    currentheight=currentheight-70;
                }
                outtake.depositPosition(currentheight);
            }
            prevRightTrigger=currRightTrigger;
            prevLeftTrigger=currLeftTrigger;


            if (gamepad2.dpad_up){//score deposit
                outtake.depositPosition(currentheight);
                outtake.setPixelLatch(true);
            }
            if (gamepad2.left_stick_button){//increase height
                intake.intakePosition2nd();
                intake.setPower(1);
            }
            if (gamepad2.right_stick_button){
                intake.intakePosition4th();
                intake.setPower(1);
            }

            if (gamepad2.touchpad){//retract
                outtake.transferPosition();
                outtake.setPixelLatch(false);
                currentheight=0;
            }
            if (gamepad2.right_bumper || !outtake.getScoring()){
                outtake.setPixelLatch(false);
            }else{
                outtake.setPixelLatch(true);
            }
            if (gamepad1.y){
                hang.releaseDrone();
            }else{
                hang.keepDrone();
            }
            if (gamepad2.share && gamepad2.options){
                intake.resetEncoder();
                outtake.resetEncoder();
            }
            if (gamepad1.right_trigger>0.5){
                hang.retract();
            } else if (gamepad1.left_trigger>0.5){
                hang.raise();
            }else if (gamepad1.left_stick_button && gamepad1.right_stick_button){
                hang.hang();
                hang.releaseServos();
            }else{
                hang.release();
            }

            transferMachine.update();
            intake.update();
            outtake.update();

            double loop = System.nanoTime();
            telemetry.addData("transfer state", transferMachine.getState().toString());
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            telemetry.addData("intake pos", intake.getEncoderPos());
            telemetry.addData("outtake pos", outtake.getEncoderPos());
            telemetry.addData("Current height", currentheight);
            loopTime = loop;
            telemetry.update();
        }
    }
}
