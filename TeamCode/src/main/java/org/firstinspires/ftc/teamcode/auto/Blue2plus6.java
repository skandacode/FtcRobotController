package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;
@Autonomous
public class Blue2plus6 extends LinearOpMode {
    Intake intake = new Intake();
    Outtake outtake = new Outtake();
    SampleMecanumDrive drive;
    double loopTime=0;
    List<LynxModule> hubs;

    @Override
    public void runOpMode() throws InterruptedException {
        intake.init(hardwareMap);
        outtake.init(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        TrajectorySequence yellowAndToStackPathLeft=drive.trajectorySequenceBuilder(new Pose2d(11.83, 62.16, Math.toRadians(270.00)))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, Math.toRadians(90), DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(50))

                .addTemporalMarker(()->{
                    intake.stay(80);
                })
                .lineTo(new Vector2d(25, 42))
                .setReversed(true)
                .addTemporalMarker(()->{
                    intake.stay(0);
                })
                .UNSTABLE_addDisplacementMarkerOffset(10, ()->{
                    outtake.depositPosition(0);
                    outtake.setPixelLatch(true);
                })
                .back(4)
                .lineToLinearHeading(new Pose2d(48, 42, Math.toRadians(180.00)))
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0.2, ()->{
                    outtake.setPixelLatch(false);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, ()->{
                    outtake.transferPosition();
                })
                .waitSeconds(0.4)
                .lineToSplineHeading(new Pose2d(30, 12, Math.toRadians(178)))
                .splineToConstantHeading(new Vector2d(20, 12), Math.toRadians(178))
                .UNSTABLE_addDisplacementMarkerOffset(0, ()->{
                    intake.intakePosition5th(900);
                    intake.setPower(1);
                })
                .forward(42.5)//first pickup
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    intake.intakePosition4th();
                })
                .waitSeconds(0.3)

                .setReversed(true)
                .addTemporalMarker(()->{
                    intake.stay(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->{
                    intake.transferPosition();
                })
                .UNSTABLE_addTemporalMarkerOffset(1.2, ()->{
                    intake.setPower(-1);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.1, ()->{
                    intake.setPower(0);
                    intake.stay(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.5, ()->{
                    outtake.depositPosition(200);
                    outtake.setPixelLatch(true);
                })
                .lineTo(new Vector2d(30, 12))
                .splineToConstantHeading(new Vector2d(43, 35), 90)
                .UNSTABLE_addTemporalMarkerOffset(0.2, ()->{
                    outtake.setPixelLatch(false);
                    outtake.setTarget(350);
                })
                .waitSeconds(0.8)
                .addTemporalMarker(()->{
                    outtake.transferPosition();
                })
                .lineToSplineHeading(new Pose2d(30, 12, Math.toRadians(179)))
                .splineToConstantHeading(new Vector2d(20, 12), Math.toRadians(179))
                .UNSTABLE_addDisplacementMarkerOffset(0, ()->{
                    intake.intakePositionExtended(900);
                    intake.setPower(1);
                })
                .forward(44)//second pickup
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()->{
                    intake.setTarget(700);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->{
                    intake.setTarget(900);
                })
                .waitSeconds(1)
                .setReversed(true)
                .addTemporalMarker(()->{
                    intake.stay(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->{
                    intake.transferPosition();
                })
                .UNSTABLE_addTemporalMarkerOffset(1.2, ()->{
                    intake.setPower(-1);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.1, ()->{
                    intake.setPower(0);
                    intake.stay(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.5, ()->{
                    outtake.depositPosition(200);
                    outtake.setPixelLatch(true);
                })
                .lineTo(new Vector2d(30, 12))
                .splineToConstantHeading(new Vector2d(44, 35), 90)
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()->{
                    outtake.setPixelLatch(false);
                    outtake.setTarget(350);
                })
                .waitSeconds(0.8)
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{
                    outtake.transferPosition();
                    outtake.setPixelLatch(false);
                })

                .lineToSplineHeading(new Pose2d(30, 12, Math.toRadians(178)))
                .splineToConstantHeading(new Vector2d(20, 9), Math.toRadians(178))
                .splineTo(new Vector2d(-26, 9), Math.toRadians(160))//intake for third time
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()->{
                    intake.intakePosition5th(900);
                    intake.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{
                    intake.intakePositionExtended(800);
                    intake.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->{
                    intake.intakePositionExtended(900);
                    intake.setPower(1);
                })

                .waitSeconds(0.9)
                .addTemporalMarker(()->{
                    intake.transferPosition();
                    outtake.transferPosition();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.8, ()->{
                    intake.setPower(-1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.6, ()->{
                    intake.stay(0);
                    intake.setPower(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.9, ()->{
                    outtake.depositPosition(200);
                    outtake.setPixelLatch(true);
                })
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-9, 9, Math.toRadians(180)), 0)
                .splineToSplineHeading(new Pose2d(30, 12, Math.toRadians(180)), 0)
                .splineToSplineHeading(new Pose2d(44.3, 35, Math.toRadians(180)), Math.toRadians(80))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, ()->{
                    outtake.setPixelLatch(false);
                    outtake.setTarget(350);
                })
                .waitSeconds(0.7)
                .build();

        hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
        telemetry.addLine("inited");
        telemetry.update();
        waitForStart();
        drive.setPoseEstimate(yellowAndToStackPathLeft.start());
        drive.followTrajectorySequenceAsync(yellowAndToStackPathLeft);
        while (drive.isBusy() && opModeIsActive()){
            updateSystems();
        }

    }
    public void updateSystems(){
        hubs.forEach(LynxModule::clearBulkCache);
        drive.update();
        drive.update();
        intake.update();
        outtake.update();
        telemetry.addData("outtake position", outtake.getEncoderPos());
        telemetry.addData("intake position", intake.getEncoderPos());
        telemetry.addData("intake target position", intake.getTarget());

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();
    }
}
