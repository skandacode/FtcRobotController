package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.notshown.PropPosition;
import org.firstinspires.ftc.teamcode.auto.notshown.RedPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.*;
import java.lang.*;
@Config
@Autonomous
public class RedClosetwopluszero extends LinearOpMode
{
    OpenCvWebcam webcam;
    PropPosition randomization=PropPosition.NONE;
    Intake intake = new Intake();
    Outtake outtake = new Outtake();

    public static String ObjectDirection;

    @Override
    public void runOpMode() throws InterruptedException{
        intake.init(hardwareMap);
        outtake.init(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        RedPipeline pipeline = new RedPipeline(telemetry, ObjectDirection);
        webcam.setPipeline(pipeline);
        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });


        TrajectorySequence right=drive.trajectorySequenceBuilder(new Pose2d(11.83, -62.16, Math.toRadians(90)))
                .lineTo(new Vector2d(26, -40))
                .setReversed(true)
                .UNSTABLE_addDisplacementMarkerOffset(10, ()->{
                    intake.intakePosition5th(0);
                })
                .UNSTABLE_addDisplacementMarkerOffset(20, ()->{
                    outtake.depositPosition(0);
                    outtake.setPixelLatch(true);
                })
                .splineTo(new Vector2d(48, -43), Math.toRadians(-2.00))
                .setReversed(false)
                .addTemporalMarker(()->{
                    outtake.setPixelLatch(false);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, ()->{
                    outtake.transferPosition();
                })
                .waitSeconds(2.5)
                .build();

        TrajectorySequence middle=drive.trajectorySequenceBuilder(new Pose2d(11.83, -62.16, Math.toRadians(90.00)))
                .splineTo(new Vector2d(12, -35), Math.toRadians(90))
                .setReversed(true)
                .UNSTABLE_addDisplacementMarkerOffset(10, ()->{
                    intake.intakePosition5th(0);
                })
                .UNSTABLE_addDisplacementMarkerOffset(30, ()->{
                    outtake.depositPosition(0);
                    outtake.setPixelLatch(true);
                })
                .splineTo(new Vector2d(48, -37), Math.toRadians(-2.00))
                .setReversed(false)
                .addTemporalMarker(()->{
                    outtake.setPixelLatch(false);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, ()->{
                    outtake.transferPosition();
                })
                .waitSeconds(2.5)
                .build();
        TrajectorySequence left = drive.trajectorySequenceBuilder(new Pose2d(11.83, -62.16, Math.toRadians(90.00)))
                .splineTo(new Vector2d(6.5, -37), Math.toRadians(-228.62))
                .setReversed(true)
                .UNSTABLE_addDisplacementMarkerOffset(10, ()->{
                    intake.intakePosition5th(0);
                })
                .UNSTABLE_addDisplacementMarkerOffset(20, ()->{
                    outtake.depositPosition(0);
                    outtake.setPixelLatch(true);
                })
                .splineTo(new Vector2d(48, -29), Math.toRadians(-2.00))
                .setReversed(false)
                .addTemporalMarker(()->{
                    outtake.setPixelLatch(false);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, ()->{
                    outtake.transferPosition();
                })
                .waitSeconds(2.5)
                .build();


        while (opModeInInit()){
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());

            telemetry.update();

            if (Objects.equals(pipeline.getPosition(), "LEFT")) {
                telemetry.addData("Position", "LEFTpo");
                randomization=PropPosition.LEFT;
            }
            else if (Objects.equals(pipeline.getPosition(), "MIDDLE")){
                telemetry.addData("Position", "MIDDLEE");
                randomization=PropPosition.MIDDLE;
            }
            else if (Objects.equals(pipeline.getPosition(), "RIGHT")){
                telemetry.addData("Position", "RIGHTO");
                randomization=PropPosition.RIGHT;
            }

            sleep(100);
        }

        webcam.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener()
        {
            @Override
            public void onClose() {

            }
        });
        waitForStart();
        outtake.resetEncoder();
        intake.resetEncoder();
        ElapsedTime timer=new ElapsedTime();
        outtake.transferPosition();
        if (randomization==PropPosition.LEFT){
            intake.transferPosition();
            intake.setTarget(40);
            drive.setPoseEstimate(left.start());
            drive.followTrajectorySequenceAsync(left);
        }
        if (randomization==PropPosition.MIDDLE){
            intake.transferPosition();
            intake.setTarget(40);
            drive.setPoseEstimate(middle.start());
            drive.followTrajectorySequenceAsync(middle);
        }
        if (randomization==PropPosition.RIGHT){
            intake.transferPosition();
            intake.setTarget(40);
            drive.setPoseEstimate(right.start());
            drive.followTrajectorySequenceAsync(right);
        }
        while (drive.isBusy() && opModeIsActive()){
            drive.update();
            intake.update();
            outtake.update();
            telemetry.addData("outtake position", outtake.getEncoderPos());
            telemetry.addData("intake position", intake.getEncoderPos());

            telemetry.update();
        }
    }
}