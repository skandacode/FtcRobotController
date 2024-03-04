package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.notshown.BluePipeline;
import org.firstinspires.ftc.teamcode.auto.notshown.PropPosition;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Objects;

@Config
@Autonomous
public class BlueAutotwoplusfour extends LinearOpMode
{
    OpenCvWebcam webcam;
    PropPosition randomization=PropPosition.NONE;
    Intake intake = new Intake();
    Outtake outtake = new Outtake();


    public static String ObjectDirection;

    @Override
    public void runOpMode()
    {
        intake.init(hardwareMap);
        outtake.init(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        BluePipeline pipeline = new BluePipeline(telemetry, ObjectDirection);
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
        TrajectorySequence left=drive.trajectorySequenceBuilder(new Pose2d(11.83, 62.16, Math.toRadians(270.00)))
                .lineTo(new Vector2d(27, 42))
                .setReversed(true)
                .UNSTABLE_addDisplacementMarkerOffset(10, ()->{
                    intake.stay(0);
                })
                .UNSTABLE_addDisplacementMarkerOffset(20, ()->{
                    outtake.depositPosition(0, 0);
                    outtake.setPixelLatch(true);
                })
                .splineTo(new Vector2d(48.5, 44), Math.toRadians(0.00))
                .setReversed(false)
                .addTemporalMarker(()->{
                    outtake.setPixelLatch(false);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, ()->{
                    outtake.transferPosition();
                })
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(30, 16, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(20, 12), Math.toRadians(180))
                .UNSTABLE_addDisplacementMarkerOffset(0, ()->{
                    intake.intakePosition4th(900);
                    intake.setPower(1);
                })
                .splineTo(new Vector2d(-23, 12), Math.toRadians(190))//first pickup
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    intake.transferPosition();
                })
                .UNSTABLE_addTemporalMarkerOffset(1.1, ()->{
                    intake.setPower(-0.2);
                })
                .UNSTABLE_addTemporalMarkerOffset(2, ()->{
                    intake.setPower(0);
                    intake.stay(60);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.5, ()->{
                    outtake.depositPosition(300, 0);
                    outtake.setPixelLatch(true);
                })
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(12, 12, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(43, 35), Math.toRadians(75.00))
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    outtake.depositPosition(300, 0);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, ()->{
                    outtake.setPixelLatch(false);
                })
                .waitSeconds(1.5)
                .addTemporalMarker(()->{
                    outtake.transferPosition();
                })
                .lineToLinearHeading(new Pose2d(30, 16, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(20, 12), Math.toRadians(180))
                .UNSTABLE_addDisplacementMarkerOffset(0, ()->{
                    intake.intakePosition2nd(900);
                    intake.setPower(1);
                })
                .splineTo(new Vector2d(-23, 9), Math.toRadians(190))//second pickup
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    intake.transferPosition();
                })
                .UNSTABLE_addTemporalMarkerOffset(1.3, ()->{
                    intake.setPower(-0.2);
                })
                .UNSTABLE_addTemporalMarkerOffset(2, ()->{
                    intake.setPower(0);
                    intake.stay(60);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.5, ()->{
                    outtake.depositPosition(300, 0);
                    outtake.setPixelLatch(true);
                })
                .UNSTABLE_addTemporalMarkerOffset(3, ()->{
                    intake.setTarget(0);
                })
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(12, 12, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(43, 35), Math.toRadians(75.00))
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()->{
                    outtake.setPixelLatch(false);
                })
                .waitSeconds(1)
                .build();

        TrajectorySequence middle=drive.trajectorySequenceBuilder(new Pose2d(11.83, 62.16, Math.toRadians(270.00)))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(70), DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(35))
                .splineTo(new Vector2d(12, 35), Math.toRadians(270))
                .setReversed(true)
                .UNSTABLE_addDisplacementMarkerOffset(10, ()->{
                    intake.intakePosition5th(60);
                })
                .UNSTABLE_addDisplacementMarkerOffset(30, ()->{
                    outtake.depositPosition(0, 0);
                    outtake.setPixelLatch(true);
                })
                .splineTo(new Vector2d(47.5, 37), Math.toRadians(2.00))
                .setReversed(false)
                .addTemporalMarker(()->{
                    outtake.setPixelLatch(false);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, ()->{
                    outtake.transferPosition();
                })
                .waitSeconds(2)
                .splineTo(new Vector2d(12, 12), Math.toRadians(180))
                .UNSTABLE_addDisplacementMarkerOffset(0, ()->{
                    intake.intakePosition4th(900);
                    intake.setPower(1);
                })
                .lineToSplineHeading(new Pose2d(-25, 12, Math.toRadians(180)))//first pickup
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    intake.transferPosition();
                })
                .UNSTABLE_addTemporalMarkerOffset(1.1, ()->{
                    intake.setPower(-0.2);
                })
                .UNSTABLE_addTemporalMarkerOffset(2, ()->{
                    intake.setPower(0);
                    intake.intakePosition5th(60);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.5, ()->{
                    outtake.depositPosition(200, 0);
                    outtake.setPixelLatch(true);
                })
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(12, 12, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(43, 35), Math.toRadians(30.00))
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    outtake.depositPosition(300, 0);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, ()->{
                    outtake.setPixelLatch(false);
                })
                .waitSeconds(1.5)
                .addTemporalMarker(()->{
                    outtake.transferPosition();
                })
                .splineTo(new Vector2d(12, 12), Math.toRadians(180))
                .UNSTABLE_addDisplacementMarkerOffset(0, ()->{
                    intake.intakePosition2nd(900);
                    intake.setPower(1);
                })
                .splineTo(new Vector2d(-28, 13), Math.toRadians(177))//pickup 2nd
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    intake.transferPosition();
                })
                .UNSTABLE_addTemporalMarkerOffset(1.3, ()->{
                    intake.setPower(-0.2);
                })
                .UNSTABLE_addTemporalMarkerOffset(2, ()->{
                    intake.setPower(0);
                    intake.intakePosition5th(60);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.5, ()->{
                    outtake.depositPosition(200, 0);
                    outtake.setPixelLatch(true);
                })
                .UNSTABLE_addTemporalMarkerOffset(3, ()->{
                    intake.setTarget(0);
                })
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(12, 12, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(43, 35), Math.toRadians(30.00))
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()->{
                    outtake.setPixelLatch(false);
                })
                .waitSeconds(1)
                .build();
        TrajectorySequence right = drive.trajectorySequenceBuilder(new Pose2d(11.83, 62.16, Math.toRadians(270.00)))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(70), DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(35))

                .splineTo(new Vector2d(8, 37), Math.toRadians(228.62))
                .setReversed(true)
                .UNSTABLE_addDisplacementMarkerOffset(10, ()->{
                    intake.intakePosition5th(60);
                })
                .UNSTABLE_addDisplacementMarkerOffset(20, ()->{
                    outtake.depositPosition(0, 0);
                    outtake.setPixelLatch(true);
                })
                .splineTo(new Vector2d(47, 29), Math.toRadians(2.00))
                .setReversed(false)
                .addTemporalMarker(()->{
                    outtake.setPixelLatch(false);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, ()->{
                    outtake.transferPosition();
                })
                .waitSeconds(1)
                .splineTo(new Vector2d(12, 12), Math.toRadians(180))
                .UNSTABLE_addDisplacementMarkerOffset(0, ()->{
                    intake.intakePosition4th(900);
                    intake.setPower(1);
                })
                .lineToSplineHeading(new Pose2d(-24, 13, Math.toRadians(180)))//first pickup
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    intake.transferPosition();
                })
                .UNSTABLE_addTemporalMarkerOffset(1.1, ()->{
                    intake.setPower(-0.2);
                })
                .UNSTABLE_addTemporalMarkerOffset(2, ()->{
                    intake.setPower(0);
                    intake.intakePosition5th(60);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.5, ()->{
                    outtake.depositPosition(300, 0);
                    outtake.setPixelLatch(true);
                })
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(12, 12, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(41.5, 35), Math.toRadians(30.00))
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    outtake.depositPosition(300, 0);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, ()->{
                    outtake.setPixelLatch(false);
                })
                .waitSeconds(1.5)
                .addTemporalMarker(()->{
                    outtake.transferPosition();
                })
                .splineTo(new Vector2d(12, 12), Math.toRadians(182))
                .UNSTABLE_addDisplacementMarkerOffset(0, ()->{
                    intake.intakePosition2nd(900);
                    intake.setPower(1);
                })
                .splineToConstantHeading(new Vector2d(-27, 13), Math.toRadians(182))//second pickup
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    intake.transferPosition();
                })
                .UNSTABLE_addTemporalMarkerOffset(1.3, ()->{
                    intake.setPower(-0.2);
                })
                .UNSTABLE_addTemporalMarkerOffset(2, ()->{
                    intake.setPower(0);
                    intake.intakePosition5th(60);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.5, ()->{
                    outtake.depositPosition(300, 0);
                    outtake.setPixelLatch(true);
                })
                .UNSTABLE_addTemporalMarkerOffset(3, ()->{
                    intake.setTarget(0);
                })
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(12, 12, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(41.5, 35), Math.toRadians(30.00))
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()->{
                    outtake.setPixelLatch(false);
                })
                .waitSeconds(1)
                .build();

        TrajectorySequence park_left=drive.trajectorySequenceBuilder(left.end())
                .splineToConstantHeading(new Vector2d(35, 61), Math.toRadians(90))
                .back(20)
                .build();
        TrajectorySequence park_middle=drive.trajectorySequenceBuilder(middle.end())
                .splineToConstantHeading(new Vector2d(35, 61), Math.toRadians(90))
                .back(20)
                .build();
        TrajectorySequence park_right=drive.trajectorySequenceBuilder(right.end())
                .splineToConstantHeading(new Vector2d(35, 61), Math.toRadians(90))
                .back(20)
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
        }
        timer.reset();
        intake.setTarget(0);
        while (timer.milliseconds()<200){
            intake.update();
        }
    }
}