package org.firstinspires.ftc.teamcode.auto.far;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
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
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;
import java.util.Objects;

@Config
@Autonomous
public class RedFar2plus2 extends LinearOpMode
{
    OpenCvWebcam webcam;
    PropPosition randomization=PropPosition.NONE;
    Intake intake = new Intake();
    Outtake outtake = new Outtake();
    enum StackState {TOP, BOTTOM, GROUND}
    AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    public static String ObjectDirection;

    @Override
    public void runOpMode() throws InterruptedException{
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
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
        TrajectorySequence right = drive.trajectorySequenceBuilder(new Pose2d(-36.11, -62.17, Math.toRadians(90)))
                .splineTo(new Vector2d(-32.23, -39.31), Math.toRadians(42.71))
                .setReversed(true)
                .UNSTABLE_addDisplacementMarkerOffset(10, ()->{
                    intake.intakePosition5th(0);
                })
                .splineTo(new Vector2d(-35.08, -52.85), Math.toRadians(-60))
                .splineTo(new Vector2d(-9, -58), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(1, ()->{
                    outtake.depositPosition(0, 0);
                    outtake.setPixelLatch(true);
                })
                .splineTo(new Vector2d(46, -43), Math.toRadians(0.00))
                .setReversed(false)
                .addTemporalMarker(()->{
                    outtake.setPixelLatch(false);
                })
                .waitSeconds(1)
                .addTemporalMarker(()->{
                    outtake.transferPosition();
                    intake.intakePosition(0);
                })
                ////////////////////////////////////////////////////////////////////////////////////
                .splineTo(new Vector2d(-9, -58), Math.toRadians(180))
                .splineTo(new Vector2d(-56, -44), Math.toRadians(150))
                //.splineTo(new Vector2d(-49, 49), Math.toRadians(216))
                .UNSTABLE_addDisplacementMarkerOffset(-20, ()->{
                    intake.intakePosition4th(0);
                    intake.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    intake.transferPosition();
                })
                .UNSTABLE_addTemporalMarkerOffset(1.3, ()->{
                    intake.setPower(-0.2);
                })
                .UNSTABLE_addTemporalMarkerOffset(2, ()->{
                    intake.setPower(0);
                    intake.intakePosition5th(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.5, ()->{
                    outtake.depositPosition(150, 0);
                    outtake.setPixelLatch(true);
                })
                .setReversed(true)
                .splineTo(new Vector2d(-9, -58), Math.toRadians(0))
                .splineTo(new Vector2d(45, -39), Math.toRadians(0.00))
                .addTemporalMarker(()->{
                    outtake.setPixelLatch(false);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, ()->{
                    outtake.transferPosition();
                    intake.intakePosition(0);
                })
                .waitSeconds(2)
                .build();
        TrajectorySequence middle = drive.trajectorySequenceBuilder(new Pose2d(-36.11, -62.17, Math.toRadians(90)))
                .splineTo(new Vector2d(-36.11, -34.51), Math.toRadians(90))
                .setReversed(true)
                .UNSTABLE_addDisplacementMarkerOffset(10, ()->{
                    intake.intakePosition5th(0);
                })
                .splineTo(new Vector2d(-35.08, -52.85), Math.toRadians(-60))
                .splineTo(new Vector2d(-9, -58), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(1, ()->{
                    outtake.depositPosition(0, 0);
                    outtake.setPixelLatch(true);
                })
                .splineTo(new Vector2d(43, -43), Math.toRadians(0.00))
                .setReversed(false)
                .lineTo(new Vector2d(46, -36))
                .waitSeconds(0.5)
                .addTemporalMarker(()->{
                    outtake.setPixelLatch(false);
                })
                .waitSeconds(1)
                .addTemporalMarker(()->{
                    outtake.transferPosition();
                    intake.intakePosition(0);
                })
                .splineTo(new Vector2d(-9, -58), Math.toRadians(180))
                .splineTo(new Vector2d(-56, -41), Math.toRadians(150))
                //.splineTo(new Vector2d(-49, 49), Math.toRadians(216))
                .UNSTABLE_addDisplacementMarkerOffset(-20, ()->{
                    intake.intakePosition4th(0);
                    intake.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    intake.transferPosition();
                })
                .UNSTABLE_addTemporalMarkerOffset(1.3, ()->{
                    intake.setPower(-0.2);
                })
                .UNSTABLE_addTemporalMarkerOffset(2, ()->{
                    intake.setPower(0);
                    intake.intakePosition5th(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.5, ()->{
                    outtake.depositPosition(150, 0);
                    outtake.setPixelLatch(true);
                })
                .setReversed(true)
                .splineTo(new Vector2d(-9, -58), Math.toRadians(0))
                .splineTo(new Vector2d(45, -39), Math.toRadians(0.00))
                .addTemporalMarker(()->{
                    outtake.setPixelLatch(false);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, ()->{
                    outtake.transferPosition();
                    intake.intakePosition(0);
                })
                .waitSeconds(2)
                .build();
        TrajectorySequence left = drive.trajectorySequenceBuilder(new Pose2d(-36.11, -62.17, Math.toRadians(90.00)))
                .lineToConstantHeading(new Vector2d(-49, -39.31))
                .setReversed(true)
                .UNSTABLE_addDisplacementMarkerOffset(10, ()->{
                    intake.intakePosition5th(0);
                })
                .back(7)
                .splineTo(new Vector2d(-49, -49), -120)
                //.splineTo(new Vector2d(-35.08, 52.85), Math.toRadians(120))
                .splineTo(new Vector2d(-9, -58), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(1, ()->{
                    outtake.depositPosition(0, 0);
                    outtake.setPixelLatch(true);
                })
                .splineTo(new Vector2d(43, -43), Math.toRadians(0.00))
                .setReversed(false)
                .lineTo(new Vector2d(46.5, -29))
                .waitSeconds(0.5)
                .addTemporalMarker(()->{
                    outtake.setPixelLatch(false);
                })
                .waitSeconds(1)
                .addTemporalMarker(()->{
                    outtake.transferPosition();
                    intake.intakePosition(0);
                })
                .splineTo(new Vector2d(-9, -58), Math.toRadians(180))
                .splineTo(new Vector2d(-56, -41), Math.toRadians(150))
                //.splineTo(new Vector2d(-49, 49), Math.toRadians(216))
                .UNSTABLE_addDisplacementMarkerOffset(-20, ()->{
                    intake.intakePosition4th(0);
                    intake.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    intake.transferPosition();
                })
                .UNSTABLE_addTemporalMarkerOffset(1.3, ()->{
                    intake.setPower(-0.2);
                })
                .UNSTABLE_addTemporalMarkerOffset(2, ()->{
                    intake.setPower(0);
                    intake.intakePosition5th(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.5, ()->{
                    outtake.depositPosition(150, 0);
                    outtake.setPixelLatch(true);
                })
                .setReversed(true)
                .splineTo(new Vector2d(-9, -58), Math.toRadians(0))
                .splineTo(new Vector2d(44, -39), Math.toRadians(0.00))
                .addTemporalMarker(()->{
                    outtake.setPixelLatch(false);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, ()->{
                    outtake.transferPosition();
                    intake.intakePosition(0);
                })
                .waitSeconds(2)
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
            intake.setTarget(60);
            drive.setPoseEstimate(left.start());
            drive.followTrajectorySequenceAsync(left);
        }
        if (randomization==PropPosition.MIDDLE){
            intake.transferPosition();
            intake.setTarget(60);
            drive.setPoseEstimate(middle.start());
            drive.followTrajectorySequenceAsync(middle);
        }
        if (randomization==PropPosition.RIGHT){
            intake.transferPosition();
            intake.setTarget(60);
            drive.setPoseEstimate(right.start());
            drive.followTrajectorySequenceAsync(right);
        }
        int numberOfTimesRead=0;
        double loopTime=0.0;
        while (drive.isBusy() && opModeIsActive()){
            hubs.forEach(LynxModule::clearBulkCache);
            drive.update();
            intake.update();
            outtake.update();

            telemetry.addData("outtake position", outtake.getEncoderPos());
            telemetry.addData("intake position", intake.getEncoderPos());
            telemetry.addData("times read", numberOfTimesRead);
            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.update();
        }
        if (opModeIsActive()){
            sleep(1000);
        }
    }
}