package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.auto.notshown.BluePipeline;
import org.firstinspires.ftc.teamcode.auto.notshown.PropPosition;
import org.firstinspires.ftc.teamcode.auto.notshown.RedPipeline;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
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
public class RedFarSensors extends LinearOpMode
{
    OpenCvWebcam webcam;
    PropPosition randomization=PropPosition.NONE;
    Intake intake = new Intake();
    Outtake outtake = new Outtake();
    Rev2mDistanceSensor left2m, right2m;
    double loopTime=0;
    List<LynxModule> hubs;
    SampleMecanumDrive drive;
    AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    public static String ObjectDirection;

    @Override
    public void runOpMode() throws InterruptedException{
        intake.init(hardwareMap);
        outtake.init(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        drive = new SampleMecanumDrive(hardwareMap);

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

        left2m=hardwareMap.get(Rev2mDistanceSensor.class, "left2m");
        right2m=hardwareMap.get(Rev2mDistanceSensor.class, "right2m");

        TrajectorySequence rightPurple = drive.trajectorySequenceBuilder(new Pose2d(-36.11, -62.17, Math.toRadians(90)))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(90), DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(45))
                .splineTo(new Vector2d(-32.23, -39.31), Math.toRadians(42.71))
                .setReversed(true)
                .UNSTABLE_addDisplacementMarkerOffset(10, ()->{
                    intake.intakePosition5th(0);
                })
                .splineTo(new Vector2d(-35.08, -52.85), Math.toRadians(-60))
                .splineTo(new Vector2d(-9, -58), Math.toRadians(0))
                .back(48)
                .UNSTABLE_addTemporalMarkerOffset(-0.7, ()->{
                    outtake.depositPosition(0);
                    outtake.setPixelLatch(true);
                })
                .build();
        TrajectorySequence rightYellow = drive.trajectorySequenceBuilder(rightPurple.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(90), DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(45))
                .lineTo(new Vector2d(46, -44))
                .setReversed(false)
                .addTemporalMarker(()->{
                    outtake.setPixelLatch(false);
                })
                .waitSeconds(1)
                .addTemporalMarker(()->{
                    outtake.transferPosition();
                    intake.intakePosition(0);
                })
                .build();
        TrajectorySequence rightWhite = drive.trajectorySequenceBuilder(rightYellow.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(90), DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(45))
                .splineTo(new Vector2d(-9, -58), Math.toRadians(180))
                .splineTo(new Vector2d(-56, -43), Math.toRadians(150))
                .UNSTABLE_addDisplacementMarkerOffset(-20, ()->{
                    intake.intakePosition4th(0);
                    intake.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->{
                    intake.intakePosition2nd();
                })
                .waitSeconds(1)
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
                    outtake.depositPosition(150);
                    outtake.setPixelLatch(true);
                })
                .setReversed(true)
                .splineTo(new Vector2d(-9, -58), Math.toRadians(0))
                .back(48)
                .lineTo(new Vector2d(43.5, -39))
                .addTemporalMarker(()->{
                    outtake.setPixelLatch(false);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, ()->{
                    outtake.transferPosition();
                    intake.intakePosition(0);
                })
                .waitSeconds(2)
                .build();
        //===============================================================================================

        TrajectorySequence middlePurple = drive.trajectorySequenceBuilder(new Pose2d(-36.11, -62.17, Math.toRadians(90)))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(90), DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(45))
                .splineTo(new Vector2d(-36.11, -34.51), Math.toRadians(90))
                .setReversed(true)
                .UNSTABLE_addDisplacementMarkerOffset(10, ()->{
                    intake.intakePosition5th(0);
                })
                .splineTo(new Vector2d(-35.08, -52.85), Math.toRadians(-60))
                .splineTo(new Vector2d(-9, -58), Math.toRadians(0))
                .back(48)
                .UNSTABLE_addTemporalMarkerOffset(-0.7, ()->{
                    outtake.depositPosition(0);
                    outtake.setPixelLatch(true);
                })
                .build();
        TrajectorySequence middleYellow = drive.trajectorySequenceBuilder(middlePurple.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(90), DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(45))
                .lineTo(new Vector2d(46, -34))
                .setReversed(false)
                .addTemporalMarker(()->{
                    outtake.setPixelLatch(false);
                })
                .waitSeconds(1)
                .addTemporalMarker(()->{
                    outtake.transferPosition();
                    intake.intakePosition(0);
                })
                .build();
        TrajectorySequence middleWhite = drive.trajectorySequenceBuilder(middleYellow.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(90), DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(45))
                .splineTo(new Vector2d(-9, -58), Math.toRadians(180))
                .splineTo(new Vector2d(-56, -42), Math.toRadians(150))
                .UNSTABLE_addDisplacementMarkerOffset(-20, ()->{
                    intake.intakePosition4th(0);
                    intake.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->{
                    intake.intakePosition2nd();
                })
                .waitSeconds(1)
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
                    outtake.depositPosition(150);
                    outtake.setPixelLatch(true);
                })
                .setReversed(true)
                .splineTo(new Vector2d(-9, -58), Math.toRadians(0))
                .back(48)
                .lineTo(new Vector2d(43.5, -39))
                .addTemporalMarker(()->{
                    outtake.setPixelLatch(false);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, ()->{
                    outtake.transferPosition();
                    intake.intakePosition(0);
                })
                .waitSeconds(2)
                .build();
        //============================================================================

        TrajectorySequence leftPurple = drive.trajectorySequenceBuilder(new Pose2d(-36.11, -62.17, Math.toRadians(90)))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(90), DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(45))
                .lineToConstantHeading(new Vector2d(-49, -39.31))
                .setReversed(true)
                .UNSTABLE_addDisplacementMarkerOffset(10, ()->{
                    intake.intakePosition5th(0);
                })
                .back(6)
                .splineTo(new Vector2d(-45, -52.85), Math.toRadians(-45))
                .splineTo(new Vector2d(-9, -58), Math.toRadians(0))
                .back(48)
                .UNSTABLE_addTemporalMarkerOffset(-0.7, ()->{
                    outtake.depositPosition(0);
                    outtake.setPixelLatch(true);
                })
                .build();
        TrajectorySequence leftYellow = drive.trajectorySequenceBuilder(leftPurple.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(90), DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(45))
                .lineTo(new Vector2d(46, -29))
                .setReversed(false)
                .addTemporalMarker(()->{
                    outtake.setPixelLatch(false);
                })
                .waitSeconds(1)
                .addTemporalMarker(()->{
                    outtake.transferPosition();
                    intake.intakePosition(0);
                })
                .build();
        TrajectorySequence leftWhite = drive.trajectorySequenceBuilder(leftYellow.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(90), DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(45))
                .splineTo(new Vector2d(10, -58), Math.toRadians(180))
                .splineTo(new Vector2d(-9, -58), Math.toRadians(180))
                .splineTo(new Vector2d(-55.5, -43.5), Math.toRadians(150))
                .UNSTABLE_addDisplacementMarkerOffset(-20, ()->{
                    intake.intakePosition4th(0);
                    intake.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->{
                    intake.intakePosition();
                })
                .waitSeconds(1)
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
                    outtake.depositPosition(150);
                    outtake.setPixelLatch(true);
                })
                .setReversed(true)
                .splineTo(new Vector2d(-9, -58), Math.toRadians(0))
                .back(48)
                .lineTo(new Vector2d(42.5, -39))
                .addTemporalMarker(()->{
                    outtake.setPixelLatch(false);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, ()->{
                    outtake.transferPosition();
                    intake.intakePosition(0);
                })
                .waitSeconds(2)
                .build();

        //===============================================================

        TrajectorySequence parkCorner=drive.trajectorySequenceBuilder(leftWhite.end())
                .addTemporalMarker(()->{
                    outtake.transferPosition();
                    intake.intakePosition5th(0);
                })
                .splineToConstantHeading(new Vector2d(55, -60), Math.toRadians(0.00))
                .build();


        hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

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
            drive.setPoseEstimate(leftPurple.start());
            drive.followTrajectorySequenceAsync(leftPurple);
            while (drive.isBusy() && opModeIsActive()){updateSystems();}
            while (right2m.getDistance(DistanceUnit.CM)<60){
                if (timer.seconds()>25){
                    break;
                }
            }
            drive.followTrajectorySequenceAsync(leftYellow);
            while (drive.isBusy() && opModeIsActive()){updateSystems();}
            if (timer.seconds()<20){
                drive.followTrajectorySequenceAsync(leftWhite);
                while (drive.isBusy() && opModeIsActive()){updateSystems();}
                drive.followTrajectorySequenceAsync(parkCorner);
                while (drive.isBusy() && opModeIsActive()){updateSystems();}

            }

        }
        if (randomization==PropPosition.MIDDLE){
            intake.transferPosition();
            intake.setTarget(60);
            drive.setPoseEstimate(middlePurple.start());
            drive.followTrajectorySequenceAsync(middlePurple);
            while (drive.isBusy() && opModeIsActive()){updateSystems();}
            while (right2m.getDistance(DistanceUnit.CM)<60){
                if (timer.seconds()>25){
                    break;
                }
            }
            drive.followTrajectorySequenceAsync(middleYellow);
            while (drive.isBusy() && opModeIsActive()){updateSystems();}
            if (timer.seconds()<20){
                drive.followTrajectorySequenceAsync(middleWhite);
                while (drive.isBusy() && opModeIsActive()){updateSystems();}
                drive.followTrajectorySequenceAsync(parkCorner);
                while (drive.isBusy() && opModeIsActive()){updateSystems();}

            }
        }
        if (randomization==PropPosition.RIGHT){
            intake.transferPosition();
            intake.setTarget(60);
            drive.setPoseEstimate(rightPurple.start());
            drive.followTrajectorySequenceAsync(rightPurple);
            while (drive.isBusy() && opModeIsActive()){updateSystems();}
            while (right2m.getDistance(DistanceUnit.CM)<60){
                if (timer.seconds()>25){
                    break;
                }
            }
            drive.followTrajectorySequenceAsync(rightYellow);
            while (drive.isBusy() && opModeIsActive()){updateSystems();}
            if (timer.seconds()<20){
                drive.followTrajectorySequenceAsync(rightWhite);
                while (drive.isBusy() && opModeIsActive()){updateSystems();}
                drive.followTrajectorySequenceAsync(parkCorner);
                while (drive.isBusy() && opModeIsActive()){updateSystems();}

            }
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