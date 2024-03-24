package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
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
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;
import java.util.Objects;

@Autonomous
public class RedAutoSensors extends LinearOpMode {

    OpenCvWebcam webcam;
    PropPosition randomization=PropPosition.NONE;
    Intake intake = new Intake();
    Outtake outtake = new Outtake();

    public static String ObjectDirection;
    SampleMecanumDrive drive;
    Rev2mDistanceSensor left2m, right2m;
    enum possiblePark{MIDDLE, FAR}
    double loopTime=0;
    List<LynxModule> hubs;
    @Override
    public void runOpMode()
    {
        intake.init(hardwareMap);
        outtake.init(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        drive = new SampleMecanumDrive(hardwareMap);

        RedPipeline pipeline = new RedPipeline(telemetry, ObjectDirection);
        webcam.setPipeline(pipeline);
        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.


        left2m=hardwareMap.get(Rev2mDistanceSensor.class, "left2m");
        right2m=hardwareMap.get(Rev2mDistanceSensor.class, "right2m");

        TrajectorySequence yellowAndToStackPathRight=drive.trajectorySequenceBuilder(new Pose2d(11.83, -62.16, Math.toRadians(90)))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, Math.toRadians(90), DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(50))

                .addTemporalMarker(()->{
                    intake.stay(80);
                })
                .lineTo(new Vector2d(25, -42))
                .setReversed(true)
                .addTemporalMarker(()->{
                    intake.stay(0);
                })
                .UNSTABLE_addDisplacementMarkerOffset(10, ()->{
                    outtake.depositPosition(0);
                    outtake.setPixelLatch(true);
                })
                .back(4)
                .lineToLinearHeading(new Pose2d(47.5, -41, Math.toRadians(180.00)))
                .setReversed(false)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(90), DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(45))
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->{
                    outtake.setPixelLatch(false);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, ()->{
                    outtake.transferPosition();
                })
                .waitSeconds(1.5)
                .lineToSplineHeading(new Pose2d(30, -12, Math.toRadians(183)))
                .splineToConstantHeading(new Vector2d(20, -12), Math.toRadians(183))
                .UNSTABLE_addDisplacementMarkerOffset(0, ()->{
                    intake.intakePosition5th(900);
                    intake.setPower(1);
                })
                .forward(43.5)//first pickup
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->{
                    intake.intakePosition4th();
                })
                .waitSeconds(1)
                .build();
        TrajectorySequence toStagefromStackPathRight=drive.trajectorySequenceBuilder(yellowAndToStackPathRight.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(90), DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(45))
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
                .lineTo(new Vector2d(43, -12))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()->{
                    outtake.depositPosition(0);
                    outtake.setPixelLatch(true);
                })
                .build();
        TrajectorySequence toStackFromStageRight=drive.trajectorySequenceBuilder(toStagefromStackPathRight.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(90), DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(45))
                .addTemporalMarker(()->{
                    outtake.setPixelLatch(false);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->{
                    outtake.transferPosition();
                })
                .waitSeconds(1)
                .addTemporalMarker(()->{
                    intake.intakePositionExtended(900);
                    intake.setPower(1);
                })
                .lineToLinearHeading(new Pose2d(-25, -8, Math.toRadians(183)))
                .waitSeconds(1)
                .build();
        TrajectorySequence toBackdropfromStageRight=drive.trajectorySequenceBuilder(toStagefromStackPathRight.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(90), DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(45))
                .lineTo(new Vector2d(43, -35))
                .addTemporalMarker(()->{
                    outtake.depositPosition(330);
                })
                .waitSeconds(1)
                .addTemporalMarker(()->{
                    outtake.setPixelLatch(false);
                })
                .waitSeconds(1)
                .build();
        TrajectorySequence toStackfromBackdropRight=drive.trajectorySequenceBuilder(toBackdropfromStageRight.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(90), DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(45))
                .addTemporalMarker(()->{
                    outtake.transferPosition();
                })
                .lineToSplineHeading(new Pose2d(30, -12, Math.toRadians(183)))
                .splineToConstantHeading(new Vector2d(20, -14), Math.toRadians(183))
                .UNSTABLE_addDisplacementMarkerOffset(0, ()->{
                    intake.intakePositionExtended(900);
                    intake.setPower(1);
                })
                .forward(43.5)
                .waitSeconds(1)
                .build();


        //=============================================================================

        TrajectorySequence yellowAndToStackPathMiddle=drive.trajectorySequenceBuilder(new Pose2d(11.83, -62.16, Math.toRadians(90)))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, Math.toRadians(90), DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(50))

                .addTemporalMarker(()->{
                    intake.stay(80);
                })
                .lineTo(new Vector2d(11.75, -34))
                .setReversed(true)
                .addTemporalMarker(()->{
                    intake.stay(0);
                })
                .UNSTABLE_addDisplacementMarkerOffset(10, ()->{
                    outtake.depositPosition(0);
                    outtake.setPixelLatch(true);
                })
                .back(4)
                .lineToLinearHeading(new Pose2d(48, -34.5, Math.toRadians(180.00)))
                .setReversed(false)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(90), DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(45))
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->{
                    outtake.setPixelLatch(false);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, ()->{
                    outtake.transferPosition();
                })
                .waitSeconds(1.5)
                .lineToSplineHeading(new Pose2d(30, -12, Math.toRadians(183)))
                .splineToConstantHeading(new Vector2d(20, -12), Math.toRadians(183))
                .UNSTABLE_addDisplacementMarkerOffset(0, ()->{
                    intake.intakePosition5th(900);
                    intake.setPower(1);
                })
                .forward(42)//first pickup
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->{
                    intake.intakePosition4th();
                })
                .waitSeconds(1)
                .build();
        TrajectorySequence toStagefromStackPathMiddle=drive.trajectorySequenceBuilder(yellowAndToStackPathMiddle.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(90), DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(45))
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
                .lineTo(new Vector2d(43, -12))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()->{
                    outtake.depositPosition(0);
                    outtake.setPixelLatch(true);
                })
                .build();
        TrajectorySequence toStackFromStageMiddle=drive.trajectorySequenceBuilder(toStagefromStackPathMiddle.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(90), DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(45))
                .addTemporalMarker(()->{
                    outtake.setPixelLatch(false);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->{
                    outtake.transferPosition();
                })
                .waitSeconds(1)
                .addTemporalMarker(()->{
                    intake.intakePositionExtended(900);
                    intake.setPower(1);
                })
                .lineToLinearHeading(new Pose2d(-24.5, -10, Math.toRadians(182)))
                .waitSeconds(1)
                .build();
        TrajectorySequence toBackdropfromStageMiddle=drive.trajectorySequenceBuilder(toStagefromStackPathMiddle.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(90), DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(45))
                .lineTo(new Vector2d(43, -35))
                .addTemporalMarker(()->{
                    outtake.depositPosition(330);
                })
                .waitSeconds(1)
                .addTemporalMarker(()->{
                    outtake.setPixelLatch(false);
                })
                .waitSeconds(1)
                .build();
        TrajectorySequence toStackfromBackdropMiddle=drive.trajectorySequenceBuilder(toBackdropfromStageMiddle.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(90), DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(45))
                .addTemporalMarker(()->{
                    outtake.transferPosition();
                })
                .lineToSplineHeading(new Pose2d(30, -12, Math.toRadians(182)))
                .splineToConstantHeading(new Vector2d(20, -10), Math.toRadians(182))
                .UNSTABLE_addDisplacementMarkerOffset(0, ()->{
                    intake.intakePositionExtended(900);
                    intake.setPower(1);
                })
                .forward(43)
                .waitSeconds(1)
                .build();
        //===========================================================================
        TrajectorySequence yellowAndToStackPathLeft=drive.trajectorySequenceBuilder(new Pose2d(11.83, -62.16, Math.toRadians(90)))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, Math.toRadians(90), DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(50))

                .addTemporalMarker(()->{
                    intake.transferPosition();
                    intake.setTarget(80);
                })
                .splineTo(new Vector2d(7, -37), Math.toRadians(360-228.62))
                .setReversed(true)
                .addTemporalMarker(()->{
                    intake.stay(0);
                })
                .UNSTABLE_addDisplacementMarkerOffset(10, ()->{
                    outtake.depositPosition(0);
                    outtake.setPixelLatch(true);
                })
                .back(4)
                .lineToLinearHeading(new Pose2d(47.5, -29, Math.toRadians(180.00)))
                .setReversed(false)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(90), DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(45))
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->{
                    outtake.setPixelLatch(false);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, ()->{
                    outtake.transferPosition();
                })
                .waitSeconds(1.5)
                .lineToSplineHeading(new Pose2d(30, -12, Math.toRadians(182)))
                .splineToConstantHeading(new Vector2d(20, -12), Math.toRadians(182))
                .UNSTABLE_addDisplacementMarkerOffset(0, ()->{
                    intake.intakePosition5th(900);
                    intake.setPower(1);
                })
                .forward(44)//first pickup
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->{
                    intake.intakePosition4th();
                })
                .waitSeconds(1)
                .build();
        TrajectorySequence toStagefromStackPathLeft=drive.trajectorySequenceBuilder(yellowAndToStackPathRight.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(90), DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(45))
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
                .lineTo(new Vector2d(41, -12))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()->{
                    outtake.depositPosition(0);
                    outtake.setPixelLatch(true);
                })
                .build();
        TrajectorySequence toStackFromStageLeft=drive.trajectorySequenceBuilder(toStagefromStackPathRight.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(90), DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(45))
                .addTemporalMarker(()->{
                    outtake.setPixelLatch(false);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->{
                    outtake.transferPosition();
                })
                .waitSeconds(1)
                .addTemporalMarker(()->{
                    intake.intakePositionExtended(900);
                    intake.setPower(1);
                })
                .lineToLinearHeading(new Pose2d(-24.5, -10, Math.toRadians(184)))
                .waitSeconds(1)
                .build();
        TrajectorySequence toBackdropfromStageLeft=drive.trajectorySequenceBuilder(toStagefromStackPathRight.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(90), DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(45))
                .lineTo(new Vector2d(41.5, -35))
                .addTemporalMarker(()->{
                    outtake.depositPosition(330);
                })
                .waitSeconds(1)
                .addTemporalMarker(()->{
                    outtake.setPixelLatch(false);
                })
                .waitSeconds(1)
                .build();
        TrajectorySequence toStackfromBackdropLeft=drive.trajectorySequenceBuilder(toBackdropfromStageRight.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(90), DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(45))
                .addTemporalMarker(()->{
                    outtake.transferPosition();
                })
                .lineToSplineHeading(new Pose2d(30, -12, Math.toRadians(184)))
                .splineToConstantHeading(new Vector2d(20, -11), Math.toRadians(184))
                .UNSTABLE_addDisplacementMarkerOffset(0, ()->{
                    intake.intakePositionExtended(900);
                    intake.setPower(1);
                })
                .forward(44)
                .waitSeconds(1)
                .build();
        //===========================================================================
        possiblePark parkPos= possiblePark.MIDDLE;
        TrajectorySequence parkCorner=drive.trajectorySequenceBuilder(toBackdropfromStageLeft.end())
                .addTemporalMarker(()->{
                    outtake.transferPosition();
                    intake.intakePosition5th(0);
                })
                .splineToConstantHeading(new Vector2d(55, -60), Math.toRadians(0.00))
                .build();
        TrajectorySequence parkMiddle=drive.trajectorySequenceBuilder(toBackdropfromStageLeft.end())
                .addTemporalMarker(()->{
                    outtake.transferPosition();
                    intake.intakePosition5th(0);
                })
                .splineToConstantHeading(new Vector2d(55, -10), Math.toRadians(0.00))
                .build();

        hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

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
        timer.reset();

        outtake.transferPosition();
        intake.transferPosition();
        int timesSeen=0;
        if (randomization==PropPosition.LEFT){
            drive.setPoseEstimate(yellowAndToStackPathLeft.start());
            drive.followTrajectorySequenceAsync(yellowAndToStackPathLeft);
            while (drive.isBusy() && opModeIsActive()){updateSystems();}
            drive.followTrajectorySequenceAsync(toStagefromStackPathLeft);
            while (drive.isBusy() && opModeIsActive()){updateSystems();}
            if (left2m.getDistance(DistanceUnit.CM)<60){//senses other robot
                drive.followTrajectorySequenceAsync(toStackFromStageLeft);
                while (drive.isBusy() && opModeIsActive()){updateSystems();}
                timesSeen++;
            }else{
                drive.followTrajectorySequenceAsync(toBackdropfromStageLeft);
                while (drive.isBusy() && opModeIsActive()){updateSystems();}
                drive.followTrajectorySequenceAsync(toStackfromBackdropLeft);
                while (drive.isBusy() && opModeIsActive()){updateSystems();}
            }
            drive.followTrajectorySequenceAsync(toStagefromStackPathLeft);
            while (drive.isBusy() && opModeIsActive()){updateSystems();}
            if (left2m.getDistance(DistanceUnit.CM)<60){//senses other robot
                outtake.setPixelLatch(false);
                sleep(500);
                outtake.transferPosition();
                sleep(500);
            }else{
                drive.followTrajectorySequenceAsync(toBackdropfromStageLeft);
                while (drive.isBusy() && opModeIsActive()){updateSystems();}
                if (timesSeen==1) {
                    if (parkPos == possiblePark.FAR) {
                        drive.followTrajectorySequenceAsync(parkCorner);
                    } else {
                        drive.followTrajectorySequenceAsync(parkMiddle);
                    }
                    while (drive.isBusy() && opModeIsActive()) {
                        updateSystems();
                    }
                }
            }
        }
        if (randomization==PropPosition.MIDDLE){
            drive.setPoseEstimate(yellowAndToStackPathMiddle.start());
            drive.followTrajectorySequenceAsync(yellowAndToStackPathMiddle);
            while (drive.isBusy() && opModeIsActive()){updateSystems();}
            drive.followTrajectorySequenceAsync(toStagefromStackPathMiddle);
            while (drive.isBusy() && opModeIsActive()){updateSystems();}
            if (left2m.getDistance(DistanceUnit.CM)<60){//senses other robot
                drive.followTrajectorySequenceAsync(toStackFromStageMiddle);
                while (drive.isBusy() && opModeIsActive()){updateSystems();}
                timesSeen++;
            }else{
                drive.followTrajectorySequenceAsync(toBackdropfromStageMiddle);
                while (drive.isBusy() && opModeIsActive()){updateSystems();}
                drive.followTrajectorySequenceAsync(toStackfromBackdropMiddle);
                while (drive.isBusy() && opModeIsActive()){updateSystems();}
            }
            drive.followTrajectorySequenceAsync(toStagefromStackPathMiddle);
            while (drive.isBusy() && opModeIsActive()){updateSystems();}
            if (left2m.getDistance(DistanceUnit.CM)<60){//senses other robot
                outtake.setPixelLatch(false);
                sleep(500);
                outtake.transferPosition();
                sleep(500);
            }else{
                drive.followTrajectorySequenceAsync(toBackdropfromStageMiddle);
                while (drive.isBusy() && opModeIsActive()){updateSystems();}
                if (timesSeen==1) {
                    if (parkPos == possiblePark.FAR) {
                        drive.followTrajectorySequenceAsync(parkCorner);
                    } else {
                        drive.followTrajectorySequenceAsync(parkMiddle);
                    }
                    while (drive.isBusy() && opModeIsActive()) {
                        updateSystems();
                    }
                }
            }
        }
        if (randomization==PropPosition.RIGHT){
            drive.setPoseEstimate(yellowAndToStackPathRight.start());
            drive.followTrajectorySequenceAsync(yellowAndToStackPathRight);
            while (drive.isBusy() && opModeIsActive()){updateSystems();}
            drive.followTrajectorySequenceAsync(toStagefromStackPathRight);
            while (drive.isBusy() && opModeIsActive()){updateSystems();}
            if (left2m.getDistance(DistanceUnit.CM)<60){//senses other robot
                drive.followTrajectorySequenceAsync(toStackFromStageRight);
                while (drive.isBusy() && opModeIsActive()){updateSystems();}
                timesSeen++;
            }else{
                drive.followTrajectorySequenceAsync(toBackdropfromStageRight);
                while (drive.isBusy() && opModeIsActive()){updateSystems();}
                drive.followTrajectorySequenceAsync(toStackfromBackdropRight);
                while (drive.isBusy() && opModeIsActive()){updateSystems();}
            }
            drive.followTrajectorySequenceAsync(toStagefromStackPathRight);
            while (drive.isBusy() && opModeIsActive()){updateSystems();}
            if (left2m.getDistance(DistanceUnit.CM)<60){//senses other robot
                outtake.setPixelLatch(false);
                sleep(500);
                outtake.transferPosition();
                sleep(500);
            }else{
                drive.followTrajectorySequenceAsync(toBackdropfromStageRight);
                while (drive.isBusy() && opModeIsActive()){updateSystems();}
                if (timesSeen==1) {
                    if (parkPos == possiblePark.FAR) {
                        drive.followTrajectorySequenceAsync(parkCorner);
                    } else {
                        drive.followTrajectorySequenceAsync(parkMiddle);
                    }
                    while (drive.isBusy() && opModeIsActive()) {
                        updateSystems();
                    }
                }
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