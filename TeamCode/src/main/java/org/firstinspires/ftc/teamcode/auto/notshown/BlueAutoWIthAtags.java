package org.firstinspires.ftc.teamcode.auto.notshown;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;
import java.util.Objects;

@Config
@Autonomous
public class BlueAutoWIthAtags extends LinearOpMode
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
        System.out.println("starting");
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
        intake.init(hardwareMap);
        outtake.init(hardwareMap);
        int cameraMonitorViewId;
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        BluePipeline pipeline = new BluePipeline(telemetry, ObjectDirection);
        webcam.setPipeline(pipeline);
        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        System.out.println("after setting timeout");
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
                System.out.println("could not be opened");
            }
        });


        TrajectorySequence left=drive.trajectorySequenceBuilder(new Pose2d(11.83, 62.16, Math.toRadians(270.00)))
                .lineTo(new Vector2d(27, 40))
                .setReversed(true)
                .UNSTABLE_addDisplacementMarkerOffset(10, ()->{
                    intake.intakePosition5th(0);
                })
                .UNSTABLE_addDisplacementMarkerOffset(20, ()->{
                    outtake.depositPosition(0);
                    outtake.setPixelLatch(true);
                })
                .splineTo(new Vector2d(48, 43), Math.toRadians(2.00))
                .setReversed(false)
                .addTemporalMarker(()->{
                    outtake.setPixelLatch(false);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, ()->{
                    outtake.transferPosition();
                })
                .waitSeconds(2.5)
                .build();

        TrajectorySequence middle=drive.trajectorySequenceBuilder(new Pose2d(11.83, 62.16, Math.toRadians(270.00)))
                .splineTo(new Vector2d(12, 35), Math.toRadians(270))
                .setReversed(true)
                .UNSTABLE_addDisplacementMarkerOffset(10, ()->{
                    intake.intakePosition5th(0);
                })
                .UNSTABLE_addDisplacementMarkerOffset(30, ()->{
                    outtake.depositPosition(0);
                    outtake.setPixelLatch(true);
                })
                .splineTo(new Vector2d(48, 37), Math.toRadians(2.00))
                .setReversed(false)
                .addTemporalMarker(()->{
                    outtake.setPixelLatch(false);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, ()->{
                    outtake.transferPosition();
                })
                .waitSeconds(2.5)
                .build();
        TrajectorySequence right = drive.trajectorySequenceBuilder(new Pose2d(11.83, 62.16, Math.toRadians(270.00)))
                .splineTo(new Vector2d(8, 37), Math.toRadians(228.62))
                .setReversed(true)
                .UNSTABLE_addDisplacementMarkerOffset(10, ()->{
                    intake.intakePosition5th(0);
                })
                .UNSTABLE_addDisplacementMarkerOffset(20, ()->{
                    outtake.depositPosition(0);
                    outtake.setPixelLatch(true);
                })
                .splineTo(new Vector2d(48, 31), Math.toRadians(2.00))
                .setReversed(false)
                .addTemporalMarker(()->{
                    outtake.setPixelLatch(false);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, ()->{
                    outtake.transferPosition();
                })
                .waitSeconds(2.5)
                .build();

        TrajectorySequence cycle = drive.trajectorySequenceBuilder(new Pose2d(46.24, 25.17, Math.toRadians(220)))
                .splineTo(new Vector2d(-27.25, 10.00), Math.toRadians(181.00))
                .addTemporalMarker(()->{
                    intake.intakePosition5th(800);
                    intake.setPower(1);
                })
                .setReversed(true)
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()->{
                    intake.transferPosition();
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, ()->{
                    intake.setPower(-1);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.5, ()->{
                    intake.setPower(0);
                    intake.intakePosition5th(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(3.5, ()->{
                    outtake.depositPosition(300);
                    outtake.setPixelLatch(true);
                })
                .splineTo(new Vector2d(45.24, 25.17), Math.toRadians(220+180))
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->{
                    outtake.setPixelLatch(false);
                })
                .waitSeconds(2)
                .setReversed(false)
                .UNSTABLE_addDisplacementMarkerOffset(20, ()->{
                    outtake.transferPosition();
                })
                .splineTo(new Vector2d(-26.25, 10.00), Math.toRadians(180.00))
                .UNSTABLE_addDisplacementMarkerOffset(-20, ()->{
                    intake.intakePosition3rd(800);
                    intake.setPower(1);
                })
                .setReversed(true)
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()->{
                    intake.transferPosition();
                })
                /*.UNSTABLE_addTemporalMarkerOffset(1.5, ()->{
                    intake.setPower(-1);
                })*/
                .UNSTABLE_addTemporalMarkerOffset(2.5, ()->{
                    //intake.setPower(0);
                    intake.intakePosition3rd(0);
                })
                /*.UNSTABLE_addTemporalMarkerOffset(3.5, ()->{
                    outtake.depositPosition(300, 0.2);
                })*/
                .splineTo(new Vector2d(45, 25.17), Math.toRadians(220+180))
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
        aprilTag = new AprilTagProcessor.Builder()
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);
        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
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
        int numberOfTimesRead=0;
        double loopTime=0.0;
        while (drive.isBusy() && opModeIsActive()){
            hubs.forEach(LynxModule::clearBulkCache);
            drive.update();
            intake.update();
            outtake.update();

            telemetry.addData("outtake position", outtake.getEncoderPos());
            telemetry.addData("intake position", intake.getEncoderPos());
            Pose2d updated= telemetryAprilTag(drive.getPoseEstimate());
            if (updated.equals(new Pose2d())){

            }else{
                drive.setPoseEstimate(updated);
                numberOfTimesRead++;
            }
            telemetry.addData("times read", numberOfTimesRead);
            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.update();
        }
        drive.followTrajectorySequenceAsync(cycle);
        while (drive.isBusy() && opModeIsActive()){
            hubs.forEach(LynxModule::clearBulkCache);
            drive.update();
            intake.update();
            outtake.update();
            Pose2d updated= telemetryAprilTag(drive.getPoseEstimate());
            if (updated.equals(new Pose2d())){

            }else{
                drive.setPoseEstimate(updated);
                numberOfTimesRead++;
            }
            telemetry.addData("outtake position", outtake.getEncoderPos());
            telemetry.addData("intake position", intake.getEncoderPos());

            telemetry.addData("times read", numberOfTimesRead);

            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.update();
        }
        //visionPortal.close();
    }
    public Pose2d telemetryAprilTag(Pose2d currPos) {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        FtcDashboard dashboard=FtcDashboard.getInstance();

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (detection.id==10 || detection.id==7){
                }else{
                    continue;
                }
                Translation2d aprilPose=new Translation2d(0, 72);
                Translation2d toApril=new Translation2d(detection.ftcPose.x, detection.ftcPose.y+8.5).rotateBy(new Rotation2d(-Math.toRadians(detection.ftcPose.yaw)));
                Translation2d robotTranslation=aprilPose.minus(toApril);
                Rotation2d robotRotation=Rotation2d.fromDegrees((180)-detection.ftcPose.yaw);
                com.arcrobotics.ftclib.geometry.Pose2d robotPosition=new com.arcrobotics.ftclib.geometry.Pose2d(robotTranslation.rotateBy(Rotation2d.fromDegrees(180)), robotRotation);
                telemetry.addLine(robotPosition.toString());
                TelemetryPacket packet = new TelemetryPacket();
                double adding=40.932;
                if (detection.id==7){
                    adding=adding*-1;
                }
                com.arcrobotics.ftclib.geometry.Pose2d position=new com.arcrobotics.ftclib.geometry.Pose2d(robotPosition.getY(), adding-robotPosition.getX(), robotPosition.getRotation());

                Pose2d updatedPosition = new Pose2d(position.getX(),position.getY(),position.getHeading());

                packet.fieldOverlay().setFill("blue")
                        .strokeCircle(position.getX(), position.getY(), 9)
                        .strokeLine(position.getX(), position.getY(),
                                (position.getRotation().getCos()*10)+ position.getX(),
                                (position.getRotation().getSin()*10)+ position.getY());

                dashboard.sendTelemetryPacket(packet);
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                return updatedPosition;
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    return new Pose2d();
    }
}