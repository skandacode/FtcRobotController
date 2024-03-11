package org.firstinspires.ftc.teamcode.auto;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.auto.notshown.BluePipeline;
import org.firstinspires.ftc.teamcode.auto.notshown.PropPosition;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;
import java.util.Objects;
@Autonomous
public class BlueAutoSensors extends LinearOpMode {

    OpenCvWebcam webcam;
    PropPosition randomization=PropPosition.NONE;
    Intake intake = new Intake();
    Outtake outtake = new Outtake();

    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;
    IMU imu;

    public static String ObjectDirection;

    enum movementStates{
        yellowAndToStack,
        toStackFromBackdrop,
        toStackFromStage,
        toStagefromStack,
        toBackdropfromStage,
        PARK
    }
    enum possiblePark{MIDDLE, FAR}

    @Override
    public void runOpMode()
    {
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
        intake.init(hardwareMap);
        outtake.init(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        BluePipeline pipeline = new BluePipeline(telemetry, ObjectDirection);
        webcam.setPipeline(pipeline);
        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.


        Rev2mDistanceSensor left2m=hardwareMap.get(Rev2mDistanceSensor.class, "left2m");
        Rev2mDistanceSensor right2m=hardwareMap.get(Rev2mDistanceSensor.class, "right2m");

        TrajectorySequence yellowAndToStackPathLeft=drive.trajectorySequenceBuilder(new Pose2d(11.83, 62.16, Math.toRadians(270.00)))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(90), DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(35))

                .addTemporalMarker(()->{
                    intake.transferPosition();
                    intake.setTarget(40);
                })
                .lineTo(new Vector2d(26, 42))
                .setReversed(true)
                .UNSTABLE_addDisplacementMarkerOffset(10, ()->{
                    intake.stay(0);
                })
                .UNSTABLE_addDisplacementMarkerOffset(20, ()->{
                    outtake.depositPosition(0, 0);
                    outtake.setPixelLatch(true);
                })
                .splineTo(new Vector2d(43, 44), Math.toRadians(0.00))
                .setReversed(false)
                .addTemporalMarker(()->{
                    outtake.setPixelLatch(false);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, ()->{
                    outtake.transferPosition();
                })
                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(30, 9, Math.toRadians(180)), Math.toRadians(180))
                .UNSTABLE_addDisplacementMarkerOffset(0, ()->{
                    intake.stay( 0);
                    intake.setPower(1);
                })
                .lineToConstantHeading(new Vector2d(-23, 9))//first pickup
                .build();
        TrajectorySequence toStagefromStackPathLeft=drive.trajectorySequenceBuilder(yellowAndToStackPathLeft.end())
                .lineTo(new Vector2d(47, 12))
                .build();
        TrajectorySequence toStackFromStageLeft=drive.trajectorySequenceBuilder(new Pose2d(11.83, 62.16, Math.toRadians(270.00)))
                .lineTo(new Vector2d(12, 62))
                .build();
        TrajectorySequence toBackdropfromStageLeft=drive.trajectorySequenceBuilder(new Pose2d(11.83, 62.16, Math.toRadians(270.00)))
                .lineTo(new Vector2d(12, 62))
                .build();
        TrajectorySequence toStackfromBackdropLeft=drive.trajectorySequenceBuilder(new Pose2d(11.83, 62.16, Math.toRadians(270.00)))
                .lineTo(new Vector2d(12, 62))
                .build();
        ElapsedTime timer=new ElapsedTime();
        possiblePark parkPos=possiblePark.FAR;

        StateMachine autoMachineLeft=new StateMachineBuilder()
                .state(movementStates.yellowAndToStack)
                .onEnter(()->drive.followTrajectorySequenceAsync(yellowAndToStackPathLeft))
                .transition(()->!drive.isBusy(), movementStates.toStagefromStack)//gets to the stack

                .state(movementStates.toStagefromStack)
                .onEnter(()->drive.followTrajectorySequenceAsync(toStagefromStackPathLeft))
                .transition(()->(!drive.isBusy() && right2m.getDistance(DistanceUnit.CM)<60), movementStates.toStackFromStage)//senses other robot
                .transition(()->(!drive.isBusy() && !(right2m.getDistance(DistanceUnit.CM)<60)), movementStates.toBackdropfromStage)//does not see
                .transition(()->timer.milliseconds()>20000, movementStates.PARK)

                .state(movementStates.toStackFromStage)
                .onEnter(()->drive.followTrajectorySequenceAsync(toStackFromStageLeft))
                .transition(()->!drive.isBusy(), movementStates.toStagefromStack)

                .state(movementStates.toBackdropfromStage)
                .onEnter(()->drive.followTrajectorySequenceAsync(toBackdropfromStageLeft))
                .transition(()->!drive.isBusy(), movementStates.toStackFromBackdrop)

                .state(movementStates.toStackFromBackdrop)
                .onEnter(()->drive.followTrajectorySequenceAsync(toStackfromBackdropLeft))
                .transition(()->!drive.isBusy(), movementStates.toStagefromStack)

                .state(movementStates.PARK)
                .onEnter(()->{
                    if (parkPos==possiblePark.MIDDLE){

                    }else{

                    }
                })
                .build();


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

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                        )
                )
        );
        imu.resetYaw();

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

        initAprilTag();

        timer.reset();
        outtake.transferPosition();
        intake.transferPosition();
        if (randomization==PropPosition.LEFT){
            drive.setPoseEstimate(yellowAndToStackPathLeft.start());
            autoMachineLeft.start();
        }
        if (randomization==PropPosition.MIDDLE){
        }
        if (randomization==PropPosition.RIGHT){
        }
        int numberOfTimesRead=0;
        double loopTime=0.0;
        while (drive.isBusy() && opModeIsActive()){
            hubs.forEach(LynxModule::clearBulkCache);
            autoMachineLeft.update();
            drive.update();
            intake.update();
            outtake.update();
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            Pose2d updated= telemetryAprilTag(orientation.getYaw(AngleUnit.DEGREES));
            if (updated.equals(new Pose2d())){

            }else{
                drive.setPoseEstimate(updated);
                numberOfTimesRead++;
            }
            telemetry.addData("outtake position", outtake.getEncoderPos());
            telemetry.addData("intake position", intake.getEncoderPos());
            telemetry.addData("MOREIMU", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("times read", numberOfTimesRead);

            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.update();
        }
    }
    public void initAprilTag() {

        aprilTag = new AprilTagProcessor.Builder().build();
        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));



        builder.addProcessor(new VisionProcessor() {
            @Override
            public void init(int width, int height, CameraCalibration calibration) {

            }

            @Override
            public Mat processFrame(Mat frame, long captureTimeNanos) {
                return null;
            }

            @Override
            public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

            }
        });
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();

    }   // end method initAprilTag()


    /**
     * Function to add telemetry about AprilTag detections.
     */
    public Pose2d telemetryAprilTag(double heading) {

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
                Translation2d toApril=new Translation2d(detection.ftcPose.x, detection.ftcPose.y+8.5).rotateBy(new Rotation2d(-Math.toRadians(-heading-90)));
                Translation2d robotTranslation=aprilPose.minus(toApril);
                Rotation2d robotRotation=Rotation2d.fromDegrees(180-(-heading-90));
                com.arcrobotics.ftclib.geometry.Pose2d robotPosition=new com.arcrobotics.ftclib.geometry.Pose2d(robotTranslation.rotateBy(Rotation2d.fromDegrees(180)), robotRotation);
                telemetry.addLine(robotPosition.toString());
                telemetry.addData("IMU", heading);
                telemetry.addData("What we see", heading+90);
                telemetry.addData("Tag", detection.ftcPose.yaw);
                double adding=40.932;
                if (detection.id==7){
                    adding=adding*-1;
                }

                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                return new Pose2d(robotPosition.getY(), adding-robotPosition.getX(), robotPosition.getRotation().getRadians());
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
    }   // end method telemetryAprilTag(
}