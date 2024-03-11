package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.List;

import android.graphics.Canvas;
import android.util.Size;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.opencv.core.Mat;

/**
 * This 2023-2024 OpMode illustrates the basics of AprilTag recognition and pose estimation,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp
public class AprilTagTest extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;

    public Pose2d position=new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    IMU imu;
    @Override
    public void runOpMode() throws InterruptedException{

        initAprilTag();
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
        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

                telemetryAprilTag(orientation.getYaw(AngleUnit.DEGREES));
                telemetry.addData("MOREIMU", orientation.getYaw(AngleUnit.DEGREES));
                telemetry.update();

                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                // Share the CPU.
                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end method runOpMode()

    /**
     * Initialize the AprilTag processor.
     */
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
    public void telemetryAprilTag(double heading) {

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
                Pose2d robotPosition=new Pose2d(robotTranslation.rotateBy(Rotation2d.fromDegrees(180)), robotRotation);
                telemetry.addLine(robotPosition.toString());
                telemetry.addData("IMU", heading);
                telemetry.addData("What we see", heading+90);
                telemetry.addData("Tag", detection.ftcPose.yaw);

                TelemetryPacket packet = new TelemetryPacket();
                double adding=40.932;
                if (detection.id==7){
                    adding=adding*-1;
                }
                position=new Pose2d(robotPosition.getY(), adding-robotPosition.getX(), robotPosition.getRotation());

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

            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()

}   // end class