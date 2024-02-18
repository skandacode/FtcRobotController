package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
public class BlueAuto extends LinearOpMode
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
                .lineToLinearHeading(new Pose2d(49.5, 41, Math.toRadians(200.00)))
                .UNSTABLE_addDisplacementMarkerOffset(5, ()-> {
                    outtake.setPixelLatch(true);
                    outtake.depositPosition(0, 0);
                })
                .build();

        TrajectorySequence middle=drive.trajectorySequenceBuilder(new Pose2d(11.83, 62.16, Math.toRadians(270.00)))
                .lineToLinearHeading(new Pose2d(50, 35, Math.toRadians(200.00)))
                .UNSTABLE_addDisplacementMarkerOffset(5, ()-> {
                    outtake.setPixelLatch(true);
                    outtake.depositPosition(0, 0);
                })
                .build();
        TrajectorySequence right=drive.trajectorySequenceBuilder(new Pose2d(11.83, 62.16, Math.toRadians(270.00)))
                .lineToLinearHeading(new Pose2d(49, 31, Math.toRadians(180)))
                .UNSTABLE_addDisplacementMarkerOffset(5, ()-> {
                    outtake.setPixelLatch(true);
                    outtake.depositPosition(0, 0);
                })
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
        if (randomization==PropPosition.LEFT){
            drive.setPoseEstimate(left.start());
            intake.purplePosition(0);
            intake.setPower(0.5);
            outtake.transferPosition();
            drive.followTrajectorySequence(left);
            intake.purplePosition(300);
            outtake.setPixelLatch(true);
            outtake.depositPosition(0, -0.05);
            timer.reset();
            while (opModeIsActive() && timer.milliseconds()<2000){
                intake.update();
                outtake.update();
            }
            intake.setPower(-0.25);
            outtake.setPixelLatch(false);
            timer.reset();
            while (opModeIsActive() && timer.milliseconds()<1400){
                if (timer.milliseconds()>400){
                    intake.setPower(0);
                }
                if (timer.milliseconds()>1000){
                    intake.setServos(0.5, 0.2);
                }
            }
            intake.setPower(0);
            intake.intakePosition5th(0);
            outtake.transferPosition();
            outtake.setPixelLatch(false);
            while (opModeIsActive() && timer.milliseconds()<5000){
                intake.update();
                outtake.update();
            }
        }
        if (randomization==PropPosition.MIDDLE){
            drive.setPoseEstimate(middle.start());
            intake.purplePosition(0);
            intake.setPower(0.5);
            outtake.transferPosition();
            drive.followTrajectorySequence(middle);
            intake.purplePosition(500);
            outtake.setPixelLatch(true);
            outtake.depositPosition(0, -0.05);
            timer.reset();
            while (opModeIsActive() && timer.milliseconds()<2000){
                intake.update();
                outtake.update();
            }
            intake.setPower(-0.25);
            outtake.setPixelLatch(false);
            timer.reset();
            while (opModeIsActive() && timer.milliseconds()<1000){
                if (timer.milliseconds()>1000){
                    intake.setPower(0);
                }
                if (timer.milliseconds()>700){
                    intake.setServos(0.5, 0.2);
                }
            }
            intake.setPower(0);
            intake.intakePosition5th(0);
            outtake.transferPosition();
            outtake.setPixelLatch(false);
            while (opModeIsActive() && timer.milliseconds()<5000){
                intake.update();
                outtake.update();
            }
        }
        if (randomization==PropPosition.RIGHT){
            drive.setPoseEstimate(right.start());
            intake.purplePosition(0);
            intake.setPower(0.5);
            outtake.transferPosition();
            drive.followTrajectorySequence(right);
            intake.purplePosition(820);
            outtake.setPixelLatch(true);
            outtake.depositPosition(0, 0);
            timer.reset();
            while (opModeIsActive() && timer.milliseconds()<2000){
                intake.update();
                outtake.update();
            }
            intake.setPower(-0.1);
            outtake.setPixelLatch(false);
            timer.reset();
            while (opModeIsActive() && timer.milliseconds()<1000){

            }
            intake.setPower(0);
            intake.intakePosition5th(0);
            outtake.transferPosition();
            outtake.setPixelLatch(false);
            while (opModeIsActive() && timer.milliseconds()<2000){
                intake.update();
                outtake.update();
            }
        }
    }
}