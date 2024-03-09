package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

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

public class BlueAutoSensors extends LinearOpMode {

    OpenCvWebcam webcam;
    PropPosition randomization=PropPosition.NONE;
    Intake intake = new Intake();
    Outtake outtake = new Outtake();


    public static String ObjectDirection;

    enum movementStates{
        yellowAndToStack,
        toStackFromBackdrop,
        toStackFromStage,
        toStagefromStack,
        toBackdropfromStage,
    }

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


        Rev2mDistanceSensor left2m=hardwareMap.get(Rev2mDistanceSensor.class, "left2m");
        Rev2mDistanceSensor right2m=hardwareMap.get(Rev2mDistanceSensor.class, "right2m");

        TrajectorySequence yellowAndToStackPathLeft=drive.trajectorySequenceBuilder(new Pose2d(11.83, 62.16, Math.toRadians(270.00)))
                .build();
        TrajectorySequence toStagefromStackPath=drive.trajectorySequenceBuilder(new Pose2d(11.83, 62.16, Math.toRadians(270.00)))
                .build();


        StateMachine autoMachineLeft=new StateMachineBuilder()
                .state(movementStates.yellowAndToStack)
                .onEnter(()->drive.followTrajectorySequenceAsync(yellowAndToStackPathLeft))
                .transition(()->drive.isBusy())//gets to the stack
                .state(movementStates.toStagefromStack)
                .onEnter(()->drive.followTrajectorySequenceAsync(toStagefromStackPath))
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
        }
        if (randomization==PropPosition.MIDDLE){
            intake.transferPosition();
        }
        if (randomization==PropPosition.RIGHT){
            intake.transferPosition();
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