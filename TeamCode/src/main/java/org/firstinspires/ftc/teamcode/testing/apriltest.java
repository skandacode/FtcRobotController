package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.AprilTagLocalizer;

@TeleOp
public class apriltest extends LinearOpMode {
    AprilTagLocalizer aprilTagLocalizer=new AprilTagLocalizer();
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard dashboard=FtcDashboard.getInstance();
        aprilTagLocalizer.initAprilTag(hardwareMap, telemetry);
        waitForStart();
        while (opModeIsActive()){
            if (aprilTagLocalizer.updatePosition()) {
                TelemetryPacket packet = new TelemetryPacket();
                Pose2d position=aprilTagLocalizer.robotPosition;
                packet.fieldOverlay().setFill("blue")
                        .strokeCircle(position.getX(), position.getY(), 9)
                        .strokeLine(position.getX(), position.getY(),
                                (position.getRotation().getCos() * 10) + position.getX(),
                                (position.getRotation().getSin() * 10) + position.getY());

                telemetry.addLine(position.toString());
                telemetry.update();
                dashboard.sendTelemetryPacket(packet);
            }
        }
    }
}
