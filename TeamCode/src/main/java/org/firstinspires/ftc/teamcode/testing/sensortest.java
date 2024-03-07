package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@TeleOp
public class sensortest extends LinearOpMode {
    Rev2mDistanceSensor left, right;
    TouchSensor intakeEnd;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        left=hardwareMap.get(Rev2mDistanceSensor.class, "left2m");
        right=hardwareMap.get(Rev2mDistanceSensor.class, "right2m");
        intakeEnd=hardwareMap.touchSensor.get("slideend");
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("left", left.getDistance(DistanceUnit.CM));
            telemetry.addData("right", right.getDistance(DistanceUnit.CM));
            telemetry.addData("endstop", intakeEnd.isPressed());
            telemetry.update();
        }
    }
}
