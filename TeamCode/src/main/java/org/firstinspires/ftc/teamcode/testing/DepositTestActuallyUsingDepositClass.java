package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Outtake;

@TeleOp
@Config
public class DepositTestActuallyUsingDepositClass extends LinearOpMode {
    Outtake outtake=new Outtake();
    public static int depositTarget=0;
    public static boolean flipped=false;
    public static boolean extended=false;
    public static double turretAngle=0;
    public static boolean opened=false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        outtake.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            outtake.setTarget(depositTarget);
            outtake.setServos(turretAngle, flipped, extended);
            outtake.setPixelLatch(opened);
            telemetry.addData("Deposit motor Position", outtake.getEncoderPos());
            telemetry.addData("TurretAngle", outtake.readAxonAnalog()[0]);
            telemetry.addData("Deposit Position analog", outtake.readAxonAnalog()[1]);
            telemetry.update();
            outtake.update();
        }
    }
}
