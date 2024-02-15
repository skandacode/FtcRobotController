package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class intaketest extends LinearOpMode {
    CRServo intake1, intake2;
    Motor intakeMotor;
    AnalogInput intakeheightsin, intakeflapsin;
    public static double power1=0;
    public static double power2=0;
    public static double intakePower=0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        intakeMotor=new Motor(hardwareMap, "intake0", Motor.GoBILDA.RPM_1150);
        intake1=hardwareMap.crservo.get("intakeservo0");
        intake2=hardwareMap.crservo.get("intakeservo1");
        intakeMotor.setInverted(true);
        intake1.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeheightsin=hardwareMap.analogInput.get("analogintake1");
        intakeflapsin=hardwareMap.analogInput.get("analogintake2");


        waitForStart();
        double previntake1=0;
        double previntake2=0;

        double looptime=System.nanoTime();

        while (opModeIsActive()){
            intake1.setPower(power1);
            intake2.setPower(power2);
            intakeMotor.set(intakePower);
            double result1=intakeheightsin.getVoltage();
            double result2=intakeflapsin.getVoltage();

            double loop=System.nanoTime();
            double hertz=1000000000/(loop-looptime);
            looptime=loop;
            telemetry.addData("intake1", result1);
            telemetry.addData("intake2", result2);
            telemetry.addData("intakePosition", intakeMotor.getCurrentPosition());
            //telemetry.addData("change1", (result1-previntake1)*hertz);
            //telemetry.addData("change2", (result2-previntake2)*hertz);
            telemetry.addData("hertz", hertz);
            telemetry.update();
            previntake1=result1;
            previntake2=result2;
        }
    }
}
