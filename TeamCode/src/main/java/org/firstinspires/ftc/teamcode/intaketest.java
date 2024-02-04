package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class intaketest extends LinearOpMode {
    CRServo intake1, intake2;
    AnalogInput input1, input2;
    public static double power1=0;
    public static double power2=0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        intake1=hardwareMap.crservo.get("intake1");
        intake2=hardwareMap.crservo.get("intake2");
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);

        input1=hardwareMap.analogInput.get("analog1");
        input2=hardwareMap.analogInput.get("analog2");


        waitForStart();
        double previntake1=0;
        double previntake2=0;

        double looptime=System.nanoTime();

        while (opModeIsActive()){
            intake1.setPower(power1);
            intake2.setPower(power2);
            double result1=input1.getVoltage();
            double result2=input2.getVoltage();

            double loop=System.nanoTime();
            double hertz=1000000000/(loop-looptime);
            looptime=loop;
            telemetry.addData("intake1", result1);
            telemetry.addData("intake2", result2);
            telemetry.addData("change1", (result1-previntake1)*hertz);
            telemetry.addData("change2", (result2-previntake2)*hertz);
            telemetry.addData("hertz", hertz);
            if (-2<(result1-previntake1)*hertz && (result1-previntake1)*hertz<3){
                telemetry.addLine("Full");
            }else{
                telemetry.addLine("No full");
            }
            telemetry.update();
            previntake1=result1;
            previntake2=result2;
        }
    }
}
