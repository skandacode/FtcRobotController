package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

@Config
public class Intake {
    Motor intakemotor;
    Servo intakeheights;
    Servo pitchcontrol;
    CRServo intake1, intake2;
    AnalogInput intakeheightsEncoder, pitchcontrolEncoder;

    public PIDFController intakeController=new PIDFController(0.001, 0, 0, 0);
    public void init(HardwareMap hardwareMap){
        intakemotor=new Motor(hardwareMap, "intakemotor", Motor.GoBILDA.RPM_1150);
        intakeheights=hardwareMap.servo.get("intakeheights");
        pitchcontrol=hardwareMap.servo.get("pitchcontrol");
        intake1=hardwareMap.crservo.get("intake1");
        intake2=hardwareMap.crservo.get("intake2");
        intakeheightsEncoder=hardwareMap.analogInput.get("intakeheightsEncoder");
        pitchcontrolEncoder=hardwareMap.analogInput.get("pitchcontrolEncoder");
        intakemotor.resetEncoder();
    }
    public void setTarget(int setPoint){
        intakeController.setSetPoint(setPoint);
    }
    public double getTarget(){
        return intakeController.getSetPoint();
    }
    public void update(){
        if (intakeController.getSetPoint()!=0) {
            double targetPower = intakeController.calculate(this.getEncoderPos());
            intakemotor.set(targetPower);
        }else{
            if (intakemotor.encoder.getPosition()<5){
                intakemotor.set(0);
            }else{
                intakemotor.set(-1);
            }
        }
    }
    public int getEncoderPos(){
        return intakemotor.getCurrentPosition();
    }
    public double[] readAxonAnalog(){
        double[] degrees = new double[2];
        degrees[0] =intakeheightsEncoder.getVoltage()*360/3.3;
        degrees[1]= pitchcontrolEncoder.getVoltage()*360/3.3;
        return degrees;
    }
    public void setServos(double height, double pitch){
        intakeheights.setPosition(height);
        pitchcontrol.setPosition(pitch);
    }
    public void intakePosition(int extend){
        this.setTarget(extend);
        //set servos
    }
    public void intakePosition5th(int extend){
        this.setTarget(extend);
        //set servos
    }
    public void intakePosition3rd(int extend){
        this.setTarget(extend);
        //set servos
    }
    public void transferPosition(){
        this.setTarget(0);
        //set servos
    }
    public void setPower(double power){
        intake1.setPower(power);
        intake2.setPower(-power);
    }
    public boolean canEject(){
        return false;
    }
}
