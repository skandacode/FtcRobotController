package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.ArrayList;
import java.util.List;

@Config
public class Intake {
    ServoImplEx intakeheights;
    Motor intakemotor;
    Servo pitchcontrol;
    CRServo intakeservo1, intakeservo2;
    AnalogInput intakeheightsEncoder, intakeflapsEncoder;

    public PIDFController intakeController=new PIDFController(0.003, 0, 0, 0);
    public static double kF=0.4;
    public static double positionTolerance=5;
    public void init(HardwareMap hardwareMap){
        intakemotor=new Motor(hardwareMap, "intake0", Motor.GoBILDA.RPM_1150);
        intakeheights=(ServoImplEx) hardwareMap.servo.get("intakeheights0");
        pitchcontrol=hardwareMap.servo.get("intakepitch1");
        intakeservo1=hardwareMap.crservo.get("intakeservo0");
        intakeservo2=hardwareMap.crservo.get("intakeservo1");

        intakeheights.setPwmRange(new PwmControl.PwmRange(510, 2490));

        intakeheightsEncoder=hardwareMap.analogInput.get("analogintake1");
        intakeflapsEncoder=hardwareMap.analogInput.get("analogintake2");
        intakeservo1.setDirection(DcMotorSimple.Direction.REVERSE);
        intakemotor.setInverted(true);
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
            if (Math.abs(this.getEncoderPos()-intakeController.getSetPoint())>positionTolerance) {
                if (this.getEncoderPos() > intakeController.getSetPoint()) {
                    targetPower = targetPower - kF;
                }
                if (this.getEncoderPos() < intakeController.getSetPoint()) {
                    targetPower = targetPower + kF;
                }
            }
            intakemotor.set(targetPower);
        }else{
            if (intakemotor.encoder.getPosition()<10){
                intakemotor.set(0);
            }else{
                intakemotor.set(-1);
            }
        }
    }
    public int getEncoderPos(){
        return intakemotor.getCurrentPosition();
    }
    public double readHeightAnalog(){
        return intakeheightsEncoder.getVoltage()*360/3.3;
    }
    public void setServos(double height, double pitch){
        intakeheights.setPosition(height);
        pitchcontrol.setPosition(pitch);
    }
    public void intakePosition(int extend){
        this.setTarget(extend);
        //set servos
        this.setServos(0.81, 0.65);
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
        setServos(0.23, 0.3);

    }
    public void setPower(double power){
        intakeservo1.setPower(power);
        intakeservo2.setPower(power);
    }
    public boolean canEject(){
        if (this.getEncoderPos()<=5){
            return this.readHeightAnalog()>255;
        }else{
            return false;
        }
    }
}
