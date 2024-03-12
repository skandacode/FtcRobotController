package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.util.ArrayList;
import java.util.List;

@Config
public class Intake {
    ServoImplEx intakeheights;
    Motor intakemotor;
    Servo pitchcontrol;
    CRServo intakeservo1, intakeservo2;
    AnalogInput intakeheightsEncoder, intakeflapsEncoder;
    TouchSensor intakeEnd;

    public PIDFController intakeController=new PIDFController(0.003, 0, 0, 0);
    public static double kF=0.6;
    public static double positionTolerance=10;

    public static int startPosition=0;
    public int height=1;

    public void init(HardwareMap hardwareMap){
        intakemotor=new Motor(hardwareMap, "intake0", Motor.GoBILDA.RPM_1150);
        intakeheights=(ServoImplEx) hardwareMap.servo.get("intakeheights0");
        pitchcontrol=hardwareMap.servo.get("intakepitch1");
        intakeservo1=hardwareMap.crservo.get("intakeservo0");
        intakeservo2=hardwareMap.crservo.get("intakeservo1");
        intakeEnd=hardwareMap.touchSensor.get("slideend");


        intakeheights.setPwmRange(new PwmControl.PwmRange(510, 2490));

        intakeheightsEncoder=hardwareMap.analogInput.get("analogintake1");
        intakeflapsEncoder=hardwareMap.analogInput.get("analogintake2");
        intakeservo1.setDirection(DcMotorSimple.Direction.REVERSE);
        intakemotor.setInverted(true);
        intakemotor.resetEncoder();

        startPosition=this.getEncoderPos();

    }
    public void setTarget(int setPoint){
        intakeController.setSetPoint(setPoint);
    }
    public double getTarget(){
        return intakeController.getSetPoint();
    }
    public void update(){
        if (this.getTarget()!=0) {
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
            if (intakeEnd.isPressed()){
                intakemotor.set(0);
            }else{
                intakemotor.set(-1);
            }
        }
    }
    public int getEncoderPos(){
        return intakemotor.getCurrentPosition()-startPosition;
    }
    public double readHeightAnalog(){
        return intakeheightsEncoder.getVoltage()*360/3.3;
    }
    public void resetEncoder(){
        intakemotor.resetEncoder();
    }
    public void setServos(double height, double pitch){
        intakeheights.setPosition(height);
        pitchcontrol.setPosition(pitch);
    }
    public void stay(int extend){
        this.setTarget(extend);
        this.setServos(0.6, 0.52);
    }
    public void intakePosition(int extend){
        this.setTarget(extend);
        //set servos
        this.setServos(0.82, 0.52);
        height=1;
    }
    public void intakePositionExtended(int extend){
        this.setTarget(extend);
        this.setServos(0.8, 0.52);
        height=1;
    }
    public void intakePositionExtended(){
        this.intakePositionExtended((int) this.getTarget());
    }
    public void intakePosition(){
        this.intakePosition((int) this.getTarget());
    }
    public void purplePosition(int extend){
        this.setTarget(extend);
        this.setServos(0.73, 0.25);
    }
    public void purplePosition(){
        this.purplePosition((int) this.getTarget());
    }
    public void intakePosition5th(int extend){
        this.setTarget(extend);
        //set servos
        this.setServos(0.72, 0.52);
        height=5;
    }
    public void intakePosition5th(){
        this.intakePosition5th((int) this.getTarget());
    }
    public void intakePosition4th(int extend){
        this.setTarget(extend);
        //set servos
        this.setServos(0.755, 0.52);
        height=4;
    }
    public void intakePosition4th(){
        this.intakePosition4th((int) this.getTarget());
    }
    public void intakePosition3rd(int extend){
        this.setTarget(extend);
        //set servos
        this.setServos(0.76, 0.52);
        height=3;
    }
    public void intakePosition3rd(){
        this.intakePosition3rd((int) this.getTarget());
    }
    public void intakePosition2nd(int extend){
        this.setTarget(extend);
        //set servos
        this.setServos(0.775, 0.52);
        height=2;
    }
    public void intakePosition2nd(){
        this.intakePosition2nd((int) this.getTarget());
    }
    public void transferPosition(){
        this.setTarget(0);
        //set servos
        setServos(0.26, 0.2);
    }
    public void setPower(double power){
        intakeservo1.setPower(power);
        intakeservo2.setPower(power);
    }
    public boolean canEject(){
        if (intakeEnd.isPressed()){
            return this.readHeightAnalog()>250;
        }else{
            return false;
        }
    }
}
