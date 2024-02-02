package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Outtake {
    Motor outtake1, outtake2;
    Servo miniTurret, depositFlip, depositExtendo, pixelLatch;
    AnalogInput turretEncoder, flipEncoder;
    MotorGroup outtakeMotors;
    public PIDFController intakeController=new PIDFController(0.001, 0, 0, 0);
    public static double kf=0.2;
    public void init(HardwareMap hardwareMap){
        outtake1=new Motor(hardwareMap, "outtake1", Motor.GoBILDA.RPM_1150);
        outtake2=new Motor(hardwareMap, "outtake2", Motor.GoBILDA.RPM_1150);
        outtake2.setInverted(true);
        outtake1.resetEncoder();
        outtake2.resetEncoder();
        outtakeMotors=new MotorGroup(outtake1, outtake2);

        miniTurret=hardwareMap.servo.get("miniTurret");
        depositFlip=hardwareMap.servo.get("depositFlip");
        depositExtendo=hardwareMap.servo.get("depositExtendo");
        pixelLatch=hardwareMap.servo.get("pixelLatch");


        turretEncoder=hardwareMap.analogInput.get("turretEncoder");
        flipEncoder=hardwareMap.analogInput.get("flipEncoder");
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
            outtakeMotors.set(targetPower+kf);
        }else{
            if (outtakeMotors.encoder.getPosition()<5){
                outtakeMotors.set(0+kf);
            }else{
                outtakeMotors.set(-1+kf);
            }
        }
    }
    public int getEncoderPos(){
        return outtakeMotors.getCurrentPosition();
    }
    public double[] readAxonAnalog(){
        double[] degrees = new double[2];
        degrees[0] =turretEncoder.getVoltage()*360/3.3;
        degrees[1]= flipEncoder.getVoltage()*360/3.3;

        return degrees;
    }
    public void setServos(double turret, boolean flipped, boolean extended, boolean latchClosed){
        miniTurret.setPosition(turret);

        if (flipped){
            //set postion
        }else{
            //set position
        }

        if (extended){
            //set postion
        }else{
            //set position
        }

        if (latchClosed){
            //set position
        }else{
            //set position
        }
    }
    public void depositPosition(int height, double angle){
        this.setTarget(height);
        //set servos
    }
    public void transferPosition(){
        this.setTarget(0);
        //set servos
    }
}
