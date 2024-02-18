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
    boolean scoring=false;
    public PIDFController intakeController=new PIDFController(0.01, 0, 0, 0);
    public static double kf=0.11;
    public void init(HardwareMap hardwareMap){
        outtake1=new Motor(hardwareMap, "depositfirst1", Motor.GoBILDA.RPM_1150);
        outtake2=new Motor(hardwareMap, "depositsecond2", Motor.GoBILDA.RPM_1150);
        outtake1.setInverted(true);
        outtake1.resetEncoder();
        outtake2.resetEncoder();
        outtakeMotors=new MotorGroup(outtake1, outtake2);
        outtake1.encoder.setDirection(Motor.Direction.REVERSE);

        miniTurret=hardwareMap.servo.get("miniturret2");
        depositFlip=hardwareMap.servo.get("depositflip4");
        depositExtendo=hardwareMap.servo.get("depositextendo3");
        pixelLatch=hardwareMap.servo.get("depositlatch5");


        turretEncoder=hardwareMap.analogInput.get("turretencoder");
        flipEncoder=hardwareMap.analogInput.get("flipencoder");
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
            if (this.getEncoderPos()<5){
                outtakeMotors.set(0+kf);
            }else{
                outtakeMotors.set(-1+kf);
            }
        }
    }
    public int getEncoderPos(){
        return outtake1.getCurrentPosition();
    }
    public double[] readAxonAnalog(){
        double[] degrees = new double[2];
        degrees[0] =turretEncoder.getVoltage()*360/3.3;
        degrees[1]= flipEncoder.getVoltage()*360/3.3;

        return degrees;
    }
    public void setServos(double turret, boolean flipped, boolean extended){
        miniTurret.setPosition(0.475+turret);

        if (flipped){
            //set postion
            depositFlip.setPosition(0.7);
        }else{
            //set position
            depositFlip.setPosition(0.2);
        }

        if (extended){
            //set postion
            depositExtendo.setPosition(0.3);
        }else{
            //set position
            depositExtendo.setPosition(0.87);
        }
    }
    public void setPixelLatch(boolean latchClosed){
        if (latchClosed){
            //set position
            pixelLatch.setPosition(0);
        }else{
            //set position
            pixelLatch.setPosition(0.55);
        }
    }
    public void depositPosition(int height, double angle){
        this.scoring=true;
        this.setTarget(height);
        //set servos
        this.setServos(angle, true, true);
    }
    public void resetEncoder(){
        outtake1.resetEncoder();
    }
    public void transferPosition(){
        this.scoring=false;
        this.setTarget(0);
        //set servos
        this.setServos(0, false, false);
    }
    public boolean getScoring(){
        return this.scoring;
    }
    public void purpleHold(){
        this.setServos(0, false, false);
        depositExtendo.setPosition(0.98);
        this.setTarget(0);
    }
}
