package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public class Hang {
    ServoImplEx hang1, hang2;
    Servo drone;
    Motor hangMotor;
    public void init(HardwareMap hardwareMap){
        hang1=(ServoImplEx) hardwareMap.servo.get("hang1");
        hang2=(ServoImplEx) hardwareMap.servo.get("hang2");
        drone=hardwareMap.servo.get("drone");
        hangMotor=new Motor(hardwareMap, "hangMotor", Motor.GoBILDA.RPM_312);
        hangMotor.resetEncoder();
    }
    public void raise(){
        //set servo positions
    }
    public void hang(){
        hang1.setPwmDisable();
        hang2.setPwmDisable();
        hangMotor.set(1);
    }
    public void keepDrone(){
        //set position
    }
    public void releaseDrone(){
        //set position
    }
}
