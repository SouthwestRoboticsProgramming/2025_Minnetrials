package com.swrobotics.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveBase extends SubsystemBase {
   
    // assigns value to talon's
    TalonSRX leftMotor1 = new TalonSRX(1);
    TalonSRX leftMotor2 = new TalonSRX(2);
    TalonSRX rightMotor1 = new TalonSRX(3);
    TalonSRX rightMotor2 = new TalonSRX(4);

    //second modors follow first motors
    public DriveBase() {
        leftMotor2.follow(leftMotor1);
        rightMotor2.follow(rightMotor1);
    }

    public void move(double forward){
        leftMotor1.set(ControlMode.PercentOutput, forward);
        rightMotor1.set(ControlMode.PercentOutput, -forward);
    }

    public void move(double forward, double turn){
        leftMotor1.set(ControlMode.PercentOutput, forward + turn);
        rightMotor1.set(ControlMode.PercentOutput, -(forward - turn));
    }

    } 

