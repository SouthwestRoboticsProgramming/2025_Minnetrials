package com.swrobotics.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.swrobotics.robot.config.Constants;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    TalonFX motor = new TalonFX(7);

    public Arm() {
        // Set up configuration for the motor
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Proportional and derivative coefficients for the PID controller
        config.Slot0.kP = Constants.kArmKP.get();
        config.Slot0.kD = Constants.kArmKD.get();
        config.Slot0.kG = Constants.kArmKG.get();
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        // These are determined by the gears in the gearbox and the sprocket on the chain
        config.Feedback.SensorToMechanismRatio = (60.0 / 8.0) * (50.0 / 26.0) * (60.0 / 10.0);
        
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Apply the configuration we created to the motor
        motor.getConfigurator().apply(config);

        // Tell KP and KD to run updatePid() when they are changed by the driver station
        Constants.kArmKP.onChange((unused) -> updatePid());
        Constants.kArmKD.onChange((unused) -> updatePid());

        // Tell motor controller where the arm is initially
        motor.setPosition(0.0);
    }

    // Update the motor PID configuration with the values from the driver station
    private void updatePid() {
        // Get new values from the tunable inputs
        Slot0Configs config = new Slot0Configs();
        config.kP = Constants.kArmKP.get();
        config.kD = Constants.kArmKD.get();
        config.kG = Constants.kArmKG.get();
        config.GravityType = GravityTypeValue.Arm_Cosine;

        // Update motor configuration to match the inputs
        motor.getConfigurator().apply(config);
    }

    public void set(double angleDeg) {
        // Convert units from degrees to rotations
        double angleRotations = Units.degreesToRotations(angleDeg);

        // Send control request to the motor to set the target angle
        motor.setControl(new PositionVoltage(angleRotations));
    }
}
