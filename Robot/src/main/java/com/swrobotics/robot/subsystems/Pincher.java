package com.swrobotics.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pincher extends SubsystemBase {
    private final Solenoid solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    
    public void set(boolean pinched) {
        solenoid.set(pinched);
    }
}
