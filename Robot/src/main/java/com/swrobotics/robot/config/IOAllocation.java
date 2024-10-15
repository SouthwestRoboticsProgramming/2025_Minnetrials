package com.swrobotics.robot.config;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Describes the IDs of the various devices within the robot.
 */
// For now, IDs are set such that this code can run on the 2024 robot
public final class IOAllocation {
    public static final class CAN {
        private static final String RIO = "";
        public static final String GERALD = "Gerald";

        public static final CanId PDP = new CanId(62, RIO);
    }

    public static final class RIO {
        public static final int PWM_LEDS = 4;
    }

    /** Location of a CAN device. This includes both the ID and the CAN bus */
    public static final class CanId {
        public static int uniqueifyForSim(int id, String bus) {
            // CTRE sim doesn't support CANivore, so ids need to be globally unique
            if (RobotBase.isSimulation() && bus.equals(CAN.GERALD)) {
                // If we somehow have more than 32 devices on the RoboRIO bus
                // we have other problems
                return id + 32;
            }

            return id;
        }

        private final int id;
        private final String bus;

        public CanId(int id, String bus) {
            this.id = id;
            this.bus = bus;
        }

        public int id() {
            return uniqueifyForSim(id, bus);
        }

        public String bus() {
            if (RobotBase.isSimulation())
                return CAN.RIO;

            return bus;
        }

        public TalonFX createTalonFX() {
            return new TalonFX(id, bus);
        }

        public CANcoder createCANcoder() {
            return new CANcoder(id, bus);
        }
    }
}
