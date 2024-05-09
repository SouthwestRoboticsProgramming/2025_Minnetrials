package com.swrobotics.robot.subsystems.swerve;

import com.swrobotics.robot.utils.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public final class SwerveSetpointGenerator {
    private final Translation2d[] modules;
    private final SwerveDriveKinematics kinematics;

    public SwerveSetpointGenerator(Translation2d[] modules) {
        this.modules = modules;
        this.kinematics = new SwerveDriveKinematics(modules);
    }

    /**
     * Check if it would be faster to go to the opposite of the goal heading (and reverse drive direction).
     *
     * @param prevToGoal The rotation from the previous state to the goal state (i.e. prev.inverse().rotateBy(goal)).
     * @return True if the shortest path to achieve this rotation involves flipping the drive direction.
     */
    private boolean flipHeading(Rotation2d prevToGoal) {
        return Math.abs(prevToGoal.getRadians()) > Math.PI / 2.0;
    }

    private double unwrapAngle(double ref, double angle) {
        double diff = angle - ref;
        if (diff > Math.PI) {
            return angle - 2.0 * Math.PI;
        } else if (diff < -Math.PI) {
            return angle + 2.0 * Math.PI;
        } else {
            return angle;
        }
    }

    @FunctionalInterface
    private interface Function2d {
        double f(double x, double y);
    }

    /**
     * Find the root of the generic 2D parametric function 'func' using the regula falsi technique. This is a pretty naive way to
     * do root finding, but it's usually faster than simple bisection while being robust in ways that e.g. the Newton-Raphson
     * method isn't.
     * @param func The Function2d to take the root of.
     * @param x0 x value of the lower bracket.
     * @param y0 y value of the lower bracket.
     * @param f0 value of 'func' at x0, y0 (passed in by caller to save a call to 'func' during recursion)
     * @param x1 x value of the upper bracket.
     * @param y1 y value of the upper bracket.
     * @param f1 value of 'func' at x1, y1 (passed in by caller to save a call to 'func' during recursion)
     * @param iterationsLeft Number of iterations of root finding left.
     * @return The parameter value 's' that interpolating between 0 and 1 that corresponds to the (approximate) root.
     */
    private double findRoot(Function2d func, double x0, double y0, double f0, double x1, double y1, double f1, int iterationsLeft) {
        if (iterationsLeft < 0 || MathUtil.fuzzyEquals(f0, f1)) {
            return 1.0;
        }

        double sGuess = MathUtil.clamp(-f0 / (f1 - f0), 0, 1);
        double xGuess = MathUtil.lerp(x0, x1, sGuess);
        double yGuess = MathUtil.lerp(y0, y1, sGuess);
        double fGuess = func.f(xGuess, yGuess);

        if (Math.signum(f0) == Math.signum(fGuess)) {
            // 0 and guess on same side of root, so use upper bracket.
            return sGuess + (1.0 - sGuess) * findRoot(func, xGuess, yGuess, fGuess, x1, y1, f1, iterationsLeft - 1);
        } else {
            // Use lower bracket.
            return sGuess * findRoot(func, x0, y0, f0, xGuess, yGuess, fGuess, iterationsLeft - 1);
        }
    }

    protected double findSteeringMaxS(double x0, double y0, double f0, double x1, double y1, double f1, double maxDeviation, int maxIterations) {
        f1 = unwrapAngle(f0, f1);
        double diff = f1 - f0;
        if (Math.abs(diff) <= maxDeviation) {
            // Can go all the way to s=1.
            return 1.0;
        }
        double offset = f0 + Math.signum(diff) * maxDeviation;
        Function2d func = (x,y) -> {
            return unwrapAngle(f0, Math.atan2(y, x)) - offset;
        };
        return findRoot(func, x0, y0, f0 - offset, x1, y1, f1 - offset, maxIterations);
    }

    private boolean inRange(double s) {
        return Double.isFinite(s) && s >= 0 && s <= 1;
    }

    private double findDriveMaxS(double x0, double y0, double x1, double y1, double maxVelStep) {
        double l0 = x0 * x0 + y0 * y0;
        double l1 = x1 * x1 + y1 * y1;
        double sqrtL0 = Math.sqrt(l0);
        double diff = Math.sqrt(l1) - sqrtL0;
        if (Math.abs(diff) <= maxVelStep)
            return 1.0;

        double offset = sqrtL0 + Math.copySign(maxVelStep, diff);
        double xp = x0 * x1;
        double yp = y0 * y1;

        // Quadratic of s
        double a = l0 + l1 - 2 * (xp + yp);
        double b = 2 * (xp + yp - l0);
        double c = l0 - offset * offset;
        double root = Math.sqrt(b * b - 4 * a * c);

        // Check if either of the solutions are valid
        double s1 = (-b + root) / (2 * a);
        if (inRange(s1))
            return s1;
        double s2 = (-b - root) / (2 * a);
        if (inRange(s2))
            return s2;

        // Shouldn't happen, but if it does just don't limit movement
        return 1.0;
    }

    private Twist2d toTwist2d(ChassisSpeeds speeds) {
        return new Twist2d(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond
        );
    }

    private boolean twistEpsilonEquals(Twist2d a, Twist2d b) {
        return MathUtil.fuzzyEquals(a.dx, b.dx)
                && MathUtil.fuzzyEquals(a.dy, b.dy)
                && MathUtil.fuzzyEquals(a.dtheta, b.dtheta);
    }

    private Rotation2d flipWithTrig(Rotation2d rot) {
        return new Rotation2d(-rot.getCos(), -rot.getSin());
    }

    private Rotation2d inverseWithTrig(Rotation2d rot) {
        return new Rotation2d(rot.getCos(), -rot.getSin());
    }

    /**
     * Generate a new setpoint.
     *
     * @param limits The kinematic limits to respect for this setpoint.
     * @param prevSetpoints The previous setpoint motion. Normally, you'd pass in the previous iteration setpoint instead of the actual
     *                     measured/estimated kinematic state.
     * @param desiredState The desired state of motion, such as from the driver sticks or a path following algorithm.
     * @param dt The loop time.
     * @return A Setpoint object that satisfies all of the KinematicLimits while converging to desiredState quickly.
     */
    public SwerveSetpoints generateSetpoint(final SwerveKinematicLimits limits, final SwerveSetpoints prevSetpoints, ChassisSpeeds desiredState, double dt) {
        SwerveModuleState[] desiredModuleState = kinematics.toSwerveModuleStates(desiredState);
        // Make sure desiredState respects velocity limits.
        if (limits.kMaxDriveVelocity > 0.0) {
            SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleState, limits.kMaxDriveVelocity);
            desiredState = kinematics.toChassisSpeeds(desiredModuleState);
        }

        // Special case: desiredState is a complete stop. In this case, module angle is arbitrary, so just use the previous angle.
        boolean needToSteer = true;
        if (twistEpsilonEquals(toTwist2d(desiredState), new Twist2d())) {
            needToSteer = false;
            for (int i = 0; i < modules.length; ++i) {
                desiredModuleState[i].angle = prevSetpoints.moduleStates[i].angle;
                desiredModuleState[i].speedMetersPerSecond = 0.0;
            }
        }

        // For each module, compute local Vx and Vy vectors.
        double[] prevVX = new double[modules.length];
        double[] prevVY = new double[modules.length];
        Rotation2d[] prevHeading = new Rotation2d[modules.length];
        double[] desiredVX = new double[modules.length];
        double[] desiredVY = new double[modules.length];
        Rotation2d[] desiredHeading = new Rotation2d[modules.length];
        boolean allModulesShouldFlip = true;
        for (int i = 0; i < modules.length; ++i) {
            SwerveModuleState prevSetpoint = prevSetpoints.moduleStates[i];
            prevVX[i] = prevSetpoint.angle.getCos() * prevSetpoint.speedMetersPerSecond;
            prevVY[i] = prevSetpoint.angle.getSin() * prevSetpoint.speedMetersPerSecond;
            prevHeading[i] = prevSetpoint.angle;
            if (prevSetpoint.speedMetersPerSecond < 0.0) {
                prevHeading[i] = flipWithTrig(prevHeading[i]);
            }

            SwerveModuleState desiredSetpoint = desiredModuleState[i];
            desiredVX[i] = desiredSetpoint.angle.getCos() * desiredSetpoint.speedMetersPerSecond;
            desiredVY[i] = desiredSetpoint.angle.getSin() * desiredSetpoint.speedMetersPerSecond;
            desiredHeading[i] = desiredSetpoint.angle;
            if (desiredSetpoint.speedMetersPerSecond < 0.0) {
                desiredHeading[i] = flipWithTrig(desiredHeading[i]);
            }

            if (allModulesShouldFlip) {
                double requiredRotationRad = Math.abs(inverseWithTrig(prevHeading[i]).rotateBy(desiredHeading[i]).getRadians());
                if (requiredRotationRad < Math.PI / 2.0) {
                    allModulesShouldFlip = false;
                }
            }
        }
        if (allModulesShouldFlip &&
                !twistEpsilonEquals(toTwist2d(prevSetpoints.chassisSpeeds), new Twist2d()) &&
                !twistEpsilonEquals(toTwist2d(desiredState), new Twist2d())) {
            // It will (likely) be faster to stop the robot, rotate the modules in place to the complement of the desired
            // angle, and accelerate again.
            return generateSetpoint(limits, prevSetpoints, new ChassisSpeeds(), dt);
        }

        // Compute the deltas between start and goal. We can then interpolate from the start state to the goal state; then
        // find the amount we can move from start towards goal in this cycle such that no kinematic limit is exceeded.
        double dx = desiredState.vxMetersPerSecond - prevSetpoints.chassisSpeeds.vxMetersPerSecond;
        double dy = desiredState.vyMetersPerSecond - prevSetpoints.chassisSpeeds.vyMetersPerSecond;
        double dtheta = desiredState.omegaRadiansPerSecond - prevSetpoints.chassisSpeeds.omegaRadiansPerSecond;

        // 's' interpolates between start and goal. At 0, we are at prevState and at 1, we are at desiredState.
        double minS = 1.0;

        // In cases where an individual module is stopped, we want to remember the right steering angle to command (since
        // inverse kinematics doesn't care about angle, we can be opportunistically lazy).
        List<Optional<Rotation2d>> overrideSteering = new ArrayList<>(modules.length);
        // Enforce steering velocity limits. We do this by taking the derivative of steering angle at the current angle,
        // and then backing out the maximum interpolant between start and goal states. We remember the minimum across all modules, since
        // that is the active constraint.
        final double maxThetaStep = dt * limits.kMaxSteeringVelocity;
        for (int i = 0; i < modules.length; ++i) {
            if (!needToSteer) {
                overrideSteering.add(Optional.of(prevSetpoints.moduleStates[i].angle));
                continue;
            }
            overrideSteering.add(Optional.empty());
            if (MathUtil.fuzzyEquals(prevSetpoints.moduleStates[i].speedMetersPerSecond, 0.0)) {
                // If module is stopped, we know that we will need to move straight to the final steering angle, so limit based
                // purely on rotation in place.
                if (MathUtil.fuzzyEquals(desiredModuleState[i].speedMetersPerSecond, 0.0)) {
                    // Goal angle doesn't matter. Just leave module at its current angle.
                    overrideSteering.set(i, Optional.of(prevSetpoints.moduleStates[i].angle));
                    continue;
                }

                var necessaryRotation = inverseWithTrig(prevSetpoints.moduleStates[i].angle).rotateBy(
                        desiredModuleState[i].angle);
                if (flipHeading(necessaryRotation)) {
                    necessaryRotation = necessaryRotation.rotateBy(new Rotation2d(Math.PI));
                }
                // getRadians() bounds to +/- Pi.
                final double numStepsNeeded = Math.abs(necessaryRotation.getRadians()) / maxThetaStep;

                if (numStepsNeeded <= 1.0) {
                    // Steer directly to goal angle.
                    overrideSteering.set(i, Optional.of(desiredModuleState[i].angle));
                    // Don't limit the global minS;
                } else {
                    // Adjust steering by maxThetaStep.
                    overrideSteering.set(i, Optional.of(prevSetpoints.moduleStates[i].angle.rotateBy(
                            Rotation2d.fromRadians(Math.signum(necessaryRotation.getRadians()) * maxThetaStep))));
                    minS = 0.0;
                }
                continue;
            }
            if (minS == 0.0) {
                // s can't get any lower. Save some CPU.
                continue;
            }

            final int kMaxIterations = 10;
            double s = findSteeringMaxS(prevVX[i], prevVY[i], prevHeading[i].getRadians(),
                    desiredVX[i], desiredVY[i], desiredHeading[i].getRadians(),
                    maxThetaStep, kMaxIterations);
            minS = Math.min(minS, s);
        }

        // Enforce drive wheel acceleration limits.
        final double maxVelStep = dt * limits.kMaxDriveAcceleration;
        for (int i = 0; i < modules.length; ++i) {
            if (minS == 0.0) {
                // No need to carry on.
                break;
            }
            double vxMinS = minS == 1.0 ? desiredVX[i] : (desiredVX[i] - prevVX[i]) * minS + prevVX[i];
            double vyMinS = minS == 1.0 ? desiredVY[i] : (desiredVY[i] - prevVY[i]) * minS + prevVY[i];
            // Find the max s for this drive wheel. Search on the interval between 0 and minS, because we already know we can't go faster
            // than that.
            double s = minS * findDriveMaxS(prevVX[i], prevVY[i], vxMinS, vyMinS, maxVelStep);
            minS = Math.min(minS, s);
        }

        ChassisSpeeds retSpeeds = new ChassisSpeeds(
                prevSetpoints.chassisSpeeds.vxMetersPerSecond + minS * dx,
                prevSetpoints.chassisSpeeds.vyMetersPerSecond + minS * dy,
                prevSetpoints.chassisSpeeds.omegaRadiansPerSecond + minS * dtheta);
        var retStates = kinematics.toSwerveModuleStates(retSpeeds);
        for (int i = 0; i < modules.length; ++i) {
            final var maybeOverride = overrideSteering.get(i);
            if (maybeOverride.isPresent()) {
                var override = maybeOverride.get();
                if (flipHeading(inverseWithTrig(retStates[i].angle).rotateBy(override))) {
                    retStates[i].speedMetersPerSecond *= -1.0;
                }
                retStates[i].angle = override;
            }
            final var deltaRotation = inverseWithTrig(prevSetpoints.moduleStates[i].angle).rotateBy(retStates[i].angle);
            if (flipHeading(deltaRotation)) {
                retStates[i].angle = flipWithTrig(retStates[i].angle);
                retStates[i].speedMetersPerSecond *= -1.0;
            }
        }
        return new SwerveSetpoints(retSpeeds, desiredModuleState, retStates);
    }
}
