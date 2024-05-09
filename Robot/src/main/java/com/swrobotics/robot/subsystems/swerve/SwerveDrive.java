package com.swrobotics.robot.subsystems.swerve;

import com.swrobotics.lib.net.NTBoolean;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.logging.FieldView;

import com.swrobotics.robot.subsystems.swerve.io.*;
import edu.wpi.first.wpilibj.DriverStation;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import com.swrobotics.lib.field.FieldInfo;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public final class SwerveDrive extends SubsystemBase {
    private static final NTBoolean CALIBRATE_OFFSETS = new NTBoolean("Drive/Modules/Calibrate", false);

    public static final int DRIVER_PRIORITY = 0;
    public static final int AUTO_PRIORITY = 1;

    // Priority should be one of the priority levels above
    public static record DriveRequest(int priority, Translation2d robotRelTranslation, DriveRequestType type) {
    }

    public static record TurnRequest(int priority, Rotation2d turn) {
    }

    private static final DriveRequest NULL_DRIVE = new DriveRequest(Integer.MIN_VALUE, new Translation2d(0, 0), DriveRequestType.OpenLoopVoltage);
    private static final TurnRequest NULL_TURN = new TurnRequest(Integer.MIN_VALUE, new Rotation2d(0));

    private final GyroIO gyroIO;
    private final GyroIO.Inputs gyroInputs;
    private final SwerveModuleIO[] moduleIOs;
    private final SwerveModuleIO.Inputs[] moduleInputs;

    private final SwerveKinematics kinematics;
    private final SwerveEstimator estimator;
    private final SwerveSetpointGenerator setpointGenerator;

    private SwerveModulePosition[] prevPositions;
    private double prevGyroAngle;
    private SwerveSetpoint prevSetpoints;

    private DriveRequest currentDriveRequest;
    private TurnRequest currentTurnRequest;
    private SwerveKinematicLimits limits;
    private int lastSelectedPriority;

    public SwerveDrive() {
//        gyro = new AHRS(SPI.Port.kMXP);
        if (RobotBase.isReal()) {
            gyroIO = new NavXGyroIO();
        } else {
            gyroIO = new SimGyroIO();
        }
        gyroInputs = new GyroIO.Inputs();

        SwerveModule.Info[] infos = Constants.kSwerveModuleInfos;

        moduleIOs = new SwerveModuleIO[infos.length];
        moduleInputs = new SwerveModuleIO.Inputs[infos.length];
        Translation2d[] positions = new Translation2d[infos.length];
        for (int i = 0; i < moduleIOs.length; i++) {
            SwerveModule.Info info = infos[i];

            if (RobotBase.isReal()) {
                SwerveModuleConstants moduleConstants = Constants.kSwerveConstantsFactory.createModuleConstants(
                        info.turnId(), info.driveId(), info.encoderId(),
                        info.offset().get(),
                        info.position().getX(), info.position().getY(),
                        false);

                moduleIOs[i] = new CtreSwerveModuleIO(info.name(), moduleConstants, info.canBus());
            } else {
                moduleIOs[i] = new SimSwerveModuleIO(info.name(), info.offset().get());
            }

            positions[i] = info.position();
            moduleInputs[i] = new SwerveModuleIO.Inputs();
        }

        this.kinematics = new SwerveKinematics(positions, Constants.kMaxAchievableSpeed);
        this.estimator = new SwerveEstimator();
        this.setpointGenerator = new SwerveSetpointGenerator(positions);

        prevPositions = null;
        prevSetpoints = SwerveSetpoint.createInitial(infos.length);
        currentDriveRequest = NULL_DRIVE;
        currentTurnRequest = NULL_TURN;

        // Cheesy constants
        limits = new SwerveKinematicLimits();
        limits.kMaxDriveVelocity = Constants.kMaxAchievableSpeed;
        limits.kMaxDriveAcceleration = limits.kMaxDriveVelocity / 0.1;
        limits.kMaxSteeringVelocity = Math.toRadians(1500);

        // Configure PathPlanner
        AutoBuilder.configureHolonomic(
                this::getEstimatedPose,
                this::setPose,
                this::getRobotRelativeSpeeds,
                (speeds) ->
                    driveAndTurn(AUTO_PRIORITY, speeds, DriveRequestType.Velocity),
                new HolonomicPathFollowerConfig(
                        new PIDConstants(Constants.kAutoDriveKp, Constants.kAutoDriveKd),
                        new PIDConstants(Constants.kAutoTurnKp.get(), Constants.kAutoTurnKd.get()),
                        Constants.kMaxAchievableSpeed,
                        Constants.kDriveRadius,
                        new ReplanningConfig(),
                        0.020),
                () -> FieldInfo.getAlliance() == DriverStation.Alliance.Red,
                this);

        PathPlannerLogging.setLogActivePathCallback(FieldView.pathPlannerPath::setPoses);
        PathPlannerLogging.setLogTargetPoseCallback(FieldView.pathPlannerSetpoint::setPose);
    }

    public SwerveModuleState[] getCurrentModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[moduleIOs.length];
        for (int i = 0; i < states.length; i++) {
            states[i] = moduleInputs[i].getState();
        }
        return states;
    }

    public SwerveModulePosition[] getCurrentModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[moduleIOs.length];
        for (int i = 0; i < positions.length; i++) {
            positions[i] = moduleInputs[i].getPosition();
        }
        return positions;
    }

    public void driveAndTurn(int priority, ChassisSpeeds speeds, DriveRequestType type) {
        drive(new DriveRequest(priority, new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), type));
        turn(new TurnRequest(priority, new Rotation2d(speeds.omegaRadiansPerSecond)));
    }

    public void drive(DriveRequest request) {
        if (request.priority > currentDriveRequest.priority)
            currentDriveRequest = request;
    }

    public void turn(TurnRequest request) {
        if (request.priority > currentTurnRequest.priority)
            currentTurnRequest = request;
    }

    private void calibrate() {
        for (int i = 0; i < moduleIOs.length; i++) {
            SwerveModule.Info info = Constants.kSwerveModuleInfos[i];

            double position = moduleInputs[i].canCoderPos;
            info.offset().set(info.offset().get() - position);
            moduleIOs[i].setCANcoderMagnetOffset(info.offset().get());
        }
    }

    @Override
    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Gyro", gyroInputs);

        for (int i = 0; i < moduleIOs.length; i++) {
            SwerveModuleIO io = moduleIOs[i];
            SwerveModuleIO.Inputs inputs = moduleInputs[i];

            io.updateInputs(inputs);
            Logger.processInputs("Swerve Modules/" + io.getName(), inputs);
        }
        Logger.recordOutput("Drive/Module Positions", getCurrentModulePositions());
        Logger.recordOutput("Drive/Module States", getCurrentModuleStates());

        if (CALIBRATE_OFFSETS.get()) {
            CALIBRATE_OFFSETS.set(false);
            calibrate();
        }

        // Update estimator
        SwerveModulePosition[] positions = getCurrentModulePositions();
        if (prevPositions != null) {
            Twist2d twist = kinematics.getTwistDelta(prevPositions, positions);

            // We trust the gyro more than the kinematics estimate
            if (gyroInputs.connected) {
                twist.dtheta = gyroInputs.yaw - prevGyroAngle;
            }

            estimator.update(twist);
        }
        prevPositions = positions;
        prevGyroAngle = gyroInputs.yaw;

        // Apply drive request when robot enabled
        if (!DriverStation.isDisabled()) {
            ChassisSpeeds requestedSpeeds = new ChassisSpeeds(
                    currentDriveRequest.robotRelTranslation.getX(),
                    currentDriveRequest.robotRelTranslation.getY(),
                    currentTurnRequest.turn.getRadians());
            lastSelectedPriority = Math.max(currentDriveRequest.priority, currentTurnRequest.priority);
            currentDriveRequest = NULL_DRIVE;
            currentTurnRequest = NULL_TURN;

            // Apply the drive request
            requestedSpeeds = ChassisSpeeds.discretize(requestedSpeeds, Constants.kDriveDriftComp);
            SwerveSetpoint setpoints = setpointGenerator.generateSetpoint(limits, prevSetpoints, requestedSpeeds, 0.02);
            SwerveModuleState[] moduleSetpoints = setpoints.mModuleStates;
            for (int i = 0; i < moduleIOs.length; i++) {
                moduleIOs[i].apply(moduleSetpoints[i], currentDriveRequest.type);
            }
            prevSetpoints = setpoints;
            Logger.recordOutput("Drive/Module Setpoints", moduleSetpoints);
        }
    }

    public Pose2d getEstimatedPose() {
        return estimator.getEstimatedPose();
    }

    public void setPose(Pose2d newPose) {
        estimator.resetPose(newPose);
    }

    public void setRotation(Rotation2d newRotation) {
        estimator.resetPose(new Pose2d(getEstimatedPose().getTranslation(), newRotation));
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(getCurrentModuleStates());
    }

    public Translation2d toRobotRelativeDrive(Translation2d fieldRelDrive) {
        return fieldRelDrive.rotateBy(getEstimatedPose().getRotation().unaryMinus());
    }

    public ChassisSpeeds getFieldRelativeSpeeds() {
        ChassisSpeeds robotRel = getRobotRelativeSpeeds();

        Rotation2d facing = getEstimatedPose().getRotation();
        Translation2d tx = new Translation2d(robotRel.vxMetersPerSecond, robotRel.vyMetersPerSecond)
                .rotateBy(facing);

        return new ChassisSpeeds(
                tx.getX(), tx.getY(),
                robotRel.omegaRadiansPerSecond
        );
    }

    public Rotation2d getRawGyroRotation() {
        return Rotation2d.fromRadians(gyroInputs.yaw);
    }

    public int getLastSelectedPriority() {
        return lastSelectedPriority;
    }

    public void setEstimatorIgnoreVision(boolean ignoreVision) {
        estimator.setIgnoreVision(ignoreVision);
    }
}
