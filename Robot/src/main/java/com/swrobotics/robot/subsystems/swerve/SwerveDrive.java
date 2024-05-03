package com.swrobotics.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.swrobotics.lib.net.NTBoolean;
import com.swrobotics.robot.config.Constants;
import com.swrobotics.robot.config.IOAllocation;
import com.swrobotics.robot.logging.FieldView;

import edu.wpi.first.wpilibj.DriverStation;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.kauailabs.navx.frc.AHRS;
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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.swrobotics.robot.subsystems.swerve.SwerveConstants.SWERVE_MODULE_BUILDER;

public final class SwerveDrive extends SubsystemBase {
    private static final NTBoolean CALIBRATE_OFFSETS = new NTBoolean("Drive/Modules/Calibrate", false);

    public static final int DRIVER_PRIORITY = 0;
    public static final int AUTO_PRIORITY = 1;
    public static final int SNAP_PRIORITY = 2;
    public static final int AIM_PRIORITY = 3;

    // Priority should be one of the priority levels above
    public static record DriveRequest(int priority, Translation2d robotRelTranslation, DriveRequestType type) {
    }

    public static record TurnRequest(int priority, Rotation2d turn) {
    }

    private static final DriveRequest NULL_DRIVE = new DriveRequest(Integer.MIN_VALUE, new Translation2d(0, 0), DriveRequestType.OpenLoopVoltage);
    private static final TurnRequest NULL_TURN = new TurnRequest(Integer.MIN_VALUE, new Rotation2d(0));

    private final FieldInfo fieldInfo;

    private final AHRS gyro;
    private final SwerveModule[] modules;
    private final SwerveKinematics kinematics;
    private final SwerveEstimator estimator;

    private SwerveModulePosition[] prevPositions;
    private Rotation2d prevGyroAngle;

    private DriveRequest currentDriveRequest;
    private TurnRequest currentTurnRequest;
    private int lastSelectedPriority;

    public SwerveDrive(FieldInfo fieldInfo) {
        this.fieldInfo = fieldInfo;
        gyro = new AHRS(SPI.Port.kMXP);

        SwerveModule.Info[] infos = Constants.kSwerveModuleInfos;

        modules = new SwerveModule[infos.length];
        Translation2d[] positions = new Translation2d[infos.length];
        for (int i = 0; i < modules.length; i++) {
            SwerveModule.Info info = infos[i];

            SwerveModuleConstants moduleConstants = SWERVE_MODULE_BUILDER.createModuleConstants(
                    info.turnId(), info.driveId(), info.encoderId(),
                    info.offset().get(),
                    info.position().getX(), info.position().getY(),
                    false);

            modules[i] = new SwerveModule(moduleConstants, info.name(), info.canBus());
            positions[i] = info.position();
        }

        this.kinematics = new SwerveKinematics(positions, Constants.kMaxAchievableSpeed);
        this.estimator = new SwerveEstimator(fieldInfo);

        prevPositions = null;
        currentDriveRequest = NULL_DRIVE;
        currentTurnRequest = NULL_TURN;

        // Configure pathing
        AutoBuilder.configureHolonomic(
                this::getEstimatedPose,
                this::setPose,
                this::getRobotRelativeSpeeds,
                (speeds) ->
                    driveAndTurn(AUTO_PRIORITY, speeds, DriveRequestType.Velocity),
                new HolonomicPathFollowerConfig(
                        new PIDConstants(8.0),
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
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < states.length; i++) {
            states[i] = modules[i].getCurrentState();
        }
        return states;
    }

    public SwerveModulePosition[] getCurrentModulePositions(boolean refresh) {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < positions.length; i++) {
            positions[i] = modules[i].getPosition(refresh);
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
        for (int i = 0; i < modules.length; i++) {
            SwerveModule.Info info = Constants.kSwerveModuleInfos[i];

            StatusSignal<Double> position = modules[i].getCANcoder().getAbsolutePosition();
            position.refresh();

            info.offset().set(info.offset().get() - position.getValue());

            CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
            cancoderConfigs.MagnetSensor.MagnetOffset = info.offset().get();
            modules[i].getCANcoder().getConfigurator().apply(cancoderConfigs);
        }
    }

    @Override
    public void periodic() {
        if (CALIBRATE_OFFSETS.get()) {
            CALIBRATE_OFFSETS.set(false);
            calibrate();
        }

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
            ChassisSpeeds robotRelSpeeds = ChassisSpeeds.discretize(requestedSpeeds, 0.020);
            SwerveModuleState[] targetStates = kinematics.getStates(robotRelSpeeds);
            for (int i = 0; i < modules.length; i++) {
                modules[i].apply(targetStates[i], currentDriveRequest.type, SteerRequestType.MotionMagic);
            }
        }

        // Update estimator
        // Do refresh here, so we get the most up-to-date data
        SwerveModulePosition[] positions = getCurrentModulePositions(true);
        Rotation2d gyroAngle = gyro.getRotation2d();
        if (prevPositions != null) {
            Twist2d twist = kinematics.getTwistDelta(prevPositions, positions);

            // We trust the gyro more than the kinematics estimate
            if (RobotBase.isReal() && gyro.isConnected()) {
                twist.dtheta = gyroAngle.getRadians() - prevGyroAngle.getRadians();
            }

            estimator.update(twist);
        }
        prevPositions = positions;
        prevGyroAngle = gyroAngle;
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
        return gyro.getRotation2d();
    }

    @Override
    public void simulationPeriodic() {
        for (SwerveModule module : modules) {
            module.updateSim(0.02, RobotController.getBatteryVoltage());
        }
    }

    public int getLastSelectedPriority() {
        return lastSelectedPriority;
    }

    public TalonFX getDriveMotor(int module) {
        return modules[module].getDriveMotor();
    }

    public TalonFX getTurnMotor(int module) {
        return modules[module].getSteerMotor();
    }

    public void setEstimatorIgnoreVision(boolean ignoreVision) {
        estimator.setIgnoreVision(ignoreVision);
    }

    public boolean hasSeenWhereWeAre() {
        return estimator.hasSeenWhereWeAre();
    }
}
