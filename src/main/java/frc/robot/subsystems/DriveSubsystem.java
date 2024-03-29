// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import badlog.lib.BadLog;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import lib.Loggable;
import lib.MotorFaultLogger;

@SuppressWarnings("FieldCanBeLocal")
public class DriveSubsystem extends SubsystemBase implements Loggable {

    // The motors on the left side of the drive.
    private final WPI_TalonFX m_leftMaster = new WPI_TalonFX(DriveConstants.kLeftMotor1Port);
    private final WPI_TalonFX m_leftFollower = new WPI_TalonFX(DriveConstants.kLeftMotor2Port);
    public SimpleMotorFeedforward feedForward =  new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
    DriveConstants.kaVoltSecondsSquaredPerMeter);

    // The motors on the right side of the drive.
    private final WPI_TalonFX m_rightMaster = new WPI_TalonFX(DriveConstants.kRightMotor1Port);
    @SuppressWarnings("FieldCanBeLocal")
    private final WPI_TalonFX m_rightFollower = new WPI_TalonFX(DriveConstants.kRightMotor2Port);

    // The robot's drive
    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMaster, m_rightMaster);

    // The left-side drive encoder
    private final WPI_TalonFX m_leftEncoder = new WPI_TalonFX(DriveConstants.kLeftMotor1Port);

    // The right-side drive encoder
    private final WPI_TalonFX m_rightEncoder = new WPI_TalonFX(DriveConstants.kRightMotor1Port);
    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry m_odometry;
    public final PIDController leftPIDController;
    public final PIDController rightPIDController;
    final NetworkTableEntry m_xEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("X");
    final NetworkTableEntry m_yEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Y");
    final NetworkTableEntry m_thetaEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("theta");
    // The gyro sensor
    private final AHRS navX = new AHRS(SerialPort.Port.kUSB1);
    private final Gyro m_gyro = navX;
    private final Field2d m_field;
    private double maxSpeed;
    private boolean direction;

    /**
     * Creates a new DriveSubsystem.
     */
    public DriveSubsystem() {
        // Sets the distance per pulse for the encoders
        MotorFaultLogger.getInstance().add("LeftMaster", m_leftMaster);
        MotorFaultLogger.getInstance().add("LeftFollower", m_leftFollower);
        MotorFaultLogger.getInstance().add("RightMaster", m_rightMaster);
        MotorFaultLogger.getInstance().add("RightFollower", m_rightFollower);
        resetEncoders();
        m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
        m_rightFollower.follow(m_rightMaster);
        m_leftFollower.follow(m_leftMaster);
        m_field = new Field2d();
        SmartDashboard.putData("Field", m_field);
        leftPIDController = new PIDController(DriveConstants.kPDriveVel, DriveConstants.kIDriveVel, DriveConstants.kDDriveVel);
        rightPIDController = new PIDController(DriveConstants.kPDriveVel, DriveConstants.kIDriveVel, DriveConstants.kDDriveVel);
        direction = true;
    }

    public void setMaxSpeed(double maxSpeed) {
        this.maxSpeed = maxSpeed;
    }

    public void logInit() {
        //DriveConstants
        BadLog.createValue("DriveConstants/TrackWidthMeters", "" + Constants.DriveConstants.kTrackwidthMeters);
        BadLog.createValue("DriveConstants/EncoderCPR", "" + Constants.DriveConstants.kEncoderCPR);
        BadLog.createValue("DriveConstants/GearRatio", "" + Constants.DriveConstants.kGearRatio);
        BadLog.createValue("DriveConstants/WheelDiameterMeters", "" + Constants.DriveConstants.kWheelDiameterMeters);
        BadLog.createValue("DriveConstants/EncoderDistancePerPulse", "" + Constants.DriveConstants.kEncoderDistancePerPulse);
        BadLog.createValue("DriveConstants/ksVolts", "" + Constants.DriveConstants.ksVolts);
        BadLog.createValue("DriveConstants/kvVoltSecondsPerMeter", "" + Constants.DriveConstants.kvVoltSecondsPerMeter);
        BadLog.createValue("DriveConstants/kaVoltSecondsSquaredPerMeter", "" + Constants.DriveConstants.kaVoltSecondsSquaredPerMeter);
        BadLog.createValue("DriveConstants/kPDriveVel", "" + Constants.DriveConstants.kPDriveVel);
        BadLog.createValue("DriveConstants/kDDriveVel", "" + Constants.DriveConstants.kDDriveVel);

        //AutoConstants
        BadLog.createValue("AutoConstants/MaxSpeedMetersPerSecond", "" + Constants.AutoConstants.kMaxSpeedMetersPerSecond);
        BadLog.createValue("AutoConstants/MaxAccelerationMetersPerSecondSquared", "" + Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        BadLog.createValue("AutoConstants/RamseteB", "" + Constants.AutoConstants.kRamseteB);
        BadLog.createValue("AutoConstants/RamseteZeta", "" + Constants.AutoConstants.kRamseteZeta);

        BadLog.createTopic("FPGA Time", "s", Timer::getFPGATimestamp, "xaxis");
        BadLog.createTopic("Drivetrain/Odometry Pose X", "m", () -> getPose().getX(), "join:Drivetrain/Robot Pose X");
        BadLog.createTopic("Drivetrain/Odometry Pose Y", "m", () -> getPose().getY(), "join:Drivetrain/Robot Pose Y");
        BadLog.createTopic("Drivetrain/Odometry Pose Heading", "degrees", () -> getPose().getRotation().getDegrees(), "join:Drivetrain/Robot Pose Heading");
        BadLog.createTopic("Drivetrain/Left Wheel Measured Speed", "m/s", () -> getWheelSpeeds().leftMetersPerSecond, "join:Drivetrain/LeftWheelSpeed");
        BadLog.createTopic("Drivetrain/Right Wheel Measured Speed", "m/s", () -> getWheelSpeeds().rightMetersPerSecond, "join:Drivetrain/RightWheelSpeed");
        BadLog.createTopic("Drivetrain/Left Wheel Setpoint", "m/s", leftPIDController::getSetpoint, "join:Drivetrain/LeftWheelSpeed");
        BadLog.createTopic("Drivetrain/Right Wheel Setpoint", "m/s", rightPIDController::getSetpoint, "join:Drivetrain/RightWheelSpeed");
        BadLog.createTopic("Battery Voltage", "V", RobotController::getBatteryVoltage);

        //Gyro Data
        BadLog.createTopic("NavX/WorldLinearAccelX", "m/s^2", () -> (double) navX.getWorldLinearAccelX());
        BadLog.createTopic("NavX/WorldLinearAccelY", "m/s^2", () -> (double) navX.getWorldLinearAccelX());
        BadLog.createTopic("NavX/WorldLinearAccelZ", "m/s^2", () -> (double) navX.getWorldLinearAccelX());
        BadLog.createTopic("NavX/RawAccelX", "m/s^2", () -> (double) navX.getRawAccelX());
        BadLog.createTopic("NavX/RawAccelY", "m/s^2", () -> (double) navX.getRawAccelX());
        BadLog.createTopic("NavX/RawAccelZ", "m/s^2", () -> (double) navX.getRawAccelX());
        BadLog.createTopic("NavX/Yaw", "degrees", navX::getAngle, "join:Drivetrain/Robot Pose Heading");
    }

    public void customPeriodic() {
        // Update the odometry in the periodic block
        m_odometry.update(
                m_gyro.getRotation2d(), m_leftEncoder.getSelectedSensorPosition() * DriveConstants.kEncoderDistancePerPulse, -1 * m_rightEncoder.getSelectedSensorPosition() * DriveConstants.kEncoderDistancePerPulse);
        m_field.setRobotPose(m_odometry.getPoseMeters());
        Pose2d translation = m_odometry.getPoseMeters();
        m_xEntry.setNumber(translation.getX());
        m_yEntry.setNumber(translation.getY());
        m_thetaEntry.setNumber(translation.getRotation().getDegrees());
        m_field.setRobotPose(m_odometry.getPoseMeters());
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void setBrakeMode() {
        m_leftMaster.setNeutralMode(NeutralMode.Brake);
        m_leftFollower.setNeutralMode(NeutralMode.Brake);
        m_rightMaster.setNeutralMode(NeutralMode.Brake);
        m_rightFollower.setNeutralMode(NeutralMode.Brake);
    }

    public void setCoastMode() {
        m_leftMaster.setNeutralMode(NeutralMode.Coast);
        m_leftFollower.setNeutralMode(NeutralMode.Coast);
        m_rightMaster.setNeutralMode(NeutralMode.Coast);
        m_rightFollower.setNeutralMode(NeutralMode.Coast);
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        SmartDashboard.putNumber("left encoder speed", m_leftEncoder.getSelectedSensorVelocity());
        SmartDashboard.putNumber("right encoder speed", m_rightEncoder.getSelectedSensorVelocity());
        return new DifferentialDriveWheelSpeeds(m_leftEncoder.getSelectedSensorVelocity() * DriveConstants.kEncoderDistancePerPulse * 10, m_rightEncoder.getSelectedSensorVelocity() * DriveConstants.kEncoderDistancePerPulse * -10);
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, m_gyro.getRotation2d());
    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void arcadeDrive(double fwd, double rot) {
        m_drive.arcadeDrive(fwd, rot);
    }

    public void tankDrivePercentOutput(double left, double right) {
        if (direction) {
            m_drive.tankDrive(-1 * maxSpeed * left, -1 * maxSpeed * right);

        } else {
            m_drive.tankDrive(maxSpeed * right,  maxSpeed * left);
        }
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts  the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        m_leftMaster.setVoltage(leftVolts);
        m_rightMaster.setVoltage(-rightVolts);
        m_drive.feed();
    }

    public void GoForwards(double speed) {
        double currentLeftRPM = getWheelSpeeds().leftMetersPerSecond;
        double LeftmotorVoltage = leftPIDController.calculate(currentLeftRPM, speed) + feedForward.calculate(speed);
        m_leftMaster.setVoltage(-1*LeftmotorVoltage);

        double currentRightRPM = getWheelSpeeds().rightMetersPerSecond;
        double RightmotorVoltage = rightPIDController.calculate(currentRightRPM, speed) + feedForward.calculate(speed);
        m_rightMaster.setVoltage(RightmotorVoltage);
        
        SmartDashboard.putNumber("Drivetrain/Right Target Velocity", rightPIDController.getSetpoint());
        SmartDashboard.putNumber("Drivetrain/Right Error", rightPIDController.getPositionError()); // Actually velocity error
        SmartDashboard.putNumber("Drivetrain/Right Velocity", currentRightRPM);

        SmartDashboard.putNumber("Drivetrain/Left Target Velocity", leftPIDController.getSetpoint());
        SmartDashboard.putNumber("Drivetrain/Left Error", leftPIDController.getPositionError()); // Actually velocity error
        SmartDashboard.putNumber("Drivetrain/Left Velocity", currentLeftRPM);
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        m_leftEncoder.setSelectedSensorPosition(0);
        m_rightEncoder.setSelectedSensorPosition(0);
    }

    /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        m_gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return m_gyro.getRotation2d().getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return -m_gyro.getRate();
    }

    public void toggleDirection() {
        direction = !direction;
    }
}
