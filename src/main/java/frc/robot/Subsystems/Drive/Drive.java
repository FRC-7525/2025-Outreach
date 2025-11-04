// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.Subsystems.Drive;

import static frc.robot.Subsystems.Drive.DriveConstants.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GlobalConstants;
import frc.robot.GlobalConstants.RobotMode;
import frc.robot.Subsystems.Drive.DriveIO.DriveIOInputs;
import frc.robot.Subsystems.Drive.GyroIO.GyroIOInputs;
import frc.robot.TeamLib.subsystem.*;

public class Drive extends Subsystem<DriveStates> {
  private static Drive instance;
  private final DriveIO io;
  private final DriveIOInputs inputs = new DriveIOInputs();
  private final GyroIO gyroIO;
  private final GyroIOInputs gyroInputs = new GyroIOInputs();
  private final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(trackWidth);
  private final double kS = GlobalConstants.ROBOT_MODE == RobotMode.SIM ? simKs : realKs;
  private final double kV = GlobalConstants.ROBOT_MODE == RobotMode.SIM ? simKv : realKv;
  private final DifferentialDrivePoseEstimator poseEstimator =
      new DifferentialDrivePoseEstimator(kinematics, Rotation2d.kZero, 0.0, 0.0, Pose2d.kZero);
  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private double lastLeftPositionMeters = 0.0;
  private double lastRightPositionMeters = 0.0;
  private XboxController controller = new XboxController(0);

  private Drive(DriveIO io, GyroIO gyroIO) {
	super("Drive", DriveStates.TANK_DRIVE);
    this.io = io;
    this.gyroIO = gyroIO;
    // Configure SysId
  }
  public XboxController getController() {
	  return controller;
  }

  public static Drive getInstance() {
	if (instance == null) {
		switch (GlobalConstants.ROBOT_MODE) {
			case REAL:
				instance = new Drive(new DriveIOReal(), new GyroIOReal());
				break;
			case SIM:
				instance = new Drive(new DriveIOSim(), new GyroIO() {});
				break;
			case TESTING:
				instance = new Drive(new DriveIOReal(), new GyroIOReal());
				break;
			default:
				throw new IllegalStateException("Unexpected value: " + GlobalConstants.ROBOT_MODE);
		}
	}
	return instance;
  }

  @Override
  public void runState() {
    io.updateInputs(inputs);
    gyroIO.updateInputs(gyroInputs);
	getState().driveRobot();

    // Update gyro angle
    if (gyroInputs.connected) {
      // Use the real gyro angle
      rawGyroRotation = gyroInputs.yawPosition;
    } else {
      // Use the angle delta from the kinematics and module deltas
      Twist2d twist =
          kinematics.toTwist2d(
              getLeftPositionMeters() - lastLeftPositionMeters,
              getRightPositionMeters() - lastRightPositionMeters);
      rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      lastLeftPositionMeters = getLeftPositionMeters();
      lastRightPositionMeters = getRightPositionMeters();
    }

    // Update odometry
    poseEstimator.update(rawGyroRotation, getLeftPositionMeters(), getRightPositionMeters());
  }

  /** Runs the drive at the desired velocity. */
  public void runClosedLoop(ChassisSpeeds speeds) {
    var wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    runClosedLoop(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
  }

  /** Runs the drive at the desired left and right velocities. */
  public void runClosedLoop(double leftMetersPerSec, double rightMetersPerSec) {
    double leftRadPerSec = leftMetersPerSec / wheelRadiusMeters;
    double rightRadPerSec = rightMetersPerSec / wheelRadiusMeters;
    SmartDashboard.putNumber("Drive/LeftSetpointRadPerSec", leftRadPerSec);
    SmartDashboard.putNumber("Drive/RightSetpointRadPerSec", rightRadPerSec);

    double leftFFVolts = kS * Math.signum(leftRadPerSec) + kV * leftRadPerSec;
    double rightFFVolts = kS * Math.signum(rightRadPerSec) + kV * rightRadPerSec;
    io.setVelocity(leftRadPerSec, rightRadPerSec, leftFFVolts, rightFFVolts);
  }

  /** Runs the drive in open loop. */
  public void runOpenLoop(double leftVolts, double rightVolts) {
    io.setVoltage(leftVolts, rightVolts);
  }

  /** Stops the drive. */
  public void stop() {
    runOpenLoop(0.0, 0.0);
  }
  
  public void tankDrive(double leftSpeed, double rightSpeed) {
		  double left = MathUtil.applyDeadband(leftSpeed, 0.02);
		  double right = MathUtil.applyDeadband(rightSpeed, 0.02);


		  runClosedLoop(left * maxSpeedMetersPerSec, right * maxSpeedMetersPerSec);
  }

  public void arcadeDrive(double forward, double rotation) {
          double x = MathUtil.applyDeadband(forward, 0.02);
          double z = MathUtil.applyDeadband(rotation, 0.02);

          // Calculate speeds
          var speeds = DifferentialDrive.arcadeDriveIK(x, z, true);

          // Apply output
          runClosedLoop(speeds.left * maxSpeedMetersPerSec, speeds.right * maxSpeedMetersPerSec);
  }

  /** Returns the current odometry pose. */
  
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(
        rawGyroRotation, getLeftPositionMeters(), getRightPositionMeters(), pose);
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevMeters) {
    poseEstimator.addVisionMeasurement(visionPose, timestamp, visionMeasurementStdDevMeters);
  }

  /** Returns the position of the left wheels in meters. */
  
  public double getLeftPositionMeters() {
    return inputs.leftPositionRad * wheelRadiusMeters;
  }

  /** Returns the position of the right wheels in meters. */
  
  public double getRightPositionMeters() {
    return inputs.rightPositionRad * wheelRadiusMeters;
  }

  /** Returns the velocity of the left wheels in meters/second. */
  
  public double getLeftVelocityMetersPerSec() {
    return inputs.leftVelocityRadPerSec * wheelRadiusMeters;
  }

  /** Returns the velocity of the right wheels in meters/second. */
  
  public double getRightVelocityMetersPerSec() {
    return inputs.rightVelocityRadPerSec * wheelRadiusMeters;
  }

  /** Returns the average velocity in radians/second. */
  public double getCharacterizationVelocity() {
    return (inputs.leftVelocityRadPerSec + inputs.rightVelocityRadPerSec) / 2.0;
  }
}