package frc.robot.Subsystems.Drive;

import static frc.robot.Subsystems.Drive.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GlobalConstants;
import frc.robot.GlobalConstants.RobotMode;
import frc.robot.Subsystems.Vision.*;
import frc.robot.Subsystems.Drive.DriveIO.DriveIOInputs;
import frc.robot.TeamLib.subsystem.*;

public class Drive extends Subsystem<DriveStates> {

	private static Drive instance;
	private final DriveIO io;
	private final DriveIOInputs inputs = new DriveIOInputs();
	private final double kS = GlobalConstants.ROBOT_MODE == RobotMode.SIM ? SIM_KS : REAL_KS;
	private final double kV = GlobalConstants.ROBOT_MODE == RobotMode.SIM ? SIM_KV : REAL_KV;
	private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
		TRACK_WIDTH
	);
	private XboxController controller = new XboxController(0);

	private boolean slowMode = false; // Slow mode variable

	private Drive(DriveIO io) {
		super("Drive", DriveStates.TANK_DRIVE);
		this.io = io;
		// Configure SysId
	}

	private final PIDController anglePID = new PIDController(0, 0, 0);
	private final PIDController distancePID = new PIDController(0, 0, 0);

	public XboxController getController() {
		return controller;
	}

	public static Drive getInstance() {
		if (instance == null) {
			switch (GlobalConstants.ROBOT_MODE) {
				case REAL:
					instance = new Drive(new DriveIOReal());
					break;
				case SIM:
					instance = new Drive(new DriveIOSim());
					break;
				case TESTING:
					instance = new Drive(new DriveIOReal());
					break;
				default:
					throw new IllegalStateException(
						"Unexpected value: " + GlobalConstants.ROBOT_MODE
					);
			}
		}
		return instance;
	}

	@Override
	public void runState() {
		io.updateInputs(inputs);
		getState().driveRobot();
	}

	/** Runs the drive at the desired velocity. */
	public void runClosedLoop(ChassisSpeeds speeds) {
		var wheelSpeeds = kinematics.toWheelSpeeds(speeds);
		runClosedLoop(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
	}

	/** Runs the drive at the desired left and right velocities. */
	public void runClosedLoop(double leftMetersPerSec, double rightMetersPerSec) {
		double leftRadPerSec = leftMetersPerSec / WHEEL_RADIUS_METERS;
		double rightRadPerSec = rightMetersPerSec / WHEEL_RADIUS_METERS;
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
		double left = MathUtil.applyDeadband(leftSpeed, DEADBAND);
		double right = MathUtil.applyDeadband(rightSpeed, DEADBAND);

		runClosedLoop(left * MAX_SPEED_METERS_PER_SEC, right * MAX_SPEED_METERS_PER_SEC);
	}

	public void arcadeDrive(double forward, double rotation) {
		double x = MathUtil.applyDeadband(forward, DEADBAND);
		double z = MathUtil.applyDeadband(rotation, DEADBAND);

		// Calculate speeds
		var speeds = DifferentialDrive.arcadeDriveIK(x, z, true);

		// Apply output
		runClosedLoop(
			speeds.left * MAX_SPEED_METERS_PER_SEC,
			speeds.right * MAX_SPEED_METERS_PER_SEC
		);
	}


	/**
	 * Adds a vision measurement to the estimator.
	 *
	 * @param visionPose The pose of the robot as measured by the vision camera.
	 * @param timestamp The timestamp of the vision measurement in seconds.
	 

	/** Returns the position of the left wheels in meters. */

	public double getLeftPositionMeters() {
		return inputs.leftPositionRad * WHEEL_RADIUS_METERS;
	}

	/** Returns the position of the right wheels in meters. */

	public double getRightPositionMeters() {
		return inputs.rightPositionRad * WHEEL_RADIUS_METERS;
	}


	public void alignToBucket() {
		Vision vision = Vision.getInstance();
		Rotation2d yaw = vision.getYaw();
		Rotation2d pitch = vision.getPitch();
	
		double distanceToBucket = calculateDistanceFromPitch(pitch);
	
		double angleError = yaw.getRadians();
		double distanceError = distanceToBucket - 0; //replace with distance setpoint
	
		double angleCorrection = anglePID.calculate(angleError);
		double distanceCorrection = distancePID.calculate(distanceError);
	
		runClosedLoop(distanceCorrection, angleCorrection);
	}
	
	private double calculateDistanceFromPitch(Rotation2d pitch) {
		return 0 / Math.tan(pitch.getRadians());  // Replace 0 with the bucket height 
	}

	/** Returns the velocity of the left wheels in meters/second. */

	public double getLeftVelocityMetersPerSec() {
		return inputs.leftVelocityRadPerSec * WHEEL_RADIUS_METERS;
	}

	/** Returns the velocity of the right wheels in meters/second. */

	public double getRightVelocityMetersPerSec() {
		return inputs.rightVelocityRadPerSec * WHEEL_RADIUS_METERS;
	}

	/** Returns the average velocity in radians/second. */
	public double getCharacterizationVelocity() {
		return (inputs.leftVelocityRadPerSec + inputs.rightVelocityRadPerSec) / TWO;
	}

	/** Returns true if slow mode is enabled. */
	public boolean isSlowMode() {
		return slowMode;
	}

	// Optionally, you may want to add a setter for slow mode:
	public void setSlowMode(boolean enabled) {
		slowMode = enabled;
	}
}
