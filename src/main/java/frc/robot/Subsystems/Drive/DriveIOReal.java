package frc.robot.Subsystems.Drive;

import static frc.robot.Subsystems.Drive.DriveConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.GlobalConstants.Controllers;
import frc.robot.Subsystems.Vision.Vision;
import java.io.File;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class DriveIOReal implements DriveIO {

	public DriveIOInputs inputs;

	private SwerveInputStream swerveInputs;
	private final SwerveDrive swerveDrive;
	private boolean slow;

	public DriveIOReal() {
		SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
		try {
			File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
			swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(
				MAX_SPEED.magnitude(),
				new Pose2d(INIT_POSE_X, INIT_POSE_Y, Rotation2d.fromDegrees(0))
			);
		} catch (Exception e) {
			throw new RuntimeException("Failed to create SwerveDrive", e);
		}
		swerveDrive.setMotorIdleMode(true);

		swerveInputs = SwerveInputStream.of(
			swerveDrive,
			() -> -Controllers.DRIVER_CONTROLLER.getLeftY(),
			() -> -Controllers.DRIVER_CONTROLLER.getLeftX()
		)
			.withControllerRotationAxis(() -> -Controllers.DRIVER_CONTROLLER.getRightX())
			.allianceRelativeControl(true)
			.driveToPoseEnabled(false);

		slow = false;
	}

	@Override
	public void updateInputs(DriveIOInputs inputs) {}

	@Override
	public SwerveDrive getDrive() {
		swerveInputs.aim(Vision.getInstance().getTargetPose(0));
		return swerveDrive;
	}

	@Override
	public ChassisSpeeds getSwerveInputs() {
		return swerveInputs.get();
	}

	@Override
	public void setSpeed() {
		if (slow) {
			if (Controllers.DRIVER_CONTROLLER.getBButtonPressed()) {
				slow = false;
				swerveInputs.scaleTranslation(SLOW_SPEED);
				swerveInputs.scaleRotation(SLOW_SPEED);
			}
		} else if (Controllers.DRIVER_CONTROLLER.getBButtonPressed()) {
			slow = true;
			swerveInputs.scaleTranslation(NORMAL_SPEED);
			swerveInputs.scaleRotation(NORMAL_SPEED);
		}
	}

	@Override
	public void zeroGyro() {
		swerveDrive.resetOdometry(
			new Pose2d(
				swerveDrive.getPose().getX(),
				swerveDrive.getPose().getY(),
				Rotation2d.fromDegrees(0)
			)
		);
	}

	@Override
	public void addVisionMeasurement(
		Pose2d visionPose,
		double timestamp,
		Matrix<N3, N1> visionMeasurementStdDevs
	) {
		swerveDrive.addVisionMeasurement(visionPose, timestamp, visionMeasurementStdDevs);
		swerveDrive.updateOdometry();
	}

	@Override
	public SwerveInputStream getSwerveInputStream() {
		return swerveInputs;
	}
}
