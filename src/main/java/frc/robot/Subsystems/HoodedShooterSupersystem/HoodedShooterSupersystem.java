package frc.robot.Subsystems.HoodedShooterSupersystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Subsystems.HoodedShooterSupersystem.HoodedShooterSupersystemConstants.*;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Subsystems.AdjustableHood.AdjustableHood;
import frc.robot.Subsystems.Shooter.Shooter;
import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class HoodedShooterSupersystem extends Subsystem<HoodedShooterSupersystemStates> {

	private static HoodedShooterSupersystem instance;
	private AdjustableHood hood;
	private Shooter shooter;

	public static HoodedShooterSupersystem getInstance() {
		if (instance == null) {
			instance = new HoodedShooterSupersystem();
		}
		return instance;
	}

	private HoodedShooterSupersystem() {
		super(SUBSYSTEM_NAME, HoodedShooterSupersystemStates.IDLE);
		hood = AdjustableHood.getInstance();
		shooter = Shooter.getInstance();
	}

	@Override
	public void runState() {
		if (getState() == HoodedShooterSupersystemStates.DYNAMIC) {
			hood.setDynamicHoodSetpoint(calculateHoodSetpoint());
			shooter.setDynamicShooterSetpoint(calculateShooterSetpoint());
		}

		hood.setState(getState().getAdjustableHoodState());
		shooter.setState(getState().getShooterState());

		hood.periodic();
		shooter.periodic();

		Logger.recordOutput(SUBSYSTEM_NAME + "/State", getState().getStateString());
	}

	/*
	 * Checks if the hood and shooter are both at their setpoints.
	 * The manager subsystem should use this method to determine when to switch from warming up to shooting.
	 * Those states should be identical but one tells the indexer to pass the ball to the shooter.
	 */
	public boolean readyToShoot() {
		return hood.atSetpoint() && shooter.atSetpoint();
	}

	private Angle calculateHoodSetpoint() {
		// Placeholder for actual hood setpoint calculation logic
		return Degrees.of(FAKE_VALUE); // Replace with actual calculation
	}

	private AngularVelocity calculateShooterSetpoint() {
		// Placeholder for actual shooter setpoint calculation logic
		return RotationsPerSecond.of(FAKE_VALUE_2); // Replace with actual calculation
	}

	// prolly want to call this in manager or move to shooter
	public ArrayList<Translation3d> calculateShotTrajectory(LinearVelocity shooterTangentialVelocity, AngularVelocity shooterAngularVel, Angle shooterAngle, Pose2d robotPose, ChassisSpeeds robotFieldRelSpeed) {
		ArrayList<Translation3d> trajectory = new ArrayList<>();
		Translation3d startingPosition = new Translation3d(robotPose.getX(), robotPose.getY(), GROUND_TO_SHOOTER.in(Meters));

		double vx = (Math.cos(shooterAngle.in(Radian)) * shooterTangentialVelocity.in(MetersPerSecond) * robotPose.getRotation().getCos()) + robotFieldRelSpeed.vxMetersPerSecond;
		double vy = (Math.cos(shooterAngle.in(Radian)) * shooterTangentialVelocity.in(MetersPerSecond) * robotPose.getRotation().getCos()) + robotFieldRelSpeed.vyMetersPerSecond;
		double vz = Math.sin(shooterAngle.in(Radian)) * shooterTangentialVelocity.in(MetersPerSecond);
		
		double firstTerm = shooterTangentialVelocity.in(MetersPerSecond) * Math.sin(shooterAngle.in(Radian));
		double secondTerm = Math.sqrt(firstTerm * firstTerm + 2 * GRAVITY.in(MetersPerSecondPerSecond) * GROUND_TO_SHOOTER.in(Meter));
		double aproximateTimeToGround = (firstTerm + secondTerm) / GRAVITY.in(MetersPerSecondPerSecond);

		for (int i = 0; i < aproximateTimeToGround; i += 0.02) {

			double netVelocity = Math.sqrt(vx * vx + vy * vy + vz * vz);
            double dragAccel = 0.5 * AIR_DENSITY * DRAG_COEFFICIENT * Math.PI * BALL_RADIUS.in(Meters) * BALL_RADIUS.in(Meters) * netVelocity / BALL_MASS.in(Kilogram);
			vx -= dragAccel * vx / netVelocity * dt;
			vy += MAGNUS_COEFFICIENT * shooterAngularVel.in(RotationsPerSecond) * netVelocity / BALL_MASS.in(Kilogram) * dt;
			vz -= (GRAVITY.in(MetersPerSecondPerSecond) + dragAccel * vy / netVelocity) * dt;

			Translation3d deltaPosition = new Translation3d(vx * dt, vy * dt, vz * dt);
			startingPosition = startingPosition.plus(deltaPosition);
			trajectory.add(startingPosition);
		}

		return trajectory;
	}
}
