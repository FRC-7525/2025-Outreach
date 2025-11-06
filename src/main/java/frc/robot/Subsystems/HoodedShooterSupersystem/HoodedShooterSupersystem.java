package frc.robot.Subsystems.HoodedShooterSupersystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Subsystems.HoodedShooterSupersystem.HoodedShooterSupersystemConstants.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Subsystems.AdjustableHood.AdjustableHood;
import frc.robot.Subsystems.Shooter.Shooter;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

class ShooterDataPoint {

	private AngularVelocity speed;
	private Angle angle;
	private Distance distance;

	public ShooterDataPoint(AngularVelocity speed, Angle angle, Distance distance) {
		this.speed = speed;
		this.angle = angle;
		this.distance = distance;
	}

	public AngularVelocity getSpeed() {
		return speed;
	}

	public Angle getAngle() {
		return angle;
	}

	public Distance getDistance() {
		return distance;
	}
}

public class HoodedShooterSupersystem extends Subsystem<HoodedShooterSupersystemStates> {

	private static HoodedShooterSupersystem instance;
	private AdjustableHood hood;
	private Shooter shooter;
	private int dataLength;
	private List<ShooterDataPoint> shots;
	private ShooterDataPoint currentShot;

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
		dataLength = SHOOTER_DATA_POINTS.size() - 1;
		shots = SHOOTER_DATA_POINTS;
		currentShot = null;
	}

	@Override
	public void runState() {
		hood.setState(getState().getAdjustableHoodState());
		shooter.setState(getState().getShooterState());
		if (getState() == HoodedShooterSupersystemStates.DYNAMIC) {
			currentShot = interpolateShotValues(Limelight.getInstance().getDistanceToTarget());
		}

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

	public ShooterDataPoint interpolateShotValues(Distance targetDistance) {
		shots.sort((a, b) -> a.getDistance().compareTo(b.getDistance()));
		for (int i = 0; i < dataLength; i++) {
			ShooterDataPoint s1 = shots.get(i);
			ShooterDataPoint s2 = shots.get(i + 1);

			if (
				targetDistance.in(Meters) >= s1.getDistance().in(Meters) &&
				targetDistance.in(Meters) <= s2.getDistance().in(Meters)
			) {
				double ratio =
					(targetDistance.in(Meters) - s1.getDistance().in(Meters)) /
					(s2.getDistance().in(Meters) - s1.getDistance().in(Meters));
				// Interpolate speed
				double interpolatedSpeedValue =
					s1.getDistance().in(Meters) +
					ratio *
					(s1.getSpeed().in(RotationsPerSecond) - s1.getSpeed().in(RotationsPerSecond));
				AngularVelocity interpolatedSpeed = RotationsPerSecond.of(interpolatedSpeedValue);
				// Interpolate angle
				double interpolatedAngleValue =
					s1.getAngle().in(Degrees) +
					ratio * (s2.getAngle().in(Degrees) - s1.getAngle().in(Degrees));
				Angle interpolatedAngle = Degrees.of(interpolatedAngleValue);

				// Set states
				return new ShooterDataPoint(interpolatedSpeed, interpolatedAngle, targetDistance); // Successful interpolation
			}
		}
		return null;
	}
}
