package frc.robot.Subsystems.HoodedShooterSupersystem;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.Subsystems.HoodedShooterSupersystem.HoodedShooterSupersystemConstants.*;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Subsystems.AdjustableHood.AdjustableHood;
import frc.robot.Subsystems.Shooter.Shooter;
import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class HoodedShooterSupersystem extends Subsystem<HoodedShooterSupersystemStates> {

	private static HoodedShooterSupersystem instance;
	private AdjustableHood hood;
	private Shooter shooter;
	private InterpolatingDoubleTreeMap hoodCalculator;

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
		hoodCalculator = new InterpolatingDoubleTreeMap();
		for (int i = 0; i < ANGLES.length; i++) {
			hoodCalculator.put(DISTANCES[i], ANGLES[i]);
		}
	}

	@Override
	public void runState() {
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

	public Angle calculateHoodSetpoint() {
		// ZERO is placeholder for logic to get distance from the target.
		return Degrees.of(hoodCalculator.get((double) 0));
	}

	public AngularVelocity calculateShooterSetpoint() {
		return MEDIUM_SPEED;
	}
}
