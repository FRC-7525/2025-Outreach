package frc.robot.Subsystems.Shooter;

import static frc.robot.GlobalConstants.*;
import static frc.robot.Subsystems.Shooter.ShooterConstants.SUBSYSTEM_NAME;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.Shooter.ShooterIO.ShooterIOInputs;
import frc.robot.TeamLib.subsystem.*;

public class Shooter extends Subsystem<ShooterStates> {

	private ShooterIO io;
	private ShooterIOInputs inputs;
	private AngularVelocity dynamicSetpoint;
	private static Shooter instance;

	public static Shooter getInstance() {
		if (instance == null) {
			ShooterIO shooterIO =
				switch (ROBOT_MODE) {
					case SIM -> new ShooterIOSim();
					case REAL -> new ShooterIOReal();
					case TESTING -> new ShooterIOReal();
				};
			instance = new Shooter(shooterIO);
		}

		return instance;
	}

	private Shooter(ShooterIO io) {
		super(SUBSYSTEM_NAME, ShooterStates.IDLE);
		this.io = io;
		inputs = new ShooterIOInputs();
	}

	@Override
	public void runState() {
		if (getState() == ShooterStates.DYNAMIC) {
			io.setShooterSetpoint(dynamicSetpoint);
		} else {
			io.setShooterSetpoint(getState().getSetpoint());
		}

		io.updateInputs(inputs);
		SmartDashboard.putString("Shooter/State", getState().getStateString());
	}

	public boolean atSetpoint() {
		return io.atTargetSpeed();
	}
}
