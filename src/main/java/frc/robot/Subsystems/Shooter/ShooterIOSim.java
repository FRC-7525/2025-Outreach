package frc.robot.Subsystems.Shooter;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.GlobalConstants.SIMULATION_PERIOD;
import static frc.robot.Subsystems.Shooter.ShooterConstants.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {

	private TalonFX flywheelMotor;
	private TalonFXSimState flywheelMotorSim;
	private FlywheelSim flywheelSim;
	private SimpleMotorFeedforward flywheelController;
	private AngularVelocity flywheelSetpoint;

	public ShooterIOSim() {
		flywheelMotor = new TalonFX(0);
		flywheelMotorSim = flywheelMotor.getSimState();
		flywheelSim = new FlywheelSim(
			LinearSystemId.createFlywheelSystem(
				DCMotor.getFalcon500(1),
				FLYWHEEL_MOI.in(KilogramSquareMeters),
				FLYWHEEL_GEARING
			),
			DCMotor.getFalcon500(1)
		); // Lowkey idk

		flywheelController = FLYWHEEL_FF.get();
	}

	@Override
	public void updateInputs(ShooterIOInputs inputs) {
		flywheelSim.update(SIMULATION_PERIOD);

		// Update inputs
		inputs.shooterSetpoint = flywheelSetpoint;
		inputs.shooterVelocity = Units.radiansToDegrees(flywheelSim.getAngularVelocityRadPerSec());

		// Update simulated motor state
		flywheelMotorSim.setRotorVelocity(
			Units.radiansToRotations(flywheelSim.getAngularVelocityRadPerSec())
		);
	}

	@Override
	public void setShooterSetpoint(AngularVelocity setpoint) {
		flywheelSetpoint = setpoint;
		flywheelSim.setInputVoltage(
			flywheelController.calculate(
				Units.radiansToRotations(flywheelSim.getAngularVelocityRadPerSec()),
				flywheelSetpoint.in(RotationsPerSecond)
			)
		);
	}

	@Override
	public boolean atTargetSpeed() {
		return Math.abs(
            Units.radiansToRotations(flywheelSim.getAngularVelocityRadPerSec()) - flywheelSetpoint.in(RotationsPerSecond)
        ) < FLYWHEEL_TOLERANCE.in(RotationsPerSecond);
	}
}
