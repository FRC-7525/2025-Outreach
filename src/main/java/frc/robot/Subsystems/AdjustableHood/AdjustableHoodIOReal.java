package frc.robot.Subsystems.AdjustableHood;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.GlobalConstants.ROBOT_MODE;
import static frc.robot.Subsystems.AdjustableHood.AdjustableHoodConstants.PIVOT_PID;
import static frc.robot.Subsystems.AdjustableHood.AdjustableHoodConstants.Real.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GlobalConstants.RobotMode;

public class AdjustableHoodIOReal implements AdjustableHoodIO {

	private TalonFX pivotMotor;
	private Angle hoodSetpoint;
	private ProfiledPIDController pivotController;

	public AdjustableHoodIOReal() {
		pivotMotor = new TalonFX(PIVOT_CAN_ID);
		pivotMotor.setPosition(0); // Zero the motor position
		pivotController = PIVOT_PID.get();
		pivotController.setTolerance(PIVOT_TOLERANCE.in(Rotations));
	}

	@Override
	public void updateInputs(AdjustableHoodIOInputs inputs) {
		inputs.hoodSetpoint = hoodSetpoint;
		inputs.hoodPosition = Units.rotationsToDegrees(
			pivotMotor.getPosition().getValueAsDouble() / GEAR_RATIO
		);

		if (ROBOT_MODE == RobotMode.TESTING) {
			SmartDashboard.putData("Hood PID Controller", pivotController);
		}
	}

	@Override
	public void setHoodSetpoint(Angle hoodSetpoint) {
		this.hoodSetpoint = hoodSetpoint;
		pivotMotor.set(
			pivotController.calculate(
				pivotMotor.getPosition().getValueAsDouble() / GEAR_RATIO,
				hoodSetpoint.in(Rotations)
			)
		);
	}

	@Override
	public boolean atHoodSetpoint() {
		return Math.abs((pivotMotor.getPosition().getValueAsDouble() / GEAR_RATIO) - hoodSetpoint.in(Rotations)) < PIVOT_TOLERANCE.in(Rotations);
	}
}
