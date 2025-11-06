package frc.robot.Subsystems.Drive;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** IO implementation for NavX. */
public class GyroIOReal implements GyroIO {

	private final AHRS navX = new AHRS(NavXComType.kMXP_SPI);

	@Override
	public void updateInputs(GyroIOInputs inputs) {
		inputs.connected = navX.isConnected();
		inputs.yawPosition = Rotation2d.fromDegrees(-navX.getAngle());
		inputs.yawVelocityRadPerSec = Units.degreesToRadians(-navX.getRawGyroZ());
	}
}
