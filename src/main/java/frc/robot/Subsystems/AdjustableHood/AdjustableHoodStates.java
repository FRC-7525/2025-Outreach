package frc.robot.Subsystems.AdjustableHood;

import org.team7525.subsystem.SubsystemStates;
import edu.wpi.first.units.measure.Angle;
import static frc.robot.Subsystems.AdjustableHood.AdjustableHoodConstants.*;

public enum AdjustableHoodStates implements SubsystemStates {
    IDLE("IDLE", IDLE_ANGLE),
    FIXED("FIXED", FIXED_ANGLE),
    DYNAMIC("DYNAMIC", null);

    private final String stateName;
    private final Angle setpoint;


    AdjustableHoodStates(String stateName, Angle setpoint) {
        this.stateName = stateName;
        this.setpoint = setpoint;

    }

    public String getStateString() {
        return stateName;
    }

    public Angle getSetpoint() {
        return setpoint;
    }


}
