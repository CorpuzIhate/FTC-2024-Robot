package Command_Based_TeleOp_2024_08_17.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class VacuumSubsystem extends SubsystemBase {
    private final Motor m_shoulder;
    public VacuumSubsystem( Motor shoulderMotor){
        m_shoulder = shoulderMotor;

    }
}
