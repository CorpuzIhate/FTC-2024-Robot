package Command_Based_TeleOp_2024_08_17.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import Command_Based_TeleOp_2024_08_17.Constants;

public class ShoulderSubsystem extends SubsystemBase {
    private final Motor m_ShoulderMotor;
    private final PIDFController m_ShoulderfeedForward;
    public ShoulderSubsystem(Motor shoulder_Motor){
        m_ShoulderMotor = shoulder_Motor;
        m_ShoulderfeedForward = new PIDFController(
                Constants.kP,
                Constants.kI,
                Constants.kD,
                Constants.kF);

    }
    public Motor getShoulderMotor(){
        return m_ShoulderMotor;
    }
    public PIDFController getShoulderFeedforward(){
        return  m_ShoulderfeedForward;
    }


}
