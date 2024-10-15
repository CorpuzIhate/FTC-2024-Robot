package Command_Based_TeleOp_2024_08_17.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Command_Based_TeleOp_2024_08_17.Subsystems.ShoulderSubsystem;

public class MoveShoulderCMD extends CommandBase {
    private final ShoulderSubsystem m_shoulderSub;
    private final Motor m_shoulderMotor;
    private final Telemetry m_dashboardTelemetry;

    public MoveShoulderCMD(ShoulderSubsystem shoulderSub, Motor shoulderMotor, Telemetry dashboardTelemetry){
        m_shoulderSub = shoulderSub;
        m_dashboardTelemetry = dashboardTelemetry;
        m_shoulderMotor = shoulderMotor;
        addRequirements(m_shoulderSub);
    }
//yolo
    @Override
    public void execute(){
        //m_shoulderMotor.set(1);
    }


}
