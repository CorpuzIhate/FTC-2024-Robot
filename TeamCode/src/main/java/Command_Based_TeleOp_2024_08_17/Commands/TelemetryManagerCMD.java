package Command_Based_TeleOp_2024_08_17.Commands;

import com.arcrobotics.ftclib.command.CommandBase;


import Command_Based_TeleOp_2024_08_17.Subsystems.TelemetryManagerSubsystem;


public class TelemetryManagerCMD extends CommandBase {
    TelemetryManagerSubsystem m_TelemetryManagerSubsystem;

    public TelemetryManagerCMD(TelemetryManagerSubsystem telemetryManagerSubsystem) {
        m_TelemetryManagerSubsystem = telemetryManagerSubsystem;
        addRequirements(telemetryManagerSubsystem);
    }

    @Override
    public void initialize() {
        //delete ftc dashboard during competition to prevent errors.
        //FTC Dashboard is PROHIBITED during games but allowed during pits
    }
        @Override
        public void execute() {
            m_TelemetryManagerSubsystem.getTelemetryObject().update();
        }

    }