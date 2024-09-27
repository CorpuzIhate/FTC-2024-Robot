package Command_Based_TeleOp_2024_08_17.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Command_Based_TeleOp_2024_08_17.Subsystems.VacuumSubsystem;

public class PowerVacuumCMD extends CommandBase {
    private final VacuumSubsystem m_vacuumSub;
    private final double m_power;
    public PowerVacuumCMD(VacuumSubsystem vacuumSubsystem,
                          double power){
        m_power = power;
        m_vacuumSub = vacuumSubsystem;

        addRequirements(vacuumSubsystem);
    }
    @Override
    public void execute(){
        m_vacuumSub.setVacuumPower(m_power);
    }
    @Override
    public boolean isFinished(){
        return false;
    }



}
