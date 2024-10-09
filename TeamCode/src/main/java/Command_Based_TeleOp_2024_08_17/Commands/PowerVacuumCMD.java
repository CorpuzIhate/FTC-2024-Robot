package Command_Based_TeleOp_2024_08_17.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Command_Based_TeleOp_2024_08_17.Subsystems.VacuumSubsystem;

public class PowerVacuumCMD extends CommandBase {
    private final VacuumSubsystem m_vacuumSub;
    private final CRServo m_ContinousVacuumServo;
    private final double m_power;

    public PowerVacuumCMD(VacuumSubsystem vacuumSubsystem,
                          double power,CRServo ContinousVacuumServo){
        m_power = power;
        m_vacuumSub = vacuumSubsystem;
        m_ContinousVacuumServo = ContinousVacuumServo;

        addRequirements(vacuumSubsystem);

    }
    @Override
    public void execute(){
        m_ContinousVacuumServo.set(m_power);
    }
    @Override
    public boolean isFinished() {

        return false;
    }
};