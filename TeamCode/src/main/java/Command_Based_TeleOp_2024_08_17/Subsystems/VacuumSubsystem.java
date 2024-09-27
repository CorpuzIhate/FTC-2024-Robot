package Command_Based_TeleOp_2024_08_17.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Command_Based_TeleOp_2024_08_17.Constants;

public class VacuumSubsystem extends SubsystemBase {
    CRServo ContinousVacuumServo;
    ServoEx targetVacuumServo;
    public VacuumSubsystem(HardwareMap hMap){
        ContinousVacuumServo = hMap.get(CRServo.class, Constants.Servos.VacuumTag);

    }
    public void setVacuumPower(double power){
        ContinousVacuumServo.set(power);
    }

}
