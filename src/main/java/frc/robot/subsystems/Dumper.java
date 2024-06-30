package frc.robot.subsystems;

import frc.robot.Constants.*;
import frc.robot.Constants.DumperConstants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Dumper extends SubsystemBase {

    public static WPI_TalonSRX dumperMotor = new WPI_TalonSRX(DumperConstants.DUMPER_MOTOR_PORT);

    private PIDController rotPID = new PIDController(0.000005, 0, 0.0000001);

    public static double currentPosition = 0.0;
    private double wantedPosition = 0.0;

    public Dumper() {
        dumperMotor.setSelectedSensorPosition(0.0);
        currentPosition = dumperMotor.getSelectedSensorPosition();
        wantedPosition = 0;
    }

    public double calamp(double mi, double ma, double v) {
        return Math.max(mi, Math.min(ma, v));
    }

    @Override
    public void periodic() {
        currentPosition = dumperMotor.getSelectedSensorPosition();
        SmartDashboard.putNumber("Dumper Position", currentPosition);
        SmartDashboard.putNumber("Wanted DUmper", wantedPosition);
        dumperMotor.set(calamp(-0.7, 0.7, rotPID.calculate(currentPosition, wantedPosition)));
        System.out.println(rotPID.calculate(currentPosition, wantedPosition));
    }

    public void rotate() {
        // Rotate 60 degrees (Just value from testing)
        wantedPosition = wantedPosition + ((4096.f * 100.f) / 6.0d);
    }

}
