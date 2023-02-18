// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.drivers;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Preferences;

/** Add your docs here. */
public class PearadoxSparkMax extends CANSparkMax {
    /**
     * Creates a new CANSparkMax with the necessary configurations.
     * @param deviceId The device ID.
     * @param motorType The motor type (Brushed/Brushless).
     * @param idleMode The idle mode (kBrake/kCoast).
     * @param currentLimit The current limit.
     * @param isInverted The invert type of the motor.
     */
    public PearadoxSparkMax(int deviceId, MotorType motorType, IdleMode idleMode, int currentLimit, boolean isInverted){
        super(deviceId, motorType);
        this.restoreFactoryDefaults();
        this.setSmartCurrentLimit(currentLimit);
        this.setInverted(isInverted);
        this.setIdleMode(idleMode);
        this.burnFlash();
        String key = "Spark " + this.getDeviceId() + " Flashes";
        Preferences.setDouble(key, Preferences.getDouble(key, 0) + 1);
    }
}
